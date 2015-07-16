/**
 * RailCAN driver for srcpd.
 *
 * Copyright (C) 2015 Reinder Feenstra <reinderfeenstra@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#include <string.h>

#include "config-srcpd.h"
#include "railcan.h"
#include "srcp-gl.h"
#include "srcp-power.h"
#include "srcp-error.h"
#include "syslogmessage.h"

#define __railcan ((RAILCAN_DATA*)buses[bus].driverdata)

#define SYSLOG_BUS(bus, level, ...) \
    if (buses[bus].debuglevel >= level) { \
        syslog_bus(bus, level, __VA_ARGS__); \
    }


int readconfig_RAILCAN(xmlDocPtr doc, xmlNodePtr node, bus_t bus)
{
    buses[bus].driverdata = calloc(1, sizeof(struct _RAILCAN_DATA));

    if (buses[bus].driverdata == NULL)
        goto memory_error;

    buses[bus].type = SERVER_RAILCAN;
    buses[bus].init_func = &init_bus_RAILCAN;
    buses[bus].thr_func = &thr_RAILCAN;
    buses[bus].init_gl_func = &init_gl_RAILCAN;
    strcpy(buses[bus].description, "GL POWER LOCK DESCRIPTION");

    if (init_GL(bus, 127))
        SYSLOG_BUS(bus, DBG_ERROR, "Can't create array for locomotives");

    return 1;

  memory_error:
    free(buses[bus].driverdata);
    SYSLOG_BUS(bus, DBG_ERROR,
               "Memory allocation error in module '%s'.", node->name);
    return 0;
}

static int bus_scan_callback_RAILCAN(struct librailcan_bus *_bus,
                                     uint8_t address, uint8_t type)
{
    if (type == LIBRAILCAN_MODULETYPE_DCC) {
        void *user_data;

        int r = librailcan_bus_get_user_data(_bus, &user_data);
        if (r != LIBRAILCAN_STATUS_SUCCESS)
            return 0;

        bus_t bus = (bus_t) user_data;

        r = librailcan_module_open(__railcan->bus, address,
                                   &__railcan->module_dcc);
        if (r != LIBRAILCAN_STATUS_SUCCESS) {
            SYSLOG_BUS(bus, DBG_FATAL,
                       "librailcan_module_open failed (r=%d)", r);
            return 0;
        }
        else
            SYSLOG_BUS(bus, DBG_INFO,
                       "Found DCC module at address: %u", address);
    }
    return 0;
}

int init_bus_RAILCAN(bus_t bus)
{
    static char *protocols = "N";
    buses[bus].protocols = protocols;

    /* open railcan bus */
    int r = librailcan_bus_open_socketcan(buses[bus].device.interface.name,
                                          &__railcan->bus);
    if (r != LIBRAILCAN_STATUS_SUCCESS) {
        SYSLOG_BUS(bus, DBG_FATAL,
                   "Failed opening railcan bus: %s (r=%d)",
                   buses[bus].device.interface.name, r);
        return 0;               /* error */
    }
    else
        SYSLOG_BUS(bus, DBG_INFO, "Opened railcan bus: %s",
                   buses[bus].device.interface.name);

    /* set bus number as user data */
    r = librailcan_bus_set_user_data(__railcan->bus, (void *) bus);
    if (r != LIBRAILCAN_STATUS_SUCCESS) {
        SYSLOG_BUS(bus, DBG_FATAL,
                   "librailcan_bus_set_user_data failed (r=%d)", r);
        return 0;               /* error */
    }

    /* register scan callback */
    r = librailcan_bus_set_scan_callback(__railcan->bus,
                                         bus_scan_callback_RAILCAN);
    if (r != LIBRAILCAN_STATUS_SUCCESS) {
        SYSLOG_BUS(bus, DBG_FATAL,
                   "librailcan_bus_set_scan_callback failed (r=%d)", r);
        return 0;               /* error */
    }

    /* scan railcan bus */
    r = librailcan_bus_scan(__railcan->bus, 1, 126);
    if (r != LIBRAILCAN_STATUS_SUCCESS) {
        SYSLOG_BUS(bus, DBG_FATAL, "librailcan_bus_scan failed (r=%d)", r);
        return 0;               /* error */
    }
    else
        SYSLOG_BUS(bus, DBG_INFO, "Scanning railcan bus");

    int i;
    for (i = 0; i < 50; i++) {
        r = librailcan_bus_process(__railcan->bus, 0);
        if (r < LIBRAILCAN_STATUS_SUCCESS) {
            SYSLOG_BUS(bus, DBG_FATAL,
                       "librailcan_bus_process failed (r=%d)", r);
            return 0;           /* error */
        }

        if (__railcan->module_dcc != NULL)
            break;

        usleep(100 * 1000);     /* 100ms */
    }

    if (__railcan->module_dcc == NULL) {
        SYSLOG_BUS(bus, DBG_FATAL, "No DCC module found");
        return 0;               /* error */
    }

    /* activate DCC module */
    r = librailcan_module_set_active(__railcan->module_dcc,
                                     LIBRAILCAN_BOOL_TRUE);
    if (r != LIBRAILCAN_STATUS_SUCCESS) {
        if (buses[bus].debuglevel >= DBG_FATAL) {
            SYSLOG_BUS(bus, DBG_FATAL,
                       "librailcan_module_set_active failed (r=%d)", r);
        }
        return 0;               /* error */
    }

    return 0;
}

int init_gl_RAILCAN(gl_state_t * gl)
{
    if (gl->protocol == 'N') {
        switch (gl->protocolversion) {
            case 1:
                return ((gl->n_fs == 14) || (gl->n_fs == 28)
                        || (gl->n_fs == 128)) ? SRCP_OK : SRCP_WRONGVALUE;
        }
    }
    return SRCP_UNSUPPORTEDDEVICEPROTOCOL;
}

static void handle_power_command(bus_t bus)
{
    int r =
        librailcan_dcc_set_enabled(__railcan->module_dcc,
                                   buses[bus].
                                   power_state ? LIBRAILCAN_BOOL_TRUE :
                                   LIBRAILCAN_BOOL_FALSE);
    if (r != LIBRAILCAN_STATUS_SUCCESS)
        SYSLOG_BUS(bus, DBG_FATAL,
                   "librailcan_dcc_set_enabled failed (r=%d)", r);

    buses[bus].power_changed = 0;
    char msg[110];

    infoPower(bus, msg);
    enqueueInfoMessage(msg);
}

static void handle_gl_command(bus_t bus)
{
    int r;
    gl_state_t gl;
    dequeueNextGL(bus, &gl);

    uint16_t address = gl.id;
    if (gl.protocolversion == 2)
        address |= LIBRAILCAN_DCC_ADDRESS_LONG;

    /* speed and direction */
    if (gl.direction == 2) {
        r = librailcan_dcc_emergency_stop(__railcan->module_dcc, address);
        if (r != LIBRAILCAN_STATUS_SUCCESS)
            SYSLOG_BUS(bus, DBG_ERROR,
                       "librailcan_dcc_emergency_stop failed (r=%d)", r);
    }
    else {
        /* direction */
        librailcan_dcc_set_direction(__railcan->module_dcc, address,
                                     gl.
                                     direction ?
                                     LIBRAILCAN_DCC_DIRECTION_FORWARD :
                                     LIBRAILCAN_DCC_DIRECTIOM_REVERSE);
        if (r != LIBRAILCAN_STATUS_SUCCESS)
            SYSLOG_BUS(bus, DBG_ERROR,
                       "librailcan_dcc_set_direction failed (r=%d)", r);

        /* speed */
        uint8_t speed = gl.speed;

        switch (gl.n_fs) {
            case 14:
                speed |= LIBRAILCAN_DCC_SPEED_14;
                break;
            case 28:
                speed |= LIBRAILCAN_DCC_SPEED_28;
                break;
            case 128:
                speed |= LIBRAILCAN_DCC_SPEED_128;
                break;
        }

        librailcan_dcc_set_speed(__railcan->module_dcc, address, speed);
        if (r != LIBRAILCAN_STATUS_SUCCESS)
            SYSLOG_BUS(bus, DBG_ERROR,
                       "librailcan_dcc_set_speed failed (r=%d)", r);
    }

    /* functions */
    int i;
    for (i = 0; i < gl.n_fs; i++) {
        r = librailcan_dcc_set_function(__railcan->module_dcc, address, i,
                                        (gl.
                                         funcs & (1 << i)) ?
                                        LIBRAILCAN_DCC_FUNCTION_ENABLED :
                                        LIBRAILCAN_DCC_FUNCTION_DISABLED);
        if (r != LIBRAILCAN_STATUS_SUCCESS)
            SYSLOG_BUS(bus, DBG_ERROR,
                       "librailcan_dcc_set_function failed (r=%d)", r);
    }

    /* update cache */
    cacheSetGL(bus, gl.id, gl);
}

static void end_bus_thread(bus_thread_t * btd)
{
    int r;

    SYSLOG_BUS(btd->bus, DBG_INFO, "RailCAN bus terminated.");

    r = librailcan_module_close(((RAILCAN_DATA*)buses[btd->bus].driverdata)->module_dcc);
    if (r != LIBRAILCAN_STATUS_SUCCESS)
        SYSLOG_BUS(btd->bus, DBG_WARN,
                   "Failed closing railcan bus: %s (r=%d)",
                   buses[btd->bus].device.interface.name, r);

    r = librailcan_bus_close(((RAILCAN_DATA*)buses[btd->bus].driverdata)->bus);
    if (r != LIBRAILCAN_STATUS_SUCCESS)
        SYSLOG_BUS(btd->bus, DBG_WARN,
                   "Failed closing railcan bus: %s (r=%d)",
                   buses[btd->bus].device.interface.name, r);

    r = pthread_mutex_destroy(&buses[btd->bus].transmit_mutex);
    if (r != 0) {
        SYSLOG_BUS(btd->bus, DBG_WARN,
                   "pthread_mutex_destroy() failed: %s (errno = %d).",
                   strerror(r), r);
    }

    r = pthread_cond_destroy(&buses[btd->bus].transmit_cond);
    if (r != 0) {
        SYSLOG_BUS(btd->bus, DBG_WARN,
                   "pthread_mutex_init() failed: %s (errno = %d).",
                   strerror(r), r);
    }

    free(buses[btd->bus].driverdata);
    free(btd);
}

void *thr_RAILCAN(void *v)
{
    int last_cancel_state, last_cancel_type;
    bus_thread_t *btd = (bus_thread_t *) malloc(sizeof(bus_thread_t));

    if (btd == NULL)
        pthread_exit((void *) 1);
    btd->bus = (bus_t) v;
    btd->fd = -1;

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &last_cancel_state);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &last_cancel_type);

    /* register cleanup routine */
    pthread_cleanup_push((void *) end_bus_thread, (void *) btd);

    SYSLOG_BUS(btd->bus, DBG_INFO, "RailCAN bus started (interface = %s).",
               buses[btd->bus].device.interface.name);

    /* enter endless loop to process work tasks */
    while (1) {
        librailcan_bus_process(((RAILCAN_DATA *) buses[btd->bus].
                                driverdata)->bus, 10);

        if (buses[btd->bus].power_changed == 1)
            handle_power_command(btd->bus);

        if (!queue_GL_isempty(btd->bus))
            handle_gl_command(btd->bus);
    }

    /* run the cleanup routine */
    pthread_cleanup_pop(1);
    return NULL;
}
