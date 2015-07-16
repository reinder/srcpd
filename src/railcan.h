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

#ifndef _RAILCAN_H
#define _RAILCAN_H

#include <libxml/tree.h>        /*xmlDocPtr, xmlNodePtr */
#include <librailcan.h>

typedef struct _RAILCAN_DATA {
    struct librailcan_bus *bus;
    struct librailcan_module *module_dcc;
} RAILCAN_DATA;

int readconfig_RAILCAN(xmlDocPtr doc, xmlNodePtr node, bus_t bus);
int init_bus_RAILCAN(bus_t bus);
int init_gl_RAILCAN(gl_state_t *);
void *thr_RAILCAN(void *);

#endif
