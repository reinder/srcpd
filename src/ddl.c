/* $Id$ */

/* DDL:  Busdriver connected with a booster only without any special hardware.
 *
 */

#include "stdincludes.h"

#include "config-srcpd.h"
#include "io.h"
#include "ddl.h"
#include "srcp-fb.h"
#include "srcp-ga.h"
#include "srcp-gl.h"
#include "srcp-power.h"
#include "srcp-srv.h"
#include "srcp-info.h"
#include "srcp-error.h"

#define __DDL ((DDL_DATA*)busses[busnumber].driverdata)

/* +----------------------------------------------------------------------+ */
/* | DDL - Digital Direct for Linux                                       | */
/* +----------------------------------------------------------------------+ */
/* | Copyright (c) 2002 - 2003 Vogt IT                                    | */
/* +----------------------------------------------------------------------+ */
/* | This source file is subject of the GNU general public license 2,     | */
/* | that is bundled with this package in the file COPYING, and is        | */
/* | available at through the world-wide-web at                           | */
/* | http://www.gnu.org/licenses/gpl.txt                                  | */
/* | If you did not receive a copy of the PHP license and are unable to   | */
/* | obtain it through the world-wide-web, please send a note to          | */
/* | gpl-license@vogt-it.com so we can mail you a copy immediately.       | */
/* +----------------------------------------------------------------------+ */
/* | Authors:   Torsten Vogt vogt@vogt-it.com                             | */
/* |                                                                      | */
/* +----------------------------------------------------------------------+ */

/***************************************************************/
/* erddcd - Electric Railroad Direct Digital Command Daemon    */
/*    generates without any other hardware digital commands    */
/*    to control electric model railroads                      */
/***************************************************************/

#include "config-srcpd.h"
#include "ddl_maerklin.h"
#include "ddl_nmra.h"

#ifdef __CYGWIN__
#define TIOCOUTQ 0x5411
#endif

/********* Q U E U E *****************/

void queue_init(long int busnumber)
{
    int error, i;

    error = pthread_mutex_init(&__DDL->queue_mutex, NULL);
    if (error) {
        DBG(0, DBG_ERROR, "DDL Engine: Cannot create mutex. Abort!");
        exit(1);
    }

    pthread_mutex_lock(&__DDL->queue_mutex);
    for (i = 0; i < QSIZE; i++) {
        __DDL->QData[i].packet_type = QNOVALIDPKT;
        __DDL->QData[i].addr = 0;
        memset(__DDL->QData[i].packet, 0, PKTSIZE);
    }
    __DDL->queue_in = 0;
    __DDL->queue_out = 0;

    __DDL->queue_initialized = TRUE;
    pthread_mutex_unlock(&__DDL->queue_mutex);
}

int queue_empty(long int busnumber)
{
    return (__DDL->queue_in == __DDL->queue_out);
}

void queue_add(long int busnumber, int addr, char *const packet,
               int packet_type, int packet_size)
{
    pthread_mutex_lock(&__DDL->queue_mutex);
    memset(__DDL->QData[__DDL->queue_in].packet, 0, PKTSIZE);
    memcpy(__DDL->QData[__DDL->queue_in].packet, packet, packet_size);
    __DDL->QData[__DDL->queue_in].packet_type = packet_type;
    __DDL->QData[__DDL->queue_in].packet_size = packet_size;
    __DDL->QData[__DDL->queue_in].addr = addr;
    __DDL->queue_in++;
    if (__DDL->queue_in == QSIZE)
        __DDL->queue_in = 0;
    pthread_mutex_unlock(&__DDL->queue_mutex);
}

int queue_get(long int busnumber, int *addr, char *packet,
              int *packet_size)
{
    int rtc;

    if (!__DDL->queue_initialized || queue_empty(busnumber))
        return QEMPTY;

    pthread_mutex_lock(&__DDL->queue_mutex);
    memcpy(packet, __DDL->QData[__DDL->queue_out].packet, PKTSIZE);
    rtc = __DDL->QData[__DDL->queue_out].packet_type;
    *packet_size = __DDL->QData[__DDL->queue_out].packet_size;
    *addr = __DDL->QData[__DDL->queue_out].addr;
    __DDL->QData[__DDL->queue_out].packet_type = QNOVALIDPKT;
    __DDL->queue_out++;
    if (__DDL->queue_out == QSIZE)
        __DDL->queue_out = 0;
    pthread_mutex_unlock(&__DDL->queue_mutex);

    return rtc;
}


/* functions to open, initialize and close comport */

//#if linux
//int init_serinfo(int fd, int divisor, struct serial_struct **serinfo)
//{
//  if (*serinfo==NULL)
//  {
//    *serinfo=malloc(sizeof(struct serial_struct));
//    if (!*serinfo) return -1;
//  }
//  if (ioctl(fd, TIOCGSERIAL, *serinfo) < 0) return -1;
//  (*serinfo)->custom_divisor = divisor;
//  (*serinfo)->flags = ASYNC_SPD_CUST | ASYNC_SKIP_TEST;
//  return 0;
//}
//
//int set_customdivisor(int fd, struct serial_struct *serinfo)
//{
//  if (ioctl(fd, TIOCSSERIAL, serinfo) < 0) return -1;
//  return 0;
//}
//
//int reset_customdivisor(int fd)
//{
//  struct serial_struct *serinfo = NULL;
//
//  serinfo=malloc(sizeof(struct serial_struct));
//  if (!serinfo) return -1;
//  if (ioctl(fd, TIOCGSERIAL, serinfo) < 0) return -1;
//  (serinfo)->custom_divisor = 0;
//  (serinfo)->flags = 0;
//  if (ioctl(fd, TIOCSSERIAL, serinfo) < 0) return -1;
//  return 0;
//}
//#endif

int setSerialMode(long int busnumber, int mode)
{
    switch (mode) {
    case SDM_MAERKLIN:
        if (__DDL->SERIAL_DEVICE_MODE != SDM_MAERKLIN) {
            if (tcsetattr
                (busses[busnumber].fd, TCSANOW,
                 &__DDL->maerklin_dev_termios) != 0) {
                DBG(busnumber, DBG_ERROR,
                    "error setting serial device mode to marklin!");
                return -1;
            }
//#if linux
//        if (__DDL->IMPROVE_NMRADCC_TIMING)
//        {
//          if (set_customdivisor(busses[busnumber].fd, __DDL->serinfo_marklin)!=0)
//          {
//            DBG(busnumber, DBG_ERROR, "cannot set custom divisor for maerklin of serial device!");
//            return -1;
//          }
//        }
//#endif
            __DDL->SERIAL_DEVICE_MODE = SDM_MAERKLIN;
        }
        break;
    case SDM_NMRA:
        if (__DDL->SERIAL_DEVICE_MODE != SDM_NMRA) {
            if (tcsetattr
                (busses[busnumber].fd, TCSANOW,
                 &__DDL->nmra_dev_termios) != 0) {
                DBG(busnumber, DBG_ERROR,
                    "error setting serial device mode to NMRA!");
                return -1;
            }
//#if linux
//        if (__DDL -> IMPROVE_NMRADCC_TIMING)
//        {
//          if (set_customdivisor(busses[busnumber].fd, __DDL->serinfo_nmradcc)!=0)
//          {
//            DBG(busnumber, DBG_ERROR, "cannot set custom divisor for nmra dcc of serial device!");
//            return -1;
//          }
//        }
//#endif
            __DDL->SERIAL_DEVICE_MODE = SDM_NMRA;
        }
        break;
    default:
        DBG(busnumber, DBG_ERROR,
            "error setting serial device to unknown mode!");
        return -1;
    }
    return 0;
}

int init_lineDDL(long int busnumber)
{
    /* opens and initializes the selected comport */
    /* returns a file handle                      */

    int dev;

    dev = open(busses[busnumber].device, O_WRONLY);     /* open comport                 */
    if (dev < 0) {
        DBG(busnumber, DBG_FATAL,
            "device %s can not be opened for writing. Abort!",
            busses[busnumber].device);
        exit(1);                /* theres no chance to continue */
    }

//#if linux
//  if (reset_customdivisor(dev)!=0)
//  {
//    DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!", busses[busnumber].device);
//    exit(1);
//  }
//#endif

    tcflush(dev, TCOFLUSH);
    tcflow(dev, TCOOFF);        /* suspend output */

    if (tcgetattr(dev, &__DDL->maerklin_dev_termios) != 0) {
        DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!",
            busses[busnumber].device);
        exit(1);
    }
    if (tcgetattr(dev, &__DDL->nmra_dev_termios) != 0) {
        DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!",
            busses[busnumber].device);
        exit(1);
    }

    /* init termios structur for maerklin mode */
    __DDL->maerklin_dev_termios.c_lflag &=
        ~(ECHO | ICANON | IEXTEN | ISIG);
    __DDL->maerklin_dev_termios.c_oflag &= ~(OPOST);
    __DDL->maerklin_dev_termios.c_iflag &=
        ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    __DDL->maerklin_dev_termios.c_cflag &= ~(CSIZE | PARENB);
    __DDL->maerklin_dev_termios.c_cflag |= CS6; /* 6 data bits      */
    cfsetospeed(&__DDL->maerklin_dev_termios, B38400);  /* baud rate: 38400 */
    cfsetispeed(&__DDL->maerklin_dev_termios, B38400);  /* baud rate: 38400 */

    /* init termios structur for nmra mode */
    __DDL->nmra_dev_termios.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    __DDL->nmra_dev_termios.c_oflag &= ~(OPOST);
    __DDL->nmra_dev_termios.c_iflag &=
        ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    __DDL->nmra_dev_termios.c_cflag &= ~(CSIZE | PARENB);
    __DDL->nmra_dev_termios.c_cflag |= CS8;     /* 8 data bits      */
    if (__DDL->IMPROVE_NMRADCC_TIMING) {
        cfsetospeed(&__DDL->nmra_dev_termios, B115200); /* baud rate: 115200 */
        cfsetispeed(&__DDL->nmra_dev_termios, B115200); /* baud rate: 115200 */
    }
    else {
        cfsetospeed(&__DDL->nmra_dev_termios, B19200);  /* baud rate: 19200 */
        cfsetispeed(&__DDL->nmra_dev_termios, B19200);  /* baud rate: 19200 */
    }

//#if linux
//  // if IMPROVE_NMRADCC_TIMING is set, we have to initialize some
//  // structures
//  if (__DDL -> IMPROVE_NMRADCC_TIMING)
//  {
//    if (init_serinfo(dev,3,&__DDL->serinfo_marklin)!=0)
//    {
//      DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!", busses[busnumber].device);
//      exit(1);
//    }
//    if (init_serinfo(dev,7,&__DDL->serinfo_nmradcc)!=0)
//    {
//      DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!",  busses[busnumber].device);
//      exit(1);
//    }
//  }
//#endif
    busses[busnumber].fd = dev; /* we need that value at the next step */
    /* setting serial device to default mode */
    if (!setSerialMode(busnumber, SDM_DEFAULT) == 0) {
        DBG(busnumber, DBG_FATAL, "error initializing device %s. Abort!",
            busses[busnumber].device);
        exit(1);
    }

    return dev;
}


/****** routines for maerklin packet pool *********************/

void init_MaerklinPacketPool(long int busnumber)
{
    int i, j;

    if (pthread_mutex_init(&__DDL->maerklin_pktpool_mutex, NULL)) {
        DBG(busnumber, DBG_ERROR,
            "cannot create mutex (maerklin packet pool). Abort!");
        exit(1);
    }

    pthread_mutex_lock(&__DDL->maerklin_pktpool_mutex);

    for (i = 0; i <= MAX_MARKLIN_ADDRESS; i++)
        __DDL->MaerklinPacketPool.knownAdresses[i] = 0;

    __DDL->MaerklinPacketPool.NrOfKnownAdresses = 1;
    __DDL->MaerklinPacketPool.knownAdresses[__DDL->MaerklinPacketPool.
                                            NrOfKnownAdresses - 1] = 81;
    /* generate idle packet */
    for (i = 0; i < 4; i++) {
        __DDL->MaerklinPacketPool.packets[81].packet[2 * i] = HI;
        __DDL->MaerklinPacketPool.packets[81].packet[2 * i + 1] = LO;
        for (j = 0; j < 4; j++) {
            __DDL->MaerklinPacketPool.packets[81].f_packets[j][2 * i] = HI;
            __DDL->MaerklinPacketPool.packets[81].f_packets[j][2 * i + 1] =
                LO;
        }
    }
    for (i = 4; i < 9; i++) {
        __DDL->MaerklinPacketPool.packets[81].packet[2 * i] = LO;
        __DDL->MaerklinPacketPool.packets[81].packet[2 * i + 1] = LO;
        for (j = 0; j < 4; j++) {
            __DDL->MaerklinPacketPool.packets[81].f_packets[j][2 * i] = LO;
            __DDL->MaerklinPacketPool.packets[81].f_packets[j][2 * i + 1] =
                LO;
        }
    }

    pthread_mutex_unlock(&__DDL->maerklin_pktpool_mutex);
}

char *get_maerklin_packet(long int busnumber, int adr, int fx)
{
    return __DDL->MaerklinPacketPool.packets[adr].f_packets[fx];
}

void update_MaerklinPacketPool(long int busnumber, int adr,
                               char const *const sd_packet,
                               char const *const f1, char const *const f2,
                               char const *const f3, char const *const f4)
{
    int i, found;

    DBG(busnumber, DBG_INFO, "update MM Packet Pool: %d", adr);
    found = 0;
    for (i = 0; i < __DDL->MaerklinPacketPool.NrOfKnownAdresses && !found;
         i++)
        if (__DDL->MaerklinPacketPool.knownAdresses[i] == adr)
            found = TRUE;

    pthread_mutex_lock(&__DDL->maerklin_pktpool_mutex);
    memcpy(__DDL->MaerklinPacketPool.packets[adr].packet, sd_packet, 18);
    memcpy(__DDL->MaerklinPacketPool.packets[adr].f_packets[0], f1, 18);
    memcpy(__DDL->MaerklinPacketPool.packets[adr].f_packets[1], f2, 18);
    memcpy(__DDL->MaerklinPacketPool.packets[adr].f_packets[2], f3, 18);
    memcpy(__DDL->MaerklinPacketPool.packets[adr].f_packets[3], f4, 18);
    pthread_mutex_unlock(&__DDL->maerklin_pktpool_mutex);

    if (__DDL->MaerklinPacketPool.NrOfKnownAdresses == 1
        && __DDL->MaerklinPacketPool.knownAdresses[0] == 81)
        __DDL->MaerklinPacketPool.NrOfKnownAdresses = 0;

    if (!found) {
        __DDL->MaerklinPacketPool.knownAdresses[__DDL->MaerklinPacketPool.
                                                NrOfKnownAdresses] = adr;
        __DDL->MaerklinPacketPool.NrOfKnownAdresses++;
    }
}

/**********************************************************/

/****** routines for NMRA packet pool *********************/

void init_NMRAPacketPool(long int busnumber)
{
    int i, j;
    char idle_packet[] = "11111111111111101111111100000000001111111110";
    char idle_pktstr[PKTSIZE];

    if (pthread_mutex_init(&__DDL->nmra_pktpool_mutex, NULL)) {
        DBG(busnumber, DBG_ERROR,
            "cannot create mutex (nmra packet pool). Abort!");
        exit(1);
    }

    pthread_mutex_lock(&__DDL->nmra_pktpool_mutex);
    for (i = 0; i <= MAX_NMRA_ADDRESS; i++)
        __DDL->NMRAPacketPool.knownAdresses[i] = 0;

    __DDL->NMRAPacketPool.NrOfKnownAdresses = 0;

    pthread_mutex_unlock(&__DDL->nmra_pktpool_mutex);

    /* put idle packet in packet pool */
    j = translateBitstream2Packetstream(__DDL->NMRADCC_TR_V, idle_packet,
                                        idle_pktstr, FALSE);
    update_NMRAPacketPool(busnumber, 255, idle_pktstr, j, idle_pktstr, j);

    /* generate and override idle_data */
    for (i = 0; i < MAXDATA; i++)
        __DDL->idle_data[i] = idle_pktstr[i % j];
    for (i = (MAXDATA / j) * j; i < MAXDATA; i++)
        __DDL->idle_data[i] = 0xC6;
    memcpy(__DDL->NMRA_idle_data, idle_pktstr, j);
}

void update_NMRAPacketPool(long int busnumber, int adr,
                           char const *const packet, int packet_size,
                           char const *const fx_packet, int fx_packet_size)
{
    int i, found;

    found = 0;
    for (i = 0; i <= __DDL->NMRAPacketPool.NrOfKnownAdresses && !found;
         i++)
        if (__DDL->NMRAPacketPool.knownAdresses[i] == adr)
            found = TRUE;

    pthread_mutex_lock(&__DDL->nmra_pktpool_mutex);
    memcpy(__DDL->NMRAPacketPool.packets[adr].packet, packet, packet_size);
    __DDL->NMRAPacketPool.packets[adr].packet_size = packet_size;
    memcpy(__DDL->NMRAPacketPool.packets[adr].fx_packet, fx_packet,
           fx_packet_size);
    __DDL->NMRAPacketPool.packets[adr].fx_packet_size = fx_packet_size;
    pthread_mutex_unlock(&__DDL->nmra_pktpool_mutex);

    if (__DDL->NMRAPacketPool.NrOfKnownAdresses == 1
        && __DDL->NMRAPacketPool.knownAdresses[0] == 255)
        __DDL->NMRAPacketPool.NrOfKnownAdresses = 0;

    if (!found) {
        __DDL->NMRAPacketPool.knownAdresses[__DDL->NMRAPacketPool.
                                            NrOfKnownAdresses] = adr;
        __DDL->NMRAPacketPool.NrOfKnownAdresses++;
    }
}


void waitUARTempty_COMMON(long int busnumber)
{
    int result;
    do {                        /* wait until UART is empty */
#if linux
        ioctl(busses[busnumber].fd, TIOCSERGETLSR, &result);
#else
        ioctl(busses[busnumber].fd, TCSADRAIN, &result);
#endif
    } while (!result);
}

void waitUARTempty_COMMON_USLEEPPATCH(long int busnumber)
{
    int result;
    do {                        /* wait until UART is empty */
#if linux
        ioctl(busses[busnumber].fd, TIOCSERGETLSR, &result);
#else
        ioctl(busses[busnumber].fd, TCSADRAIN, &result);
#endif
        usleep(__DDL->WAITUART_USLEEP_USEC);
    } while (!result);
}

/* new Version of waitUARTempty() for a clean NMRA-DCC signal */
/* from Harald Barth */
#define SLEEPFACTOR 48000l      /* used in waitUARTempty() */
#define NUMBUFFERBYTES 1024     /* used in waitUARTempty() */

void waitUARTempty_CLEANNMRADCC(long int busnumber)
{
    int outbytes;

    /* look how many bytes are in UART's outbuffer */
    ioctl(busses[busnumber].fd, TIOCOUTQ, &outbytes);

    if (outbytes > NUMBUFFERBYTES) {
        struct timespec sleeptime;
        /* calculate sleeptime */
        sleeptime.tv_sec = outbytes / SLEEPFACTOR;
        sleeptime.tv_nsec =
            (outbytes % SLEEPFACTOR) * (1000000000l / SLEEPFACTOR);

        nanosleep(&sleeptime, NULL);
    }
}

int checkRingIndicator(long int busnumber)
{
    int result, arg;
    result = ioctl(busses[busnumber].fd, TIOCMGET, &arg);
    if (result >= 0) {
        if (arg & TIOCM_RI) {
            DBG(busnumber, DBG_INFO,
                "ring indicator alert. Power on tracks stopped!");
            return 1;
        }
        return 0;
    }
    else {
        DBG(busnumber, DBG_ERROR,
            "ioctl() call failed. Power on tracks stopped!");
        return 1;
    }
}

int checkShortcut(long int busnumber)
{
    int result, arg;
    time_t short_now = 0;
    struct timeval tv_shortcut = { 0, 0 };

    result = ioctl(busses[busnumber].fd, TIOCMGET, &arg);
    if (result >= 0) {
        if (((arg & TIOCM_DSR) && (!__DDL->DSR_INVERSE))
            || ((~arg & TIOCM_DSR) && (__DDL->DSR_INVERSE))) {
            if (__DDL->short_detected == 0) {
                gettimeofday(&tv_shortcut, NULL);
                __DDL->short_detected =
                    tv_shortcut.tv_sec * 1000000 + tv_shortcut.tv_usec;
            }
            gettimeofday(&tv_shortcut, NULL);
            short_now = tv_shortcut.tv_sec * 1000000 + tv_shortcut.tv_usec;
            if (__DDL->SHORTCUTDELAY <=
                (short_now - __DDL->short_detected)) {
                DBG(busnumber, DBG_INFO,
                    "shortcut detected. Power on tracks stopped!");
                return 1;
            }
        }
        else {
            __DDL->short_detected = 0;
            return 0;
        }
    }
    else {
        DBG(busnumber, DBG_INFO,
            "ioctl() call failed. Power on tracks stopped!");
        return 1;
    }
    return 0;
}

void send_packet(long int busnumber, int addr, char *packet,
                 int packet_size, int packet_type, int refresh)
{
    int i, laps;
    /* arguments for nanosleep and marklin loco decoders (19KHz) */
    struct timespec rqtp_btw19K = { 0, 1250000 };     /* ==> busy waiting */
    struct timespec rqtp_end19K = { 0, 1700000 };     /* ==> busy waiting */
    /* arguments for nanosleep and marklin solenoids/func decoders (38KHz) */
    struct timespec rqtp_btw38K = { 0, 625000 };      /* ==> busy waiting */
    struct timespec rqtp_end38K = { 0, 850000 };      /* ==> busy waiting */

    waitUARTempty(busnumber);

    switch (packet_type) {
    case QM1LOCOPKT:
    case QM2LOCOPKT:
        if (setSerialMode(busnumber, SDM_MAERKLIN) < 0)
            return;
        if (refresh)
            laps = 2;
        else
            laps = 4;           /* YYTV 9 */
        for (i = 0; i < laps; i++) {
            nanosleep(&rqtp_end19K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 18);
            waitUARTempty(busnumber);
            nanosleep(&rqtp_btw19K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 18);
            waitUARTempty(busnumber);
        }
        break;
    case QM2FXPKT:
        if (setSerialMode(busnumber, SDM_MAERKLIN) < 0)
            return;
        if (refresh)
            laps = 2;
        else
            laps = 3;           /* YYTV 6 */
        for (i = 0; i < laps; i++) {
            nanosleep(&rqtp_end19K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 18);
            waitUARTempty(busnumber);
            nanosleep(&rqtp_btw19K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 18);
            waitUARTempty(busnumber);
        }
        break;
    case QM1FUNCPKT:
    case QM1SOLEPKT:
        if (setSerialMode(busnumber, SDM_MAERKLIN) < 0)
            return;
        for (i = 0; i < 2; i++) {
            nanosleep(&rqtp_end38K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 9);
            waitUARTempty(busnumber);
            nanosleep(&rqtp_btw38K, &__DDL->rmtp);
            write(busses[busnumber].fd, packet, 9);
            waitUARTempty(busnumber);
        }
        break;
    case QNBLOCOPKT:
    case QNBACCPKT:
        if (setSerialMode(busnumber, SDM_NMRA) < 0)
            return;
        if (__DDL->IMPROVE_NMRADCC_TIMING) {
            improve_nmradcc_write(busses[busnumber].fd, packet,
                                  packet_size);
            waitUARTempty(busnumber);
            improve_nmradcc_write(busses[busnumber].fd,
                                  __DDL->NMRA_idle_data, 13);
            waitUARTempty(busnumber);
            improve_nmradcc_write(busses[busnumber].fd, packet,
                                  packet_size);
        }
        else {
            write(busses[busnumber].fd, packet, packet_size);
            waitUARTempty(busnumber);
            write(busses[busnumber].fd, __DDL->NMRA_idle_data, 13);
            waitUARTempty(busnumber);
            write(busses[busnumber].fd, packet, packet_size);
        }
        break;
    }
    if (__DDL->ENABLED_PROTOCOLS & EP_MAERKLIN)
        nanosleep(&rqtp_end38K, &__DDL->rmtp);
    if (setSerialMode(busnumber, SDM_DEFAULT) < 0)
        return;
}

void improve_nmradcc_write(long int busnumber, char *packet,
                           int packet_size)
{
    // Idee: NMRA l�uft mit 17000 Baud
    // 115200 Baud / 7 = 16457 Baud
    // -> jedes Bit 7 mal senden
    char improve_nmradcc_packet[packet_size * 7];
    int i, j;
    for (i = 0; i < packet_size; i++) {
        for (j = 0; j < 7; j++) {
            improve_nmradcc_packet[i * 7 + j] = packet[i];
        }
    }
    write(busses[busnumber].fd, improve_nmradcc_packet, (packet_size * 7));
}

void refresh_loco(long int busnumber)
{
    int adr;

    if (__DDL->maerklin_refresh) {
        adr =
            __DDL->MaerklinPacketPool.knownAdresses[__DDL->
                                                    last_refreshed_maerklin_loco];
        tcflush(busses[busnumber].fd, TCOFLUSH);
        if (__DDL->last_refreshed_maerklin_fx < 0)
            send_packet(busnumber, adr,
                        __DDL->MaerklinPacketPool.packets[adr].packet, 18,
                        QM2LOCOPKT, TRUE);
        else
            send_packet(busnumber, adr,
                        __DDL->MaerklinPacketPool.packets[adr].
                        f_packets[__DDL->last_refreshed_maerklin_fx], 18,
                        QM2FXPKT, TRUE);
        __DDL->last_refreshed_maerklin_fx++;
        if (__DDL->last_refreshed_maerklin_fx == 4) {
            __DDL->last_refreshed_maerklin_fx = -1;
            __DDL->last_refreshed_maerklin_loco++;
            if (__DDL->last_refreshed_maerklin_loco >=
                __DDL->MaerklinPacketPool.NrOfKnownAdresses)
                __DDL->last_refreshed_maerklin_loco = 0;
        }
    }
    else {
        adr =
            __DDL->NMRAPacketPool.knownAdresses[__DDL->
                                                last_refreshed_nmra_loco];
        if (adr >= 0) {
            if (__DDL->last_refreshed_nmra_fx < 0) {
                send_packet(busnumber, adr,
                            __DDL->NMRAPacketPool.packets[adr].packet,
                            __DDL->NMRAPacketPool.packets[adr].packet_size,
                            QNBLOCOPKT, TRUE);
                __DDL->last_refreshed_nmra_fx = 0;
            }
            else {
                send_packet(busnumber, adr,
                            __DDL->NMRAPacketPool.packets[adr].fx_packet,
                            __DDL->NMRAPacketPool.packets[adr].
                            fx_packet_size, QNBLOCOPKT, TRUE);
                __DDL->last_refreshed_nmra_fx = 1;
            }
        }
        if (__DDL->last_refreshed_nmra_fx == 1) {
            __DDL->last_refreshed_nmra_loco++;
            __DDL->last_refreshed_nmra_fx = -1;
            if (__DDL->last_refreshed_nmra_loco >=
                __DDL->NMRAPacketPool.NrOfKnownAdresses)
                __DDL->last_refreshed_nmra_loco = 0;
        }
    }
    if (__DDL->ENABLED_PROTOCOLS == (EP_MAERKLIN | EP_NMRADCC))
        __DDL->maerklin_refresh = !__DDL->maerklin_refresh;
}

long int compute_delta(struct timeval tv1, struct timeval tv2)
{
    long int delta_sec;
    long int delta_usec;

    delta_sec = tv2.tv_sec - tv1.tv_sec;
    if (delta_sec == 0)
        delta_usec = tv2.tv_usec - tv1.tv_usec;
    else {
        if (delta_sec == 1)
            delta_usec = tv2.tv_usec + (1000000 - tv1.tv_usec);
        else
            delta_usec =
                1000000 * (delta_sec - 1) + tv2.tv_usec + (1000000 -
                                                           tv1.tv_usec);
    }
    return delta_usec;
}

void set_SerialLine(long int busnumber, int line, int mode)
{
    int result, arg;
    result = ioctl(busses[busnumber].fd, TIOCMGET, &arg);
    if (result >= 0) {
        switch (line) {
        case SL_DTR:
            if (mode == OFF)
                arg &= ~TIOCM_DTR;
            if (mode == ON)
                arg |= TIOCM_DTR;
            break;
        case SL_DSR:
            if (mode == OFF)
                arg &= ~TIOCM_DSR;
            if (mode == ON)
                arg |= TIOCM_DSR;
            break;
        case SL_RI:
            if (mode == OFF)
                arg &= ~TIOCM_RI;
            if (mode == ON)
                arg |= TIOCM_RI;
            break;
        case SL_RTS:
            if (mode == OFF)
                arg &= ~TIOCM_RTS;
            if (mode == ON)
                arg |= TIOCM_RTS;
            break;
        case SL_CTS:
            if (mode == OFF)
                arg &= ~TIOCM_CTS;
            if (mode == ON)
                arg |= TIOCM_CTS;
            break;
        }
        result = ioctl(busses[busnumber].fd, TIOCMSET, &arg);
    }
    if (result < 0)
        DBG(busnumber, DBG_ERROR,
            "ioctl() call failed. Serial line not set! (%d: %d)", line,
            mode);
}

/* ************************************************ */

void set_lines_off(long int busnumber)
{
    /* set interface lines to the off state */
    tcflush(busses[busnumber].fd, TCOFLUSH);
    tcflow(busses[busnumber].fd, TCOOFF);
    set_SerialLine(busnumber, SL_DTR, OFF);
}

int check_lines(long int busnumber)
{
    char msg[110];
    if (__DDL->CHECKSHORT)
        if (checkShortcut(busnumber) == 1) {
            busses[busnumber].power_changed = 1;
            strcpy(busses[busnumber].power_msg, "SHORTCUT DETECTED");
            infoPower(busnumber, msg);
            queueInfoMessage(msg);
        }
    if (__DDL->RI_CHECK)
        if (checkRingIndicator(busnumber) == 1) {
            busses[busnumber].power_changed = 1;
            strcpy(busses[busnumber].power_msg, "RINGINDICATOR DETECTED");
            infoPower(busnumber, msg);
            queueInfoMessage(msg);
        }

    if (busses[busnumber].power_changed == 1) {
        if (busses[busnumber].power_state == 0) {
            set_lines_off(busnumber);
            DBG(busnumber, DBG_INFO, "refresh cycle stopped.");
        }
        if (busses[busnumber].power_state == 1) {
            set_SerialLine(busnumber, SL_DTR, ON);
            DBG(busnumber, DBG_INFO, "refresh cycle restarted.");
        }
        busses[busnumber].power_changed = 0;
        infoPower(busnumber, msg);
        queueInfoMessage(msg);
    }

    if (busses[busnumber].power_state == 0) {
        usleep(1000);
        return (1);
    }
    return (0);
}

void *thr_refresh_cycle(void *v)
{
    int l;
    struct sched_param sparam;
    int policy;
    int packet_size;
    int packet_type;
    char packet[PKTSIZE];
    int addr;
    long int busnumber = (long int) v;
    struct timeval tv1, tv2;
    struct timezone tz;

    /* argument for nanosleep to do non-busy waiting */
    struct timespec rqtp_sleep = {0, 2500000};    /* ==> non-busy waiting */

    /* set the best waitUARTempty-Routine */
    waitUARTempty = waitUARTempty_COMMON;
    if (__DDL->WAITUART_USLEEP_PATCH)
        waitUARTempty = waitUARTempty_COMMON_USLEEPPATCH;
    if ((__DDL->ENABLED_PROTOCOLS & EP_NMRADCC)
        && !(__DDL->ENABLED_PROTOCOLS & EP_MAERKLIN))
        waitUARTempty = waitUARTempty_CLEANNMRADCC;

    pthread_getschedparam(pthread_self(), &policy, &sparam);
    sparam.sched_priority = 10;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sparam);

    /* some boosters like the Maerklin 6017 must be initialized */
    tcflow(busses[busnumber].fd, TCOON);
    set_SerialLine(busnumber, SL_DTR, ON);
    write(busses[busnumber].fd, "SRCP-DAEMON", 11);
    tcflush(busses[busnumber].fd, TCOFLUSH);

    /* now set some serial lines */
    tcflow(busses[busnumber].fd, TCOOFF);
    set_SerialLine(busnumber, SL_RTS, ON);      /* +12V for ever on RTS   */
    set_SerialLine(busnumber, SL_CTS, OFF);     /* -12V for ever on CTS   */
    set_SerialLine(busnumber, SL_DTR, OFF);     /* disable booster output */

    tcflow(busses[busnumber].fd, TCOON);
    set_SerialLine(busnumber, SL_DTR, ON);

    l = 0;
    gettimeofday(&tv1, &tz);
    for (;;) {
        if (check_lines(busnumber))
            continue;
        write(busses[busnumber].fd, __DDL->idle_data, MAXDATA);
        packet_type = queue_get(busnumber, &addr, packet, &packet_size);
        /*now,look at commands */
        if (packet_type > QNOVALIDPKT) {
            tcflush(busses[busnumber].fd, TCOFLUSH);
            while (packet_type > QNOVALIDPKT) {
                if (check_lines(busnumber))
                    continue;
                send_packet(busnumber, addr, packet, packet_size,
                            packet_type, FALSE);
                if (__DDL->ENABLED_PROTOCOLS == (EP_MAERKLIN | EP_NMRADCC))
                    write(busses[busnumber].fd, __DDL->NMRA_idle_data, 13);
                packet_type =
                    queue_get(busnumber, &addr, packet, &packet_size);
            }
        }
        /* no commands? Then we do a refresh */
        else {
            if (check_lines(busnumber))
                continue;
            gettimeofday(&tv2, &tz);
            /* but not every time! */
            if (compute_delta(tv1, tv2) > 100000) {
                refresh_loco(busnumber);
                gettimeofday(&tv1, &tz);
            }
            else
                nanosleep(&rqtp_sleep, &__DDL->rmtp);
        }
    }

    return NULL;
}

long int init_gl_DDL(struct _GLSTATE *gl)
{
    switch (gl->protocol) {
    case 'M':                  /* Motorola Codes */
        return (gl->protocolversion > 0
                && gl->protocolversion <= 5) ? SRCP_OK : SRCP_WRONGVALUE;
        break;
    case 'N':
        return (gl->protocolversion > 0
                && gl->protocolversion <= 5) ? SRCP_OK : SRCP_WRONGVALUE;
        break;
    }
    return SRCP_UNSUPPORTEDDEVICEPROTOCOL;
}

long int init_ga_DDL(struct _GASTATE *ga)
{
    switch (ga->protocol) {
    case 'M':                  /* Motorola Codes */
        return SRCP_OK;
        break;
    case 'N':
        return SRCP_OK;
        break;
    }
    return SRCP_UNSUPPORTEDDEVICEPROTOCOL;
}


int readconfig_DDL(xmlDocPtr doc, xmlNodePtr node, long int busnumber)
{
    busses[busnumber].type = SERVER_DDL;
    busses[busnumber].init_func = &init_bus_DDL;
    busses[busnumber].term_func = &term_bus_DDL;
    busses[busnumber].init_gl_func = &init_gl_DDL;
    busses[busnumber].init_ga_func = &init_ga_DDL;

    busses[busnumber].thr_func = &thr_sendrec_DDL;

    busses[busnumber].driverdata = malloc(sizeof(struct _DDL_DATA));
    strcpy(busses[busnumber].description, "GA GL POWER LOCK DESCRIPTION");

    __DDL->number_gl = 81;
    __DDL->number_ga = 324;

    __DDL->RI_CHECK = FALSE;    /* ring indicator checking      */
    __DDL->CHECKSHORT = FALSE;  /* default no shortcut checking */
    __DDL->DSR_INVERSE = FALSE; /* controls how DSR is used to  */
    /* check shorts                 */
    __DDL->SHORTCUTDELAY = 0;   /* usecs shortcut delay         */
    __DDL->NMRADCC_TR_V = 3;    /* version of the nmra dcc      */
    /* translation routine (1 or 2) */
    __DDL->ENABLED_PROTOCOLS = (EP_MAERKLIN | EP_NMRADCC);      /* enabled p's */
    __DDL->IMPROVE_NMRADCC_TIMING = 0;  /* NMRA DCC: improve timing     */

    __DDL->WAITUART_USLEEP_PATCH = 0;   /* enable/disbable usleep patch */
    __DDL->WAITUART_USLEEP_USEC = 0;    /* usecs for usleep patch       */

    __DDL->SERIAL_DEVICE_MODE = SDM_NOTINITIALIZED;

    xmlNodePtr child = node->children;
    xmlChar *txt = NULL;

    while (child != NULL) {
        if (xmlStrncmp(child->name, BAD_CAST "text", 4) == 0) {
            child = child->next;
            continue;
        }

        if (xmlStrcmp(child->name, BAD_CAST "number_gl") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->number_gl = atoi((char *) txt);
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "number_ga") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->number_ga = atoi((char *) txt);
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp
            (child->name, BAD_CAST "enable_ringindicator_checking") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->RI_CHECK =
                    (xmlStrcmp(txt, BAD_CAST "yes") == 0) ? TRUE : FALSE;
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "enable_checkshort_checking")
            == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->CHECKSHORT =
                    (xmlStrcmp(txt, BAD_CAST "yes") == 0) ? TRUE : FALSE;
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "inverse_dsr_handling") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            __DDL->DSR_INVERSE =
                (xmlStrcmp(txt, BAD_CAST "yes") == 0) ? TRUE : FALSE;
            xmlFree(txt);
        }

        else if (xmlStrcmp(child->name, BAD_CAST "enable_maerklin") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                if (xmlStrcmp(txt, BAD_CAST "yes") == 0)
                    __DDL->ENABLED_PROTOCOLS |= EP_MAERKLIN;
                else
                    __DDL->ENABLED_PROTOCOLS &= ~EP_MAERKLIN;

                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "enable_nmradcc") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                if (xmlStrcmp(txt, BAD_CAST "yes") == 0)
                    __DDL->ENABLED_PROTOCOLS |= EP_NMRADCC;
                else
                    __DDL->ENABLED_PROTOCOLS &= ~EP_NMRADCC;

                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "improve_nmradcc_timing") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->IMPROVE_NMRADCC_TIMING = (xmlStrcmp(txt,
                                                           BAD_CAST "yes")
                                                 == 0) ? TRUE : FALSE;
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "shortcut_failure_delay") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->SHORTCUTDELAY = atoi((char *) txt);
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "nmradcc_translation_routine")
            == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->NMRADCC_TR_V = atoi((char *) txt);
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "enable_usleep_patch") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->WAITUART_USLEEP_PATCH = (xmlStrcmp(txt,
                                                          BAD_CAST "yes")
                                                == 0) ? TRUE : FALSE;
                xmlFree(txt);
            }
        }

        else if (xmlStrcmp(child->name, BAD_CAST "usleep_usec") == 0) {
            txt = xmlNodeListGetString(doc, child->xmlChildrenNode, 1);
            if (txt != NULL) {
                __DDL->WAITUART_USLEEP_USEC = atoi((char *) txt);
                xmlFree(txt);
            }
        }

        else
            DBG(busnumber, DBG_INFO,
                    "WARNING, unknown tag found: \"%s\"!\n",
                    child->name);;


        child = child->next;
    }                           // while

    if (init_GA(busnumber, __DDL->number_ga)) {
        __DDL->number_ga = 0;
        DBG(busnumber, DBG_ERROR, "Can't create array for assesoirs");
    }

    if (init_GL(busnumber, __DDL->number_gl)) {
        __DDL->number_gl = 0;
        DBG(busnumber, DBG_ERROR, "Can't create array for locomotivs");
    }

    return (1);
}

long int term_bus_DDL(long int busnumber)
{
    /* store thread return value here */
    void *pThreadReturn;
    /* send cancel to refresh cycle */
    pthread_cancel(__DDL->ptid);
    /* wait until the refresh cycle has terminated */
    pthread_join(__DDL->ptid, &pThreadReturn);

    set_lines_off(busnumber);

    //pthread_cond_destroy(&(__DDL->refresh_cond));
    DBG(busnumber, DBG_INFO, "DDL bus %d terminating", busnumber);
    return 0;
}

/* Initialisiere den Bus, signalisiere Fehler */
/* Einmal aufgerufen mit busnummer als einzigem Parameter */
/* return code wird ignoriert (vorerst) */
long int init_bus_DDL(long int busnumber)
{
    DBG(busnumber, DBG_INFO, "DDL init with debug level %d",
        busses[busnumber].debuglevel);
    int i, error;

    busses[busnumber].fd = init_lineDDL(busnumber);

    __DDL->short_detected = 0;

    __DDL->queue_initialized = FALSE;
    __DDL->queue_out = 0;
    __DDL->queue_in = 0;
    queue_init(busnumber);

    /* generate idle_data */
    for (i = 0; i < MAXDATA; i++)
        __DDL->idle_data[i] = 0x55;     /* 0x55 = 01010101 */
    for (i = 0; i < PKTSIZE; i++)
        __DDL->NMRA_idle_data[i] = 0x55;

    /*
     * ATTENTION:
     *   If nmra dcc mode is activated __DDL->idle_data[] and
     *   __DDL->NMRA_idle_data must be overidden from init_NMRAPacketPool().
     *   This means, that init_NMRAPacketPool()
     *   must be called after init_MaerklinPacketPool().
     */

    __DDL->last_refreshed_maerklin_loco = 0;
    __DDL->last_refreshed_maerklin_fx = -1;
    __DDL->last_refreshed_nmra_loco = 0;
    __DDL->last_refreshed_nmra_fx = -1;

    if (__DDL->ENABLED_PROTOCOLS & EP_MAERKLIN) {
        init_MaerklinPacketPool(busnumber);
        __DDL->maerklin_refresh = 1;
    }
    else
        __DDL->maerklin_refresh = 0;
    if (__DDL->ENABLED_PROTOCOLS & EP_NMRADCC)
        init_NMRAPacketPool(busnumber);

    /*
     * Starting the thread that is responsible for the signals on 
     * serial port.
     */
    error =
        pthread_create(&(__DDL->ptid), NULL, thr_refresh_cycle,
                       (void *) busnumber);
    if (error) {
        DBG(busnumber, DBG_FATAL,
            "cannot create thread: refresh_cycle. Abort!");
        exit(1);
    }

    DBG(busnumber, DBG_INFO, "DDL init done");
    return 0;
}

void *thr_sendrec_DDL(void *v)
{
    struct _GLSTATE gltmp;
    struct _GASTATE gatmp;
    int addr;
    long int busnumber = (long int) v;

    DBG(busnumber, DBG_INFO, "DDL started on device %s",
        busses[busnumber].device);

    busses[busnumber].watchdog = 1;

    while (1) {
        if (!queue_GL_isempty(busnumber)) {
            char p;
            int pv;
            int speed;
            int direction;

            unqueueNextGL(busnumber, &gltmp);
            p = gltmp.protocol;
            /* need to compute from the n_fs and n_func parameters */
            pv = gltmp.protocolversion;
            addr = gltmp.id;
            speed = gltmp.speed;
            direction = gltmp.direction;
            DBG(busnumber, DBG_DEBUG, "next command: %c (%x) %d %d", p, p,
                pv, addr);
            switch (p) {
            case 'M':          /* Motorola Codes */
                if (speed == 1)
                    speed++;    /* Never send FS1 */
                if (direction == 2)
                    speed = 0;
                switch (pv) {
                case 1:
                    comp_maerklin_1(busnumber, addr, gltmp.direction,
                                    speed, gltmp.funcs & 0x01);
                    break;
                case 2:
                    comp_maerklin_2(busnumber, addr, gltmp.direction,
                                    speed, gltmp.funcs & 0x01,
                                    ((gltmp.funcs >> 1) & 0x01),
                                    ((gltmp.funcs >> 2) & 0x01),
                                    ((gltmp.funcs >> 3) & 0x01),
                                    ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 3:
                    comp_maerklin_3(busnumber, addr, gltmp.direction,
                                    speed, gltmp.funcs & 0x01,
                                    ((gltmp.funcs >> 1) & 0x01),
                                    ((gltmp.funcs >> 2) & 0x01),
                                    ((gltmp.funcs >> 3) & 0x01),
                                    ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 4:
                    comp_maerklin_4(busnumber, addr, gltmp.direction,
                                    speed, gltmp.funcs & 0x01,
                                    ((gltmp.funcs >> 1) & 0x01),
                                    ((gltmp.funcs >> 2) & 0x01),
                                    ((gltmp.funcs >> 3) & 0x01),
                                    ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 5:
                    comp_maerklin_5(busnumber, addr, gltmp.direction,
                                    speed, gltmp.funcs & 0x01,
                                    ((gltmp.funcs >> 1) & 0x01),
                                    ((gltmp.funcs >> 2) & 0x01),
                                    ((gltmp.funcs >> 3) & 0x01),
                                    ((gltmp.funcs >> 4) & 0x01));
                    break;
                }
                break;
            case 'N':          /* NMRA / DCC Codes */
                if (speed == 1)
                    speed++;
                switch (pv) {
                case 1:
                    if (direction != 2)
                        comp_nmra_baseline(busnumber, addr, direction,
                                           speed);
                    else
                        /* emergency halt: FS 1 */
                        comp_nmra_baseline(busnumber, addr, 0, 1);
                    break;
                case 2:
                    if (direction != 2)
                        comp_nmra_f4b7s28(busnumber, addr, direction,
                                          speed, gltmp.funcs & 0x01,
                                          ((gltmp.funcs >> 1) & 0x01),
                                          ((gltmp.funcs >> 2) & 0x01),
                                          ((gltmp.funcs >> 3) & 0x01),
                                          ((gltmp.funcs >> 4) & 0x01));
                    else        // emergency halt: FS 1
                        comp_nmra_f4b7s28(busnumber, addr, 0, 1,
                                          gltmp.funcs & 0x01,
                                          ((gltmp.funcs >> 1) & 0x01),
                                          ((gltmp.funcs >> 2) & 0x01),
                                          ((gltmp.funcs >> 3) & 0x01),
                                          ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 3:
                    if (direction != 2)
                        comp_nmra_f4b7s128(busnumber, addr, direction,
                                           speed, gltmp.funcs & 0x01,
                                           ((gltmp.funcs >> 1) & 0x01),
                                           ((gltmp.funcs >> 2) & 0x01),
                                           ((gltmp.funcs >> 3) & 0x01),
                                           ((gltmp.funcs >> 4) & 0x01));
                    else        // emergency halt: FS 1
                        comp_nmra_f4b7s128(busnumber, addr, 0, 1,
                                           gltmp.funcs & 0x01,
                                           ((gltmp.funcs >> 1) & 0x01),
                                           ((gltmp.funcs >> 2) & 0x01),
                                           ((gltmp.funcs >> 3) & 0x01),
                                           ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 4:
                    if (direction != 2)
                        comp_nmra_f4b14s28(busnumber, addr, direction,
                                           speed, gltmp.funcs & 0x01,
                                           ((gltmp.funcs >> 1) & 0x01),
                                           ((gltmp.funcs >> 2) & 0x01),
                                           ((gltmp.funcs >> 3) & 0x01),
                                           ((gltmp.funcs >> 4) & 0x01));
                    else        // emergency halt: FS 1
                        comp_nmra_f4b14s28(busnumber, addr, 0, 1,
                                           gltmp.funcs & 0x01,
                                           ((gltmp.funcs >> 1) & 0x01),
                                           ((gltmp.funcs >> 2) & 0x01),
                                           ((gltmp.funcs >> 3) & 0x01),
                                           ((gltmp.funcs >> 4) & 0x01));
                    break;
                case 5:
                    if (direction != 2)
                        comp_nmra_f4b14s128(busnumber, addr, direction,
                                            speed, gltmp.funcs & 0x01,
                                            ((gltmp.funcs >> 1) & 0x01),
                                            ((gltmp.funcs >> 2) & 0x01),
                                            ((gltmp.funcs >> 3) & 0x01),
                                            ((gltmp.funcs >> 4) & 0x01));
                    else        // emergency halt: FS 1
                        comp_nmra_f4b14s128(busnumber, addr, 0, 1,
                                            gltmp.funcs & 0x01,
                                            ((gltmp.funcs >> 1) & 0x01),
                                            ((gltmp.funcs >> 2) & 0x01),
                                            ((gltmp.funcs >> 3) & 0x01),
                                            ((gltmp.funcs >> 4) & 0x01));
                    break;
                }
            }
            setGL(busnumber, addr, gltmp);
        }
        busses[busnumber].watchdog = 4;

        if (!queue_GA_isempty(busnumber)) {
            char p;
            unqueueNextGA(busnumber, &gatmp);
            addr = gatmp.id;
            p = gatmp.protocol;
            DBG(busnumber, DBG_DEBUG, "next GA command: %c (%x) %d", p, p,
                addr);
            switch (p) {
            case 'M':          /* Motorola Codes */
                comp_maerklin_ms(busnumber, addr, gatmp.port,
                                 gatmp.action);
                break;
            case 'N':
                comp_nmra_accessory(busnumber, addr, gatmp.port,
                                    gatmp.action);
                break;
            }
            setGA(busnumber, addr, gatmp);
            busses[busnumber].watchdog = 5;

            if (gatmp.activetime >= 0) {
                usleep(1000L * (unsigned long) gatmp.activetime);
                gatmp.action = 0;
                DBG(busnumber, DBG_DEBUG, "delayed GA command: %c (%x) %d",
                    p, p, addr);
                switch (p) {
                case 'M':      /* Motorola Codes */
                    comp_maerklin_ms(busnumber, addr, gatmp.port,
                                     gatmp.action);
                    break;
                case 'N':
                    comp_nmra_accessory(busnumber, addr, gatmp.port,
                                        gatmp.action);
                    break;
                }
                setGA(busnumber, addr, gatmp);
            }
        }
        usleep(1000);
    }
}