//  SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (c) 2018 MediaTek Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/******************************************************************************
*                         C O M P I L E R   F L A G S
*******************************************************************************
*/

/******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
*******************************************************************************
*/
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
//#include <syslog.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/poll.h>
#include <sys/param.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <linux/serial.h> /* struct serial_struct  */
#include "uart_launcher.h"

//---------------------------------------------------------------------------
static int set_speed(int fd, struct termios *ti, int speed);
int setup_uart_param (int hComPort, int iBaudrate, struct UART_CONFIG *sUartConfig);

#ifdef __ANDROID__
static int notifyFd = -1;
#endif
static int gTtyFd = -1;
static int cont = 1;    /** loop continue running */
struct flock fl;

//---------------------------------------------------------------------------
/* Used as host uart param setup callback */
int setup_uart_param (
    int hComPort,
    int iBaudrate,
    struct UART_CONFIG *sUartConfig)
{
    struct termios ti;
    int  fd;
    BPRINT_I("setup_uart_param begin");
    if(!sUartConfig){
        BPRINT_E("Invalid sUartConfig");
        return -2;
    }

    BPRINT_I("setup_uart_param Baud %d FC %d", iBaudrate, sUartConfig->fc);

    fd = hComPort;
    if (fd < 0) {
        BPRINT_E("Invalid serial port");
        return -2;
    }

    tcflush(fd, TCIOFLUSH);

    if (tcgetattr(fd, &ti) < 0) {
        BPRINT_E("Can't get port settings");
        return -3;
    }

    cfmakeraw(&ti);

    BPRINT_I("ti.c_cflag = 0x%08x", ti.c_cflag);
    ti.c_cflag |= CLOCAL;
    BPRINT_I("CLOCAL = 0x%x", CLOCAL);
    BPRINT_I("(ori)ti.c_iflag = 0x%08x", ti.c_iflag);
    BPRINT_I("(ori)ti.c_cflag = 0x%08x", ti.c_cflag);
    BPRINT_I("sUartConfig->fc= %d (0:none,sw,hw,linux)", sUartConfig->fc);

    switch (sUartConfig->fc) {
     /* HW FC Enable */
    case UART_HW_FC:
        ti.c_cflag |= CRTSCTS;
        ti.c_iflag &= ~(NOFLSH);
        break;
    /* Linux Software FC */
    case UART_LINUX_FC:
        ti.c_iflag |= (IXON | IXOFF | IXANY);
        ti.c_cflag &= ~(CRTSCTS);
        ti.c_iflag &= ~(NOFLSH);
        break;
    /* MTK Software FC */
    case UART_MTK_SW_FC:
        ti.c_iflag |= CRTSCTS;
        ti.c_cflag &= ~(NOFLSH);
        break;
    /* default disable flow control */
    default:
        ti.c_cflag &= ~(CRTSCTS);
        ti.c_iflag &= ~(NOFLSH|CRTSCTS);
    }

    BPRINT_D("c_c CRTSCTS = 0x%16x", CRTSCTS);
    BPRINT_D("c_i IXON = 0x%08x", IXON);
    BPRINT_D("c_i IXOFF = 0x%08x", IXOFF);
    BPRINT_D("c_i IXANY = 0x%08x", IXANY);
    BPRINT_D("(aft)ti.c_iflag = 0x%08x", ti.c_iflag);
    BPRINT_D("(aft)ti.c_cflag = 0x%08x", ti.c_cflag);

    if (tcsetattr(fd, TCSANOW, &ti) < 0) {
        BPRINT_E("Can't set port settings");
        return -4;
    }

    /* Set baudrate */
    if (set_speed(fd, &ti, iBaudrate) < 0) {
        BPRINT_E("Can't set initial baud rate");
        return -5;
    }

    tcflush(fd, TCIOFLUSH);
    BPRINT_I("%s done", __func__);
    return 0;
}

//---------------------------------------------------------------------------
static speed_t get_speed (int baudrate)
{
    unsigned int idx;
    for (idx = 0; idx < sizeof(speeds)/sizeof(speeds[0]); idx++) {
        if (baudrate == (int)speeds[idx].baud) {
            return speeds[idx].speed;
        }
    }
    return CBAUDEX;
}

//---------------------------------------------------------------------------
int set_speed(int fd, struct termios *ti, int speed)
{
    struct serial_struct ss;
    int baudenum = get_speed(speed);

    if (speed != CBAUDEX) {
        //printf("%s: standard baudrate: %d -> 0x%08x\n", __FUNCTION__, speed, baudenum);
        if ((ioctl(fd, TIOCGSERIAL, &ss)) < 0) {
            BPRINT_E("%s: BAUD: error to get the serial_struct info:%s\n", __func__, strerror(errno));
            return -1;
        }
#ifdef ANDROID
        ss.flags &= ~ASYNC_SPD_CUST;
#if defined(SERIAL_STRUCT_EXT) /*modified in serial_struct.h*/
        memset(ss.reserved, 0x00, sizeof(ss.reserved));
#endif
        ss.flags |= (1 << 13);    /*set UPFLOWLATENCY flat to tty, or serial_core will reset tty->low_latency to 0*/
        /*set standard buadrate setting*/
        if ((ioctl(fd, TIOCSSERIAL, &ss)) < 0) {
            BPRINT_E("%s: BAUD: error to set serial_struct:%s\n", __func__, strerror(errno));
            return -2;
        }
#endif
        cfsetospeed(ti, baudenum);
        cfsetispeed(ti, baudenum);
        return tcsetattr(fd, TCSANOW, ti);
    }
    else {
        BPRINT_E("%s: unsupported non-standard baudrate: %d -> 0x%08x\n", __func__, speed, baudenum);
        return -3;
    }
}

//---------------------------------------------------------------------------
void init_flock(struct flock *flk)
{
    flk->l_type = F_WRLCK;
    flk->l_whence = SEEK_SET;
    flk->l_pid = getpid();
    flk->l_start = 0;
    flk->l_len = 0;
}

void unlock_flock(int fd, struct flock *fl, int type, int whence)
{
    fl->l_type = type;
    fl->l_whence = whence;
    if (fcntl(fd, F_SETLKW, fl) < 0)
        BPRINT_E("%s: fcntl failed(%d)", __func__, errno);
}

//---------------------------------------------------------------------------
int cmd_hdr_baud (struct UART_CONFIG *sUartConfig, int baudrate) {
    return (gTtyFd != -1) ? setup_uart_param(gTtyFd, baudrate, sUartConfig) : -1;
}

//---------------------------------------------------------------------------
int osi_system(const char *cmd)
{
    FILE *fp;
    int ret;

    if (cmd == NULL) {
        BPRINT_E("%s: cmd is NULL", __func__);
        return -1;
    }

    fp = popen(cmd, "w");
    if (fp == NULL) {
        BPRINT_E("%s: (%s) failed", __func__, cmd);
        return -1;
    }

    BPRINT_I("Command: %s", cmd);

    ret = pclose(fp);
    if (ret < 0) {
        BPRINT_E("%s: pclose ret = %d", __func__, ret);
    } else if (ret > 0) {
        BPRINT_I("%s: pclose ret = %d", __func__, ret);
    }
    return ret;
}

//---------------------------------------------------------------------------
static void uart_launcher_sig_handler(int signum)
{
    BPRINT_I("%s: sig[%d] fd[%d]", __func__, signum, gTtyFd);
    cont = 0;
}

//---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    BPRINT_I("Bluetooth Uart Launcher Ver %s", VERSION);
    int ld = 0;
    struct pollfd fds;
    int fd_num = 0;
    int err = 0;
    int opt;
    int baudrate = 115200;
    int chang_baud_rate = 0;
    int retry = 0;
    int flow_control = UART_DISABLE_FC;
    char *tty_path = "/dev/ttyUSB0";
    struct UART_CONFIG sUartConfig;
    struct sigaction sigact;

    memset(&sUartConfig, 0, sizeof(struct UART_CONFIG));
    memset(&fds, 0, sizeof(struct pollfd));
    sUartConfig.fc = flow_control;

    /* Register signal handler */
    sigact.sa_handler = uart_launcher_sig_handler;
    sigact.sa_flags = 0;
    sigemptyset(&sigact.sa_mask);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);
    sigaction(SIGQUIT, &sigact, NULL);
    /* SIGKILL, SIGSTOP may not be catched */
    sigaction(SIGKILL, &sigact, NULL);
    sigaction(SIGSTOP, &sigact, NULL);
    init_flock(&fl);
    ld = N_MTK;

    while ((opt = getopt(argc, argv, "c:f:p:k:l::")) != -1) {
        switch (opt) {
            /* change baudrate */
            case 'c':
                baudrate = atoi(optarg);
                BPRINT_I("baudrate = %d", baudrate);
                /* not need to change if baudrate is same */
                if (baudrate == CUST_BAUDRATE_DFT) {
                    chang_baud_rate = 0;
                } else
                    chang_baud_rate = 1;
                break;
            /* flow control */
            case 'f':
                flow_control = atoi(optarg);
                BPRINT_I("flow_control = %d", flow_control);
                break;
            /* tty path */
            case 'p':
                tty_path = optarg;
                BPRINT_I("Log path is %s", tty_path);
                break;
            /* kill process */
            case 'k':
                osi_system("killall uart_launcher");
                BPRINT_I("Kill uart_launcher");
                return 0;
            /* set ldisc */
            case 'l':
                ld = atoi(optarg);
                BPRINT_I("set ldisc[%d]", ld);
                break;
            case '?':
        default:
                BPRINT_I("set baud:\t uart_launcher -c [baudrate]");
                BPRINT_I("flow control:\t uart_launcher -f [flow_control]");
                BPRINT_I("\t\t 0:disable FC  1:MTK_SW_FC  2: SW_FC  3: HW_FC");
                BPRINT_I("tty path:\t uart_launcher -p [path]");
                BPRINT_I("kill process:\t uart_launcher -k");
                goto exit;
                break;
        }
    }

    /* open ttyUSB */
    BPRINT_I("Running...");
    /* node may not ready, retry 20 times */
    while (1) {
        gTtyFd = open(tty_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        BPRINT_I("open done ttyfd %d", gTtyFd);
        if (gTtyFd < 0) {
            if (retry > 20) {
                BPRINT_E("ttyfd %d, error", gTtyFd);
                goto exit;
            } else {
                retry++;
                (void)usleep(1000 * 1000);
            }
        } else
            break;
    }

#ifdef __ANDROID__
    /* open bt_uart_launcher_notify */
    BPRINT_I("open /proc/stpbt/bt_uart_launcher_notify");
    /* node may not ready, retry 20 times */
    while (1) {
        notifyFd= open("/proc/stpbt/bt_uart_launcher_notify", O_RDONLY);
        BPRINT_I("open done bt_uart_launcher_notify fd[%d]", notifyFd);
        if (notifyFd < 0) {
            if (retry > 20) {
                BPRINT_E("bt_uart_launcher_notify %d, error", notifyFd);
	        break;
            } else {
                retry++;
                (void)usleep(1000 * 100);
            }
        } else
            break;
    }
#endif

    /* flock the device node */
    BPRINT_I("flock the device node");
    if (fcntl(gTtyFd, F_SETLK, &fl) < 0) {
        BPRINT_E("lock device node failed, uart_launcher already running.");
        goto exit;
    }

    /* to ensure driver register TIOCSETD, retry 20 times */
    retry = 0;
    while (1) {
        if (ioctl(gTtyFd, TIOCSETD, &ld) < 0) {
            if (retry > 20) {
                BPRINT_E("set TIOCSETD N_MTK error");
                goto exit;
            } else {
                retry++;
                (void)usleep(1000 * 1000);
            }
        } else
            break;
    }

restart:
    /* Set default Baud rate */
    sUartConfig.iBaudrate = CUST_BAUDRATE_DFT;
    BPRINT_I("set baudtate = %d", CUST_BAUDRATE_DFT);
    cmd_hdr_baud(&sUartConfig, CUST_BAUDRATE_DFT);
    fds.fd = gTtyFd;
    fds.events = POLLIN;
    ++fd_num;

    err = ioctl(gTtyFd, HCIUARTINIT, NULL);
    if (err < 0) {
        BPRINT_E("set HCIUARTINIT error %d", err);
        goto exit;
    }

    err = ioctl(gTtyFd, HCIUARTGETBAUD, NULL);
    if (err < 0) {
        BPRINT_E("set HCIUARTGETBAUD error %d", err);
        goto exit;
    }

    /* chang baud rate */
    if (chang_baud_rate | flow_control) {
        sUartConfig.iBaudrate = baudrate;
        sUartConfig.fc = flow_control;
        err = ioctl(gTtyFd, HCIUARTSETBAUD, &sUartConfig);
        if (err < 0) {
            BPRINT_E("set HCIUARTSETBAUD error %d", err);
            goto exit;
        }

        BPRINT_I("set baudtate %d", baudrate);
        cmd_hdr_baud(&sUartConfig, baudrate);

        err = ioctl(gTtyFd, HCIUARTSETWAKEUP, NULL);
        if (err < 0) {
            BPRINT_E("set HCIUARTSETWAKEUP error %d", err);
            goto exit;
        }
    }

    if (ioctl(gTtyFd, HCIUARTLOADPATCH, NULL) < 0) {
        BPRINT_E("set HCIUARTLOADPATCH error");
        goto exit;
    }

    while (cont) {
        err = poll(&fds, fd_num, 20000);
        if (err < 0) {
            if (errno == EINTR) {
                continue;
            }
            else {
                BPRINT_E("poll error:%d errno:%d, %s", err, errno, strerror(errno));
                break;
            }
        } else if (!err) {
            if (fds.revents & POLLIN) {
                goto restart;
            } else {
                continue;
            }
        }
        goto restart;
    }

    BPRINT_I("%s: deinit flow fd[%d]", __func__, gTtyFd);

    if (gTtyFd < 0)
        goto exit;

#if !defined(__ANDROID__) // In sp project, baudrate is controled by driver
    /* before exit daemon, return baud to default */
    if (chang_baud_rate | flow_control) {
        sUartConfig.iBaudrate = CUST_BAUDRATE_DFT;
        sUartConfig.fc = UART_DISABLE_FC;
        err = ioctl(gTtyFd, HCIUARTSETBAUD, &sUartConfig);
        if (err < 0) {
            BPRINT_E("ioctl HCIUARTSETBAUD error:[%d] %s", errno, strerror(errno));
            goto exit;
        }

        BPRINT_I("set baudtate %d", CUST_BAUDRATE_DFT);
        cmd_hdr_baud(&sUartConfig, CUST_BAUDRATE_DFT);

        err = ioctl(gTtyFd, HCIUARTSETWAKEUP, NULL);
        if (err < 0) {
            BPRINT_E("ioctl HCIUARTSETWAKEUP error:[%d] %s", errno, strerror(errno));
            goto exit;
        }
    }
#endif

exit:

#ifdef __ANDROID__
    BPRINT_I("%s: exit notifyFd[%d]", __func__, notifyFd);
    if (notifyFd > 0) {
        close(notifyFd);
        notifyFd= -1;
    }
#endif

    BPRINT_I("%s: exit ttyFd[%d]", __func__, gTtyFd);

    /* unlock ttyFd */
    if (gTtyFd > 0) {
        err = ioctl(gTtyFd, HCIUARTDEINIT, NULL);
        if (err < 0) {
            BPRINT_E("ioctl HCIUARTDEINIT error:[%d] %s", errno, strerror(errno));
            BPRINT_I("deinit fail, wait...");
            (void)usleep(1000 * 1000);
        }
        BPRINT_I("unlock_flock");
        unlock_flock(gTtyFd, &fl, F_UNLCK, SEEK_SET);
        close(gTtyFd);
        gTtyFd = -1;
    }
    BPRINT_I("uart_launcher stop");
    return 0;
}
//---------------------------------------------------------------------------
