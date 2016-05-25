/****************************************************************************
 * arch/sim/src/up_tapdev.c
 *
 *   Copyright (C) 2007-2009, 2011, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __CYGWIN__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/uio.h>
#include <sys/socket.h>

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#ifdef CONFIG_SIM_NET_HOST_ROUTE
#  include <net/route.h>
#endif

#include <net/if.h>
#include <linux/sockios.h>
#include <linux/if_tun.h>
#include <linux/net.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define TAPDEV_DEBUG  1

#define DEVTAP        "/dev/net/tun"

/* Syslog priority (must match definitions in nuttx/include/syslog.h) */

#define LOG_INFO      1  /* Informational message */
#define LOG_ERR       4  /* Error conditions */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Warning: This is very much Linux version specific! */

struct sel_arg_struct
{
  unsigned long   n;
  fd_set         *inp;
  fd_set         *outp;
  fd_set         *exp;
  struct timeval *tvp;
};

/****************************************************************************
 * NuttX Domain Public Function Prototypes
 ****************************************************************************/

int syslog(int priority, const char *format, ...);
int netdriver_setmacaddr(unsigned char *macaddr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static int  gdrop = 0;
#endif
static int  gtapdevfd;
static char gdevname[IFNAMSIZ];

#ifdef CONFIG_SIM_NET_HOST_ROUTE
static struct rtentry ghostroute;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static inline void dump_ethhdr(const char *msg, unsigned char *buf, int buflen)
{
  syslog(LOG_INFO, "TAPDEV: %s %d bytes\n", msg, buflen);
  syslog(LOG_INFO,
         "        %02x:%02x:%02x:%02x:%02x:%02x %02x:%02x:%02x:%02x:%02x:%02x %02x%02x\n",
         buf[0], buf[1], buf[2], buf[3], buf[4],  buf[5],
         buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
#ifdef CONFIG_ENDIAN_BIG
         buf[13], buf[12]);
#else
         buf[12], buf[13]);
#endif
}
#else
#  define dump_ethhdr(m,b,l)
#endif

static int up_setmacaddr(void)
{
  unsigned char mac[7];
  int ret = -1;

  /* Assign a random locally-created MAC address.
   *
   * This previously took the address from the TAP interface; that was
   * incorrect, as that hardware address belongs to the host system.  Packets
   * destined for the application aren't guaranteed to reach it if you do
   * that, as the host may handle them at its discretion.
   *
   * With a unique MAC address, we get ALL the packets.
   *
   * TODO:  The generated MAC address should be checked to see if it
   *        conflicts with something else on the network.
   */

  srand(time(NULL));
  mac[0] = 0x42;
  mac[1] = rand() % 256;
  mac[2] = rand() % 256;
  mac[3] = rand() % 256;
  mac[4] = rand() % 256;
  mac[5] = rand() % 256;
  mac[6] = 0;

  ret = netdriver_setmacaddr(mac);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void tapdev_init(void)
{
  struct ifreq ifr;
  int ret;

#ifdef CONFIG_SIM_NET_BRIDGE
  int sockfd;
#endif

  /* Open the tap device */

  gtapdevfd = open(DEVTAP, O_RDWR, 0644);
  if (gtapdevfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: open failed: %d\n", -gtapdevfd);
      return;
    }

  /* Configure the tap device */

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
  ret = ioctl(gtapdevfd, TUNSETIFF, (unsigned long) &ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed: %d\n", -ret);
      return;
   }

  /* Save the tap device name */

  strncpy(gdevname, ifr.ifr_name, IFNAMSIZ);

#ifdef CONFIG_SIM_NET_BRIDGE
  /* Get a socket with which to manipulate the tap device; the remaining
   * ioctl calls unfortunately won't work on the tap device fd.
   */

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
      return;
    }

  /* Assign the tap device to a bridge */

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, CONFIG_SIM_NET_BRIDGE_DEVICE, IFNAMSIZ);
  ifr.ifr_ifindex = if_nametoindex(gdevname);

  ret = ioctl(sockfd, SIOCBRADDIF, &ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't add interface %s to bridge %s): %d\n",
             gdevname, CONFIG_SIM_NET_BRIDGE_DEVICE, -ret);
    }

  close(sockfd);
#endif

  /* Set the MAC address */

  up_setmacaddr();
}

unsigned int tapdev_read(unsigned char *buf, unsigned int buflen)
{
  fd_set                fdset;
  struct timeval        tv;
  int                   ret;

  /* We can't do anything if we failed to open the tap device */

  if (gtapdevfd < 0)
    {
      return 0;
    }

  /* Wait for data on the tap device (or a timeout) */

  tv.tv_sec  = 0;
  tv.tv_usec = 1000;

  FD_ZERO(&fdset);
  FD_SET(gtapdevfd, &fdset);

  ret = select(gtapdevfd + 1, &fdset, NULL, NULL, &tv);
  if (ret == 0)
    {
      return 0;
    }

  ret = read(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: read failed: %d\n", -ret);
      return 0;
    }

  dump_ethhdr("read", buf, ret);
  return ret;
}

void tapdev_send(unsigned char *buf, unsigned int buflen)
{
  int ret;
#ifdef TAPDEV_DEBUG
  syslog(LOG_INFO, "tapdev_send: sending %d bytes\n", buflen);

  gdrop++;
  if (gdrop % 8 == 7)
    {
      syslog(LOG_ERR, "TAPDEV: Dropped a packet!\n");
      return;
    }
#endif

  ret = write(gtapdevfd, buf, buflen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: write failed: %d", -ret);
      exit(1);
    }

  dump_ethhdr("write", buf, buflen);
}

void tapdev_ifup(in_addr_t ifaddr)
{
  struct ifreq ifr;
  int          sockfd;
  int          ret;

#ifdef CONFIG_SIM_NET_HOST_ROUTE
  struct sockaddr_in *addr;
#endif

  /* Get a socket with which to manipulate the tap device */

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
      return;
    }

  /* Bring the TAP interface up */

  strncpy(ifr.ifr_name, gdevname, IFNAMSIZ);

  ret = ioctl(sockfd, SIOCGIFFLAGS, (unsigned long)&ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't get interface flags): %d\n", -ret);
      close(sockfd);
      return;
    }

  ifr.ifr_flags |= IFF_UP;
  ret = ioctl(sockfd, SIOCSIFFLAGS, (unsigned long)&ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't set interface flags): %d\n", -ret);
      close(sockfd);
      return;
    }

#ifdef CONFIG_SIM_NET_HOST_ROUTE
  /* Add host route */

  memset(&ghostroute, 0, sizeof(ghostroute));

  addr = (struct sockaddr_in *)&ghostroute.rt_dst;
  addr->sin_family = AF_INET;
  addr->sin_addr.s_addr = ifaddr;

  ghostroute.rt_dev    = gdevname;
  ghostroute.rt_flags  = RTF_UP | RTF_HOST;
  ghostroute.rt_metric = 0;

  ret = ioctl(sockfd, SIOCADDRT, (unsigned long)&ghostroute);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't add host route): %d\n", -ret);
      close(sockfd);
      return;
    }
#endif

  close(sockfd);
}

void tapdev_ifdown(void)
{
#ifdef CONFIG_SIM_NET_HOST_ROUTE
  int sockfd;
  int ret;

  if (((struct sockaddr_in *)&ghostroute.rt_dst)->sin_addr.s_addr != 0)
    {
      /* Get a socket with which to manipulate the tap device */

      sockfd = socket(AF_INET, SOCK_DGRAM, 0);
      if (sockfd < 0)
        {
          syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
          return;
        }

      ret = ioctl(sockfd, SIOCDELRT, (unsigned long)&ghostroute);
      if (ret < 0)
        {
          syslog(LOG_ERR, "TAPDEV: ioctl failed (can't delete host route): %d\n", -ret);
        }

      close(sockfd);
    }
#endif
}

#endif /* !__CYGWIN__ */
