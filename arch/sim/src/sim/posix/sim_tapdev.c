/****************************************************************************
 * arch/sim/src/sim/posix/sim_tapdev.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
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
#include <syslog.h>
#include <time.h>
#include <net/route.h>
#include <net/if.h>
#include <linux/sockios.h>
#include <linux/if_tun.h>
#include <linux/net.h>
#include <netinet/in.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVTAP        "/dev/net/tun"

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
 * Private Data
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static int  gdrop = 0;
#endif
static int  gtapdevfd[CONFIG_SIM_NETDEV_NUMBER] =
{
  [0 ... CONFIG_SIM_NETDEV_NUMBER - 1] = -1
};
static char gdevname[CONFIG_SIM_NETDEV_NUMBER][IFNAMSIZ];
static void *g_priv[CONFIG_SIM_NETDEV_NUMBER];
static void (*g_tx_done_intr_cb[CONFIG_SIM_NETDEV_NUMBER])(void *priv);
static void (*g_rx_ready_intr_cb[CONFIG_SIM_NETDEV_NUMBER])(void *priv);

#ifdef CONFIG_SIM_NET_HOST_ROUTE
static struct rtentry ghostroute[CONFIG_SIM_NETDEV_NUMBER];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef TAPDEV_DEBUG
static inline void dump_ethhdr(const char *msg, unsigned char *buf,
                               int buflen)
{
  syslog(LOG_INFO, "TAPDEV: %s %d bytes\n", msg, buflen);
  syslog(LOG_INFO, "        %02x:%02x:%02x:%02x:%02x:%02x "
         "%02x:%02x:%02x:%02x:%02x:%02x %02x%02x\n",
         buf[0], buf[1], buf[2], buf[3], buf[4],  buf[5],
         buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
#ifdef CONFIG_ENDIAN_BIG
         buf[13], buf[12]
#else
         buf[12], buf[13]
#endif
        );
}
#else
#  define dump_ethhdr(m,b,l)
#endif

static void set_macaddr(int devidx)
{
  unsigned char mac[7];

  /* Assign a random locally-created MAC address.
   *
   * This previously took the address from the TAP interface; that was
   * incorrect, as that hardware address belongs to the host system.  Packets
   * destined for the application aren't guaranteed to reach it if you do
   * that, as the host may handle them at its discretion.
   *
   * With a unique MAC address, we get ALL the packets.
   *
   * The generated MAC addresses will be same if we use timestamp as seed and
   * create more than one device at the same time, so add index to make mac
   * address different.
   *
   * TODO:  The generated MAC address should be checked to see if it
   *        conflicts with something else on the network.
   */

  srand(time(NULL) + devidx);
  mac[0] = 0x42;
  mac[1] = rand() % 256;
  mac[2] = rand() % 256;
  mac[3] = rand() % 256;
  mac[4] = rand() % 256;
  mac[5] = rand() % 256;
  mac[6] = 0;

  sim_netdriver_setmacaddr(devidx, mac);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sim_tapdev_init(int devidx, void *priv,
                 void (*tx_done_intr_cb)(void *priv),
                 void (*rx_ready_intr_cb)(void *priv))
{
  struct ifreq ifr;
  int tapdevfd;
  int ret;
  int sockfd;

  /* Open the tap device */

  tapdevfd = open(DEVTAP, O_RDWR, 0644);
  if (tapdevfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: open failed: %d\n", -tapdevfd);
      return;
    }

  /* Configure the tap device */

  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
  ret = ioctl(tapdevfd, TUNSETIFF, (unsigned long) &ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed: %d\n", -ret);
      close(tapdevfd);
      return;
    }

  /* Save the tap device name */

  strncpy(gdevname[devidx], ifr.ifr_name, IFNAMSIZ);

  /* Get a socket with which to manipulate the tap device; the remaining
   * ioctl calls unfortunately won't work on the tap device fd.
   */

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
      close(tapdevfd);
      return;
    }

#ifdef CONFIG_SIM_NET_BRIDGE
  /* Assign the tap device to a bridge */

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, CONFIG_SIM_NET_BRIDGE_DEVICE, IFNAMSIZ);
  ifr.ifr_ifindex = if_nametoindex(gdevname[devidx]);

  ret = ioctl(sockfd, SIOCBRADDIF, &ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't add interface %s to "
             "bridge %s): %d\n",
             gdevname[devidx], CONFIG_SIM_NET_BRIDGE_DEVICE, -ret);
      close(sockfd);
      close(tapdevfd);
      return;
    }
#else
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, gdevname[devidx], IFNAMSIZ);
#endif

  ret = ioctl(sockfd, SIOCGIFMTU, &ifr);
  close(sockfd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed (can't get MTU "
             "from %s): %d\n", gdevname[devidx], -ret);
      close(tapdevfd);
      return;
    }
  else
    {
      sim_netdriver_setmtu(devidx, ifr.ifr_mtu);
    }

  gtapdevfd[devidx] = tapdevfd;
  g_priv[devidx] = priv;

  /* Register the emulated TX done interrupt callback */

  g_tx_done_intr_cb[devidx] = tx_done_intr_cb;

  /* Register the emulated RX ready interrupt callback */

  g_rx_ready_intr_cb[devidx] = rx_ready_intr_cb;

  /* Set the MAC address */

  set_macaddr(devidx);
}

int sim_tapdev_avail(int devidx)
{
  struct timeval tv;
  fd_set fdset;

  /* We can't do anything if we failed to open the tap device */

  if (gtapdevfd[devidx] < 0)
    {
      return 0;
    }

  /* Wait for data on the tap device (or a timeout) */

  tv.tv_sec  = 0;
  tv.tv_usec = 0;

  FD_ZERO(&fdset);
  FD_SET(gtapdevfd[devidx], &fdset);

  return select(gtapdevfd[devidx] + 1, &fdset, NULL, NULL, &tv) > 0;
}

unsigned int sim_tapdev_read(int devidx, unsigned char *buf,
                             unsigned int buflen)
{
  int ret;

  if (!sim_tapdev_avail(devidx))
    {
      return 0;
    }

  ret = read(gtapdevfd[devidx], buf, buflen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: read failed: %d\n", -ret);
      return 0;
    }

  dump_ethhdr("read", buf, ret);
  return ret;
}

void sim_tapdev_send(int devidx, unsigned char *buf, unsigned int buflen)
{
  int ret;

  if (gtapdevfd[devidx] < 0)
    {
      return;
    }

#ifdef TAPDEV_DEBUG
  syslog(LOG_INFO, "sim_tapdev_send: sending %d bytes\n", buflen);

  gdrop++;
  if (gdrop % 8 == 7)
    {
      syslog(LOG_ERR, "TAPDEV: Dropped a packet!\n");
      return;
    }
#endif

  ret = write(gtapdevfd[devidx], buf, buflen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: write failed: %d\n", -ret);
      exit(1);
    }

  dump_ethhdr("write", buf, buflen);

  /* Emulate TX done interrupt */

  if (g_tx_done_intr_cb[devidx] != NULL)
    {
      g_tx_done_intr_cb[devidx](g_priv[devidx]);
    }

  /* Emulate RX ready interrupt */

  if (g_rx_ready_intr_cb[devidx] != NULL && sim_tapdev_avail(devidx))
    {
      g_rx_ready_intr_cb[devidx](g_priv[devidx]);
    }
}

void sim_tapdev_ifup(int devidx, in_addr_t ifaddr)
{
  struct ifreq ifr;
  int          sockfd;
  int          ret;

#ifdef CONFIG_SIM_NET_HOST_ROUTE
  struct sockaddr_in *addr;
#endif

  if (gtapdevfd[devidx] < 0)
    {
      return;
    }

  /* Get a socket with which to manipulate the tap device */

  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0)
    {
      syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
      return;
    }

  /* Bring the TAP interface up */

  strncpy(ifr.ifr_name, gdevname[devidx], IFNAMSIZ);

  ret = ioctl(sockfd, SIOCGIFFLAGS, (unsigned long)&ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed "
             "(can't get interface flags): %d\n", -ret);
      close(sockfd);
      return;
    }

  ifr.ifr_flags |= IFF_UP;
  ret = ioctl(sockfd, SIOCSIFFLAGS, (unsigned long)&ifr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed "
             "(can't set interface flags): %d\n", -ret);
      close(sockfd);
      return;
    }

#ifdef CONFIG_SIM_NET_HOST_ROUTE
  /* Add host route */

  memset(&ghostroute[devidx], 0, sizeof(ghostroute[devidx]));

  addr = (struct sockaddr_in *)&ghostroute[devidx].rt_dst;
  addr->sin_family = AF_INET;
  addr->sin_addr.s_addr = ifaddr;

  ghostroute[devidx].rt_dev    = gdevname[devidx];
  ghostroute[devidx].rt_flags  = RTF_UP | RTF_HOST;
  ghostroute[devidx].rt_metric = 0;

  ret = ioctl(sockfd, SIOCADDRT, (unsigned long)&ghostroute[devidx]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "TAPDEV: ioctl failed"
              "(can't add host route): %d\n", -ret);
      close(sockfd);
      return;
    }
#endif

  close(sockfd);
}

void sim_tapdev_ifdown(int devidx)
{
#ifdef CONFIG_SIM_NET_HOST_ROUTE
  int sockfd;
  int ret;

  if (gtapdevfd[devidx] < 0)
    {
      return;
    }

  if (((struct sockaddr_in *)&ghostroute[devidx].rt_dst)->sin_addr.s_addr)
    {
      /* Get a socket with which to manipulate the tap device */

      sockfd = socket(AF_INET, SOCK_DGRAM, 0);
      if (sockfd < 0)
        {
          syslog(LOG_ERR, "TAPDEV: Can't open socket: %d\n", -sockfd);
          return;
        }

      ret = ioctl(sockfd, SIOCDELRT, (unsigned long)&ghostroute[devidx]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "TAPDEV: ioctl failed "
                 "(can't delete host route): %d\n", -ret);
        }

      close(sockfd);
    }
#endif
}
