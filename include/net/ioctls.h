/****************************************************************************
 * net/ioctls.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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

#ifndef __NET_IOCTLS_H
#define __NET_IOCTLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/socket.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* These are ioctl commands to use with a socket FD.  At present, commnads
 * are accepted onloy to set/get IP addresses, broadcast address, network
 * masks, and hardware address, and a few others
 */

#define _SIOCBASE       (0x8900)
#define _SIOCMASK       (0x00ff)
#define _SIOCVALID(c)   (((c) & ~_SIOCMASK) == _SIOCBASE)

#define SIOCGIFADDR     (_SIOCBASE|0x0001)  /* Get IP address */
#define SIOCSIFADDR     (_SIOCBASE|0x0002)  /* Set IP address */
#define SIOCGIFDSTADDR  (_SIOCBASE|0x0003)  /* Get P-to-P address */
#define SIOCSIFDSTADDR  (_SIOCBASE|0x0004)  /* Set P-to-P address */
#define SIOCGIFBRDADDR  (_SIOCBASE|0x0005)  /* Get broadcast IP address	*/
#define SIOCSIFBRDADDR  (_SIOCBASE|0x0006)  /* Set broadcast IP address	*/
#define SIOCGIFNETMASK  (_SIOCBASE|0x0007)  /* Get network mask */
#define SIOCSIFNETMASK  (_SIOCBASE|0x0008)  /* Set network mask */
#define SIOCGIFMTU      (_SIOCBASE|0x0009)  /* Get MTU size */
#define SIOCGIFHWADDR   (_SIOCBASE|0x000a)  /* Get hardware address */
#define SIOCSIFHWADDR   (_SIOCBASE|0x000b)  /* Set hardware address */
#define SIOCDIFADDR     (_SIOCBASE|0x000c)  /* Delete IP address */
#define SIOCGIFCOUNT    (_SIOCBASE|0x000d)  /* Get number of devices */

/* Sizing parameters */

#define IFNAMSIZ        6
#define IFHWADDRLEN     6

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct ifreq
{
  char       ifr_name[IFNAMSIZ];      /* Network device name (e.g. "eth0") */
  union
  {
    struct sockaddr ifru_addr;        /* IP Address */
    struct sockaddr ifru_dstaddr;     /* P-to-P Address */
    struct sockaddr ifru_broadaddr;   /* Broadcast address */
    struct sockaddr ifru_netmask;     /* Netmask */
    struct sockaddr ifru_hwaddr;      /* MAC address */
    int     ifru_count;               /* Number of devices */
    int     ifru_mtu;                 /* MTU size */
  } ifr_ifru;
};

#define ifr_addr        ifr_ifru.ifru_addr      /* IP address */
#define ifr_dstaddr     ifr_ifru.ifru_dstaddr   /* P-to-P Address */
#define ifr_broadaddr   ifr_ifru.ifru_broadaddr /* Broadcast address */
#define ifr_netmask     ifr_ifru.ifru_netmask   /* Interface net mask */
#define ifr_hwaddr      ifr_ifru.ifru_hwaddr    /* MAC address */
#define ifr_mtu         ifr_ifru.ifru_mtu       /* MTU */
#define ifr_count       ifr_ifru.ifru_count     /* Number of devices */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __NET_IOCTLS_H */
