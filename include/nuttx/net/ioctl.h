/****************************************************************************
 * include/nuttx/net/ioctl.h
 *
 *   Copyright (C) 2007-2008, 2010-2013, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NET_IOCTL_H
#define __INCLUDE_NUTTX_NET_IOCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/ioctl.h> /* _SIOCBASE, etc. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* These are ioctl commands to use with a socket FD.  At present, commands
 * are accepted only to set/get IP addresses, broadcast address, network
 * masks, and hardware address, and a few others
 */

/* IPv4 interface control operations */

#define SIOCGIFADDR      _SIOC(0x0001)  /* Get IP address */
#define SIOCSIFADDR      _SIOC(0x0002)  /* Set IP address */
#define SIOCGIFDSTADDR   _SIOC(0x0003)  /* Get P-to-P address */
#define SIOCSIFDSTADDR   _SIOC(0x0004)  /* Set P-to-P address */
#define SIOCGIFBRDADDR   _SIOC(0x0005)  /* Get broadcast IP address */
#define SIOCSIFBRDADDR   _SIOC(0x0006)  /* Set broadcast IP address */
#define SIOCGIFNETMASK   _SIOC(0x0007)  /* Get network mask */
#define SIOCSIFNETMASK   _SIOC(0x0008)  /* Set network mask */
#define SIOCGIFMTU       _SIOC(0x0009)  /* Get MTU size */

/* IPv6 interface control operations */

#define SIOCGLIFADDR     _SIOC(0x000a)  /* Get IP address */
#define SIOCSLIFADDR     _SIOC(0x000b)  /* Set IP address */
#define SIOCGLIFDSTADDR  _SIOC(0x000c)  /* Get P-to-P address */
#define SIOCSLIFDSTADDR  _SIOC(0x000d)  /* Set P-to-P address */
#define SIOCGLIFBRDADDR  _SIOC(0x000e)  /* Get broadcast IP address */
#define SIOCSLIFBRDADDR  _SIOC(0x000f)  /* Set broadcast IP address */
#define SIOCGLIFNETMASK  _SIOC(0x0010)  /* Get network mask */
#define SIOCSLIFNETMASK  _SIOC(0x0011)  /* Set network mask */
#define SIOCGLIFMTU      _SIOC(0x0012)  /* Get MTU size */
#define SIOCIFAUTOCONF   _SIOC(0x0013)  /* Perform ICMPv6 auto-configuration */

/* Common interface control operations */

#define SIOCGIFHWADDR    _SIOC(0x0014)  /* Get hardware address */
#define SIOCSIFHWADDR    _SIOC(0x0015)  /* Set hardware address */
#define SIOCDIFADDR      _SIOC(0x0016)  /* Delete IP address (IPv4 and IPv6) */
#define SIOCGIFCOUNT     _SIOC(0x0017)  /* Get number of devices */

/* Interface flags */

#define SIOCSIFFLAGS     _SIOC(0x0018)  /* Sets the interface flags */
#define SIOCGIFFLAGS     _SIOC(0x0019)  /* Gets the interface flags */

#define SIOCGIPMSFILTER  _SIOC(0x001a)  /* Retrieve source filter addresses */
#define SIOCSIPMSFILTER  _SIOC(0x001b)  /* Set source filter content */

/* ARP Table.  Argument is a reference to sruct arpreq as defined
 * include/nuttx/net/arp.h
 */

#define SIOCSARP         _SIOC(0x001c)  /* Set an ARP mapping */
#define SIOCDARP         _SIOC(0x001d)  /* Delete an ARP mapping */
#define SIOCGARP         _SIOC(0x001e)  /* Get an ARP mapping */

/* Routing table.  Argument is a reference to struct rtentry as defined in
 * include/net/route.h
 */

#define SIOCADDRT        _SIOC(0x001f)  /* Add an entry to the routing table */
#define SIOCDELRT        _SIOC(0x0020)  /* Delete an entry from the routing table */

/* MDIO/MCD *****************************************************************/

#define SIOCMIINOTIFY    _SIOC(0x0021)  /* Receive notificaion via signal on
                                         * PHY state change */
#define SIOCGMIIPHY      _SIOC(0x0022)  /* Get address of MII PHY in use */
#define SIOCGMIIREG      _SIOC(0x0023)  /* Get a MII register via MDIO */
#define SIOCSMIIREG      _SIOC(0x0024)  /* Set a MII register via MDIO */

/* Unix domain sockets ******************************************************/

#define SIOCINQ          _SIOC(0x0025)  /* Returns the amount of queued unread
                                         * data in the receive */

/* TUN/TAP driver ***********************************************************/

#define TUNSETIFF        _SIOC(0x0026)  /* Set TUN/TAP interface */

/* Telnet driver ************************************************************/

#define SIOCTELNET       _SIOC(0x0027)  /* Create a Telnet sessions.
                                         * See include/nuttx/net/telnet.h */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* See include/net/if.h, include/net/route.h, and include/net/arp.h */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_IOCTL_H */
