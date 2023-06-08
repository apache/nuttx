/****************************************************************************
 * include/nuttx/net/ioctl.h
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
#define SIOCSIFMTU       _SIOC(0x0033)  /* Set MTU size */

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
#define SIOCGIFCONF      _SIOC(0x0018)  /* Return an interface list (IPv4) */
#define SIOCGLIFCONF     _SIOC(0x0019)  /* Return an interface list (IPv6) */

#define SIOCGIFNAME      _SIOC(0x002A)  /* Get interface name string */
#define SIOCGIFINDEX     _SIOC(0x002B)  /* Get index based name string */
#define SIOCSIFNAME      _SIOC(0x0034)  /* Set interface name string*/

/* Interface flags */

#define SIOCSIFFLAGS     _SIOC(0x001a)  /* Sets the interface flags */
#define SIOCGIFFLAGS     _SIOC(0x001b)  /* Gets the interface flags */

#define SIOCGIPMSFILTER  _SIOC(0x001c)  /* Retrieve source filter addresses */
#define SIOCSIPMSFILTER  _SIOC(0x001d)  /* Set source filter content */

/* Routing table.  Argument is a reference to struct rtentry as defined in
 * include/net/route.h
 */

#define SIOCADDRT        _SIOC(0x0021)  /* Add an entry to the routing table */
#define SIOCDELRT        _SIOC(0x0022)  /* Delete an entry from the routing table */

/* MDIO/MCD *****************************************************************/

#define SIOCMIINOTIFY    _SIOC(0x0023)  /* Receive notificaion via signal on
                                         * PHY state change */
#define SIOCGMIIPHY      _SIOC(0x0024)  /* Get address of MII PHY in use */
#define SIOCGMIIREG      _SIOC(0x0025)  /* Get a MII register via MDIO */
#define SIOCSMIIREG      _SIOC(0x0026)  /* Set a MII register via MDIO */

/* Unix domain sockets ******************************************************/

#define SIOCINQ          _SIOC(0x0027)  /* Returns the amount of queued unread
                                         * data in the receive */

/* TUN/TAP driver ***********************************************************/

#define TUNSETIFF        _SIOC(0x0028)  /* Set TUN/TAP interface */
#define TUNGETIFF        _SIOC(0x0035)  /* Get TUN/TAP interface */

/* Telnet driver ************************************************************/

#define SIOCTELNET       _SIOC(0x0029)  /* Create a Telnet sessions.
                                         * See include/nuttx/net/telnet.h */

/* SocketCAN ****************************************************************/

#define SIOCGCANBITRATE    _SIOC(0x002C)  /* Get bitrate from a CAN controller */
#define SIOCSCANBITRATE    _SIOC(0x002D)  /* Set bitrate of a CAN controller */
#define SIOCACANEXTFILTER  _SIOC(0x002E)  /* Add hardware-level exteneded ID filter */
#define SIOCDCANEXTFILTER  _SIOC(0x002F)  /* Delete hardware-level exteneded ID filter */
#define SIOCACANSTDFILTER  _SIOC(0x0030)  /* Add hardware-level standard ID filter */
#define SIOCDCANSTDFILTER  _SIOC(0x0031)  /* Delete hardware-level standard ID filter */

/* Network socket control ***************************************************/

#define SIOCDENYINETSOCK _SIOC(0x0033) /* Deny network socket. */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* See include/net/if.h, include/net/route.h, and include/net/arp.h */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_IOCTL_H */
