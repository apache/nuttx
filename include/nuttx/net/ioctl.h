/****************************************************************************
 * include/nuttx/net/ioctl.h
 *
 *   Copyright (C) 2007-2008, 2010-2013, 2015 Gregory Nutt. All rights reserved.
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

#define _SIOCVALID(c)   (_IOC_TYPE(c)==_SIOCBASE)
#define _SIOC(nr)       _IOC(_SIOCBASE,nr)

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

/* Common interface control operations */

#define SIOCGIFHWADDR    _SIOC(0x0013)  /* Get hardware address */
#define SIOCSIFHWADDR    _SIOC(0x0014)  /* Set hardware address */
#define SIOCDIFADDR      _SIOC(0x0015)  /* Delete IP address (IPv4 and IPv6) */
#define SIOCGIFCOUNT     _SIOC(0x0016)  /* Get number of devices */

/* Interface flags */

#define SIOCSIFFLAGS     _SIOC(0x0017) /* Sets the interface flags */
#define SIOCGIFFLAGS     _SIOC(0x0018) /* Gets the interface flags */

#define SIOCGIPMSFILTER  _SIOC(0x0019)  /* Retrieve source filter addresses */
#define SIOCSIPMSFILTER  _SIOC(0x001a)  /* Set source filter content */

/* Routing table.  Argument is a reference to struct rtentry as defined in
 * include/net/route.h
 */

#define SIOCADDRT        _SIOC(0x001b)  /* Add an entry to the routing table */
#define SIOCDELRT        _SIOC(0x001c)  /* Delete an entry from the routing table */

/* Wireless ioctl commands **************************************************/

#define SIOCSIWCOMMIT    _SIOC(0x001d)  /* Commit pending changes to driver */
#define SIOCGIWNAME      _SIOC(0x001e)  /* Get name of wireless protocol */

#define SIOCSIWNWID      _SIOC(0x001f)  /* Set network ID (pre-802.11) */
#define SIOCGIWNWID      _SIOC(0x0020)  /* Get network ID (the cell) */
#define SIOCSIWFREQ      _SIOC(0x0021)  /* Set channel/frequency (Hz) */
#define SIOCGIWFREQ      _SIOC(0x0022)  /* Get channel/frequency (Hz) */
#define SIOCSIWMODE      _SIOC(0x0023)  /* Set operation mode */
#define SIOCGIWMODE      _SIOC(0x0024)  /* Get operation mode */
#define SIOCSIWSENS      _SIOC(0x0025)  /* Set sensitivity (dBm) */
#define SIOCGIWSENS      _SIOC(0x0026)  /* Get sensitivity (dBm) */

#define SIOCGIWRANGE     _SIOC(0x0027)  /* Get range of parameters */
#define SIOCGIWPRIV      _SIOC(0x0028)  /* Get private ioctl interface info */
#define SIOCGIWSTATS     _SIOC(0x0029)  /* Get wireless stats */

#define SIOCSIWSPY       _SIOC(0x002a)  /* Set spy addresses */
#define SIOCGIWSPY       _SIOC(0x002b)  /* Get spy info (quality of link) */
#define SIOCSIWTHRSPY    _SIOC(0x002c)  /* Set spy threshold (spy event) */
#define SIOCGIWTHRSPY    _SIOC(0x002d)  /* Get spy threshold */

#define SIOCSIWAP        _SIOC(0x002e)  /* Set access point MAC addresses */
#define SIOCGIWAP        _SIOC(0x002f)  /* Get access point MAC addresses */
#define SIOCGIWAPLIST    _SIOC(0x0030)  /* Deprecated in favor of scanning */
#define SIOCSIWSCAN      _SIOC(0x0031)  /* Trigger scanning (list cells) */
#define SIOCGIWSCAN      _SIOC(0x0032)  /* Get scanning results */

#define SIOCSIWESSID     _SIOC(0x0033)  /* Set ESSID (network name) */
#define SIOCGIWESSID     _SIOC(0x0034)  /* Get ESSID */
#define SIOCSIWNICKN     _SIOC(0x0035)  /* Set node name/nickname */
#define SIOCGIWNICKN     _SIOC(0x0036)  /* Get node name/nickname */

#define SIOCSIWRATE      _SIOC(0x0037)  /* Set default bit rate (bps) */
#define SIOCGIWRATE      _SIOC(0x0038)  /* Get default bit rate (bps) */
#define SIOCSIWRTS       _SIOC(0x0039)  /* Set RTS/CTS threshold (bytes) */
#define SIOCGIWRTS       _SIOC(0x003a)  /* Get RTS/CTS threshold (bytes) */
#define SIOCSIWFRAG      _SIOC(0x003b)  /* Set fragmentation thr (bytes) */
#define SIOCGIWFRAG      _SIOC(0x003c)  /* Get fragmentation thr (bytes) */
#define SIOCSIWTXPOW     _SIOC(0x003d)  /* Set transmit power (dBm) */
#define SIOCGIWTXPOW     _SIOC(0x003e)  /* Get transmit power (dBm) */
#define SIOCSIWRETRY     _SIOC(0x003f)  /* Set retry limits and lifetime */
#define SIOCGIWRETRY     _SIOC(0x0040)  /* Get retry limits and lifetime */

#define SIOCSIWPOWER     _SIOC(0x0041)  /* Set Power Management settings */
#define SIOCGIWPOWER     _SIOC(0x0042)  /* Get Power Management settings */

#define SIOCSIWGENIE     _SIOC(0x0043)  /* Set generic IE */
#define SIOCGIWGENIE     _SIOC(0x0044)  /* Get generic IE */

#define SIOCSIWMLME      _SIOC(0x0045)  /* Request MLME operation */

#define SIOCSIWAUTH      _SIOC(0x0046)  /* Set authentication mode params */
#define SIOCGIWAUTH      _SIOC(0x0047)  /* Get authentication mode params */

#define SIOCSIWENCODEEXT _SIOC(0x0048) /* Set encoding token & mode */
#define SIOCGIWENCODEEXT _SIOC(0x0049) /* Get encoding token & mode */

#define SIOCSIWPMKSA     _SIOC(0x004a) /* PMKSA cache operation */

/* MDIO/MCD *****************************************************************/

#define SIOCMIINOTIFY    _SIOC(0x004b) /* Receive notificaion via signal on
                                        * PHY state change */
#define SIOCGMIIPHY      _SIOC(0x004c) /* Get address of MII PHY in use */
#define SIOCGMIIREG      _SIOC(0x004d) /* Get a MII register via MDIO */
#define SIOCSMIIREG      _SIOC(0x004e) /* Set a MII register via MDIO */

/* Unix domain sockets ******************************************************/

#define SIOCINQ          _SIOC(0x004f) /* Returns the amount of queued unread
                                        * data in the receive */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* See include/net/if.h */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_IOCTL_H */
