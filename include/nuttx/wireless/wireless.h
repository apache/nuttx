/************************************************************************************
 * include/nuttx/wireless/wireless.h
 *
 *   Copyright (C) 2011-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Laurent Latil <gnutt@nuttx.org>
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
 ************************************************************************************/

/* This file includes common definitions to be used in all wireless drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_H
#define __INCLUDE_NUTTX_WIRELESS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <stdint.h>

#include <net/if.h>
#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_DRIVERS_WIRELESS

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Network Driver IOCTL Commands ****************************************************/
/* Use of these IOCTL commands requires a socket descriptor created by the socket()
 * interface.
 */

/* Wireless identification */

#define SIOCSIWCOMMIT       _WLIOC(0x0001)  /* Commit pending changes to driver */
#define SIOCGIWNAME         _WLIOC(0x0002)  /* Get name of wireless protocol */

/* Basic Operations */

#define SIOCSIWNWID         _WLIOC(0x0003)  /* Set network ID (pre-802.11) */
#define SIOCGIWNWID         _WLIOC(0x0004)  /* Get network ID (the cell) */
#define SIOCSIWFREQ         _WLIOC(0x0005)  /* Set channel/frequency (Hz) */
#define SIOCGIWFREQ         _WLIOC(0x0006)  /* Get channel/frequency (Hz) */
#define SIOCSIWMODE         _WLIOC(0x0007)  /* Set operation mode */
#define SIOCGIWMODE         _WLIOC(0x0008)  /* Get operation mode */
#define SIOCSIWSENS         _WLIOC(0x0009)  /* Set sensitivity (dBm) */
#define SIOCGIWSENS         _WLIOC(0x000a)  /* Get sensitivity (dBm) */

/* Informational */

#define SIOCGIWRANGE        _WLIOC(0x000b)  /* Get range of parameters */
#define SIOCGIWPRIV         _WLIOC(0x000c)  /* Get private ioctl interface info */
#define SIOCGIWRANGE        _WLIOC(0x000d)  /* Get range of parameters */
#define SIOCGIWPRIV         _WLIOC(0x000e)  /* Get private ioctl interface info */
#define SIOCGIWSTATS        _WLIOC(0x000f)  /* Get wireless stats */

/* Spy support (statistics per MAC address - used for Mobile IP support) */

#define SIOCSIWSPY          _WLIOC(0x0010)  /* Set spy addresses */
#define SIOCGIWSPY          _WLIOC(0x0011)  /* Get spy info (quality of link) */
#define SIOCSIWTHRSPY       _WLIOC(0x0012)  /* Set spy threshold (spy event) */
#define SIOCGIWTHRSPY       _WLIOC(0x0013)  /* Get spy threshold */

/* Access point manipulation */

#define SIOCSIWAP           _WLIOC(0x0014)  /* Set access point MAC addresses */
#define SIOCGIWAP           _WLIOC(0x0015)  /* Get access point MAC addresses */
#define SIOCGIWAPLIST       _WLIOC(0x0016)  /* Deprecated in favor of scanning */
#define SIOCSIWSCAN         _WLIOC(0x0017)  /* Trigger scanning (list cells) */
#define SIOCGIWSCAN         _WLIOC(0x0018)  /* Get scanning results */

/* 802.11 specific support */

#define SIOCSIWESSID        _WLIOC(0x0019)  /* Set ESSID (network name) */
#define SIOCGIWESSID        _WLIOC(0x001a)  /* Get ESSID */
#define SIOCSIWNICKN        _WLIOC(0x001b)  /* Set node name/nickname */
#define SIOCGIWNICKN        _WLIOC(0x001c)  /* Get node name/nickname */

#define SIOCSIWRATE         _WLIOC(0x001d)  /* Set default bit rate (bps) */
#define SIOCGIWRATE         _WLIOC(0x001e)  /* Get default bit rate (bps) */
#define SIOCSIWRTS          _WLIOC(0x001f)  /* Set RTS/CTS threshold (bytes) */
#define SIOCGIWRTS          _WLIOC(0x0010)  /* Get RTS/CTS threshold (bytes) */
#define SIOCSIWFRAG         _WLIOC(0x0011)  /* Set fragmentation thr (bytes) */
#define SIOCGIWFRAG         _WLIOC(0x0022)  /* Get fragmentation thr (bytes) */
#define SIOCSIWTXPOW        _WLIOC(0x0023)  /* Set transmit power (dBm) */
#define SIOCGIWTXPOW        _WLIOC(0x0024)  /* Get transmit power (dBm) */
#define SIOCSIWRETRY        _WLIOC(0x0025)  /* Set retry limits and lifetime */
#define SIOCGIWRETRY        _WLIOC(0x0026)  /* Get retry limits and lifetime */

/* Encoding */

#define SIOCSIWENCODE       _WLIOC(0x0027)  /* Set encoding token & mode */
#define SIOCGIWENCODE       _WLIOC(0x0028)  /* Get encoding token & mode */

/* Power saving */

#define SIOCSIWPOWER        _WLIOC(0x0029)  /* Set Power Management settings */
#define SIOCGIWPOWER        _WLIOC(0x002a)  /* Get Power Management settings */

/* WPA : Generic IEEE 802.11 information element */

#define SIOCSIWGENIE        _WLIOC(0x002b)  /* Set generic IE */
#define SIOCGIWGENIE        _WLIOC(0x002c)  /* Get generic IE */

/* WPA : IEEE 802.11 MLME requests */

#define SIOCSIWMLME         _WLIOC(0x002d)  /* Request MLME operation */

/* WPA : Authentication mode parameters */

#define SIOCSIWAUTH         _WLIOC(0x002e)  /* Set authentication mode params */
#define SIOCGIWAUTH         _WLIOC(0x002f)  /* Get authentication mode params */

/* WPA : Extended version of encoding configuration */

#define SIOCSIWENCODEEXT    _WLIOC(0x0030)  /* Set encoding token & mode */
#define SIOCGIWENCODEEXT    _WLIOC(0x0031)  /* Get encoding token & mode */

/* WPA2 : PMKSA cache management */

#define SIOCSIWPMKSA        _WLIOC(0x0032)  /* PMKSA cache operation */

#define WL_FIRSTCHAR        0x0033
#define WL_NNETCMDS         0x0032

/* Character Driver IOCTL commands *************************************************/
/* Non-compatible, NuttX only IOCTL definitions for use with low-level wireless
 * drivers that are accessed via a character device.  Use of these IOCTL commands
 * requires a file descriptor created by the open() interface.
 */

#define WLIOC_SETRADIOFREQ  _WLIOC(0x0033)  /* arg: Pointer to uint32_t, frequency
                                             * value (in Mhz) */
#define WLIOC_GETRADIOFREQ  _WLIOC(0x0034)  /* arg: Pointer to uint32_t, frequency
                                             * value (in Mhz) */
#define WLIOC_SETADDR       _WLIOC(0x0035)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_GETADDR       _WLIOC(0x0036)  /* arg: Pointer to address value, format
                                             * of the address is driver specific */
#define WLIOC_SETTXPOWER    _WLIOC(0x0037)  /* arg: Pointer to int32_t, output power
                                             * (in dBm) */
#define WLIOC_GETTXPOWER    _WLIOC(0x0038)  /* arg: Pointer to int32_t, output power
                                             * (in dBm) */

/* Device-specific IOCTL commands **************************************************/

#define WL_FIRST            0x0001          /* First common command */
#define WL_NCMDS            0x0038          /* Number of common commands */

/* User defined ioctl commands are also supported. These will be forwarded
 * by the upper-half QE driver to the lower-half QE driver via the ioctl()
 * method fo the QE lower-half interface.  However, the lower-half driver
 * must reserve a block of commands as follows in order prevent IOCTL
 * command numbers from overlapping.
 */

/* See include/nuttx/wireless/cc3000.h */

#define CC3000_FIRST        (WL_FIRST + WL_NCMDS)
#define CC3000_NCMDS        7

/* See include/nuttx/wireless/nrf24l01.h */

#define NRF24L01_FIRST      (CC3000_FIRST + CC3000_NCMDS)
#define NRF24L01_NCMDS      14

/* Other Common Wireless Definitions ***********************************************/

/* Maximum size of the ESSID and NICKN strings */

#define IW_ESSID_MAX_SIZE   32

/************************************************************************************
 * Public Types
 ************************************************************************************/
/* TODO:
 *
 * - Add types for statistics (struct iw_statistics and related)
 * - Add struct iw_range for use with IOCTL commands that need exchange mode data
 *   that could not fit in iwreq.
 * - Private IOCTL data support (struct iw_priv_arg)
 * - Quality
 * - WPA support.
 * - Wireless events.
 * - Various flag definitions.
 *
 * These future additions will all need to be compatible with BSD/Linux definitions.
 */

/* Generic format for most parameters that fit in a int32_t */

struct iw_param
{
  int32_t   value;          /* The value of the parameter itself */
  uint8_t   fixed;          /* Hardware should not use auto select */
  uint8_t   disabled;       /* Disable the feature */
  uint16_t  flags;          /* Optional flags */
};

/* Large data reference.  For all data larger than 16 octets, we need to use a
 * pointer to memory allocated in user space.
 */
 
struct iw_point
{
  FAR void *pointer;        /* Pointer to the data  (in user space) */
  uint16_t  length;         /* number of fields or size in bytes */
  uint16_t  flags;          /* Optional flags */
};

/* For numbers lower than 10^9, we encode the number in 'm' and set 'e' to 0
 * For number greater than 10^9, we divide it by the lowest power of 10 to
 * get 'm' lower than 10^9, with 'm'= f / (10^'e')...
 * The power of 10 is in 'e', the result of the division is in 'm'.
 */

struct iw_freq
{
  int32_t   m;              /* Mantissa */
  int16_t   e;              /* Exponent */
  uint8_t   i;              /* List index (when in range struct) */
  uint8_t   flags;          /* Flags (fixed/auto) */
};

/* Quality of the link */

struct iw_quality
{
  uint8_t   qual;           /* link quality (%retries, SNR,
                             * %missed beacons or better...) */
  uint8_t   level;          /* signal level (dBm) */
  uint8_t   noise;          /* noise level (dBm) */
  uint8_t   updated;        /* Flags to know if updated */
};

/* This union defines the data payload of an ioctl, and is used in struct iwreq
 * below.
 */

union iwreq_data
{
  char name[IFNAMSIZ];      /* Network interface name */
  struct iw_point essid;    /* Extended network name */
  struct iw_param nwid;     /* Network id (or domain - the cell) */
  struct iw_freq freq;      /* frequency or channel :
                             * 0-1000 = channel
                             * > 1000 = frequency in Hz */
  struct iw_param sens;     /* signal level threshold */
  struct iw_param bitrate;  /* default bit rate */
  struct iw_param txpower;  /* default transmit power */
  struct iw_param rts;      /* RTS threshold threshold */
  struct iw_param frag;     /* Fragmentation threshold */
  uint32_t mode;            /* Operation mode */
  struct iw_param retry;    /* Retry limits & lifetime */
 
  struct iw_point encoding; /* Encoding stuff : tokens */
  struct iw_param power;    /* PM duration/timeout */
  struct iw_quality qual;   /* Quality part of statistics */
 
  struct sockaddr ap_addr;  /* Access point address */
  struct sockaddr addr;     /* Destination address (hw/mac) */
 
  struct iw_param param;    /* Other small parameters */
  struct iw_point data;     /* Other large parameters */
};
 
/* This is the structure used to exchange data in wireless IOCTLs.  This structure
 * is the same as 'struct ifreq', but defined for use with wireless IOCTLs.
 */

struct iwreq
{
  char ifrn_name[IFNAMSIZ];    /* Interface name, e.g. "eth0" */
  union iwreq_data u;          /* Data payload */
};

#endif /* CONFIG_DRIVERS_WIRELESS */
#endif /* __INCLUDE_NUTTX_WIRELESS_H */
