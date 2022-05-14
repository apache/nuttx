/************************************************************************************
 * include/nuttx/wireless/wireless.h
 * Wireless network IOCTL commands
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
 ************************************************************************************/

/* This file includes common definitions to be used in all wireless network drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_WIRELESS_H
#define __INCLUDE_NUTTX_WIRELESS_WIRELESS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <stdint.h>

#include <net/if.h>
#include <nuttx/fs/ioctl.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Network Driver IOCTL Commands ****************************************************/

/* Use of these IOCTL commands requires a socket descriptor created by the socket()
 * interface.
 */

/* Wireless identification */

#define SIOCSIWCOMMIT       _WLIOC(0x0000)  /* Commit pending changes to driver */
#define SIOCGIWNAME         _WLIOC(0x0001)  /* Get name of wireless protocol */

/* Basic Operations */

#define SIOCSIWNWID         _WLIOC(0x0002)  /* Set network ID (pre-802.11) */
#define SIOCGIWNWID         _WLIOC(0x0003)  /* Get network ID (the cell) */
#define SIOCSIWFREQ         _WLIOC(0x0004)  /* Set channel/frequency (Hz) */
#define SIOCGIWFREQ         _WLIOC(0x0005)  /* Get channel/frequency (Hz) */
#define SIOCSIWMODE         _WLIOC(0x0006)  /* Set operation mode */
#define SIOCGIWMODE         _WLIOC(0x0007)  /* Get operation mode */
#define SIOCSIWSENS         _WLIOC(0x0008)  /* Set sensitivity (dBm) */
#define SIOCGIWSENS         _WLIOC(0x0009)  /* Get sensitivity (dBm) */

/* Informational */

#define SIOCSIWRANGE        _WLIOC(0x000a)  /* Unused */
#define SIOCGIWRANGE        _WLIOC(0x000b)  /* Get range of parameters */
#define SIOCSIWPRIV         _WLIOC(0x000c)  /* Unused */
#define SIOCGIWPRIV         _WLIOC(0x000d)  /* Get private ioctl interface info */
#define SIOCSIWSTATS        _WLIOC(0x000e)  /* Unused */
#define SIOCGIWSTATS        _WLIOC(0x000f)  /* Get wireless stats */

/* Spy support (statistics per MAC address - used for Mobile IP support) */

#define SIOCSIWSPY          _WLIOC(0x0010)  /* Set spy addresses */
#define SIOCGIWSPY          _WLIOC(0x0011)  /* Get spy info (quality of link) */
#define SIOCSIWTHRSPY       _WLIOC(0x0012)  /* Set spy threshold (spy event) */
#define SIOCGIWTHRSPY       _WLIOC(0x0013)  /* Get spy threshold */

/* Access point manipulation */

#define SIOCSIWAP           _WLIOC(0x0014)  /* Set access point MAC addresses */
#define SIOCGIWAP           _WLIOC(0x0015)  /* Get access point MAC addresses */
                                            /* 0x0016:  See SIOCSIWMLME */
#define SIOCGIWAPLIST       _WLIOC(0x0017)  /* Deprecated in favor of scanning */
#define SIOCSIWSCAN         _WLIOC(0x0018)  /* Trigger scanning (list cells) */
#define SIOCGIWSCAN         _WLIOC(0x0019)  /* Get scanning results */

/* 802.11 specific support */

#define SIOCSIWESSID        _WLIOC(0x001a)  /* Set ESSID (network name) */
#define SIOCGIWESSID        _WLIOC(0x001b)  /* Get ESSID */
#define SIOCSIWNICKN        _WLIOC(0x001c)  /* Set node name/nickname */
#define SIOCGIWNICKN        _WLIOC(0x001d)  /* Get node name/nickname */

#define SIOCSIWRATE         _WLIOC(0x0020)  /* Set default bit rate (bps) */
#define SIOCGIWRATE         _WLIOC(0x0021)  /* Get default bit rate (bps) */
#define SIOCSIWRTS          _WLIOC(0x0022)  /* Set RTS/CTS threshold (bytes) */
#define SIOCGIWRTS          _WLIOC(0x0023)  /* Get RTS/CTS threshold (bytes) */
#define SIOCSIWFRAG         _WLIOC(0x0024)  /* Set fragmentation thr (bytes) */
#define SIOCGIWFRAG         _WLIOC(0x0025)  /* Get fragmentation thr (bytes) */
#define SIOCSIWTXPOW        _WLIOC(0x0026)  /* Set transmit power (dBm) */
#define SIOCGIWTXPOW        _WLIOC(0x0027)  /* Get transmit power (dBm) */
#define SIOCSIWRETRY        _WLIOC(0x0028)  /* Set retry limits and lifetime */
#define SIOCGIWRETRY        _WLIOC(0x0029)  /* Get retry limits and lifetime */

/* Encoding */

#define SIOCSIWENCODE       _WLIOC(0x002a)  /* Set encoding token & mode */
#define SIOCGIWENCODE       _WLIOC(0x002b)  /* Get encoding token & mode */

/* Power saving */

#define SIOCSIWPOWER        _WLIOC(0x002c)  /* Set Power Management settings */
#define SIOCGIWPOWER        _WLIOC(0x002d)  /* Get Power Management settings */

/* WPA : Generic IEEE 802.11 information element */

#define SIOCSIWGENIE        _WLIOC(0x0030)  /* Set generic IE */
#define SIOCGIWGENIE        _WLIOC(0x0031)  /* Get generic IE */

/* WPA : IEEE 802.11 MLME requests */

#define SIOCSIWMLME         _WLIOC(0x0016)  /* Request MLME operation */

/* WPA : Authentication mode parameters */

#define SIOCSIWAUTH         _WLIOC(0x0032)  /* Set authentication mode params */
#define SIOCGIWAUTH         _WLIOC(0x0033)  /* Get authentication mode params */

/* WPA : Extended version of encoding configuration */

#define SIOCSIWENCODEEXT    _WLIOC(0x0034)  /* Set encoding token & mode */
#define SIOCGIWENCODEEXT    _WLIOC(0x0035)  /* Get encoding token & mode */

/* WPA2 : PMKSA cache management */

#define SIOCSIWPMKSA        _WLIOC(0x0036)  /* PMKSA cache operation */

/* Country code extension */

#define SIOCSIWCOUNTRY      _WLIOC(0x0037)  /* Set country code */
#define SIOCGIWCOUNTRY      _WLIOC(0x0038)  /* Get country code */

/* WIFI / BT coexist type */

#define SIOCSIWPTAPRIO      _WLIOC(0x0039)  /* Set PTA priority type */
#define SIOCGIWPTAPRIO      _WLIOC(0x003a)  /* Get PTA priority type */

#define WL_IS80211POINTERCMD(cmd) ((cmd) == SIOCGIWSCAN || \
                                   (cmd) == SIOCSIWSCAN || \
                                   (cmd) == SIOCSIWCOUNTRY || \
                                   (cmd) == SIOCGIWCOUNTRY || \
                                   (cmd) == SIOCGIWRANGE || \
                                   (cmd) == SIOCSIWENCODEEXT || \
                                   (cmd) == SIOCGIWENCODEEXT || \
                                   (cmd) == SIOCGIWESSID || \
                                   (cmd) == SIOCSIWESSID)

/* Device-specific network IOCTL commands *******************************************/

#define WL_NETFIRST         0x0000          /* First network command */
#define WL_NNETCMDS         0x003b          /* Number of network commands */

/* Reserved for Bluetooth network devices (see bt_ioctls.h) */

#define WL_BLUETOOTHFIRST     (WL_NETFIRST + WL_NNETCMDS)
#define WL_BLUETOOTHCMDS      (26)
#define WL_IBLUETOOTHCMD(cmd) (_WLIOCVALID(cmd) && \
                              _IOC_NR(cmd) >= WL_BLUETOOTHFIRST && \
                              _IOC_NR(cmd) < (WL_BLUETOOTHFIRST + WL_BLUETOOTHCMDS))

/* Reserved for IEEE802.15.4 wireless network devices
 * NOTE:  Not used.  Currently logic uses IOCTL commands from the IEEE802.15.4
 * character driver space.
 */

#define WL_802154FIRST        (WL_BLUETOOTHFIRST + WL_BLUETOOTHCMDS)
#define WL_N802154CMDS        (3)
#define WL_IS802154CMD(cmd)   (_WLIOCVALID(cmd) && \
                               _IOC_NR(cmd) >= WL_802154FIRST && \
                               _IOC_NR(cmd) < (WL_802154FIRST + WL_N802154CMDS))

/* Reserved for network packet radio network devices  */

#define WL_PKTRADIOFIRST      (WL_802154FIRST + WL_N802154CMDS)
#define WL_NPKTRADIOCMDS      (3)
#define WL_ISPKTRADIOCMD(cmd) (_WLIOCVALID(cmd) && \
                               _IOC_NR(cmd) >= WL_PKTRADIOFIRST && \
                               _IOC_NR(cmd) < (WL_PKTRADIOFIRST + WL_NPKTRADIOCMDS))

/* ------------------------------ WIRELESS EVENTS --------------------------------- */

/* Those are *NOT* ioctls, do not issue request on them !!! */

/* Most events use the same identifier as ioctl requests */

#define IWEVTXDROP      0x8c00        /* Packet dropped to excessive retry */
#define IWEVQUAL        0x8c01        /* Quality part of statistics (scan) */
#define IWEVCUSTOM      0x8c02        /* Driver specific ascii string */
#define IWEVREGISTERED  0x8c03        /* Discovered a new node (AP mode) */
#define IWEVEXPIRED     0x8c04        /* Expired a node (AP mode) */
#define IWEVGENIE       0x8c05        /* Generic IE (WPA, RSN, WMM, ..)
                                       * (scan results); This includes id and
                                       * length fields. One IWEVGENIE may
                                       * contain more than one IE. Scan
                                       * results may contain one or more
                                       * IWEVGENIE events. */
#define IWEVMICHAELMICFAILURE 0x8c06  /* Michael MIC failure
                                       * (struct iw_michaelmicfailure)
                                       */
#define IWEVASSOCREQIE  0x8c07        /* IEs used in (Re)Association Request.
                                       * The data includes id and length
                                       * fields and may contain more than one
                                       * IE. This event is required in
                                       * Managed mode if the driver
                                       * generates its own WPA/RSN IE. This
                                       * should be sent just before
                                       * IWEVREGISTERED event for the
                                       * association. */
#define IWEVASSOCRESPIE 0x8c08        /* IEs used in (Re)Association
                                       * Response. The data includes id and
                                       * length fields and may contain more
                                       * than one IE. This may be sent
                                       * between IWEVASSOCREQIE and
                                       * IWEVREGISTERED events for the
                                       * association. */
#define IWEVPMKIDCAND   0x8c09        /* PMKID candidate for RSN
                                       * pre-authentication
                                       * (struct iw_pmkid_cand) */

#define IWEVFIRST       0x8c00
#define IW_EVENT_IDX(cmd) ((cmd) - IWEVFIRST)

/* Other Common Wireless Definitions ************************************************/

/* Maximum size of the ESSID and NICKN strings */

#define IW_ESSID_MAX_SIZE   32

/* Modes of operation */

#define IW_MODE_AUTO        0    /* Let the driver decides */
#define IW_MODE_ADHOC       1    /* Single cell network */
#define IW_MODE_INFRA       2    /* Multi cell network, roaming, ... */
#define IW_MODE_MASTER      3    /* Synchronisation master or Access Point */
#define IW_MODE_REPEAT      4    /* Wireless Repeater (forwarder) */
#define IW_MODE_SECOND      5    /* Secondary master/repeater (backup) */
#define IW_MODE_MONITOR     6    /* Passive monitor (listen only) */
#define IW_MODE_MESH        7    /* Mesh (IEEE 802.11s) network */

/* Statistics flags (bitmask in updated) */

#define IW_QUAL_QUAL_UPDATED  0x01  /* Value was updated since last read */
#define IW_QUAL_LEVEL_UPDATED 0x02
#define IW_QUAL_NOISE_UPDATED 0x04
#define IW_QUAL_ALL_UPDATED   0x07
#define IW_QUAL_DBM           0x08  /* Level + Noise are dBm */
#define IW_QUAL_QUAL_INVALID  0x10  /* Driver doesn't provide value */
#define IW_QUAL_LEVEL_INVALID 0x20
#define IW_QUAL_NOISE_INVALID 0x40
#define IW_QUAL_RCPI          0x80  /* Level + Noise are 802.11k RCPI */
#define IW_QUAL_ALL_INVALID   0x70

/* Flags for encoding (along with the token) */

#define IW_ENCODE_INDEX      0x00FF  /* Token index (if needed) */
#define IW_ENCODE_FLAGS      0xFF00  /* Flags defined below */
#define IW_ENCODE_MODE       0xF000  /* Modes defined below */
#define IW_ENCODE_DISABLED   0x8000  /* Encoding disabled */
#define IW_ENCODE_ENABLED    0x0000  /* Encoding enabled */
#define IW_ENCODE_RESTRICTED 0x4000  /* Refuse non-encoded packets */
#define IW_ENCODE_OPEN       0x2000  /* Accept non-encoded packets */
#define IW_ENCODE_NOKEY      0x0800  /* Key is write only, so not present */
#define IW_ENCODE_TEMP       0x0400  /* Temporary key */

/* Frequency flags */

#define IW_FREQ_AUTO        0    /* Let the driver decides */
#define IW_FREQ_FIXED       1    /* Force a specific value */

#define IW_MAX_FREQUENCIES  32   /* Max. frequencies in struct iw_range */

/* ESSID flags */

#define IW_ESSID_OFF        0    /* Disconnect with access point */
#define IW_ESSID_ON         1    /* Connect  with access point */
#define IW_ESSID_DELAY_ON   2    /* Delay the connection behavior of essid */

/* Transmit Power flags available */

#define IW_TXPOW_TYPE       0x00ff  /* Type of value */
#  define IW_TXPOW_DBM      0x0000  /* Value is in dBm */
#  define IW_TXPOW_MWATT    0x0001  /* Value is in mW */
#  define IW_TXPOW_RELATIVE 0x0002  /* Value is in arbitrary units */
#define IW_TXPOW_RANGE      0x1000  /* Range of value between min/max */

/* Scan-related */

/* Scanning request flags */

#define IW_SCAN_DEFAULT    0x0000  /* Default scan of the driver */
#define IW_SCAN_ALL_ESSID  0x0001  /* Scan all ESSIDs */
#define IW_SCAN_THIS_ESSID 0x0002  /* Scan only this ESSID */
#define IW_SCAN_ALL_FREQ   0x0004  /* Scan all Frequencies */
#define IW_SCAN_THIS_FREQ  0x0008  /* Scan only this Frequency */
#define IW_SCAN_ALL_MODE   0x0010  /* Scan all Modes */
#define IW_SCAN_THIS_MODE  0x0020  /* Scan only this Mode */
#define IW_SCAN_ALL_RATE   0x0040  /* Scan all Bit-Rates */
#define IW_SCAN_THIS_RATE  0x0080  /* Scan only this Bit-Rate */

/* struct iw_scan_req scan_type */

#define IW_SCAN_TYPE_ACTIVE  0
#define IW_SCAN_TYPE_PASSIVE 1

/* Maximum size of returned data */

#define IW_SCAN_MAX_DATA     4096  /* In bytes */

/* Scan capability flags - in (struct iw_range *)->scan_capa */

#define IW_SCAN_CAPA_NONE    0x00
#define IW_SCAN_CAPA_ESSID   0x01
#define IW_SCAN_CAPA_BSSID   0x02
#define IW_SCAN_CAPA_CHANNEL 0x04
#define IW_SCAN_CAPA_MODE    0x08
#define IW_SCAN_CAPA_RATE    0x10
#define IW_SCAN_CAPA_TYPE    0x20
#define IW_SCAN_CAPA_TIME    0x40

/* SIOCSIWAUTH/SIOCGIWAUTH struct iw_param flags */

#define IW_AUTH_INDEX   0x0FFF
#define IW_AUTH_FLAGS   0xF000

/* SIOCSIWAUTH/SIOCGIWAUTH parameters (0 .. 4095)
 * (IW_AUTH_INDEX mask in struct iw_param flags; this is the index of the
 * parameter that is being set/get to; value will be read/written to
 * struct iw_param value field)
 */

#define IW_AUTH_WPA_VERSION          0
#define IW_AUTH_CIPHER_PAIRWISE      1
#define IW_AUTH_CIPHER_GROUP         2
#define IW_AUTH_KEY_MGMT             3
#define IW_AUTH_TKIP_COUNTERMEASURES 4
#define IW_AUTH_DROP_UNENCRYPTED     5
#define IW_AUTH_80211_AUTH_ALG       6
#define IW_AUTH_WPA_ENABLED          7
#define IW_AUTH_RX_UNENCRYPTED_EAPOL 8
#define IW_AUTH_ROAMING_CONTROL      9
#define IW_AUTH_PRIVACY_INVOKED      10
#define IW_AUTH_CIPHER_GROUP_MGMT    11
#define IW_AUTH_MFP                  12

/* IW_AUTH_WPA_VERSION values (bit field) */

#define IW_AUTH_WPA_VERSION_DISABLED 0x00000001
#define IW_AUTH_WPA_VERSION_WPA      0x00000002
#define IW_AUTH_WPA_VERSION_WPA2     0x00000004

/* IW_AUTH_PAIRWISE_CIPHER and IW_AUTH_GROUP_CIPHER values (bit field) */

#define IW_AUTH_CIPHER_NONE          0x00000001
#define IW_AUTH_CIPHER_WEP40         0x00000002
#define IW_AUTH_CIPHER_TKIP          0x00000004
#define IW_AUTH_CIPHER_CCMP          0x00000008
#define IW_AUTH_CIPHER_WEP104        0x00000010
#define IW_AUTH_CIPHER_AES_CMAC      0x00000020

/* IW_AUTH_KEY_MGMT values (bit field) */

#define IW_AUTH_KEY_MGMT_802_1X      1
#define IW_AUTH_KEY_MGMT_PSK         2

/* IW_AUTH_80211_AUTH_ALG values (bit field) */

#define IW_AUTH_ALG_OPEN_SYSTEM      0x00000001
#define IW_AUTH_ALG_SHARED_KEY       0x00000002
#define IW_AUTH_ALG_LEAP             0x00000004

/* IW_AUTH_ROAMING_CONTROL values */

#define IW_AUTH_ROAMING_ENABLE       0 /* driver/firmware based roaming */
#define IW_AUTH_ROAMING_DISABLE      1 /* user space program used for roaming
                                        * control */

/* SIOCSIWENCODEEXT definitions */

#define IW_ENCODE_SEQ_MAX_SIZE       8

/* struct iw_encode_ext ->alg */

#define IW_ENCODE_ALG_NONE           0
#define IW_ENCODE_ALG_WEP            1
#define IW_ENCODE_ALG_TKIP           2
#define IW_ENCODE_ALG_CCMP           3
#define IW_ENCODE_ALG_PMK            4
#define IW_ENCODE_ALG_AES_CMAC       5

/* IW_COEX_PTA_PRIORITY values */

#define IW_PTA_PRIORITY_COEX_MAXIMIZED 0
#define IW_PTA_PRIORITY_COEX_HIGH      1
#define IW_PTA_PRIORITY_BALANCED       2
#define IW_PTA_PRIORITY_WLAN_HIGH      3
#define IW_PTA_PRIORITY_WLAN_MAXIMIZED 4

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* TODO:
 *
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

/* Packet discarded in the wireless adapter due to
 * "wireless" specific problems...
 * Note : the list of counter and statistics in net_device_stats
 * is already pretty exhaustive, and you should use that first.
 * This is only additional stats...
 */

struct iw_discarded
{
  uint32_t nwid;      /* Rx : Wrong nwid/essid */
  uint32_t code;      /* Rx : Unable to code/decode (WEP) */
  uint32_t fragment;  /* Rx : Can't perform MAC reassembly */
  uint32_t retries;   /* Tx : Max MAC retries num reached */
  uint32_t misc;      /* Others cases */
};

/* Packet/Time period missed in the wireless adapter due to
 * "wireless" specific problems...
 */

struct iw_missed
{
  uint32_t beacon;    /* Missed beacons/superframe */
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
  char ifr_name[IFNAMSIZ];  /* Interface name, e.g. "eth0" */
  union iwreq_data u;       /* Data payload */
};

/* Range of parameters (currently only frequencies) */

struct iw_range
{
  uint8_t num_frequency;  /* Number of frequencies in the freq[] list */
  struct iw_freq freq[IW_MAX_FREQUENCIES];
};

/* A Wireless Event. */

struct iw_event
{
  uint16_t           len;   /* Real length of ata */
  uint16_t           cmd;   /* Wireless IOCTL command */
  union iwreq_data   u;     /* Fixed IOCTL payload */
};

/* WPA support */

struct  iw_encode_ext
{
  uint32_t ext_flags;                      /* IW_ENCODE_EXT_* */
  uint8_t  tx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
  uint8_t  rx_seq[IW_ENCODE_SEQ_MAX_SIZE]; /* LSB first */
  struct sockaddr addr;                    /* ff:ff:ff:ff:ff:ff for
                                            * broadcast/multicast
                                            * (group) keys or unicast address
                                            * for individual keys */
  uint16_t alg;                            /* IW_ENCODE_ALG_* */
  uint16_t key_len;
  uint8_t  key[0];
};

/* Optional data for scan request
 *
 * Note: these optional parameters are controlling parameters for the
 * scanning behavior, these do not apply to getting scan results
 * (SIOCGIWSCAN). Drivers are expected to keep a local BSS table and
 * provide a merged results with all BSSes even if the previous scan
 * request limited scanning to a subset, e.g., by specifying an SSID.
 * Especially, scan results are required to include an entry for the
 * current BSS if the driver is in Managed mode and associated with an AP.
 */

struct  iw_scan_req
{
  uint8_t scan_type;     /* IW_SCAN_TYPE_{ACTIVE,PASSIVE} */
  uint8_t essid_len;
  uint8_t num_channels;  /* num entries in channel_list;
                          * 0 = scan all allowed channels */
  uint8_t flags;         /* reserved as padding; use zero, this may
                          * be used in the future for adding flags
                          * to request different scan behavior */
  struct sockaddr bssid; /* ff:ff:ff:ff:ff:ff for broadcast BSSID or
                          * individual address of a specific BSS */

  /* Use this ESSID if IW_SCAN_THIS_ESSID flag is used instead of using
   * the current ESSID. This allows scan requests for specific ESSID
   * without having to change the current ESSID and potentially breaking
   * the current association.
   */

  uint8_t essid[IW_ESSID_MAX_SIZE];

  /* Optional parameters for changing the default scanning behavior.
   * These are based on the MLME-SCAN.request from IEEE Std 802.11.
   * TU is 1.024 ms. If these are set to 0, driver is expected to use
   * reasonable default values. min_channel_time defines the time that
   * will be used to wait for the first reply on each channel. If no
   * replies are received, next channel will be scanned after this. If
   * replies are received, total time waited on the channel is defined by
   * max_channel_time.
   */

  uint32_t min_channel_time; /* in TU */
  uint32_t max_channel_time; /* in TU */

  struct iw_freq  channel_list[IW_MAX_FREQUENCIES];
};

/* Wireless statistics */

struct iw_statistics
{
  uint16_t status;              /* Status
                                 * - device dependent for now
                                 */

  struct iw_quality qual;       /* Quality of the link
                                 * (instant/mean/max)
                                 */
  struct iw_discarded discard;  /* Packet discarded counts */
  struct iw_missed miss;        /* Packet missed counts */
};

/* A Wireless Event. Contains basically the same data as the ioctl...
 */

#define IW_EV_LEN(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

#endif /* __INCLUDE_NUTTX_WIRELESS_WIRELESS_H */
