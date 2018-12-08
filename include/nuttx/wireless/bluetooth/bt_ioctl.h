/****************************************************************************
 * wireless/bluetooth/bt_ioctl.h
 * Bluetooth Network IOCTL commands.
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_IOCTL_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <net/if.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_gatt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_FEATURES_SIZE    8   /* LMP features */

/* Arbitrary size limits of things for fixed allocations.  These could be
 * increased with a memory usage penalty)
 */

#define HCI_GATT_MAXHANDLES  8   /* Max handles in GATT read multiple IOCTL */
#define HCI_GATTRD_DATA     32   /* Max number of bytes in GATT read data */
#define HCI_GATTWR_DATA     16   /* Max number of bytes in GATT write data */

/* Bluetooth network device IOCTL commands. */

#if !defined(WL_BLUETOOTHCMDS) || WL_BLUETOOTHCMDS != 26
#  error Incorrect setting for number of Bluetooth IOCTL commands
#endif

/* IOCTL Commands ***********************************************************
 * Many derive from NetBSD, at least in name.
 * All of the following use an argument of type struct btreg_s:
 *
 * Bluetooth Information Queries
 *
 * SIOCGBTINFO
 *   Get Bluetooth device Info.  Given the device name, fill in the btreq_s
 *   structure including the address field for use with socket addressing as
 *   above.
 * SIOCGBTINFOA
 *   Get Bluetooth device Info from Address.  Given the device address, fill
 *   in the btreq_s structure including the name field.
 * SIOCNBTINFO
 *   Next Bluetooth device Info.  If name field is empty, the first device
 *   will be returned.  Otherwise, the next device will be returned until
 *   no more devices are found when the call will fail, with error ENXIO.
 *   Thus, you can cycle through all devices in the system.
 */

#define SIOCGBTINFO            _WLIOC(WL_BLUETOOTHFIRST + 0)
#define SIOCGBTINFOA           _WLIOC(WL_BLUETOOTHFIRST + 1)
#define SIOCNBTINFO            _WLIOC(WL_BLUETOOTHFIRST + 2)

/* Features
 *
 * SIOCGBTFEAT
 *   Get Bluetooth BR/BDR device Features.  This returns the cached basic
 *   (page 0) and extended (page 1 & 2) features.  Only page 0 is valid.
 * SIOCGBTLEFEAT
 *   Get Bluetooth LE device Features.  This returns the cached page 0-2
 *   features.  Only page 0 is value.
 */

#define SIOCGBTFEAT            _WLIOC(WL_BLUETOOTHFIRST + 3)
#define SIOCGBTLEFEAT          _WLIOC(WL_BLUETOOTHFIRST + 4)

/* Set Flags, Link Policy, and Packet Types
 *
 * SIOCSBTFLAGS
 *   Set Bluetooth device Flags.  Not all flags are settable.
 * SIOCSBTPOLICY
 *   Set Bluetooth device Link Policy bits.
 * SIOCSBTPTYPE
 *   Set Bluetooth device Packet Types.  You can only set packet types that
 *   the device supports.
 */

#define SIOCSBTFLAGS           _WLIOC(WL_BLUETOOTHFIRST + 5)
#define SIOCSBTPOLICY          _WLIOC(WL_BLUETOOTHFIRST + 6)
#define SIOCSBTPTYPE           _WLIOC(WL_BLUETOOTHFIRST + 7)

/* Get Statistics:
 *
 * SIOCGBTSTATS
 *   Read device statistics.
 * SIOCZBTSTATS
 *   Read device statistics, and zero them.
 */

#define SIOCGBTSTATS           _WLIOC(WL_BLUETOOTHFIRST + 8)
#define SIOCZBTSTATS           _WLIOC(WL_BLUETOOTHFIRST + 9)

/* Advertisement
 *
 * SIOCBTADVSTART
 *   Set advertisement data, scan response data, advertisement parameters
 *   and start advertising.
 * SIOCBTADVSTOP
 *   Stop advertising.
 */

#define SIOCBTADVSTART         _WLIOC(WL_BLUETOOTHFIRST + 10)
#define SIOCBTADVSTOP          _WLIOC(WL_BLUETOOTHFIRST + 11)

/* Scanning
 *
 * SIOCBTSCANSTART
 *   Start LE scanning.  Buffered scan results may be obtained via
 *   SIOCBTSCANGET
 * SIOCBTSCANGET
 *   Return scan results buffered since the call time that the
 *   SIOCBTSCANGET command was invoked.
 * SIOCBTSCANSTOP
 *   Stop LE scanning and discard any buffered results.
 */

#define SIOCBTSCANSTART        _WLIOC(WL_BLUETOOTHFIRST + 12)
#define SIOCBTSCANGET          _WLIOC(WL_BLUETOOTHFIRST + 13)
#define SIOCBTSCANSTOP         _WLIOC(WL_BLUETOOTHFIRST + 14)

/* Security
 *
 * SIOCBTSECURITY
 *   Enable security for a connection.
 */

#define SIOCBTSECURITY         _WLIOC(WL_BLUETOOTHFIRST + 15)

/* GATT
 *
 * SIOCBTEXCHANGE
 *   Exchange MTUs
 * SIOCBTDISCOVER
 *   Starts GATT discovery
 * SIOCBTGATTRD
 *   Initiate read of GATT data
 * SIOCBTGATTWR
 *   Write GATT data
 */

#define SIOCBTEXCHANGE         _WLIOC(WL_BLUETOOTHFIRST + 16)
#define SIOCBTDISCOVER         _WLIOC(WL_BLUETOOTHFIRST + 17)
#define SIOCBTGATTRD           _WLIOC(WL_BLUETOOTHFIRST + 18)
#define SIOCBTGATTWR           _WLIOC(WL_BLUETOOTHFIRST + 19)

/* Connect/diconnect from a peer */

#define SIOCBTCONNECT          _WLIOC(WL_BLUETOOTHFIRST + 24)
#define SIOCBTDISCONNECT       _WLIOC(WL_BLUETOOTHFIRST + 25)

/* Definitions associated with struct btreg_s *******************************/
/* struct btreq_s union field accessors */

#define btr_bdaddr             btru.btri.btri_bdaddr
#define btr_flags              btru.btri.btri_flags
#define btr_num_cmd            btru.btri.btri_num_cmd
#define btr_num_acl            btru.btri.btri_num_acl
#define btr_num_sco            btru.btri.btri_num_sco
#define btr_acl_mtu            btru.btri.btri_acl_mtu
#define btr_sco_mtu            btru.btri.btri_sco_mtu
#define btr_link_policy        btru.btri.btri_link_policy
#define btr_packet_type        btru.btri.btri_packet_type
#define btr_max_acl            btru.btri.btri_max_acl
#define btr_max_sco            btru.btri.btri_max_sco

#define btr_features0          btru.btrf.btrf_page0
#define btr_features1          btru.btrf.btrf_page1
#define btr_features2          btru.btrf.btrf_page2

#define btr_stats              btru.btrs

#define btr_advtype            btru.btras.btras_advtype
#define btr_advad              btru.btras.btras_advad
#define btr_advsd              btru.btras.btras_advsd

#define btr_dupenable          btru.btrss.btrss_dupenable

#define btr_nrsp               btru.btrsr.brtsr_nrsp
#define btr_rsp                btru.btrsr.btrsr_rsp

#define btr_secaddr            btru.btrse.btrse_secaddr
#define btr_seclevel           btru.btrse.btrse_seclevel

#define btr_expeer             btru.btmx.btmx_expeer
#define btr_exresult           btru.btmx.btmx_result

#define btr_dtype              btru.btrds.btrds_dtype
#define btr_dpeer              btru.btrds.btrds_dpeer
#define btr_duuid16            btru.btrds.btrds_duuid16
#define btr_dstart             btru.btrds.btrds_dstart
#define btr_dend               btru.btrds.btrds_dend
#define btr_gnrsp              btru.btrds.btrds_gnrsp
#define btr_grsp               btru.btrds.btrds_grsp
#define btr_indx               btru.btrds.btrds_indx

#define btr_rdpeer             btru.btgrd.btgrd_rdpeer
#define btr_rdoffset           btru.btgrd.btgrd_rdoffset
#define btr_rdnhandles         btru.btgrd.btgrd_rdnhandles
#define btr_rdhandles          btru.btgrd.btgrd_rdhandles
#define btr_rdresult           btru.btgrd.btgrd_rdresult
#define btr_rdsize             btru.btgrd.btgrd_rdsize
#define btr_rddata             btru.btgrd.btgrd_rddata

#define btr_wrpeer             btru.btgwr.btgwr_wrpeer
#define btr_wrnbytes           btru.btgwr.btgwr_wrnbytes
#define btr_wrhandle           btru.btgwr.btgwr_wrhandle
#define btr_wrdata             btru.btgwr.btgwr_wrdata
#define btr_wrresult           btru.btgwr.btgwr_wrresult

#define btr_rmtpeer            btru.btcon.btcon_peer

/* btr_flags */

#define BTF_UP                 (1 << 0)  /* Unit is up */
#define BTF_RUNNING            (1 << 1)  /* Unit is running */
#define BTF_XMIT_CMD           (1 << 2)  /* Transmitting CMD packets */
#define BTF_XMIT_ACL           (1 << 3)  /* Transmitting ACL packets */
#define BTF_XMIT_SCO           (1 << 4)  /* Transmitting SCO packets */
#define BTF_INIT_BDADDR        (1 << 5)  /* Waiting for bdaddr */
#define BTF_INIT_BUFFER_SIZE   (1 << 6)  /* Waiting for buffer size */
#define BTF_INIT_FEATURES      (1 << 7)  /* Waiting for features */
#define BTF_NOOP_ON_RESET      (1 << 8)  /* Wait for No-op on reset */
#define BTF_INIT_COMMANDS      (1 << 9)  /* Waiting for supported commands */
#define BTF_MASTER             (1 << 10) /* Request Master role */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Type of GATT discovery command */

enum bt_gatt_discover_e
{
  GATT_DISCOVER = 0,                /* Discover */
  GATT_DISCOVER_DESC,               /* Discover descriptor */
  GATT_DISCOVER_CHAR,               /* Discover characteristic */
};

/* Write-able data that accompanies the SIOCBTSCANGET IOCTL command */

struct bt_scanresponse_s
{
  bt_addr_le_t sr_addr;             /* Advertiser LE address and type */
  int8_t sr_rssi;                   /* Strength of advertiser signal */
  uint8_t sr_type;                  /* Type of advertising response */
  uint8_t sr_len;                   /* Length of advertiser data */
  uint8_t sr_data[CONFIG_BLUETOOTH_MAXSCANDATA];
};

/* Write-able data that accompanies the SIOCBTDISCGET IOCTL command */

struct bt_discresonse_s
{
  uint16_t dr_handle;               /* Discovered handled */
  uint8_t dr_perm;                  /* Permissions */
};

/* Bluetooth statistics */

struct bt_stats_s
{
  uint32_t err_tx;
  uint32_t err_rx;
  uint32_t cmd_tx;
  uint32_t evt_rx;
  uint32_t acl_tx;
  uint32_t acl_rx;
  uint32_t sco_tx;
  uint32_t sco_rx;
  uint32_t byte_tx;
  uint32_t byte_rx;
};

/* Common structure for Bluetooth IOCTL commands */

struct btreq_s
  {
    char btr_name[IFNAMSIZ];         /* IN:  Device name */
    union
    {
      /* Bluetooth information used with informational query IOCTL commands */

      struct
      {
         bt_addr_t btri_bdaddr;      /* IN/OUT: Device bdaddr */
         uint16_t btri_flags;        /* OUT: flags */
         uint16_t btri_num_cmd;      /* OUT: Number of free cmd buffers */
         uint16_t btri_num_acl;      /* OUT: Number of free ACL buffers */
         uint16_t btri_num_sco;      /* OUT: Number of free SCO buffers */
         uint16_t btri_acl_mtu;      /* OUT: ACL mtu */
         uint16_t btri_sco_mtu;      /* OUT: SCO mtu */
         uint16_t btri_link_policy;  /* OUT: Link Policy */
         uint16_t btri_packet_type;  /* OUT: Packet Type */
         uint16_t btri_max_acl;      /* OUT: max ACL buffers */
         uint16_t btri_max_sco;      /* OUT: max SCO buffers */
       } btri;

       /* Bluetooth Features */

       struct
       {
         uint8_t btrf_page0[HCI_FEATURES_SIZE]; /* OUT: Basic */
         uint8_t btrf_page1[HCI_FEATURES_SIZE]; /* OUT: Extended page 1 */
         uint8_t btrf_page2[HCI_FEATURES_SIZE]; /* OUT: Extended page 2 */
       } btrf;

      struct bt_stats_s btrs;        /* OUT: Unit statistics */

      /* Read-only data that accompanies the SIOCBTADVSTART IOCTL command.
       * Advertising types are defined in bt_hci.h.  NOTE that btras_ad and
       * btras_sd pointers to the beginning of a list of "Extended Inquire
       * Responses".   Each list is terminated with a dummy, NULL entry
       * identified with a length of zero.
       */

      struct
      {
        uint8_t btras_advtype;             /* IN:  Advertising type */
        FAR struct bt_eir_s *btras_advad;  /* IN:  Data for advertisement packets */
        FAR struct bt_eir_s *btras_advsd;  /* IN:  Data for scan response packets */
      } btras;

      /* NOTE: No additional data accompanies the SIOCBTADVSTOP */

      /* The read-only data that accompanies the SIOCBTSCANSTART IOCTL
       * command.
       */

      struct
      {
        bool btrss_dupenable;        /* IN:  True: enable duplicate filtering */
      } btrss;

      /* Write-able data that accompanies the SIOCBTSCANGET IOCTL command */

      struct
      {
        uint8_t brtsr_nrsp;          /* IN:   Max number of responses
                                      * OUT: Actual number of responses */

        /* Reference to a beginning of an array in user memory in which to
         * return the scan response data.  The size of the array is
         * btrsr_nrsp.
         */

        FAR struct bt_scanresponse_s *btrsr_rsp;
      } btrsr;

      /* NOTE: No additional data accompanies the SIOCBTSCANSTOP */

      /* Read-only data that accompanies the SIOCBTSECURITY IOCTL command */

      struct
      {
        bt_addr_le_t btrse_secaddr;        /* IN:  BLE address */
        enum bt_security_e btrse_seclevel; /* IN:  Security level */
      } btrse;

      /* Read-only data that accompanies SIOCBTEXCHANGE command */

      struct
      {
        bt_addr_le_t btmx_expeer;    /* IN:  Peer address for MTU exchange */
        uint8_t btmx_result;         /* OUT: The result of the operation */
      } btmx;

      /* Write-able data that accompanies SIOCBTDISCOVER command */

      struct
      {
        uint8_t btrds_dtype;         /* IN:  Discovery type (see enum
                                      *      bt_gatt_discover_e) */
        bt_addr_le_t btrds_dpeer;    /* IN:  Peer address */
        uint16_t btrds_duuid16;      /* IN:  Discover UUID type */
        uint16_t btrds_dstart;       /* IN:  Discover start handle */
        uint16_t btrds_dend;         /* IN:  Discover end handle */
        uint8_t btrds_gnrsp;         /* IN:  Max number of responses
                                      * OUT: Actual number of responses */
        FAR struct bt_discresonse_s *btrds_grsp;
        int btrds_indx;              /* IN:  Index of first entry */
      } btrds;

      /* Write-able data that accompanies the SIOCBTGATTRD command */

      struct
      {
        bt_addr_le_t btgrd_rdpeer;   /* IN:  Peer address */
        uint8_t btgrd_rdnhandles;    /* IN:  Number of handles in array */
        uint16_t btgrd_rdoffset;     /* IN:  Offset (Only for read single) */
        uint16_t btgrd_rdhandles[HCI_GATT_MAXHANDLES];
        uint8_t btgrd_rdresult;      /* OUT: Result of the read */
        uint8_t btgrd_rdsize;        /* IN:  Sizeof rddata[]
                                      * OUT: Number of valid bytes */
        FAR uint8_t *btgrd_rddata;   /* OUT: Values returned by read */
      } btgrd;

      /* Write-able data that accompanies the SIOCBTGATTWR command.
       * NOTE:  The write data provided by the caller is not buffered
       * and must persist until the completion of the write.
       */

      struct
      {
        bt_addr_le_t btgwr_wrpeer;   /* IN:  Peer address */
        uint8_t btgwr_wrnbytes;      /* IN:  Number of bytes to write */
        uint16_t btgwr_wrhandle;     /* IN:  GATT handle */
        FAR uint8_t btgwr_wrdata[HCI_GATTWR_DATA]; /* IN:  Data to be written */
        uint8_t btgwr_wrresult;      /* OUT: The result of the operation */
      } btgwr;

      /* Read-only data that accompanies the SIOCBTCONNECT and
       * SIOCBTDISCONNECT commands.
       */

      struct
      {
        bt_addr_le_t btcon_peer;     /* IN:  Peer address */
      } btcon;

   } btru;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_IOCTL_H */
