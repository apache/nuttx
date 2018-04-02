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

#ifndef __INCLUDE_NUTTX_WIRELESS_BT_IOCTL_H
#define __INCLUDE_NUTTX_WIRELESS_BT_IOCTL_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_DEVNAME_SIZE  32   /* Maximum size of node name */
#define HCI_FEATURES_SIZE  8   /* LMP features */

/* Bluetooth network device IOCTL commands. */

#ifndef WL_BLUETOOTHCMDS != 15
#  error Incorrect setting for number of Bluetooth IOCTL commands
#endif

/* NetBSD IOCTL commands ****************************************************/
/* All of the following use an argument of type struct btreg_s:
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
 * SIOCSBTFLAGS
 *   Set Bluetooth device Flags.  Not all flags are settable.
 * SIOCGBTFEAT
 *   Get Bluetooth device Features.  This returns the cached basic (page 0)
 *   and extended (page 1 & 2) features.
 * SIOCSBTPOLICY
 *   Set Bluetooth device Link Policy bits.
 * SIOCSBTPTYPE
 *   Set Bluetooth device Packet Types.  You can only set packet types that
 *   the device supports.
 * SIOCGBTSTATS
 *   Read device statistics.
 * SIOCZBTSTATS
 *   Read device statistics, and zero them.
 *
 * NOTE: These are here for reference.  None of the NetBSD IOCTL commands
 * have been implemented in NuttX.
 */

#define SIOCGBTINFO            _WLIOC(WL_BLUETOOTHFIRST + 0)
#define SIOCGBTINFOA           _WLIOC(WL_BLUETOOTHFIRST + 1)
#define SIOCNBTINFO            _WLIOC(WL_BLUETOOTHFIRST + 2)
#define SIOCSBTFLAGS           _WLIOC(WL_BLUETOOTHFIRST + 3)
#define SIOCGBTFEAT            _WLIOC(WL_BLUETOOTHFIRST + 4)
#define SIOCSBTPOLICY          _WLIOC(WL_BLUETOOTHFIRST + 5)
#define SIOCSBTPTYPE           _WLIOC(WL_BLUETOOTHFIRST + 6)
#define SIOCGBTSTATS           _WLIOC(WL_BLUETOOTHFIRST + 7)
#define SIOCZBTSTATS           _WLIOC(WL_BLUETOOTHFIRST + 8)

/* NuttX-specific IOCTL commands. *******************************************/
/* SIOCBT_ADVERTISESTART
 *   Description:   Set advertisement data, scan response data,
 *                  advertisement parameters and start advertising.
 *   Input:         Pointer to read-write instance of struct
 *                  bt_advertisestart_s.
 *   Output:        None
 */

#define SIOCBT_ADVERTISESTART  _WLIOC(WL_BLUETOOTHFIRST + 9)

/* SIOCBT_ADVERTISESTOP
 *   Description:   Stop advertising.
 *   Input:         A reference to a write-able instance of struct
 *                  bt_scanstop_s.
 *   Output:        None
 */

#define SIOCBT_ADVERTISESTOP   _WLIOC(WL_BLUETOOTHFIRST + 10)

/* SIOCBT_SCANSTART
 *   Description:   Start LE scanning.  Buffered scan results may be
 *                  obtained via SIOCBT_SCANGET
 *   Input:         A read-only referent to struct bt_scanstart_s.
 *   Output:        None
 */

#define SIOCBT_SCANSTART       _WLIOC(WL_BLUETOOTHFIRST + 11)

/* SIOCBT_SCANGET
 *   Description:   Return scan results buffered since the call time that
 *                  the SIOCBT_SCANGET command was invoked.
 *   Input:         A reference to a write-able instance of struct
 *                  bt_scanresult_s.
 *   Output:        Buffered scan result results are returned in the user-
 *                  provided buffer space.
 */

#define SIOCBT_SCANGET         _WLIOC(WL_BLUETOOTHFIRST + 12)

/* SIOCBT_SCANSTOP
 *   Description:   Stop LE scanning and discard any buffered results.
 *   Input:         A reference to a write-able instance of struct
 *                  bt_scanstop_s.
 *   Output:        None
 */

#define SIOCBT_SCANSTOP        _WLIOC(WL_BLUETOOTHFIRST + 13)

/* SIOCBT_SECURITY
 *   Description:   Enable security for a connection.
 *   Input:         A reference to a write-able instance of struct
 *                  bt_security_s.
 *   Output:        None
 */

#define SIOCBT_SECURITY        _WLIOC(WL_BLUETOOTHFIRST + 14)

/* Definitions associated with struct btreg_s *******************************/
/* struct btreq_s union field accessors */

#define btr_flags              btru.btri.btri_flags
#define btr_bdaddr             btru.btri.btri_bdaddr
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

/* Common structure for Bluetooth IOCTL commands */

struct bt_stats
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

struct btreq_s
  {
    char btr_name[HCI_DEVNAME_SIZE]; /* Device name */
    union
    {
      struct
      {
         bt_addr_t btri_bdaddr;      /* Device bdaddr */
         uint16_t btri_flags;        /* flags */
         uint16_t btri_num_cmd;      /* # of free cmd buffers */
         uint16_t btri_num_acl;      /* # of free ACL buffers */
         uint16_t btri_num_sco;      /* # of free SCO buffers */
         uint16_t btri_acl_mtu;      /* ACL mtu */
         uint16_t btri_sco_mtu;      /* SCO mtu */
         uint16_t btri_link_policy;  /* Link Policy */
         uint16_t btri_packet_type;  /* Packet Type */
         uint16_t btri_max_acl;      /* max ACL buffers */
         uint16_t btri_max_sco;      /* max SCO buffers */
       } btri;
       struct
       {
         uint8_t btrf_page0[HCI_FEATURES_SIZE]; /* basic */
         uint8_t btrf_page1[HCI_FEATURES_SIZE]; /* extended page 1 */
         uint8_t btrf_page2[HCI_FEATURES_SIZE]; /* extended page 2 */
       } btrf;
       struct bt_stats btrs;   /* unit stats */
   } btru;
};

/* Read-only data that accompanies the SIOCBT_ADVERTISESTART IOCTL command.
 * Advertising types are defined in bt_hci.h.
 */

struct bt_advertisestart_s
{
  char as_name[HCI_DEVNAME_SIZE];  /* Device name */
  uint8_t as_type;                 /* Advertising type */
  FAR struct bt_eir_s as_ad;       /* Data for advertisement packets */
  FAR struct bt_eir_s as_sd;       /* Data for scan response packets */
};

/* The read-only data that accompanies the SIOCBT_SCANSTOP IOCTL command */

struct bt_advertisestop_s
{
  char at_name[HCI_DEVNAME_SIZE];   /* Device name */
};

/* The read-only data that accompanies the SIOCBT_SCANSTART IOCTL command */

struct bt_scanstart_s
{
  char ss_name[HCI_DEVNAME_SIZE];   /* Device name */
  bool ss_dupenable;                /* True: enable duplicate filtering */
};

/* The read-only data that accompanies the SIOCBT_SCANSTOP IOCTL command */

struct bt_scanstop_s
{
  char st_name[HCI_DEVNAME_SIZE];   /* Device name */
};

/* Write-able data that accompanies the SIOCBT_SCANGET IOCTL command */

struct bt_scanresponse_s
{
  char sr_name[HCI_DEVNAME_SIZE];   /* Device name */
  bt_addr_le_t sr_addr;             /* Advertiser LE address and type */
  int8_t sr_rssi;                   /* Strength of advertiser signal */
  uint8_t sr_type;                  /* Type of advertising response */
  uint8_t sr_len;                   /* Length of advertiser data */
  uint8_t sr_data[CONFIG_BLUETOOTH_MAXSCANDATA];
};

struct bt_scanresult_s
{
  char sr_name[HCI_DEVNAME_SIZE];   /* Device name */
  uint8_t sr_nrsp;                  /* Input:  Max number of responses
                                     * Return: Actual number of responses */
  struct bt_scanresponse_s sr_rsp[1];
};

#define SIZEOF_BT_SCANRESULT_S(n) \
  (sizeof(struct bt_scanresult_s) + \
   ((n) - 1) * sizeof(struct bt_scanresponse_s))

/* Read-only data that accompanies the SIOCBT_SECURITY IOCTL command */

struct bt_security_s
{
  char se_name[HCI_DEVNAME_SIZE];   /* Device name */
  bt_addr_le_t se_addr;             /* BLE address */
  enum bt_security_e se_level;      /* Security level */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_WIRELESS_BT_IOCTL_H */
