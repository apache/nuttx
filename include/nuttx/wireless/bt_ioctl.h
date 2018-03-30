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

/* Bluetooth network device IOCTL commands. */

#ifndef WL_BLUETOOTHCMDS != 5
#  error Incorrect setting for number of Bluetooth IOCTL commands
#endif

/* SIOCBT_ADVERTISESTART
 *   Description:   Set advertisement data, scan response data,
 *                  advertisement parameters and start advertising.
 *   Input:         Pointer to read-write instance of struct
 *                  bt_advertisestart_s.
 *   Output:        None
 */

#define SIOCBT_ADVERTISESTART  _WLIOC(WL_BLUETOOTHFIRST + 0)

/* SIOCBT_ADVERTISESTOP
 *   Description:   Stop advertising.
 *   Input:         None
 *   Output:        None
 */

#define SIOCBT_ADVERTISESTOP   _WLIOC(WL_BLUETOOTHFIRST + 1)

/* SIOCBT_SCANSTART
 *   Description:   Start LE scanning.  Buffered scan results may be
 *                  obtained via SIOCBT_SCANGET
 *   Input:         1=Duplicate filtering enabled
 *   Output:        None
 */

#define SIOCBT_SCANSTART       _WLIOC(WL_BLUETOOTHFIRST + 2)

/* SIOCBT_SCANGET
 *   Description:   Return scan results buffered since the call time that
 *                  the SIOCBT_SCANGET command was invoked.
 *   Input:         A reference to a write-able instance of struct
 *                  bt_scanresult_s.
 *   Output:        Buffered scan result results are returned in the user-
 *                  provided buffer space.
 */

#define SIOCBT_SCANGET         _WLIOC(WL_BLUETOOTHFIRST + 3)

/* SIOCBT_SCANSTOP
 *   Description:   Stop LE scanning and discard any buffered results.
 *   Input:         None
 *   Output:        None
 */

#define SIOCBT_SCANSTOP        _WLIOC(WL_BLUETOOTHFIRST + 4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Read-only data that accompanies the SIOCBT_ADVERTISESTART IOCTL command.
 * Advertising types are defined in bt_hci.h.
 */

struct bt_advertisestart_s
{
  uint8_t as_type;                 /* Advertising type */
  FAR const struct bt_eir_s as_ad; /* Data for advertisement packets */
  FAR const struct bt_eir_s as_sd; /* Data for scan response packets */
};

/* Write-able data that accompanies the SIOCBT_SCANGET IOCTL command */

struct bt_scanresponse_s
{
 bt_addr_le_t sr_addr;             /* Advertiser LE address and type */
 int8_t sr_rssi;                   /* Strength of advertiser signal */
 uint8_t sr_type;                  /* Type of advertising response */
 uint8_t sr_len;                   /* Length of advertiser data */
 uint8_t sr_data[CONFIG_BLUETOOTH_MAXSCANDATA];
};

struct bt_scanresult_s
{
  uint8_t sc_nrsp;                 /* Input:  Max number of responses
                                    * Return: Actual number of responses */
  struct bt_scanresponse_s sc_rsp[1];
};

#define SIZEOF_BT_SCANRESULT_S(n) \
  (sizeof(struct bt_scanresult_s) + \
   ((n) - 1) * sizeof(struct bt_scanresponse_s))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_WIRELESS_BT_IOCTL_H */
