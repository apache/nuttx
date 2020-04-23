/****************************************************************************
 * wireless/bluetooth/bt_smp.h
 * Security Manager Protocol implementation.
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
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS
 * ; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLUETOOTH_BT_SMP_H
#define __WIRELESS_BLUETOOTH_BT_SMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "bt_conn.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_SMP_ERR_PASSKEY_ENTRY_FAILED      0x01
#define BT_SMP_ERR_OOB_NOT_AVAIL             0x02
#define BT_SMP_ERR_AUTH_REQUIREMENTS         0x03
#define BT_SMP_ERR_CONFIRM_FAILED            0x04
#define BT_SMP_ERR_PAIRING_NOTSUPP           0x05
#define BT_SMP_ERR_ENC_KEY_SIZE              0x06
#define BT_SMP_ERR_CMD_NOTSUPP               0x07
#define BT_SMP_ERR_UNSPECIFIED               0x08
#define BT_SMP_ERR_REPEATED_ATTEMPTS         0x09
#define BT_SMP_ERR_INVALID_PARAMS            0x0a
#define BT_SMP_ERR_DHKEY_CHECK_FAILED        0x0b
#define BT_SMP_ERR_NUMERIC_COMP_FAILED       0x0c
#define BT_SMP_ERR_BREDR_PAIRING_IN_PROGRESS 0x0d
#define BT_SMP_ERR_CROSS_TRANSP_NOT_ALLOWED  0x0e

#define BT_SMP_IO_DISPLAY_ONLY               0x00
#define BT_SMP_IO_DISPLAY_YESNO              0x01
#define BT_SMP_IO_KEYBOARD_ONLY              0x02
#define BT_SMP_IO_NO_INPUT_OUTPUT            0x03
#define BT_SMP_IO_KEYBOARD_DISPLAY           0x04

#define BT_SMP_OOB_NOT_PRESENT               0x00
#define BT_SMP_OOB_PRESENT                   0x01

#define BT_SMP_MIN_ENC_KEY_SIZE              16
#define BT_SMP_MAX_ENC_KEY_SIZE              16

#define BT_SMP_DIST_ENC_KEY                  0x01
#define BT_SMP_DIST_ID_KEY                   0x02
#define BT_SMP_DIST_SIGN                     0x04
#define BT_SMP_DIST_LINK_KEY                 0x08

#define BT_SMP_DIST_MASK                     0x0f

#define BT_SMP_AUTH_NONE                     0x00
#define BT_SMP_AUTH_BONDING                  0x01
#define BT_SMP_AUTH_MITM                     0x04
#define BT_SMP_AUTH_SC                       0x08
#define BT_SMP_AUTH_KEYPRESS                 0x10

#define BT_SMP_AUTH_MASK                     0x1f

#define BT_SMP_CMD_PAIRING_REQ               0x01
#define BT_SMP_CMD_PAIRING_RSP               0x02
#define BT_SMP_CMD_PAIRING_CONFIRM           0x03
#define BT_SMP_CMD_PAIRING_RANDOM            0x04
#define BT_SMP_CMD_PAIRING_FAIL              0x05
#define BT_SMP_CMD_ENCRYPT_INFO              0x06
#define BT_SMP_CMD_MASTER_IDENT              0x07
#define BT_SMP_CMD_IDENT_INFO                0x08
#define BT_SMP_CMD_IDENT_ADDR_INFO           0x09
#define BT_SMP_CMD_SECURITY_REQUEST          0x0b

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct bt_smp_hdr_s
{
  uint8_t code;
} end_packed_struct;

begin_packed_struct struct bt_smp_pairing_s
{
  uint8_t io_capability;
  uint8_t oob_flag;
  uint8_t auth_req;
  uint8_t max_key_size;
  uint8_t init_key_dist;
  uint8_t resp_key_dist;
} end_packed_struct;

begin_packed_struct struct bt_smp_pairing_confirm_s
{
  uint8_t val[16];
} end_packed_struct;

begin_packed_struct struct bt_smp_pairing_random_s
{
  uint8_t val[16];
} end_packed_struct;

begin_packed_struct struct bt_smp_pairing_fail_s
{
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct bt_smp_encrypt_info_s
{
  uint8_t ltk[16];
} end_packed_struct;

begin_packed_struct struct bt_smp_master_ident_s
{
  uint16_t ediv;
  uint64_t rand;
} end_packed_struct;

begin_packed_struct struct bt_smp_ident_info_s
{
  uint8_t irk[16];
} end_packed_struct;

begin_packed_struct struct bt_smp_ident_addr_info_s
{
  bt_addr_le_t addr;
} end_packed_struct;

begin_packed_struct struct bt_smp_security_request_s
{
  uint8_t auth_req;
} end_packed_struct;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool bt_smp_irk_matches(FAR const uint8_t irk[16],
                        FAR const bt_addr_t *addr);
int  bt_smp_send_pairing_req(FAR struct bt_conn_s *conn);
int  bt_smp_send_security_req(FAR struct bt_conn_s *conn);
int  bt_smp_initialize(void);

#endif /* __WIRELESS_BLUETOOTH_BT_SMP_H */
