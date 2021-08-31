/****************************************************************************
 * wireless/bluetooth/bt_att.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_ATTR_H
#define __WIRELESS_BLUETOOTH_BT_ATTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_ATT_DEFAULT_LE_MTU              23
#define BT_ATT_MAX_LE_MTU                  517

/* Error codes for Error response PDU */

#define BT_ATT_ERR_INVALID_HANDLE          0x01
#define BT_ATT_ERR_READ_NOT_PERMITTED      0x02
#define BT_ATT_ERR_WRITE_NOT_PERMITTED     0x03
#define BT_ATT_ERR_INVALID_PDU             0x04
#define BT_ATT_ERR_AUTHENTICATION          0x05
#define BT_ATT_ERR_NOT_SUPPORTED           0x06
#define BT_ATT_ERR_INVALID_OFFSET          0x07
#define BT_ATT_ERR_AUTHORIZATION           0x08
#define BT_ATT_ERR_PREPARE_QUEUE_FULL      0x09
#define BT_ATT_ERR_ATTRIBUTE_NOT_FOUND     0x0a
#define BT_ATT_ERR_ATTRIBUTE_NOT_LONG      0x0b
#define BT_ATT_ERR_ENCRYPTION_KEY_SIZE     0x0c
#define BT_ATT_ERR_INVALID_ATTRIBUTE_LEN   0x0d
#define BT_ATT_ERR_UNLIKELY                0x0e
#define BT_ATT_ERR_INSUFFICIENT_ENCRYPTION 0x0f
#define BT_ATT_ERR_UNSUPPORTED_GROUP_TYPE  0x10
#define BT_ATT_ERR_INSUFFICIENT_RESOURCES  0x11

#define BT_ATT_OP_ERROR_RSP                0x01
#define BT_ATT_OP_MTU_REQ                  0x02
#define BT_ATT_OP_MTU_RSP                  0x03

/* Find Information Request */

#define BT_ATT_OP_FIND_INFO_REQ            0x04

/* Find Information Response */

#define BT_ATT_INFO_16                     0x01
#define BT_ATT_INFO_128                    0x02
#define BT_ATT_OP_FIND_INFO_RSP            0x05

/* Find By Type Value Request */

#define BT_ATT_OP_FIND_TYPE_REQ            0x06

/* Find By Type Value Response */

#define BT_ATT_OP_FIND_TYPE_RSP            0x07

/* Read By Type Request */

#define BT_ATT_OP_READ_TYPE_REQ            0x08

/* Read By Type Response */

#define BT_ATT_OP_READ_TYPE_RSP            0x09

/* Read Request */

#define BT_ATT_OP_READ_REQ                 0x0a

/* Read Response */

#define BT_ATT_OP_READ_RSP                 0x0b

/* Read Blob Request */

#define BT_ATT_OP_READ_BLOB_REQ            0x0c

/* Read Blob Response */

#define BT_ATT_OP_READ_BLOB_RSP            0x0d

/* Read Multiple Request */

#define BT_ATT_READ_MULT_MIN_LEN_REQ       0x04
#define BT_ATT_OP_READ_MULT_REQ            0x0e

/* Read Multiple Response */

#define BT_ATT_OP_READ_MULT_RSP            0x0f

/* Read by Group Type Request */

#define BT_ATT_OP_READ_GROUP_REQ           0x10

/* Read by Group Type Response */

#define BT_ATT_OP_READ_GROUP_RSP           0x11

/* Write Request */

#define BT_ATT_OP_WRITE_REQ                0x12

/* Write Response */

#define BT_ATT_OP_WRITE_RSP                0x13

/* Prepare Write Request */

#define BT_ATT_OP_PREPARE_WRITE_REQ        0x16

/* Prepare Write Respond */

#define BT_ATT_OP_PREPARE_WRITE_RSP        0x17

/* Execute Write Request */

#define BT_ATT_FLAG_CANCEL                 0x00
#define BT_ATT_FLAG_EXEC                   0x01
#define BT_ATT_OP_EXEC_WRITE_REQ           0x18

/* Execute Write Response */

#define BT_ATT_OP_EXEC_WRITE_RSP           0x19

/* Handle Value Notification */

#define BT_ATT_OP_NOTIFY                   0x1b

/* Handle Value Indication */

#define BT_ATT_OP_INDICATE                 0x1d

/* Handle Value Confirm */

#define BT_ATT_OP_CONFIRM                  0x1f

/* Write Command */

#define BT_ATT_OP_WRITE_CMD                0x52

/* Signed Write Command */

#define BT_ATT_OP_SIGNED_WRITE_CMD         0xd2

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct bt_att_hdr_s
{
  uint8_t code;
} end_packed_struct;

begin_packed_struct struct bt_att_error_rsp_s
{
  uint8_t request;
  uint16_t handle;
  uint8_t error;
} end_packed_struct;

begin_packed_struct struct bt_att_exchange_mtu_req_s
{
  uint16_t mtu;
} end_packed_struct;

begin_packed_struct struct bt_att_exchange_mtu_rsp_s
{
  uint16_t mtu;
} end_packed_struct;

/* Find Information Request */

begin_packed_struct struct bt_att_find_info_req_s
{
  uint16_t start_handle;
  uint16_t end_handle;
} end_packed_struct;

begin_packed_struct struct bt_att_info_16_s
{
  uint16_t handle;
  uint16_t uuid;
} end_packed_struct;

begin_packed_struct struct bt_att_info_128_s
{
  uint16_t handle;
  uint8_t uuid[16];
} end_packed_struct;

/* Find Information Response */

begin_packed_struct struct bt_att_find_info_rsp_s
{
  uint8_t format;
  uint8_t info[0];
} end_packed_struct;

/* Find By Type Value Request */

begin_packed_struct struct bt_att_find_type_req_s
{
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t type;
  uint8_t value[0];
} end_packed_struct;

begin_packed_struct struct bt_att_handle_group_s
{
  uint16_t start_handle;
  uint16_t end_handle;
} end_packed_struct;

/* Find By Type Value Response */

begin_packed_struct struct bt_att_find_type_rsp_s
{
  struct bt_att_handle_group_s list[0];
} end_packed_struct;

/* Read By Type Request */

begin_packed_struct struct bt_att_read_type_req_s
{
  uint16_t start_handle;
  uint16_t end_handle;
  uint8_t uuid[0];
} end_packed_struct;

begin_packed_struct struct bt_att_data_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

/* Read By Type Response */

begin_packed_struct struct bt_att_read_type_rsp_s
{
  uint8_t len;
  struct bt_att_data_s data[0];
} end_packed_struct;

/* Read Request */

begin_packed_struct struct bt_att_read_req_s
{
  uint16_t handle;
} end_packed_struct;

/* Read Response */

begin_packed_struct struct bt_att_read_rsp_s
{
  uint8_t value[0];
} end_packed_struct;

/* Read Blob Request */

begin_packed_struct struct bt_att_read_blob_req_s
{
  uint16_t handle;
  uint16_t offset;
} end_packed_struct;

/* Read Blob Response */

begin_packed_struct struct bt_att_read_blob_rsp_s
{
  uint8_t value[0];
} end_packed_struct;

/* Read Multiple Request */

begin_packed_struct struct bt_att_read_mult_req_s
{
  uint16_t handles[0];
} end_packed_struct;

/* Read Multiple Response */

begin_packed_struct struct bt_att_read_mult_rsp_s
{
  uint8_t value[0];
} end_packed_struct;

/* Read by Group Type Request */

struct bt_att_read_group_req_s
{
  uint16_t start_handle;
  uint16_t end_handle;
  uint8_t uuid[0];
} end_packed_struct;

begin_packed_struct struct bt_att_group_data_s
{
  uint16_t start_handle;
  uint16_t end_handle;
  uint8_t value[0];
} end_packed_struct;

/* Read by Group Type Response */

begin_packed_struct struct bt_att_read_group_rsp_s
{
  uint8_t len;
  struct bt_att_group_data_s data[0];
} end_packed_struct;

/* Write Request */

begin_packed_struct struct bt_att_write_req_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

/* Write Response */

/* Prepare Write Request */

begin_packed_struct struct bt_att_prepare_write_req_s
{
  uint16_t handle;
  uint16_t offset;
  uint8_t value[0];
} end_packed_struct;

/* Prepare Write Response */

begin_packed_struct struct bt_att_prepare_write_rsp_s
{
  uint16_t handle;
  uint16_t offset;
  uint8_t value[0];
} end_packed_struct;

/* Execute Write Request */

begin_packed_struct struct bt_att_exec_write_req_s
{
  uint8_t flags;
} end_packed_struct;

/* Execute Write Response */

/* Handle Value Notification */

begin_packed_struct struct bt_att_notify_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

/* Handle Value Indication */

begin_packed_struct struct bt_att_indicate_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

/* Handle Value Confirm */

begin_packed_struct struct bt_att_signature_s
{
  uint8_t value[12];
} end_packed_struct;

/* Write Command */

begin_packed_struct struct bt_att_write_cmd_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

/* Signed Write Command */

begin_packed_struct struct bt_att_signed_write_cmd_s
{
  uint16_t handle;
  uint8_t value[0];
} end_packed_struct;

typedef void (*bt_att_func_t)(FAR struct bt_conn_s *conn, uint8_t err,
                               FAR const void *pdu, uint16_t length,
                               FAR void *user_data);
typedef void (*bt_att_destroy_t)(FAR void *user_data);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void bt_att_initialize(void);
struct bt_buf_s *bt_att_create_pdu(FAR struct bt_conn_s *conn, uint8_t op,
                                 size_t len);

/* Send ATT PDU over a connection */

int bt_att_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                bt_att_func_t func, FAR void *user_data,
                bt_att_destroy_t destroy);

/* Cancel ATT request */

void bt_att_cancel(FAR struct bt_conn_s *conn);

#endif /* __WIRELESS_BLUETOOTH_BT_ATTR_H */
