/****************************************************************************
 * wireless/bluetooth/bt_l2cap.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_L2CAP_H
#define __WIRELESS_BLUETOOTH_BT_L2CAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_L2CAP_CID_ATT             0x0004
#define BT_L2CAP_CID_LE_SIG          0x0005
#define BT_L2CAP_CID_SMP             0x0006

#define BT_L2CAP_REJ_NOT_UNDERSTOOD  0x0000
#define BT_L2CAP_REJ_MTU_EXCEEDED    0x0001
#define BT_L2CAP_REJ_INVALID_CID     0x0002

#define BT_L2CAP_CMD_REJECT          0x01
#define BT_L2CAP_CONN_PARAM_REQ      0x12
#define BT_L2CAP_CONN_PARAM_RSP      0x13

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct bt_l2cap_hdr_s
{
  uint16_t len;
  uint16_t cid;
} end_packed_struct;

begin_packed_struct struct bt_l2cap_sig_hdr_s
{
  uint8_t code;
  uint8_t ident;
  uint16_t len;
} end_packed_struct;

begin_packed_struct struct bt_l2cap_cmd_reject_s
{
  uint16_t reason;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct bt_l2cap_conn_param_req_s
{
  uint16_t min_interval;
  uint16_t max_interval;
  uint16_t latency;
  uint16_t timeout;
} end_packed_struct;

begin_packed_struct struct bt_l2cap_conn_param_rsp_s
{
  uint16_t result;
} end_packed_struct;

struct bt_l2cap_chan_s
{
  FAR struct bt_l2cap_chan_s *flink;
  FAR void *context;
  uint16_t cid;

  CODE void (*connected)(FAR struct bt_conn_s *conn,
              FAR void *context, uint16_t cid);
  CODE void (*disconnected)(FAR struct bt_conn_s *conn, FAR void *context,
              uint16_t cid);
  CODE void (*encrypt_change)(FAR struct bt_conn_s *conn,
              FAR void *context, uint16_t cid);
  CODE void (*receive)(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
              FAR void *context, uint16_t cid);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Register a fixed L2CAP channel handler for L2CAP */

void bt_l2cap_chan_register(FAR struct bt_l2cap_chan_s *chan);

/* Register a default L2CAP channel handle for L2CAP */

void bt_l2cap_chan_default(FAR struct bt_l2cap_chan_s *chan);

/* Notify L2CAP channels of a new connection */

void bt_l2cap_connected(FAR struct bt_conn_s *conn);

/* Notify L2CAP channels of a disconnect event */

void bt_l2cap_disconnected(FAR struct bt_conn_s *conn);

/* Notify L2CAP channels of a change in encryption state */

void bt_l2cap_encrypt_change(FAR struct bt_conn_s *conn);

/* Prepare an L2CAP PDU to be sent over a connection */

FAR struct bt_buf_s *bt_l2cap_create_pdu(FAR struct bt_conn_s *conn);

/* Send L2CAP PDU over a connection */

void bt_l2cap_send(FAR struct bt_conn_s *conn, uint16_t cid,
                   FAR struct bt_buf_s *buf);

/* Receive a new L2CAP PDU from a connection */

void bt_l2cap_receive(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf);

/* Perform connection parameter update request */

void bt_l2cap_update_conn_param(FAR struct bt_conn_s *conn);

/* Initialize L2CAP and supported channels */

int bt_l2cap_init(void);

#endif /* __WIRELESS_BLUETOOTH_BT_L2CAP_H */
