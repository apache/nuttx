/****************************************************************************
 * wireless/bluetooth/bt_conn.h
 * Bluetooth connection handling.
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
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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

#ifndef __WIRELESS_BLUETOOTH_BT_CONN_H
#define __WIRELESS_BLUETOOTH_BT_CONN_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mqueue.h>

#include <nuttx/wireless/bt_conn.h>

#include "bt_atomic.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum bt_conn_state_e
{
  BT_CONN_DISCONNECTED,
  BT_CONN_CONNECT_SCAN,
  BT_CONN_CONNECT,
  BT_CONN_CONNECTED,
  BT_CONN_DISCONNECT,
};

/* L2CAP signaling channel specific context */

struct bt_conn_l2cap_s
{
  uint8_t ident;
};

/* bt_conn_s flags: the flags defined here represent connection parameters */

enum bt_conn_flags_e
{
  BT_CONN_AUTO_CONNECT,
};

struct bt_conn_s
{
  uint16_t handle;
  uint8_t role;
  bt_atomic_t flags[1];

  bt_addr_le_t src;
  bt_addr_le_t dst;

  uint8_t encrypt;

  uint16_t rx_len;
  FAR struct bt_buf_s *rx;

  /* Queue for outgoing ACL data */

  mqd_t tx_queue;

  FAR struct bt_keys_s *keys;

  /* Fixed channel contexts */

  struct bt_conn_l2cap_s l2cap;
  FAR void *att;
  FAR void *smp;

  uint8_t le_conn_interval;
  bt_atomic_t ref;
  enum bt_conn_state_e state;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Process incoming data for a connection */

void bt_conn_recv(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                  uint8_t flags);

/* Send data over a connection */

void bt_conn_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf);

/* Add a new connection */

FAR struct bt_conn_s *bt_conn_add(FAR const bt_addr_le_t *peer, uint8_t role);

/* Look up an existing connection */

FAR struct bt_conn_s *bt_conn_lookup_handle(uint16_t handle);

/* Look up a connection state. For BT_ADDR_LE_ANY, returns the first connection
 * with the specific state
 */

FAR struct bt_conn_s *bt_conn_lookup_state(FAR const bt_addr_le_t * peer,
                                           enum bt_conn_state_e state);

/* Set connection object in certain state and perform action related to state */

void bt_conn_set_state(FAR struct bt_conn_s *conn, enum bt_conn_state_e state);

/* rand and ediv should be in BT order */

int bt_conn_le_start_encryption(FAR struct bt_conn_s *conn, uint64_t rand,
                                uint16_t ediv, FAR const uint8_t *ltk);

int bt_conn_le_conn_update(FAR struct bt_conn_s *conn, uint16_t min,
                           uint16_t max, uint16_t latency,
                           uint16_t timeout);

#endif /* __WIRELESS_BLUETOOTH_BT_CONN_H */
