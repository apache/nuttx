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
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
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

#ifndef __WIRELESS_BLUETOOTH_BT_CONN_H
#define __WIRELESS_BLUETOOTH_BT_CONN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <mqueue.h>

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

  /* Temporary data used by ioctl */

  void *p_iostate;
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_conn_receive
 *
 * Description:
 *   Receive packets from the HCI core on a registered connection.
 *
 * Input Parameters:
 *   conn  - The registered connection
 *   buf   - The buffer structure containing the received packet
 *   flags - Packet boundary flags
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_receive(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                     uint8_t flags);

/****************************************************************************
 * Name: bt_conn_send
 *
 * Description:
 *   Send data over a connection
 *
 * Input Parameters:
 *   conn  - The registered connection
 *   buf   - The buffer structure containing the packet to be sent
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf);

/****************************************************************************
 * Name: bt_conn_add
 *
 * Description:
 *   Add a new connection
 *
 * Input Parameters:
 *   peer - The address of the Bluetooth peer
 *   role - Either BT_HCI_ROLE_MASTER or BT_HCI_ROLE_SLAVE
 *
 * Returned Value:
 *   A reference to the new connection structure is returned on success.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_add(FAR const bt_addr_le_t *peer,
                                  uint8_t role);

/****************************************************************************
 * Name: bt_conn_set_state
 *
 * Description:
 *   Set connection object in certain state and perform actions related to
 *   state change.
 *
 * Input Parameters:
 *   conn - The connection whose state will be changed.
 *   state - The new state of the connection.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_set_state(FAR struct bt_conn_s *conn,
                       enum bt_conn_state_e state);

/****************************************************************************
 * Name: bt_conn_lookup_handle
 *
 * Description:
 *   Look up an existing connection
 *
 * Input Parameters:
 *   handle - The handle to be used to perform the lookup
 *
 * Returned Value:
 *   A reference to the connection state instance is returned on success.
 *   NULL is returned if the connection is not found.  On success, the
 *   caller gets a new reference to the connection object which must be
 *   released with bt_conn_release() once done using the connection.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_lookup_handle(uint16_t handle);

/****************************************************************************
 * Name: bt_conn_lookup_addr_le
 *
 * Description:
 *   Look up an existing connection based on the remote address.
 *
 * Input Parameters:
 *   peer - Remote address.
 *
 * Returned Value:
 *   A reference to the connection state instance is returned on success.
 *   NULL is returned if the connection is not found.  On success, the
 *   caller gets a new reference to the connection object which must be
 *   released with bt_conn_release() once done using the connection.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_lookup_addr_le(const bt_addr_le_t *peer);

/****************************************************************************
 * Name: bt_conn_lookup_state
 *
 * Description:
 *   Look up a connection state.  For BT_ADDR_LE_ANY, returns the first
 *   connection with the specific state
 *
 * Input Parameters:
 *   peer  - The peer address to match
 *   state - The connection state to match
 *
 * Returned Value:
 *   A reference to the connection state instance is returned on success.
 *   NULL is returned if the connection is not found.  On success, the
 *   caller gets a new reference to the connection object which must be
 *   released with bt_conn_release() once done using the connection.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_lookup_state(FAR const bt_addr_le_t *peer,
                                           enum bt_conn_state_e state);

/****************************************************************************
 * Name: bt_conn_addref
 *
 * Description:
 *   Increment the reference count of a connection object.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 * Returned Value:
 *   Connection object with incremented reference count.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_addref(FAR struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_conn_release
 *
 * Description:
 *   Decrement the reference count of a connection object.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_release(FAR struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_conn_get_dst
 *
 * Description:
 *   Get destination (peer) address of a connection.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 * Returned Value:
 *   Destination address.
 *
 ****************************************************************************/

FAR const bt_addr_le_t *bt_conn_get_dst(FAR const struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_conn_security
 *
 * Description:
 *   This function enable security (encryption) for a connection.
 *   If device is already paired with sufficiently strong key encryption will
 *   be enabled. If link is already encrypted with sufficiently strong key
 *   this function does nothing.
 *
 *   If device is not paired pairing will be initiated.
 *   If device is paired and keys are too weak but input output capabilities
 *   allow for strong enough keys pairing will be initiated.
 *
 *   This function may return error if required level of security is not
 *   possible to achieve due to local or remote device limitation (eg input
 *   output capabilities).
 *
 * Input Parameters:
 *   conn - Connection object.
 *   sec  - Requested security level.
 *
 * Returned Value:
 *   0 on success or negative error
 *
 ****************************************************************************/

int bt_conn_security(FAR struct bt_conn_s *conn, enum bt_security_e sec);

/****************************************************************************
 * Name:bt_conn_set_auto_conn
 *
 * Description:
 *   This function enables/disables automatic connection initiation.
 *   Every time the device looses the connection with peer, this connection
 *   will be re-established if connectible advertisement from peer is
 *   received.
 *
 * Input Parameters:
 *   conn      - Existing connection object.
 *   auto_conn - boolean value. If true, auto connect is enabled, if false,
 *               auto connect is disabled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_set_auto_conn(FAR struct bt_conn_s *conn, bool auto_conn);

/****************************************************************************
 * Name: bt_conn_disconnect
 *
 * Description:
 *   Disconnect an active connection with the specified reason code or cancel
 *   pending outgoing connection.
 *
 * Input Parameters:
 *   conn   - Connection to disconnect.
 *   reason - Reason code for the disconnection.
 *
 * Returned Value:
 *   Zero on success or (negative) error code on failure.
 *
 ****************************************************************************/

int bt_conn_disconnect(FAR struct bt_conn_s *conn, uint8_t reason);

/****************************************************************************
 * Name: bt_conn_create_le
 *
 * Description:
 *  Allows initiate new LE link to remote peer using its address.
 *  Returns a new reference that the the caller is responsible for managing.
 *
 * Input Parameters:
 *   peer - Remote address.
 *
 * Returned Value:
 *   Valid connection object on success or NULL otherwise.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_create_le(FAR const bt_addr_le_t *peer);

/****************************************************************************
 * Name: bt_conn_le_start_encryption
 *
 * Description:
 *   See the HCI start encryption command.
 *
 *   NOTE: rand and ediv should be in BT order.
 *
 * Input Parameters:
 *   conn       - The connection to send the command on.
 *   rand, ediv - Values to use for the encryption key
 *   ltk        -
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bt_conn_le_start_encryption(FAR struct bt_conn_s *conn, uint64_t rand,
                                uint16_t ediv, FAR const uint8_t *ltk);

/****************************************************************************
 * Name: bt_conn_le_conn_update
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bt_conn_le_conn_update(FAR struct bt_conn_s *conn, uint16_t min,
                           uint16_t max, uint16_t latency,
                           uint16_t timeout);

#endif /* __WIRELESS_BLUETOOTH_BT_CONN_H */
