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

#ifndef __INCLUDE_NUTTX_WIRELESS_BT_CONN_H
#define __INCLUDE_NUTTX_WIRELESS_BT_CONN_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/wireless/bt_core.h>
#include <nuttx/wireless/bt_hci.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Connection callback structure */

struct bt_conn_s; /* Forward Reference */
struct bt_conn_cb_s
{
  CODE void (*connected)(FAR struct bt_conn_s *conn);
  CODE void (*disconnected)(FAR struct bt_conn_s *conn);
  FAR struct bt_conn_cb_s *next;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: bt_conn_get
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

FAR struct bt_conn_s *bt_conn_get(FAR struct bt_conn_s *conn);

/****************************************************************************
 * Name: bt_conn_put
 *
 * Description:
 *   Decrement the reference count of a connection object.
 *
 * Input Parameters:
 *   conn - Connection object.
 *
 ****************************************************************************/

void bt_conn_put(FAR struct bt_conn_s *conn);

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
 *   Connection object or NULL if not found. The caller gets a new reference
 *   to the connection object which must be released with bt_conn_put() once
 *   done using the object.
 *
 ****************************************************************************/

FAR struct bt_conn_s *bt_conn_lookup_addr_le(const bt_addr_le_t *peer);

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

FAR struct bt_conn_s *bt_conn_create_le(const bt_addr_le_t *peer);

/****************************************************************************
 * Name: bt_conn_security
 *
 * Description:
 *   This function enable security (encryption) for a connection. If device is
 *   already paired with sufficiently strong key encryption will be enabled. If
 *   link is already encrypted with sufficiently strong key this function does
 *   nothing.
 *
 *   If device is not paired pairing will be initiated. If device is paired and
 *   keys are too weak but input output capabilities allow for strong enough keys
 *   pairing will be initiated.
 *
 *   This function may return error if required level of security is not possible
 *   to achieve due to local or remote device limitation (eg input output
 *   capabilities).
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
 * Name: bt_conn_cb_register
 *
 * Description:
 *   Register callbacks to monitor the state of connections.
 *
 * Input Parameters:
 *   cb - Callback struct.
 *
 ****************************************************************************/

void bt_conn_cb_register(struct bt_conn_cb_s *cb);

/****************************************************************************
 * Name:
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

#endif /* __INCLUDE_NUTTX_WIRELESS_BT_CONN_H */
