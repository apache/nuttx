/****************************************************************************
 * wireless/bluetooth/bt_conn.c
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/mm/iob.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>

#include "bt_atomic.h"
#include "bt_queue.h"
#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_l2cap.h"
#include "bt_keys.h"
#include "bt_smp.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bt_conn_handoff_s
{
  sem_t sync_sem;
  FAR struct bt_conn_s *conn;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_conn_s g_conns[CONFIG_BLUETOOTH_MAX_CONN];
static struct bt_conn_handoff_s g_conn_handoff =
{
  SEM_INITIALIZER(1),
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_WIRELESS_INFO
static const char *state2str(enum bt_conn_state_e state)
{
  switch (state)
    {
    case BT_CONN_DISCONNECTED:
      return "disconnected";

    case BT_CONN_CONNECT_SCAN:
      return "connect-scan";

    case BT_CONN_CONNECT:
      return "connect";

    case BT_CONN_CONNECTED:
      return "connected";

    case BT_CONN_DISCONNECT:
      return "disconnect";

    default:
      return "(unknown)";
    }
}
#endif

static void bt_conn_reset_rx_state(FAR struct bt_conn_s *conn)
{
  if (!conn->rx_len)
    {
      return;
    }

  bt_buf_release(conn->rx);
  conn->rx     = NULL;
  conn->rx_len = 0;
}

static int conn_tx_kthread(int argc, FAR char *argv[])
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_buf_s *buf;
  struct mq_attr attr;
  int ret;

  /* Get the connection instance */

  conn = g_conn_handoff.conn;
  DEBUGASSERT(conn != NULL);
  nxsem_post(&g_conn_handoff.sync_sem);

  wlinfo("Started for handle %u\n", conn->handle);

  while (conn->state == BT_CONN_CONNECTED)
    {
      /* Wait until the controller can accept ACL packets */

      wlinfo("calling nxsem_wait_uninterruptible()\n");

      ret = nxsem_wait_uninterruptible(&g_btdev.le_pkts_sem);
      if (ret < 0)
        {
          wlerr("nxsem_wait_uninterruptible() failed: %d\n", ret);
          break;
        }

      /* Check for disconnection */

      if (conn->state != BT_CONN_CONNECTED)
        {
          nxsem_post(&g_btdev.le_pkts_sem);
          break;
        }

      /* Get next ACL packet for connection */

      ret = bt_queue_receive(conn->tx_queue, &buf);
      DEBUGASSERT(ret >= 0 && buf != NULL);
      UNUSED(ret);

      if (conn->state != BT_CONN_CONNECTED)
        {
          nxsem_post(&g_btdev.le_pkts_sem);
          bt_buf_release(buf);
          break;
        }

      wlinfo("passing buf %p len %u to driver\n", buf, buf->len);
      g_btdev.btdev->send(g_btdev.btdev, buf);
      bt_buf_release(buf);
    }

  wlinfo("handle %u disconnected - cleaning up\n", conn->handle);

  /* Give back any allocated buffers */

  do
    {
      buf = NULL;

      /* Make sure the thread is not blocked forever on an empty queue.
       * SIOCBTCONNECT will fail if preceding SIOCBTDISCONNECT does not
       * result in a successful termination of this thread.
       */

      ret = mq_getattr(conn->tx_queue, &attr);
      if (ret != OK)
        {
          break;
        }

      if (attr.mq_curmsgs == 0)
        {
          break;
        }

      ret = bt_queue_receive(conn->tx_queue, &buf);
      if (ret >= 0)
        {
          DEBUGASSERT(buf != NULL);
          bt_buf_release(buf);
        }
    }
  while (ret >= OK);

  bt_conn_reset_rx_state(conn);

  wlinfo("handle %u exiting\n", conn->handle);

  /* Release reference taken when thread was created */

  bt_conn_release(conn);
  return EXIT_SUCCESS;
}

static int bt_hci_disconnect(FAR struct bt_conn_s *conn, uint8_t reason)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_disconnect_s *disconn;
  int err;

  buf = bt_hci_cmd_create(BT_HCI_OP_DISCONNECT, sizeof(*disconn));
  if (!buf)
    {
      return -ENOBUFS;
    }

  disconn         = bt_buf_extend(buf, sizeof(*disconn));
  disconn->handle = BT_HOST2LE16(conn->handle);
  disconn->reason = reason;

  err = bt_hci_cmd_send(BT_HCI_OP_DISCONNECT, buf);
  if (err)
    {
      return err;
    }

  bt_conn_set_state(conn, BT_CONN_DISCONNECT);
  return 0;
}

static int bt_hci_connect_le_cancel(FAR struct bt_conn_s *conn)
{
  int err;

  err = bt_hci_cmd_send(BT_HCI_OP_LE_CREATE_CONN_CANCEL, NULL);
  if (err)
    {
      return err;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_conn_receive
 *
 * Description:
 *   Receive packets from the HCI core on a registered connection.
 *
 * Input Parameters:
 *   conn  - The registered connection
 *   buf   - The buffer structure containing the packet
 *   flags - Packet boundary flags
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_conn_receive(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                     uint8_t flags)
{
  FAR struct bt_l2cap_hdr_s *hdr;
  uint16_t len;

  wlinfo("handle %u len %u flags %02x\n", conn->handle, buf->len, flags);

  /* Check packet boundary flags */

  switch (flags)
    {
      case 0x02:

        /* First packet */

        hdr = (void *)buf->data;
        len = BT_LE162HOST(hdr->len);

        wlinfo("First, len %u final %u\n", buf->len, len);

        if (conn->rx_len)
          {
            wlerr("ERROR: Unexpected first L2CAP frame\n");
            bt_conn_reset_rx_state(conn);
          }

        conn->rx_len = (sizeof(*hdr) + len) - buf->len;
        wlinfo("rx_len %u\n", conn->rx_len);
        if (conn->rx_len)
          {
            conn->rx = buf;
            return;
          }

        break;

      case 0x01:

        /* Continuation */

        if (!conn->rx_len)
          {
            wlerr("ERROR: Unexpected L2CAP continuation\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_release(buf);
            return;
          }

        if (buf->len > conn->rx_len)
          {
            wlerr("ERROR: L2CAP data overflow\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_release(buf);
            return;
          }

        wlinfo("Cont, len %u rx_len %u\n", buf->len, conn->rx_len);

        if (buf->len > bt_buf_tailroom(conn->rx))
          {
            wlerr("ERROR: Not enough buffer space for L2CAP data\n");
            bt_conn_reset_rx_state(conn);
            bt_buf_release(buf);
            return;
          }

        memcpy(bt_buf_extend(conn->rx, buf->len), buf->data, buf->len);
        conn->rx_len -= buf->len;
        bt_buf_release(buf);

        if (conn->rx_len)
          {
            return;
          }

        buf          = conn->rx;
        conn->rx     = NULL;
        conn->rx_len = 0;

        break;

      default:
        wlerr("ERROR: Unexpected ACL flags (0x%02x)\n", flags);
        bt_conn_reset_rx_state(conn);
        bt_buf_release(buf);
        return;
    }

  hdr = (void *)buf->data;
  len = BT_LE162HOST(hdr->len);

  if (sizeof(*hdr) + len != buf->len)
    {
      wlerr("ERROR: ACL len mismatch (%u != %u)\n", len, buf->len);
      bt_buf_release(buf);
      return;
    }

  wlinfo("Successfully parsed %u byte L2CAP packet\n", buf->len);

  bt_l2cap_receive(conn, buf);
}

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

void bt_conn_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_acl_hdr_s *hdr;
  sq_queue_t fraglist;
  uint16_t len;
  uint16_t remaining = buf->len;
  FAR uint8_t *ptr;

  DEBUGASSERT(conn != NULL && buf != NULL);

  sq_init(&fraglist);

  wlinfo("conn handle %u buf len %u\n", conn->handle, buf->len);

  if (conn->state != BT_CONN_CONNECTED)
    {
      wlerr("ERROR: not connected!\n");
      return;
    }

  len = remaining;
  if (len > g_btdev.le_mtu)
    {
      len = g_btdev.le_mtu;
    }

  hdr         = bt_buf_provide(buf, sizeof(*hdr));
  hdr->handle = BT_HOST2LE16(conn->handle);
  hdr->len    = BT_HOST2LE16(len);

  buf->len   -= remaining - len;
  ptr         = bt_buf_tail(buf);

  /* Add the fragment to the end of the list */

  sq_addlast((FAR sq_entry_t *)buf, &fraglist);
  remaining  -= len;

  while (remaining)
    {
      buf = bt_l2cap_create_pdu(conn);

      len = remaining;
      if (len < g_btdev.le_mtu)
        {
          len = g_btdev.le_mtu;
        }

      /* Copy from original buffer */

      memcpy(bt_buf_extend(buf, len), ptr, len);
      ptr        += len;

      hdr         = bt_buf_provide(buf, sizeof(*hdr));
      hdr->handle = BT_HOST2LE16(conn->handle | (1 << 12));
      hdr->len    = BT_HOST2LE16(len);

      /* Add the fragment to the end of the list */

      sq_addlast((FAR sq_entry_t *)buf, &fraglist);
      remaining  -= len;
    }

  /* Then send each fragment in the correct order */

  while ((buf = (FAR struct bt_buf_s *)sq_remfirst(&fraglist)) != NULL)
    {
      bt_queue_send(conn->tx_queue, buf, BT_NORMAL_PRIO);
    }
}

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
                                  uint8_t role)
{
  FAR struct bt_conn_s *conn = NULL;
  int i;

  for (i = 0; i < CONFIG_BLUETOOTH_MAX_CONN; i++)
    {
      if (!bt_addr_le_cmp(&g_conns[i].dst, BT_ADDR_LE_ANY))
        {
          conn = &g_conns[i];
          break;
        }
    }

  if (!conn)
    {
      return NULL;
    }

  memset(conn, 0, sizeof(*conn));

  bt_atomic_set(&conn->ref, 1);
  conn->role = role;
  bt_addr_le_copy(&conn->dst, peer);

  return conn;
}

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
                       enum bt_conn_state_e state)
{
  enum bt_conn_state_e old_state;

  wlinfo("%s -> %s\n", state2str(conn->state), state2str(state));

  if (conn->state == state)
    {
      wlwarn("no transition\n");
      return;
    }

  old_state   = conn->state;
  conn->state = state;

  /* Take a reference for the first state transition after bt_conn_add() and
   * keep it until reaching DISCONNECTED again.
   */

  if (old_state == BT_CONN_DISCONNECTED)
    {
      bt_conn_addref(conn);
    }

  switch (conn->state)
    {
      case BT_CONN_CONNECTED:
        {
          pid_t pid;
          int ret;

          ret = bt_queue_open(BT_CONN_TX, O_RDWR | O_CREAT,
                              CONFIG_BLUETOOTH_TXCONN_NMSGS,
                              &conn->tx_queue);
          DEBUGASSERT(ret >= 0 && g_btdev.tx_queue != 0);
          UNUSED(ret);

          /* Get exclusive access to the handoff structure.  The count will
           * be zero when we complete this.
           */

          ret = nxsem_wait_uninterruptible(&g_conn_handoff.sync_sem);
          if (ret >= 0)
            {
              /* Start the Tx connection kernel thread */

              g_conn_handoff.conn = bt_conn_addref(conn);
              pid = kthread_create("BT Conn Tx",
                                   CONFIG_BLUETOOTH_TXCONN_PRIORITY,
                                   CONFIG_BLUETOOTH_TXCONN_STACKSIZE,
                                   conn_tx_kthread, NULL);
              DEBUGASSERT(pid > 0);
              UNUSED(pid);

              /* Take the semaphore again.  This will force us to wait with
               * the sem_count at -1.  It will be zero again when we
               * continue.
               */

              ret = nxsem_wait_uninterruptible(&g_conn_handoff.sync_sem);
              nxsem_post(&g_conn_handoff.sync_sem);
          }
        }
        break;

      case BT_CONN_DISCONNECTED:

        /* Send dummy buffer to wake up and stop the Tx thread for states
         * where it was running.
         */

        if (old_state == BT_CONN_CONNECTED ||
           old_state == BT_CONN_DISCONNECT)
          {
            bt_queue_send(conn->tx_queue, bt_buf_alloc(BT_DUMMY, NULL, 0),
                          BT_NORMAL_PRIO);
          }

        /* Release the reference we took for the very first state
         * transition.
         */

        bt_conn_release(conn);
        break;

      case BT_CONN_CONNECT_SCAN:
      case BT_CONN_CONNECT:
      case BT_CONN_DISCONNECT:
        break;

      default:
        wlwarn("no valid (%u) state was set\n", state);
        break;
    }
}

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

FAR struct bt_conn_s *bt_conn_lookup_handle(uint16_t handle)
{
  int i;

  for (i = 0; i < CONFIG_BLUETOOTH_MAX_CONN; i++)
    {
      /* We only care about connections with a valid handle */

      if (g_conns[i].state != BT_CONN_CONNECTED &&
          g_conns[i].state != BT_CONN_DISCONNECT)
        {
          continue;
        }

      if (g_conns[i].handle == handle)
        {
          return bt_conn_addref(&g_conns[i]);
        }
    }

  return NULL;
}

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

FAR struct bt_conn_s *bt_conn_lookup_addr_le(FAR const bt_addr_le_t * peer)
{
  int i;

  for (i = 0; i < CONFIG_BLUETOOTH_MAX_CONN; i++)
    {
      if (!bt_addr_le_cmp(peer, &g_conns[i].dst))
        {
          return bt_conn_addref(&g_conns[i]);
        }
    }

  return NULL;
}

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

FAR struct bt_conn_s *bt_conn_lookup_state(FAR const bt_addr_le_t * peer,
                                           enum bt_conn_state_e state)
{
  int i;

  for (i = 0; i < CONFIG_BLUETOOTH_MAX_CONN; i++)
    {
      if (!bt_addr_le_cmp(&g_conns[i].dst, BT_ADDR_LE_ANY))
        {
          continue;
        }

      if (bt_addr_le_cmp(peer, BT_ADDR_LE_ANY) &&
          bt_addr_le_cmp(peer, &g_conns[i].dst))
        {
          continue;
        }

      if (g_conns[i].state == state)
        {
          return bt_conn_addref(&g_conns[i]);
        }
    }

  return NULL;
}

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

FAR struct bt_conn_s *bt_conn_addref(FAR struct bt_conn_s *conn)
{
  bt_atomic_incr(&conn->ref);

  wlinfo("handle %u ref %u\n", conn->handle, bt_atomic_get(&conn->ref));

  return conn;
}

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

void bt_conn_release(FAR struct bt_conn_s *conn)
{
  bt_atomic_t old_ref;

  old_ref = bt_atomic_decr(&conn->ref);

  wlinfo("handle %u ref %u\n", conn->handle, bt_atomic_get(&conn->ref));

  if (old_ref > 1)
    {
      return;
    }

  bt_addr_le_copy(&conn->dst, BT_ADDR_LE_ANY);
}

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

FAR const bt_addr_le_t *bt_conn_get_dst(FAR const struct bt_conn_s *conn)
{
  return &conn->dst;
}

/****************************************************************************
 * Name: bt_conn_security
 *
 * Description:
 *   This function enable security (encryption) for a connection. If device
 *   is already paired with sufficiently strong key encryption will be
 *   enabled. If link is already encrypted with sufficiently strong key this
 *   function does nothing.
 *
 *   If device is not paired pairing will be initiated. If device is paired
 *   and keys are too weak but input output capabilities allow for strong
 *   enough keys pairing will be initiated.
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

int bt_conn_security(FAR struct bt_conn_s *conn, enum bt_security_e sec)
{
  FAR struct bt_keys_s *keys;

  if (conn->state != BT_CONN_CONNECTED)
    {
      return -ENOTCONN;
    }

  /* Nothing to do */

  if (sec == BT_SECURITY_LOW)
    {
      return 0;
    }

  /* For now we only support JustWorks */

  if (sec > BT_SECURITY_MEDIUM)
    {
      return -EINVAL;
    }

  if (conn->encrypt)
    {
      return 0;
    }

  if (conn->role == BT_HCI_ROLE_SLAVE)
    {
      return bt_smp_send_security_req(conn);
    }

  keys = bt_keys_find(BT_KEYS_LTK, &conn->dst);
  if (keys)
    {
      return bt_conn_le_start_encryption(conn, keys->ltk.rand,
                                         keys->ltk.ediv, keys->ltk.val);
    }

  return bt_smp_send_pairing_req(conn);
}

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

void bt_conn_set_auto_conn(FAR struct bt_conn_s *conn, bool auto_conn)
{
  if (auto_conn)
    {
      bt_atomic_setbit(conn->flags, BT_CONN_AUTO_CONNECT);
    }
  else
    {
      bt_atomic_clrbit(conn->flags, BT_CONN_AUTO_CONNECT);
    }
}

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

int bt_conn_disconnect(FAR struct bt_conn_s *conn, uint8_t reason)
{
  /* Disconnection is initiated by us, so auto connection shall be disabled.
   * Otherwise the passive scan would be enabled and we could send LE Create
   * Connection as soon as the remote starts advertising.
   */

  bt_conn_set_auto_conn(conn, false);

  switch (conn->state)
    {
      case BT_CONN_CONNECT_SCAN:
        bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
        bt_le_scan_update();
        return 0;

      case BT_CONN_CONNECT:
        return bt_hci_connect_le_cancel(conn);

      case BT_CONN_CONNECTED:
        return bt_hci_disconnect(conn, reason);

      case BT_CONN_DISCONNECT:
        return 0;

      case BT_CONN_DISCONNECTED:
      default:
        return -ENOTCONN;
    }
}

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

FAR struct bt_conn_s *bt_conn_create_le(FAR const bt_addr_le_t *peer)
{
  FAR struct bt_conn_s *conn;

  /* First check if this connection exists and that it is in a proper
   * state.
   */

  conn = bt_conn_lookup_addr_le(peer);
  if (conn != NULL)
    {
      switch (conn->state)
        {
          case BT_CONN_CONNECT_SCAN:
          case BT_CONN_CONNECT:
          case BT_CONN_CONNECTED:
            return conn;

          default:
            bt_conn_release(conn);
            return NULL;
        }
    }

  /* No.. the connection does not exist.  Create it assuming MASTER role
   * and put it in the BT_CONNECT_SCAN state.
   */

  conn = bt_conn_add(peer, BT_HCI_ROLE_MASTER);
  if (!conn)
    {
      return NULL;
    }

  bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);
  bt_le_scan_update();
  return conn;
}

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
                                uint16_t ediv, FAR const uint8_t *ltk)
{
  FAR struct bt_hci_cp_le_start_encryption_s *cp;
  FAR struct bt_buf_s *buf;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_START_ENCRYPTION, sizeof(*cp));
  if (!buf)
    {
      return -ENOBUFS;
    }

  cp         = bt_buf_extend(buf, sizeof(*cp));
  cp->handle = BT_HOST2LE16(conn->handle);
  cp->rand   = rand;
  cp->ediv   = ediv;
  memcpy(cp->ltk, ltk, sizeof(cp->ltk));

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_START_ENCRYPTION, buf, NULL);
}

int bt_conn_le_conn_update(FAR struct bt_conn_s *conn, uint16_t min,
                           uint16_t max, uint16_t latency, uint16_t timeout)
{
  FAR struct hci_cp_le_conn_update_s *conn_update;
  FAR struct bt_buf_s *buf;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_CONN_UPDATE, sizeof(*conn_update));
  if (!buf)
    {
      return -ENOBUFS;
    }

  conn_update                      = bt_buf_extend(buf,
                                                   sizeof(*conn_update));
  memset(conn_update, 0, sizeof(*conn_update));
  conn_update->handle              = BT_HOST2LE16(conn->handle);
  conn_update->conn_interval_min   = BT_HOST2LE16(min);
  conn_update->conn_interval_max   = BT_HOST2LE16(max);
  conn_update->conn_latency        = BT_HOST2LE16(latency);
  conn_update->supervision_timeout = BT_HOST2LE16(timeout);

  return bt_hci_cmd_send(BT_HCI_OP_LE_CONN_UPDATE, buf);
}
