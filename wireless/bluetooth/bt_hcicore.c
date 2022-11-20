/****************************************************************************
 * wireless/bluetooth/bt_hcicore.c
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <time.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/kthread.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "bt_queue.h"
#include "bt_buf.h"
#include "bt_keys.h"
#include "bt_conn.h"
#include "bt_l2cap.h"
#include "bt_hcicore.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

/* Wait up to 2.5 seconds for a response.  This delay is arbitrary and
 * intended only to avoid hangs while waiting for a response.  It may need
 * to be adjusted.
 */

#define TIMEOUT_MSEC   2500

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* State of connected HCI device.
 *
 * NOTE:  Because this is a global singleton, multiple HCI devices may not
 * be supported.
 */

struct bt_dev_s g_btdev;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
static FAR struct bt_conn_cb_s *g_callback_list;
static bt_le_scan_cb_t *g_scan_dev_found_cb;
#else
static struct bt_hci_cb_s *g_hci_cb;
#endif

/* Lists of pending received messages.  One for low priority input that is
 * processed on the low priority work queue and one for high priority
 * input that is processed on high priority work queue.
 */

static FAR struct bt_bufferlist_s g_lp_rxlist;
static FAR struct bt_bufferlist_s g_hp_rxlist;

/* Work structures: One for high priority and one for low priority work */

static struct work_s g_lp_work;
static struct work_s g_hp_work;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_enqueue_bufwork
 *
 * Description:
 *   Add the provided buffer 'buf' to the head selected buffer list 'list'
 *
 * Input Parameters:
 *   list - The buffer list to use
 *   buf  - The buffer to be added to the head of the buffer list
 *
 * Returned Value:
 *
 ****************************************************************************/

static void bt_enqueue_bufwork(FAR struct bt_bufferlist_s *list,
                               FAR struct bt_buf_s *buf)
{
  irqstate_t flags;

  flags      = spin_lock_irqsave(NULL);
  buf->flink = list->head;
  if (list->head == NULL)
    {
      list->tail = buf;
    }

  list->head = buf;
  spin_unlock_irqrestore(NULL, flags);
}

/****************************************************************************
 * Name: bt_dequeue_bufwork
 *
 * Description:
 *   Remove and return the buffer at the tail of the buffer list specified
 *   by 'list'.
 *
 * Input Parameters:
 *   list - The buffer list to use
 *
 * Returned Value:
 *   A pointer to the buffer that was at the tail of the buffer list.  NULL
 *   is returned if the list was empty.
 *
 ****************************************************************************/

static FAR struct bt_buf_s *
  bt_dequeue_bufwork(FAR struct bt_bufferlist_s *list)
{
  FAR struct bt_buf_s *buf;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);
  buf   = list->tail;
  if (buf != NULL)
    {
      if (list->head == list->tail)
        {
          list->head = NULL;
          list->tail = NULL;
        }
      else
        {
          FAR struct bt_buf_s *prev;

          for (prev = list->head;
               prev && prev->flink != buf;
               prev = prev->flink)
            {
            }

          if (prev != NULL)
            {
              prev->flink = NULL;
              list->tail  = prev;
            }
        }

      buf->flink = NULL;
    }

  spin_unlock_irqrestore(NULL, flags);
  return buf;
}

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
static void bt_connected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_conn_cb_s *cb;

  for (cb = g_callback_list; cb; cb = cb->flink)
    {
      if (cb->connected)
        {
          cb->connected(conn, cb->context);
        }
    }
}

static void bt_disconnected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_conn_cb_s *cb;

  for (cb = g_callback_list; cb; cb = cb->flink)
    {
      if (cb->disconnected)
        {
          cb->disconnected(conn, cb->context);
        }
    }
}

static void hci_acl(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_acl_hdr_s *hdr = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;
  uint16_t len = BT_LE162HOST(hdr->len);
  uint8_t flags;

  wlinfo("buf %p\n", buf);

  handle            = BT_LE162HOST(hdr->handle);
  flags             = (handle >> 12);
  buf->u.acl.handle = bt_acl_handle(handle);

  bt_buf_consume(buf, sizeof(*hdr));

  wlinfo("handle %u len %u flags %u\n", buf->u.acl.handle, len, flags);

  if (buf->len != len)
    {
      wlerr("ERROR:  ACL data length mismatch (%u != %u)\n",
             buf->len, len);
      bt_buf_release(buf);
      return;
    }

  conn = bt_conn_lookup_handle(buf->u.acl.handle);
  if (!conn)
    {
      wlerr("ERROR:  Unable to find conn for handle %u\n",
            buf->u.acl.handle);
      bt_buf_release(buf);
      return;
    }

  bt_conn_receive(conn, buf, flags);
  bt_conn_release(conn);
}

/* HCI event processing */

static void hci_encrypt_change(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_encrypt_change_s *evt = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle = BT_LE162HOST(evt->handle);

  wlinfo("status %u handle %u encrypt 0x%02x\n",
        evt->status, handle, evt->encrypt);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR:  Unable to look up conn with handle %u\n", handle);
      return;
    }

  conn->encrypt = evt->encrypt;

  bt_l2cap_encrypt_change(conn);
  bt_conn_release(conn);
}

static void hci_reset_complete(FAR struct bt_buf_s *buf)
{
  uint8_t status = buf->data[0];

  wlinfo("status %u\n", status);

  if (status)
    {
      return;
    }

  g_scan_dev_found_cb = NULL;
  g_btdev.scan_enable = BT_LE_SCAN_DISABLE;
  g_btdev.scan_filter = BT_LE_SCAN_FILTER_DUP_ENABLE;
}

static void hci_cmd_done(uint16_t opcode, uint8_t status,
                         FAR struct bt_buf_s *buf)
{
  FAR struct bt_buf_s *sent = g_btdev.sent_cmd;

  if (sent == NULL)
    {
      return;
    }

  if (g_btdev.sent_cmd == NULL)
    {
      wlerr("ERROR: Request cmd missing!\n");
      return;
    }

  if (g_btdev.sent_cmd->u.hci.opcode != opcode)
    {
      wlerr("ERROR:  Unexpected completion of opcode 0x%04x " \
            "expected 0x%04x\n",
            opcode, g_btdev.sent_cmd->u.hci.opcode);
      return;
    }

  g_btdev.sent_cmd = NULL;

  /* If the command was synchronous wake up bt_hci_cmd_send_sync() */

  if (sent->u.hci.sync != NULL)
    {
      FAR sem_t *sem = sent->u.hci.sync;

      if (status != 0)
        {
          wlwarn("WARNING: status %u\n", status);
          sent->u.hci.sync = NULL;
        }
      else
        {
          sent->u.hci.sync = bt_buf_addref(buf);
        }

      nxsem_post(sem);
    }

  bt_buf_release(sent);
}

static void hci_cmd_complete(FAR struct bt_buf_s *buf)
{
  FAR struct hci_evt_cmd_complete_s *evt = (FAR void *)buf->data;
  uint16_t opcode = BT_LE162HOST(evt->opcode);
  FAR uint8_t *status;

  wlinfo("opcode %04x\n", opcode);

  bt_buf_consume(buf, sizeof(*evt));

  /* All command return parameters have a 1-byte status in the beginning, so
   * we can safely make this generalization.
   */

  status = buf->data;

  switch (opcode)
    {
      case BT_HCI_OP_RESET:
        hci_reset_complete(buf);
        break;

      default:
        wlinfo("Unhandled opcode %04x\n", opcode);
        break;
    }

  hci_cmd_done(opcode, *status, buf);

  if (evt->ncmd > 0 && g_btdev.ncmd == 0)
    {
      /* Allow next command to be sent */

      g_btdev.ncmd = 1;
      nxsem_post(&g_btdev.ncmd_sem);
    }
}

static void hci_cmd_status(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_cmd_status_s *evt = (FAR void *)buf->data;
  uint16_t opcode = BT_LE162HOST(evt->opcode);

  wlinfo("opcode %04x\n", opcode);

  bt_buf_consume(buf, sizeof(*evt));

  switch (opcode)
    {
      default:
        wlinfo("Unhandled opcode %04x\n", opcode);
        break;
    }

  hci_cmd_done(opcode, evt->status, buf);

  if (evt->ncmd && !g_btdev.ncmd)
    {
      /* Allow next command to be sent */

      g_btdev.ncmd = 1;
      nxsem_post(&g_btdev.ncmd_sem);
    }
}

static void hci_num_completed_packets(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_num_completed_packets_s *evt = (FAR void *)buf->data;
  uint16_t num_handles = BT_LE162HOST(evt->num_handles);
  uint16_t i;

  wlinfo("num_handles %u\n", num_handles);

  for (i = 0; i < num_handles; i++)
    {
      uint16_t handle;
      uint16_t count;

      handle = BT_LE162HOST(evt->h[i].handle);
      count  = BT_LE162HOST(evt->h[i].count);

      wlinfo("handle %u count %u\n", handle, count);
      UNUSED(handle);

      while (count--)
        {
          nxsem_post(&g_btdev.le_pkts_sem);
        }
    }
}

static void hci_encrypt_key_refresh_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_encrypt_key_refresh_complete_s *evt =
    (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;

  handle = BT_LE162HOST(evt->handle);

  wlinfo("status %u handle %u\n", evt->status, handle);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR:  Unable to look up conn with handle %u\n", handle);
      return;
    }

  bt_l2cap_encrypt_change(conn);
  bt_conn_release(conn);
}

static void copy_id_addr(FAR struct bt_conn_s *conn,
                         FAR const bt_addr_le_t *addr)
{
  FAR struct bt_keys_s *keys;

  /* If we have a keys struct we already know the identity */

  if (conn->keys)
    {
      return;
    }

  keys = bt_keys_find_irk(addr);
  if (keys)
    {
      bt_addr_le_copy(&conn->dst, &keys->addr);
      conn->keys = keys;
    }
  else
    {
      bt_addr_le_copy(&conn->dst, addr);
    }
}

static int bt_hci_start_scanning(uint8_t scan_type, uint8_t scan_filter)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_buf_s *rsp;
  FAR struct bt_hci_cp_le_set_scan_params_s *set_param;
  FAR struct bt_hci_cp_le_set_scan_enable_s *scan_enable;
  int ret;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_PARAMS, sizeof(*set_param));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  set_param = bt_buf_extend(buf, sizeof(*set_param));
  memset(set_param, 0, sizeof(*set_param));
  set_param->scan_type = scan_type;

  /* for the rest parameters apply default values according to spec 4.2,
   * vol2, part E, 7.8.10
   */

  set_param->interval      = BT_HOST2LE16(0x0010);
  set_param->window        = BT_HOST2LE16(0x0010);
  set_param->filter_policy = 0x00;
  set_param->addr_type     = 0x00;

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_SCAN_PARAMS, buf);
  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE,
                          sizeof(*scan_enable));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  scan_enable             = bt_buf_extend(buf, sizeof(*scan_enable));
  memset(scan_enable, 0, sizeof(*scan_enable));
  scan_enable->filter_dup = scan_filter;
  scan_enable->enable     = BT_LE_SCAN_ENABLE;

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
      return ret;
    }

  /* Update scan state in case of success (0) status */

  ret = rsp->data[0];
  if (!ret)
    {
      g_btdev.scan_enable = BT_LE_SCAN_ENABLE;
    }

  bt_buf_release(rsp);
  return ret;
}

static int bt_hci_stop_scanning(void)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_buf_s *rsp;
  FAR struct bt_hci_cp_le_set_scan_enable_s *scan_enable;
  int ret;

  if (g_btdev.scan_enable == BT_LE_SCAN_DISABLE)
    {
      wlwarn("WARNING:  Scan already disabled\n");
      return -EALREADY;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_ENABLE,
                          sizeof(*scan_enable));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  scan_enable             = bt_buf_extend(buf, sizeof(*scan_enable));
  memset(scan_enable, 0x0, sizeof(*scan_enable));
  scan_enable->filter_dup = 0x00;
  scan_enable->enable     = BT_LE_SCAN_DISABLE;

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_SCAN_ENABLE, buf, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
      return ret;
    }

  /* Update scan state in case of success (0) status */

  ret = rsp->data[0];
  if (!ret)
    {
      g_btdev.scan_enable = BT_LE_SCAN_DISABLE;
    }

  bt_buf_release(rsp);
  return ret;
}

static int hci_le_create_conn(FAR const bt_addr_le_t *addr)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_le_create_conn_s *cp;

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_CREATE_CONN, sizeof(*cp));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  cp                      = bt_buf_extend(buf, sizeof(*cp));
  memset(cp, 0x0, sizeof(*cp));
  bt_addr_le_copy(&cp->peer_addr, addr);
  cp->conn_interval_max   = BT_HOST2LE16(0x0028);
  cp->conn_interval_min   = BT_HOST2LE16(0x0018);
  cp->scan_interval       = BT_HOST2LE16(0x0060);
  cp->scan_window         = BT_HOST2LE16(0x0030);
  cp->supervision_timeout = BT_HOST2LE16(0x07d0);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_CREATE_CONN, buf, NULL);
}

static void hci_disconn_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_disconn_complete_s *evt = (FAR void *)buf->data;
  uint16_t handle = BT_LE162HOST(evt->handle);
  FAR struct bt_conn_s *conn;

  wlinfo("status %u handle %u reason %u\n",
         evt->status, handle, evt->reason);

  if (evt->status)
    {
      return;
    }

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR:  Unable to look up conn with handle %u\n", handle);
      return;
    }

  bt_l2cap_disconnected(conn);
  bt_disconnected(conn);

  bt_conn_set_state(conn, BT_CONN_DISCONNECTED);
  conn->handle = 0;

  if (bt_atomic_testbit(conn->flags, BT_CONN_AUTO_CONNECT))
    {
      bt_conn_set_state(conn, BT_CONN_CONNECT_SCAN);
      bt_le_scan_update();
    }

  bt_conn_release(conn);

  if (g_btdev.adv_enable)
    {
      buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
      if (buf)
        {
          memcpy(bt_buf_extend(buf, 1), &g_btdev.adv_enable, 1);
          bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_ENABLE, buf);
        }
    }
}

static void le_conn_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_conn_complete_s *evt = (FAR void *)buf->data;
  uint16_t handle = BT_LE162HOST(evt->handle);
  FAR struct bt_conn_s *conn;
  FAR struct bt_keys_s *keys;

  wlinfo("status %u handle %u role %u %s\n", evt->status, handle,
         evt->role, bt_addr_le_str(&evt->peer_addr));

  /* Make lookup to check if there's a connection object in CONNECT state
   * associated with passed peer LE address.
   */

  keys = bt_keys_find_irk(&evt->peer_addr);
  if (keys)
    {
      conn = bt_conn_lookup_state(&keys->addr, BT_CONN_CONNECT);
    }
  else
    {
      conn = bt_conn_lookup_state(&evt->peer_addr, BT_CONN_CONNECT);
    }

  if (evt->status)
    {
      if (!conn)
        {
          return;
        }

      bt_conn_set_state(conn, BT_CONN_DISCONNECTED);

      /* Drop the reference got by lookup call in CONNECT state. We are now
       * in the DISCONNECTED state since no successful LE link been made.
       */

      bt_conn_release(conn);
      return;
    }

  if (!conn)
    {
      conn = bt_conn_add(&evt->peer_addr, evt->role);
    }

  if (!conn)
    {
      wlerr("ERROR:  Unable to add new conn for handle %u\n", handle);
      return;
    }

  conn->handle           = handle;
  conn->src.type         = BT_ADDR_LE_PUBLIC;
  memcpy(conn->src.val, g_btdev.bdaddr.val, sizeof(g_btdev.bdaddr.val));
  copy_id_addr(conn, &evt->peer_addr);
  conn->le_conn_interval = BT_LE162HOST(evt->interval);

  bt_conn_set_state(conn, BT_CONN_CONNECTED);

  bt_l2cap_connected(conn);

  if (evt->role == BT_HCI_ROLE_SLAVE)
    {
      bt_l2cap_update_conn_param(conn);
    }

  bt_connected(conn);
  bt_conn_release(conn);
  bt_le_scan_update();
}

static void check_pending_conn(FAR const bt_addr_le_t *addr, uint8_t evtype,
                               FAR struct bt_keys_s *keys)
{
  FAR struct bt_conn_s *conn;

  /* Return if event is not connectible */

  if (evtype != BT_LE_ADV_IND && evtype != BT_LE_ADV_DIRECT_IND)
    {
      return;
    }

  if (keys)
    {
      conn = bt_conn_lookup_state(&keys->addr, BT_CONN_CONNECT_SCAN);
    }
  else
    {
      conn = bt_conn_lookup_state(addr, BT_CONN_CONNECT_SCAN);
    }

  if (!conn)
    {
      return;
    }

  if (bt_hci_stop_scanning())
    {
      goto done;
    }

  if (hci_le_create_conn(addr))
    {
      goto done;
    }

  bt_conn_set_state(conn, BT_CONN_CONNECT);

done:
  bt_conn_release(conn);
}

static void le_adv_report(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_ev_le_advertising_report_s *info;
  uint8_t num_reports = buf->data[0];

  wlinfo("Adv number of reports %u\n", num_reports);

  info = bt_buf_consume(buf, sizeof(num_reports));

  while (num_reports--)
    {
      int8_t rssi = info->data[info->length];
      FAR struct bt_keys_s *keys;
      bt_addr_le_t addr;

      wlinfo("%s event %u, len %u, rssi %d dBm\n",
             bt_addr_le_str(&info->addr), info->evt_type, info->length,
             rssi);

      keys = bt_keys_find_irk(&info->addr);
      if (keys)
        {
          bt_addr_le_copy(&addr, &keys->addr);
          wlinfo("Identity %s matched RPA %s\n",
                 bt_addr_le_str(&keys->addr), bt_addr_le_str(&info->addr));
        }
      else
        {
          bt_addr_le_copy(&addr, &info->addr);
        }

      if (g_scan_dev_found_cb)
        {
          g_scan_dev_found_cb(&addr, rssi, info->evt_type,
                            info->data, info->length);
        }

      check_pending_conn(&info->addr, info->evt_type, keys);

      /* Get next report iteration by moving pointer to right offset in buf
       * according to spec 4.2, Vol 2, Part E, 7.7.65.2.
       *
       * TODO: multiple reports are stored as multiple arrays not one array
       * of structs. If num_reports > 0 this will not WORK!
       */

      /* Note that info already contains one byte which accounts for RSSI */

      info = bt_buf_consume(buf, sizeof(*info) + info->length);
    }
}

static void le_ltk_request(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_ltk_request_s *evt = (FAR void *)buf->data;
  FAR struct bt_conn_s *conn;
  uint16_t handle;

  handle = BT_LE162HOST(evt->handle);

  wlinfo("handle %u\n", handle);

  conn = bt_conn_lookup_handle(handle);
  if (!conn)
    {
      wlerr("ERROR:  Unable to lookup conn for handle %u\n", handle);
      return;
    }

  if (!conn->keys)
    {
      conn->keys = bt_keys_find(BT_KEYS_SLAVE_LTK, &conn->dst);
    }

  if (conn->keys && (conn->keys->keys & BT_KEYS_SLAVE_LTK) &&
      conn->keys->slave_ltk.rand == evt->rand &&
      conn->keys->slave_ltk.ediv == evt->ediv)
    {
      FAR struct bt_hci_cp_le_ltk_req_reply_s *cp;

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_LTK_REQ_REPLY, sizeof(*cp));
      if (!buf)
        {
          wlerr("ERROR:  Out of command buffers\n");
          goto done;
        }

      cp         = bt_buf_extend(buf, sizeof(*cp));
      cp->handle = evt->handle;
      memcpy(cp->ltk, conn->keys->slave_ltk.val, 16);

      bt_hci_cmd_send(BT_HCI_OP_LE_LTK_REQ_REPLY, buf);
    }
  else
    {
      FAR struct bt_hci_cp_le_ltk_req_neg_reply_s *cp;

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_LTK_REQ_NEG_REPLY, sizeof(*cp));
      if (!buf)
        {
          wlerr("ERROR:  Out of command buffers\n");
          goto done;
        }

      cp         = bt_buf_extend(buf, sizeof(*cp));
      cp->handle = evt->handle;

      bt_hci_cmd_send(BT_HCI_OP_LE_LTK_REQ_NEG_REPLY, buf);
    }

done:
  bt_conn_release(conn);
}

static void hci_le_meta_event(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_le_meta_event_s *evt = (FAR void *)buf->data;

  bt_buf_consume(buf, sizeof(*evt));

  switch (evt->subevent)
    {
      case BT_HCI_EVT_LE_CONN_COMPLETE:
        le_conn_complete(buf);
        break;

      case BT_HCI_EVT_LE_ADVERTISING_REPORT:
        le_adv_report(buf);
        break;

      case BT_HCI_EVT_LE_LTK_REQUEST:
        le_ltk_request(buf);
        break;

      default:
        wlinfo("Unhandled LE event %04x\n", evt->subevent);
        break;
    }
}

static void hci_event(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_evt_hdr_s *hdr = (FAR void *)buf->data;

  wlinfo("event %u\n", hdr->evt);

  bt_buf_consume(buf, sizeof(struct bt_hci_evt_hdr_s));

  switch (hdr->evt)
    {
      case BT_HCI_EVT_DISCONN_COMPLETE:
        hci_disconn_complete(buf);
        break;

      case BT_HCI_EVT_ENCRYPT_CHANGE:
        hci_encrypt_change(buf);
        break;

      case BT_HCI_EVT_ENCRYPT_KEY_REFRESH_COMPLETE:
        hci_encrypt_key_refresh_complete(buf);
        break;

      case BT_HCI_EVT_LE_META_EVENT:
        hci_le_meta_event(buf);
        break;

      default:
        wlwarn("WARNING:  Unhandled event 0x%02x\n", hdr->evt);
        break;
    }

  bt_buf_release(buf);
}
#endif

/****************************************************************************
 * Name: hci_tx_kthread
 *
 * Description:
 *   This is a kernel thread that handles sending of commands.
 *
 * Input Parameters:
 *   Standard kernel thread arguments
 *
 * Returned Value:
 *   Doesn't normally return.
 *
 ****************************************************************************/

static int hci_tx_kthread(int argc, FAR char *argv[])
{
  FAR struct bt_driver_s *btdev = g_btdev.btdev;
  int ret;

  wlinfo("started\n");

  for (; ; )
    {
      FAR struct bt_buf_s *buf;

      /* Wait until ncmd > 0 */

      ret = nxsem_wait_uninterruptible(&g_btdev.ncmd_sem);
      if (ret < 0)
        {
          wlerr("nxsem_wait_uninterruptible() failed: %d\n", ret);
          return EXIT_FAILURE;
        }

      /* Get next command - wait if necessary */

      buf = NULL;
      ret = bt_queue_receive(&g_btdev.tx_queue, &buf);
      DEBUGASSERT(ret >= 0 && buf != NULL);
      UNUSED(ret);

      g_btdev.ncmd = 0;

      /* Clear out any existing sent command */

      if (g_btdev.sent_cmd)
        {
          wlerr("ERROR:  Uncleared pending sent_cmd\n");
          bt_buf_release(g_btdev.sent_cmd);
          g_btdev.sent_cmd = NULL;
        }

      g_btdev.sent_cmd = bt_buf_addref(buf);

      wlinfo("Sending command %04x buf %p to driver\n",
             buf->u.hci.opcode, buf);

      bt_send(btdev, buf);
      bt_buf_release(buf);
    }

  return EXIT_SUCCESS;  /* Can't get here */
}

/****************************************************************************
 * Name: hci_rx_work
 *
 * Description:
 *   This work function operates on the low priority work queue using the
 *   low priority buffer queue.
 *
 * Input Parameters:
 *   arg - Indicates which buffer queue should be used
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hci_rx_work(FAR void *arg)
{
  FAR struct bt_bufferlist_s *list = (FAR struct bt_bufferlist_s *)arg;
  FAR struct bt_buf_s *buf;

  wlinfo("list %p\n", list);
  DEBUGASSERT(list != NULL);

  while ((buf = bt_dequeue_bufwork(list)) != NULL)
    {
      wlinfo("buf %p type %u len %u\n", buf, buf->type, buf->len);

      /* TODO: Hook monitor callback */

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
      switch (buf->type)
        {
          case BT_ACL_IN:
            hci_acl(buf);
            break;

          case BT_EVT:
            hci_event(buf);
            break;

          default:
            wlerr("ERROR:  Unknown buf type %u\n", buf->type);
            bt_buf_release(buf);
            break;
        }
#else
      g_hci_cb->received(buf, g_hci_cb->context);
#endif
      bt_buf_release(buf);
    }
}

/****************************************************************************
 * Name: priority_rx_work
 *
 * Description:
 *   This work function operates on the high priority work thread using the
 *   high priority buffer queue.
 *
 * Input Parameters:
 *   arg - Indicates which buffer queue should be used
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void priority_rx_work(FAR void *arg)
{
  FAR struct bt_bufferlist_s *list = (FAR struct bt_bufferlist_s *)arg;
  FAR struct bt_buf_s *buf;

  wlinfo("list %p\n", list);
  DEBUGASSERT(list != NULL);

  while ((buf = bt_dequeue_bufwork(list)) != NULL)
    {
      FAR struct bt_hci_evt_hdr_s *hdr = (FAR void *)buf->data;

      wlinfo("buf %p type %u len %u\n", buf, buf->type, buf->len);

      /* TODO: Hook monitor callback */

      if (buf->type != BT_EVT)
        {
          wlerr("Unknown buf type %u\n", buf->type);
          bt_buf_release(buf);
          continue;
        }

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
      bt_buf_consume(buf, sizeof(struct bt_hci_evt_hdr_s));

      switch (hdr->evt)
        {
          case BT_HCI_EVT_CMD_COMPLETE:
            hci_cmd_complete(buf);
            break;

          case BT_HCI_EVT_CMD_STATUS:
            hci_cmd_status(buf);
            break;

          case BT_HCI_EVT_NUM_COMPLETED_PACKETS:
            hci_num_completed_packets(buf);
            break;

          default:
            wlerr("Unknown event 0x%02x\n", hdr->evt);
            break;
        }
#else
      UNUSED(hdr);

      g_hci_cb->received(buf, g_hci_cb->context);
#endif
      bt_buf_release(buf);
    }
}

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
static void read_local_features_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_local_features_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  memcpy(g_btdev.features, rp->features, sizeof(g_btdev.features));
}

static void read_local_ver_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_local_version_info_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  g_btdev.hci_version  = rp->hci_version;
  g_btdev.hci_revision = BT_LE162HOST(rp->hci_revision);
  g_btdev.manufacturer = BT_LE162HOST(rp->manufacturer);
}

static void read_bdaddr_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_bd_addr_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  bt_addr_copy(&g_btdev.bdaddr, &rp->bdaddr);
}

static void read_le_features_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_le_read_local_features_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  memcpy(g_btdev.le_features, rp->features, sizeof(g_btdev.le_features));
}

static void read_buffer_size_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_read_buffer_size_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  /* If LE-side has buffers we can ignore the BR/EDR values */

  if (g_btdev.le_mtu)
    {
      return;
    }

  g_btdev.le_mtu  = BT_LE162HOST(rp->acl_max_len);
  g_btdev.le_pkts = BT_LE162HOST(rp->acl_max_num);
}

static void le_read_buffer_size_complete(FAR struct bt_buf_s *buf)
{
  FAR struct bt_hci_rp_le_read_buffer_size_s *rp = (FAR void *)buf->data;

  wlinfo("status %u\n", rp->status);

  g_btdev.le_mtu  = BT_LE162HOST(rp->le_max_len);
  g_btdev.le_pkts = rp->le_max_num;
}

/****************************************************************************
 * Name: hci_initialize()
 *
 * Description:
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int hci_initialize(void)
{
  FAR struct bt_hci_cp_host_buffer_size_s *hbs;
  FAR struct bt_hci_cp_set_event_mask_s *ev;
  FAR struct bt_buf_s *buf;
  FAR struct bt_buf_s *rsp;
  FAR uint8_t *enable;
  int ret;

  /* Send HCI_RESET */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_RESET, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR: BT_HCI_OP_RESET failed: %d\n", ret);
      return ret;
    }

  bt_buf_release(rsp);

  /* Read Local Supported Features */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_FEATURES, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR: BT_HCI_OP_READ_LOCAL_FEATURES failed: %d\n", ret);
      return ret;
    }

  read_local_features_complete(rsp);
  bt_buf_release(rsp);

  /* Read Local Version Information */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_LOCAL_VERSION_INFO, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
      return ret;
    }

  read_local_ver_complete(rsp);
  bt_buf_release(rsp);

  /* Read Bluetooth Address */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BD_ADDR, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
      return ret;
    }

  read_bdaddr_complete(rsp);
  bt_buf_release(rsp);

  /* For now we only support LE capable controllers */

  if (!lmp_le_capable(g_btdev))
    {
      wlerr("ERROR:  Non-LE capable controller detected!\n");
      return -ENODEV;
    }

  /* Read Low Energy Supported Features */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_LOCAL_FEATURES, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  BT_HCI_OP_LE_READ_LOCAL_FEATURES failed: %d\n", ret);
      return ret;
    }

  read_le_features_complete(rsp);
  bt_buf_release(rsp);

  /* Read LE Buffer Size */

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_BUFFER_SIZE, NULL, &rsp);
  if (ret < 0)
    {
      wlerr("ERROR:  BT_HCI_OP_LE_READ_BUFFER_SIZE failed: %d\n", ret);
      return ret;
    }

  le_read_buffer_size_complete(rsp);
  bt_buf_release(rsp);

  buf = bt_hci_cmd_create(BT_HCI_OP_SET_EVENT_MASK, sizeof(*ev));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  ev = bt_buf_extend(buf, sizeof(*ev));
  memset(ev, 0, sizeof(*ev));

  ev->events[0] |= 0x10;        /* Disconnection Complete */
  ev->events[1] |= 0x08;        /* Read Remote Version Information Complete */
  ev->events[1] |= 0x20;        /* Command Complete */
  ev->events[1] |= 0x40;        /* Command Status */
  ev->events[1] |= 0x80;        /* Hardware Error */
  ev->events[2] |= 0x04;        /* Number of Completed Packets */
  ev->events[3] |= 0x02;        /* Data Buffer Overflow */
  ev->events[7] |= 0x20;        /* LE Meta-Event */

  if (g_btdev.le_features[0] & BT_HCI_LE_ENCRYPTION)
    {
      ev->events[0] |= 0x80;    /* Encryption Change */
      ev->events[5] |= 0x80;    /* Encryption Key Refresh Complete */
    }

  bt_hci_cmd_send_sync(BT_HCI_OP_SET_EVENT_MASK, buf, NULL);

  buf = bt_hci_cmd_create(BT_HCI_OP_HOST_BUFFER_SIZE, sizeof(*hbs));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  hbs = bt_buf_extend(buf, sizeof(*hbs));
  memset(hbs, 0, sizeof(*hbs));
  hbs->acl_mtu = BT_HOST2LE16(BLUETOOTH_MAX_FRAMELEN -
                              sizeof(struct bt_hci_acl_hdr_s) -
                              g_btdev.btdev->head_reserve);
  hbs->acl_pkts = BT_HOST2LE16(CONFIG_BLUETOOTH_BUFFER_PREALLOC);

  ret = bt_hci_cmd_send(BT_HCI_OP_HOST_BUFFER_SIZE, buf);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send failed: %d\n", ret);
      return ret;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_SET_CTL_TO_HOST_FLOW, 1);
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  enable  = bt_buf_extend(buf, sizeof(*enable));
  *enable = 0x01;

  ret = bt_hci_cmd_send_sync(BT_HCI_OP_SET_CTL_TO_HOST_FLOW, buf, NULL);
  if (ret < 0)
    {
      wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
      return ret;
    }

  if (lmp_bredr_capable(g_btdev))
    {
      FAR struct bt_hci_cp_write_le_host_supp_s *cp;

      /* Use BR/EDR buffer size if LE reports zero buffers */

      if (!g_btdev.le_mtu)
        {
          ret = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BUFFER_SIZE, NULL, &rsp);
          if (ret < 0)
            {
              wlerr("ERROR:  bt_hci_cmd_send_sync failed: %d\n", ret);
              return ret;
            }

          read_buffer_size_complete(rsp);
          bt_buf_release(rsp);
        }

      buf = bt_hci_cmd_create(BT_HCI_OP_LE_WRITE_LE_HOST_SUPP, sizeof(*cp));
      if (buf == NULL)
        {
          wlerr("ERROR:  Failed to create buffer\n");
          return -ENOBUFS;
        }

      /* Explicitly enable LE for dual-mode controllers */

      cp        = bt_buf_extend(buf, sizeof *cp);
      cp->le    = 0x01;
      cp->simul = 0x00;

      bt_hci_cmd_send_sync(BT_HCI_OP_LE_WRITE_LE_HOST_SUPP, buf, NULL);
    }

  wlinfo("HCI ver %u rev %u, manufacturer %u\n", g_btdev.hci_version,
         g_btdev.hci_revision, g_btdev.manufacturer);
  wlinfo("ACL buffers: pkts %u mtu %u\n", g_btdev.le_pkts, g_btdev.le_mtu);

  /* Initialize & prime the semaphore for counting controller-side available
   * ACL packet buffers.
   */

  nxsem_init(&g_btdev.le_pkts_sem, 0, g_btdev.le_pkts);
  return 0;
}
#endif

/* threads, fifos and semaphores initialization */

static void cmd_queue_init(void)
{
  int ret;

  /* When there is a command to be sent to the Bluetooth driver, it queued on
   * the Tx queue and received by logic on the Tx kernel thread.
   */

  ret = bt_queue_open(BT_HCI_TX, O_RDWR | O_CREAT,
                      CONFIG_BLUETOOTH_TXCMD_NMSGS, &g_btdev.tx_queue);
  DEBUGASSERT(ret >= 0);

  nxsem_init(&g_btdev.ncmd_sem, 0, 1);

  g_btdev.ncmd = 1;
  ret = kthread_create("BT HCI Tx", CONFIG_BLUETOOTH_TXCMD_PRIORITY,
                       CONFIG_BLUETOOTH_TXCMD_STACKSIZE,
                       hci_tx_kthread, NULL);
  DEBUGASSERT(ret > 0);
  UNUSED(ret);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_send
 *
 * Description:
 *   Send the provided buffer to the bluetooth driver
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *   buf   - The buffer to be sent by the driver
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure.
 *
 ****************************************************************************/

int bt_send(FAR struct bt_driver_s *btdev,
            FAR struct bt_buf_s *buf)
{
  /* Send to driver */

  return btdev->send(btdev, buf->type, buf->data, buf->len);
}

/****************************************************************************
 * Name: bt_initialize
 *
 * Description:
 *   Initialize Bluetooth. Must be the called before anything else.
 *
 * Returned Value:
 *    Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_initialize(void)
{
  FAR struct bt_driver_s *btdev = g_btdev.btdev;
  int ret;

  wlinfo("btdev %p\n", btdev);

  DEBUGASSERT(btdev != NULL);
  bt_buf_initialize();

  cmd_queue_init();

  ret = btdev->open(btdev);
  if (ret < 0)
    {
      wlerr("ERROR: HCI driver open failed (%d)\n", ret);
      return ret;
    }

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST
  ret = hci_initialize();
  if (ret < 0)
    {
      wlerr("ERROR:  hci_initialize failed: %d\n", ret);
      return ret;
    }

  ret = bt_l2cap_init();
#endif

  return ret;
}

/****************************************************************************
 * Name: bt_driver_register
 *
 * Description:
 *   Register the Bluetooth low-level driver with the Bluetooth stack.
 *   This is called from the low-level driver and is part of the driver
 *   interface prototyped in include/nuttx/wireless/bluetooth/bt_driver.h
 *
 *   This function associates the Bluetooth driver with the Bluetooth stack.
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *  Zero is returned on success; a negated errno value is returned on any
 *  failure.
 *
 ****************************************************************************/

int bt_driver_register(FAR struct bt_driver_s *btdev)
{
  DEBUGASSERT(btdev != NULL && btdev->open != NULL && btdev->send != NULL);

  if (g_btdev.btdev != NULL)
    {
      wlwarn("WARNING:  Already registered\n");
      return -EALREADY;
    }

  g_btdev.btdev = btdev;
  return 0;
}

/****************************************************************************
 * Name: bt_driver_unregister
 *
 * Description:
 *   Unregister a Bluetooth low-level driver previously registered with
 *   bt_driver_register.  This may be called from the low-level driver and
 *   is part of the driver interface prototyped in
 *   include/nuttx/wireless/bluetooth/bt_driver.h
 *
 * Input Parameters:
 *   btdev - An instance of the low-level drivers interface structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void bt_driver_unregister(FAR struct bt_driver_s *btdev)
{
  g_btdev.btdev = NULL;
}

/****************************************************************************
 * Name: bt_receive
 *
 * Description:
 *   Called by the Bluetooth low-level driver when new data is received from
 *   the radio.  This may be called from the low-level driver and is part of
 *   the driver interface prototyped in
 *   include/nuttx/wireless/bluetooth/bt_driver.h
 *
 *   NOTE:  This function will defer all real work to the low or to the high
 *   priority work queues.  Therefore, this function may safely be called
 *   from interrupt handling logic.
 *
 * Input Parameters:
 *   buf - An instance of the buffer structure providing the received frame.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

int bt_receive(FAR struct bt_driver_s *btdev, enum bt_buf_type_e type,
               FAR void *data, size_t len)
{
  FAR struct bt_hci_evt_hdr_s *hdr;
  struct bt_buf_s *buf;
  int ret;

  wlinfo("data %p len %zu\n", data, len);

  /* Critical command complete/status events use the high priority work
   * queue.
   */

  buf = bt_buf_alloc(type, NULL, BLUETOOTH_H4_HDRLEN);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  memcpy(bt_buf_extend(buf, len), data, len);

  if (type != BT_ACL_IN)
    {
      if (type != BT_EVT)
        {
          wlerr("ERROR: Invalid buf type %u\n", buf->type);
          bt_buf_release(buf);
          return -EINVAL;
        }

      /* Command Complete/Status events use high priority messages. */

      hdr = (FAR void *)buf->data;
      if (hdr->evt == BT_HCI_EVT_CMD_COMPLETE ||
          hdr->evt == BT_HCI_EVT_CMD_STATUS ||
          hdr->evt == BT_HCI_EVT_NUM_COMPLETED_PACKETS)
        {
          /* Add the buffer to the high priority Rx buffer list */

          bt_enqueue_bufwork(&g_hp_rxlist, buf);

          /* If there is already pending work, then do nothing.  Otherwise,
           * schedule processing of the Rx buffer list on the high priority
           * work queue.
           */

          if (work_available(&g_hp_work))
            {
              ret = work_queue(HPWORK, &g_hp_work, priority_rx_work,
                               &g_hp_rxlist, 0);
              if (ret < 0)
                {
                  wlerr("ERROR:  Failed to schedule HPWORK: %d\n", ret);
                }
            }

          return OK;
        }
    }

  /* All others use the low priority work queue */

  /* Add the buffer to the low priority Rx buffer list */

  bt_enqueue_bufwork(&g_lp_rxlist, buf);

  /* If there is already pending work, then do nothing.  Otherwise, schedule
   * processing of the Rx buffer list on the low priority work queue.
   */

  if (work_available(&g_lp_work))
    {
      ret = work_queue(LPWORK, &g_lp_work, hci_rx_work, &g_lp_rxlist, 0);
      if (ret < 0)
        {
          wlerr("ERROR:  Failed to schedule LPWORK: %d\n", ret);
        }
    }

  return OK;
}

#ifdef CONFIG_WIRELESS_BLUETOOTH_HOST

/****************************************************************************
 * Name: bt_hci_cmd_create
 *
 * Description:
 *   Allocate and initialize a buffer for a command
 *
 * Returned Value:
 *   A reference to the allocated buffer.  NULL could possibly be returned
 *   on any failure to allocate.
 *
 ****************************************************************************/

FAR struct bt_buf_s *bt_hci_cmd_create(uint16_t opcode, uint8_t param_len)
{
  FAR struct bt_hci_cmd_hdr_s *hdr;
  FAR struct bt_buf_s *buf;

  wlinfo("opcode %04x param_len %u\n", opcode, param_len);

  buf = bt_buf_alloc(BT_CMD, NULL, g_btdev.btdev->head_reserve);
  if (!buf)
    {
      wlerr("ERROR: Cannot get free buffer\n");
      return NULL;
    }

  wlinfo("buf %p\n", buf);

  buf->u.hci.opcode = opcode;
  buf->u.hci.sync   = NULL;

  hdr              = bt_buf_extend(buf, sizeof(*hdr));
  hdr->opcode      = BT_HOST2LE16(opcode);
  hdr->param_len   = param_len;

  return buf;
}

int bt_hci_cmd_send(uint16_t opcode, FAR struct bt_buf_s *buf)
{
  int ret;

  if (buf == NULL)
    {
      buf = bt_hci_cmd_create(opcode, 0);
      if (buf == NULL)
        {
          wlerr("ERROR:  Failed to create buffer\n");
          return -ENOBUFS;
        }
    }

  wlinfo("opcode %04x len %u\n", opcode, buf->len);

  /* Host Number of Completed Packets can ignore the ncmd value and does not
   * generate any cmd complete/status events.
   */

  if (opcode == BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS)
    {
      bt_send(g_btdev.btdev, buf);
      bt_buf_release(buf);
      return 0;
    }

  ret = bt_queue_send(&g_btdev.tx_queue, buf, BT_NORMAL_PRIO);
  if (ret < 0)
    {
      wlerr("ERROR: bt_queue_send() failed: %d\n", ret);
    }

  return ret;
}

int bt_hci_cmd_send_sync(uint16_t opcode, FAR struct bt_buf_s *buf,
                         FAR struct bt_buf_s **rsp)
{
  sem_t sync_sem;
  int ret;

  /* NOTE: This function cannot be called from the rx thread since it relies
   * on the very same thread in processing the cmd_complete event and giving
   * back the blocking semaphore.
   */

  if (buf == NULL)
    {
      buf = bt_hci_cmd_create(opcode, 0);
      if (buf == NULL)
        {
          wlerr("ERROR:  Failed to create buffer\n");
          return -ENOBUFS;
        }
    }

  wlinfo("opcode %04x len %u\n", opcode, buf->len);

  /* Set up for the wait */

  nxsem_init(&sync_sem, 0, 0);
  buf->u.hci.sync = &sync_sem;

  /* Send the frame */

  ret = bt_queue_send(&g_btdev.tx_queue, buf, BT_NORMAL_PRIO);
  if (ret < 0)
    {
      wlerr("ERROR: bt_queue_send() failed: %d\n", ret);
    }
  else
    {
      /* Wait for the response to the command.  An I/O error will be
       * declared if the response does not occur within the timeout
       * interval.
       *
       * REVISIT: The cause of the timeout could be a failure to receive a
       * response to a sent frame or, perhaps, a failure to send the frame.
       * Should there also be logic to flush any unsent Tx packets?
       */

      ret = nxsem_tickwait_uninterruptible(&sync_sem,
                                           MSEC2TICK(TIMEOUT_MSEC));
    }

  /* Indicate failure if we failed to get the response */

  if (ret >= 0)
    {
      if (buf->u.hci.sync == NULL)
        {
          wlerr("ERROR:  Failed get return parameters\n");
          ret = -EIO;
        }
      else
        {
          ret = 0;
        }
    }

  /* Note: if ret < 0 the packet might just be delayed and could still
   * be sent.  We cannot decrease the ref count since it if it was sent
   * it buf could be pointed a completely different request.
   */

  if (rsp != NULL)
    {
      /* If the response is expected provide the sync response */

      *rsp = buf->u.hci.sync;
    }
  else if (buf->u.hci.sync != NULL)
    {
      /* If a sync response was given but not requested drop it */

      bt_buf_release(buf->u.hci.sync);
    }

  nxsem_destroy(&sync_sem);

  return ret;
}

/****************************************************************************
 * Name: bt_start_advertising
 *
 * Description:
 *   Set advertisement data, scan response data, advertisement parameters
 *   and start advertising.
 *
 * Input Parameters:
 *   type - Advertising type.
 *   ad   - Data to be used in advertisement packets.
 *   sd   - Data to be used in scan response packets.
 *
 * Returned Value:
 *   Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_start_advertising(uint8_t type, FAR const struct bt_eir_s *ad,
                         FAR const struct bt_eir_s *sd)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_hci_cp_le_set_adv_data_s *set_data;
  FAR struct bt_hci_cp_le_set_adv_data_s *scan_rsp;
  FAR struct bt_hci_cp_le_set_adv_parameters_s *set_param;
  int i;

  if (ad == NULL)
    {
      goto send_scan_rsp;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_DATA, sizeof(*set_data));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  set_data = bt_buf_extend(buf, sizeof(*set_data));

  memset(set_data, 0, sizeof(*set_data));

  for (i = 0; ad[i].len > 0; i++)
    {
      /* Check if ad fit in the remaining buffer */

      if (set_data->len + ad[i].len + 1 > 29)
        {
          break;
        }

      memcpy(&set_data->data[set_data->len], &ad[i], ad[i].len + 1);
      set_data->len += ad[i].len + 1;
    }

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_DATA, buf);

send_scan_rsp:
  if (sd == NULL)
    {
      goto send_set_param;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_SCAN_RSP_DATA,
                          sizeof(*scan_rsp));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  scan_rsp = bt_buf_extend(buf, sizeof(*scan_rsp));

  memset(scan_rsp, 0, sizeof(*scan_rsp));

  for (i = 0; sd[i].len > 0; i++)
    {
      /* Check if ad fit in the remaining buffer */

      if (scan_rsp->len + sd[i].len + 1 > 29)
        {
          break;
        }

      memcpy(&scan_rsp->data[scan_rsp->len], &sd[i], sd[i].len + 1);
      scan_rsp->len += sd[i].len + 1;
    }

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_SCAN_RSP_DATA, buf);

send_set_param:
  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_PARAMETERS,
                          sizeof(*set_param));
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  set_param = bt_buf_extend(buf, sizeof(*set_param));

  memset(set_param, 0, sizeof(*set_param));
  set_param->min_interval = BT_HOST2LE16(300);
  set_param->max_interval = BT_HOST2LE16(300);
  set_param->type         = type;
  set_param->channel_map  = 0x07;

  bt_hci_cmd_send(BT_HCI_OP_LE_SET_ADV_PARAMETERS, buf);

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  g_btdev.adv_enable = 0x01;
  memcpy(bt_buf_extend(buf, 1), &g_btdev.adv_enable, 1);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
}

/****************************************************************************
 * Name: bt_stop_advertising
 *
 * Description:
 *   Stops ongoing advertising.
 *
 * Returned Value:
 *   Zero on success or (negative) error code otherwise.
 *
 ****************************************************************************/

int bt_stop_advertising(void)
{
  FAR struct bt_buf_s *buf;

  if (!g_btdev.adv_enable)
    {
      wlwarn("WARNING:  Already advertising\n");
      return -EALREADY;
    }

  buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_ADV_ENABLE, 1);
  if (buf == NULL)
    {
      wlerr("ERROR:  Failed to create buffer\n");
      return -ENOBUFS;
    }

  g_btdev.adv_enable = 0x00;
  memcpy(bt_buf_extend(buf, 1), &g_btdev.adv_enable, 1);

  return bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_ADV_ENABLE, buf, NULL);
}

/****************************************************************************
 * Name: bt_start_scanning
 *
 * Description:
 *   Start LE scanning with and provide results through the specified
 *   callback.
 *
 * Input Parameters:
 *   filter_dups - Enable duplicate filtering (or not).
 *   cb          - Callback to notify scan results.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_start_scanning(uint8_t scan_filter, bt_le_scan_cb_t cb)
{
  /* Return if active scan is already enabled */

  if (g_scan_dev_found_cb)
    {
      wlwarn("WARNING:  Already scanning\n");
      return -EALREADY;
    }

  g_scan_dev_found_cb = cb;
  g_btdev.scan_filter = scan_filter;

  return bt_le_scan_update();
}

/****************************************************************************
 * Name: bt_stop_scanning
 *
 * Description:
 *   Stops ongoing LE scanning.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_stop_scanning(void)
{
  /* Return if active scanning is already disabled */

  if (g_scan_dev_found_cb == NULL)
    {
      wlwarn("WARNING:  Not scanning\n");
      return -EALREADY;
    }

  g_scan_dev_found_cb = NULL;
  g_btdev.scan_filter = BT_LE_SCAN_FILTER_DUP_ENABLE;

  return bt_le_scan_update();
}

/****************************************************************************
 * Name: bt_le_scan_update
 *
 * Description:
 *   Used to determine whether to start scan and which scan type should be
 *   used.
 *
 * Returned Value:
 *   Zero on success or error code otherwise, positive in case
 *   of protocol error or negative (POSIX) in case of stack internal error
 *
 ****************************************************************************/

int bt_le_scan_update(void)
{
  FAR struct bt_conn_s *conn;
  int ret;

  if (g_btdev.scan_enable)
    {
      if (g_scan_dev_found_cb)
        {
          return 0;
        }

      ret = bt_hci_stop_scanning();
      if (ret)
        {
          return ret;
        }
    }

  if (g_scan_dev_found_cb)
    {
      return bt_hci_start_scanning(BT_LE_SCAN_ACTIVE, g_btdev.scan_filter);
    }

  conn = bt_conn_lookup_state(BT_ADDR_LE_ANY, BT_CONN_CONNECT_SCAN);
  if (!conn)
    {
      return 0;
    }

  bt_conn_release(conn);
  return bt_hci_start_scanning(BT_LE_SCAN_PASSIVE, g_btdev.scan_filter);
}

/****************************************************************************
 * Name: bt_conn_cb_register
 *
 * Description:
 *   Register callbacks to monitor the state of connections.
 *
 * Input Parameters:
 *   cb - Instance of the callback structure.
 *
 ****************************************************************************/

void bt_conn_cb_register(FAR struct bt_conn_cb_s *cb)
{
  cb->flink       = g_callback_list;
  g_callback_list = cb;
}

FAR const char *bt_addr_str(FAR const bt_addr_t *addr)
{
  static char bufs[2][18];
  static uint8_t cur;
  FAR char *str;

  str  = bufs[cur++];
  cur %= ARRAY_SIZE(bufs);
  bt_addr_to_str(addr, str, sizeof(bufs[cur]));

  return str;
}

FAR const char *bt_addr_le_str(FAR const bt_addr_le_t *addr)
{
  static char bufs[2][27];
  static uint8_t cur;
  FAR char *str;

  str  = bufs[cur++];
  cur %= ARRAY_SIZE(bufs);
  bt_addr_le_to_str(addr, str, sizeof(bufs[cur]));

  return str;
}

#else

/****************************************************************************
 * Name: bt_hci_cb_register
 *
 * Description:
 *   Register callbacks to handle RAW HCI packets
 *
 * Input Parameters:
 *   cb - Instance of the callback structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bt_hci_cb_register(FAR struct bt_hci_cb_s *cb)
{
  g_hci_cb = cb;
}

#endif

