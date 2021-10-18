/****************************************************************************
 * wireless/bluetooth/bt_l2cap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>

#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_l2cap.h"
#include "bt_att.h"
#include "bt_smp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LE_CONN_MIN_INTERVAL         0x0028
#define LE_CONN_MAX_INTERVAL         0x0038
#define LE_CONN_LATENCY              0x0000
#define LE_CONN_TIMEOUT              0x002a

#define BT_L2CAP_CONN_PARAM_ACCEPTED 0
#define BT_L2CAP_CONN_PARAM_REJECTED 1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct bt_l2cap_chan_s *g_channels;
static FAR struct bt_l2cap_chan_s *g_default;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t get_ident(FAR struct bt_conn_s *conn)
{
  conn->l2cap.ident++;

  /* Handle integer overflow (0 is not valid) */

  if (!conn->l2cap.ident)
    {
      conn->l2cap.ident++;
    }

  return conn->l2cap.ident;
}

void bt_l2cap_chan_register(FAR struct bt_l2cap_chan_s *chan)
{
  wlinfo("CID 0x%04x\n", chan->cid);

  chan->flink = g_channels;
  g_channels  = chan;
}

void bt_l2cap_chan_default(FAR struct bt_l2cap_chan_s *chan)
{
  g_default  = chan;
}

void bt_l2cap_connected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_l2cap_chan_s *chan;

  /* Notify all registered channels of the connection event */

  for (chan = g_channels; chan; chan = chan->flink)
    {
      if (chan->connected != NULL)
        {
          chan->connected(conn, chan->context, chan->cid);
        }
    }

  /* Notify any default listener of the connection event */

  chan = g_default;
  if (chan != NULL && chan->connected != NULL)
    {
      chan->connected(conn, chan->context, chan->cid);
    }
}

void bt_l2cap_disconnected(FAR struct bt_conn_s *conn)
{
  FAR struct bt_l2cap_chan_s *chan;

  /* Notify all registered channels of the disconnection event */

  for (chan = g_channels; chan; chan = chan->flink)
    {
      if (chan->disconnected != NULL)
        {
          chan->disconnected(conn, chan->context, chan->cid);
        }
    }

  /* Notify any default listener of the disconnection event */

  chan = g_default;
  if (chan != NULL && chan->disconnected != NULL)
    {
      chan->disconnected(conn, chan->context, chan->cid);
    }
}

void bt_l2cap_encrypt_change(FAR struct bt_conn_s *conn)
{
  FAR struct bt_l2cap_chan_s *chan;

  /* Notify all registered channels of the encryption change event */

  for (chan = g_channels; chan; chan = chan->flink)
    {
      if (chan->encrypt_change != NULL)
        {
          chan->encrypt_change(conn, chan->context, chan->cid);
        }
    }

  /* Notify any default listener of the encryption change event */

  chan = g_default;
  if (chan != NULL && chan->encrypt_change != NULL)
    {
      chan->encrypt_change(conn, chan->context, chan->cid);
    }
}

struct bt_buf_s *bt_l2cap_create_pdu(FAR struct bt_conn_s *conn)
{
  size_t head_reserve = sizeof(struct bt_l2cap_hdr_s) +
    sizeof(struct bt_hci_acl_hdr_s) + g_btdev.btdev->head_reserve;

  return bt_buf_alloc(BT_ACL_OUT, NULL, head_reserve);
}

void bt_l2cap_send(FAR struct bt_conn_s *conn, uint16_t cid,
                   FAR struct bt_buf_s *buf)
{
  FAR struct bt_l2cap_hdr_s *hdr;

  hdr      = bt_buf_provide(buf, sizeof(*hdr));
  hdr->len = BT_HOST2LE16(buf->len - sizeof(*hdr));
  hdr->cid = BT_HOST2LE16(cid);

  bt_conn_send(conn, buf);
}

static void rej_not_understood(FAR struct bt_conn_s *conn, uint8_t ident)
{
  FAR struct bt_l2cap_cmd_reject_s *rej;
  FAR struct bt_l2cap_sig_hdr_s *hdr;
  FAR struct bt_buf_s *buf;

  buf = bt_l2cap_create_pdu(conn);
  if (!buf)
    {
      return;
    }

  hdr         = bt_buf_extend(buf, sizeof(*hdr));
  hdr->code   = BT_L2CAP_CMD_REJECT;
  hdr->ident  = ident;
  hdr->len    = BT_HOST2LE16(sizeof(*rej));

  rej         = bt_buf_extend(buf, sizeof(*rej));
  rej->reason = BT_HOST2LE16(BT_L2CAP_REJ_NOT_UNDERSTOOD);

  bt_l2cap_send(conn, BT_L2CAP_CID_LE_SIG, buf);
}

static void le_conn_param_rsp(FAR struct bt_conn_s *conn,
                              FAR struct bt_buf_s *buf)
{
  struct bt_l2cap_conn_param_rsp_s *rsp = (void *)buf->data;

  if (buf->len < sizeof(*rsp))
    {
      wlerr("ERROR: Too small LE conn param rsp\n");
      return;
    }

  wlinfo("LE conn param rsp result %u\n", BT_LE162HOST(rsp->result));
}

static uint16_t le_validate_conn_params(uint16_t min, uint16_t max,
                                        uint16_t latency, uint16_t timeout)
{
  uint16_t max_latency;

  if (min > max || min < 6 || max > 3200)
    {
      return BT_L2CAP_CONN_PARAM_REJECTED;
    }

  if (timeout < 10 || timeout > 3200)
    {
      return BT_L2CAP_CONN_PARAM_REJECTED;
    }

  /* Calculation based on BT spec 4.2 [Vol3, PartA, 4.20] max_latency =
   * ((timeout * 10)/(max * 1.25 * 2)) - 1;
   */

  max_latency = (timeout * 4 / max) - 1;
  if (latency > 499 || latency > max_latency)
    {
      return BT_L2CAP_CONN_PARAM_REJECTED;
    }

  return BT_L2CAP_CONN_PARAM_ACCEPTED;
}

static void le_conn_param_update_req(FAR struct bt_conn_s *conn,
                                     uint8_t ident,
                                     FAR struct bt_buf_s *buf)
{
  FAR struct bt_l2cap_sig_hdr_s *hdr;
  FAR struct bt_l2cap_conn_param_rsp_s *rsp;
  FAR struct bt_l2cap_conn_param_req_s *req = (void *)buf->data;
  uint16_t min;
  uint16_t max;
  uint16_t latency;
  uint16_t timeout;
  uint16_t result;

  if (buf->len < sizeof(*req))
    {
      wlerr("ERROR: Too small LE conn update param req\n");
      return;
    }

  if (conn->role != BT_HCI_ROLE_MASTER)
    {
      return;
    }

  min     = BT_LE162HOST(req->min_interval);
  max     = BT_LE162HOST(req->max_interval);
  latency = BT_LE162HOST(req->latency);
  timeout = BT_LE162HOST(req->timeout);

  wlinfo("min 0x%4.4x max 0x%4.4x latency: 0x%4.4x timeout: 0x%4.4x",
         min, max, latency, timeout);

  buf = bt_l2cap_create_pdu(conn);
  if (!buf)
    {
      return;
    }

  result      = le_validate_conn_params(min, max, latency, timeout);

  hdr         = bt_buf_extend(buf, sizeof(*hdr));
  hdr->code   = BT_L2CAP_CONN_PARAM_RSP;
  hdr->ident  = ident;
  hdr->len    = BT_HOST2LE16(sizeof(*rsp));

  rsp         = bt_buf_extend(buf, sizeof(*rsp));
  memset(rsp, 0, sizeof(*rsp));
  rsp->result = BT_HOST2LE16(result);

  bt_l2cap_send(conn, BT_L2CAP_CID_LE_SIG, buf);

  if (result == BT_L2CAP_CONN_PARAM_ACCEPTED)
    {
      bt_conn_le_conn_update(conn, min, max, latency, timeout);
    }
}

static void le_sig(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                   FAR void *context, uint16_t cid)
{
  struct bt_l2cap_sig_hdr_s *hdr = (FAR void *)buf->data;
  uint16_t len;

  if (buf->len < sizeof(*hdr))
    {
      wlerr("ERROR: Too small L2CAP LE signaling PDU\n");
      goto drop;
    }

  len = BT_LE162HOST(hdr->len);
  bt_buf_consume(buf, sizeof(*hdr));

  wlinfo("LE signaling code 0x%02x ident %u len %u\n", hdr->code,
         hdr->ident, len);

  if (buf->len != len)
    {
      wlerr("ERROR: L2CAP length mismatch (%u != %u)\n", buf->len, len);
      goto drop;
    }

  if (!hdr->ident)
    {
      wlerr("ERROR: Invalid ident value in L2CAP PDU\n");
      goto drop;
    }

  switch (hdr->code)
    {
    case BT_L2CAP_CONN_PARAM_RSP:
      le_conn_param_rsp(conn, buf);
      break;

    case BT_L2CAP_CONN_PARAM_REQ:
      le_conn_param_update_req(conn, hdr->ident, buf);
      break;

    default:
      wlwarn("Unknown L2CAP PDU code 0x%02x\n", hdr->code);
      rej_not_understood(conn, hdr->ident);
      break;
    }

drop:
  bt_buf_release(buf);
}

void bt_l2cap_receive(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf)
{
  FAR struct bt_l2cap_hdr_s *hdr = (FAR void *)buf->data;
  FAR struct bt_l2cap_chan_s *chan;
  uint16_t cid;

  if (buf->len < sizeof(*hdr))
    {
      wlerr("ERROR: Too small L2CAP PDU received\n");
      bt_buf_release(buf);
      return;
    }

  cid = BT_LE162HOST(hdr->cid);
  bt_buf_consume(buf, sizeof(*hdr));

  wlinfo("Packet for CID %u len %u\n", cid, buf->len);

  /* Search for a subscriber to this channel */

  for (chan = g_channels; chan != NULL; chan = chan->flink)
    {
      if (chan->cid == cid)
        {
          break;
        }
    }

  /* If there is no subscriber, then send all received frames to the default
   * listener (if one is registered).
   */

  if (chan == NULL)
    {
      chan = g_default;
    }

  if (chan == NULL)
    {
      wlwarn("WARNING: No subscriber to CID 0x%04x\n", cid);
      bt_buf_release(buf);
      return;
    }

  chan->receive(conn, buf, chan->context, cid);
}

void bt_l2cap_update_conn_param(FAR struct bt_conn_s *conn)
{
  FAR struct bt_l2cap_sig_hdr_s *hdr;
  FAR struct bt_l2cap_conn_param_req_s *req;
  FAR struct bt_buf_s *buf;

  /* Check if we need to update anything */

  if (conn->le_conn_interval >= LE_CONN_MIN_INTERVAL &&
      conn->le_conn_interval <= LE_CONN_MAX_INTERVAL)
    {
      return;
    }

  buf = bt_l2cap_create_pdu(conn);
  if (!buf)
    {
      return;
    }

  hdr               = bt_buf_extend(buf, sizeof(*hdr));
  hdr->code         = BT_L2CAP_CONN_PARAM_REQ;
  hdr->ident        = get_ident(conn);
  hdr->len          = BT_HOST2LE16(sizeof(*req));

  req               = bt_buf_extend(buf, sizeof(*req));
  req->min_interval = BT_HOST2LE16(LE_CONN_MIN_INTERVAL);
  req->max_interval = BT_HOST2LE16(LE_CONN_MAX_INTERVAL);
  req->latency      = BT_HOST2LE16(LE_CONN_LATENCY);
  req->timeout      = BT_HOST2LE16(LE_CONN_TIMEOUT);

  bt_l2cap_send(conn, BT_L2CAP_CID_LE_SIG, buf);
}

int bt_l2cap_init(void)
{
  int ret;

  static struct bt_l2cap_chan_s chan =
  {
    .cid     = BT_L2CAP_CID_LE_SIG,
    .receive = le_sig,
  };

  bt_att_initialize();

  ret = bt_smp_initialize();
  if (ret < 0)
    {
      return ret;
    }

  bt_l2cap_chan_register(&chan);
  return ret;
}
