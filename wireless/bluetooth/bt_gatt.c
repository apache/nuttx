/****************************************************************************
 * wireless/bluetooth/bt_gatt.c
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

#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_buf.h>
#include <nuttx/wireless/bluetooth/bt_uuid.h>
#include <nuttx/wireless/bluetooth/bt_gatt.h>

#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_keys.h"
#include "bt_l2cap.h"
#include "bt_att.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct gatt_incl_s
{
  uint16_t start_handle;
  uint16_t end_handle;
  union
  {
    uint16_t uuid16;
    uint8_t uuid[16];
  } u;
} end_packed_struct;

begin_packed_struct struct gatt_chrc_s
{
  uint8_t properties;
  uint16_t value_handle;
  union
  {
    uint16_t uuid16;
    uint8_t uuid[16];
  } u;
} end_packed_struct;

struct notify_data_s
{
  FAR const void *data;
  size_t len;
  uint8_t handle;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct bt_gatt_attr_s *g_db = NULL;
static size_t g_attr_count = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_gatt_register(FAR const struct bt_gatt_attr_s *attrs, size_t count)
{
  g_db         = attrs;
  g_attr_count = count;
}

int bt_gatt_attr_read(FAR struct bt_conn_s *conn,
                      FAR const struct bt_gatt_attr_s *attr, FAR void *buf,
                      uint8_t buf_len, uint16_t offset,
                      FAR const void *value, uint8_t value_len)
{
  uint8_t maxlen;
  uint8_t len;

  if (offset > value_len)
    {
      return -EINVAL;
    }

  len    = buf_len;
  maxlen = value_len - offset;
  if (len > maxlen)
    {
      len = maxlen;
    }

  wlinfo("handle 0x%04x offset %u length %u\n", attr->handle, offset, len);

  memcpy(buf, value + offset, len);
  return len;
}

int bt_gatt_attr_read_service(FAR struct bt_conn_s *conn,
                              FAR const struct bt_gatt_attr_s *attr,
                              FAR void *buf, uint8_t len, uint16_t offset)
{
  FAR struct bt_uuid_s *uuid = attr->user_data;

  if (uuid->type == BT_UUID_16)
    {
      uint16_t uuid16 = BT_HOST2LE16(uuid->u.u16);

      return bt_gatt_attr_read(conn, attr, buf, len, offset, &uuid16,
                               sizeof(uuid16));
    }

  return bt_gatt_attr_read(conn, attr, buf, len, offset, uuid->u.u128,
                           sizeof(uuid->u.u128));
}

int bt_gatt_attr_read_included(FAR struct bt_conn_s *conn,
                               FAR const struct bt_gatt_attr_s *attr,
                               FAR void *buf, uint8_t len, uint16_t offset)
{
  FAR struct bt_gatt_include_s *incl = attr->user_data;
  struct gatt_incl_s pdu;
  uint8_t value_len;

  pdu.start_handle = BT_HOST2LE16(incl->start_handle);
  pdu.end_handle   = BT_HOST2LE16(incl->end_handle);
  value_len        = sizeof(pdu.start_handle) + sizeof(pdu.end_handle);

  if (incl->uuid->type == BT_UUID_16)
    {
      pdu.u.uuid16 = BT_HOST2LE16(incl->uuid->u.u16);
      value_len += sizeof(pdu.u.uuid16);
    }
  else
    {
      memcpy(pdu.u.uuid, incl->uuid->u.u128, sizeof(incl->uuid->u.u128));
      value_len += sizeof(incl->uuid->u.u128);
    }

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pdu, value_len);
}

int bt_gatt_attr_read_chrc(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR void *buf, uint8_t len, uint16_t offset)
{
  FAR struct bt_gatt_chrc_s *chrc = attr->user_data;
  struct gatt_chrc_s pdu;
  uint8_t value_len;

  pdu.properties   = chrc->properties;
  pdu.value_handle = BT_HOST2LE16(chrc->value_handle);
  value_len        = sizeof(pdu.properties) + sizeof(pdu.value_handle);

  if (chrc->uuid->type == BT_UUID_16)
    {
      pdu.u.uuid16 = BT_HOST2LE16(chrc->uuid->u.u16);
      value_len += sizeof(pdu.u.uuid16);
    }
  else
    {
      memcpy(pdu.u.uuid, chrc->uuid->u.u128, sizeof(chrc->uuid->u.u128));
      value_len += sizeof(chrc->uuid->u.u128);
    }

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &pdu, value_len);
}

void bt_gatt_foreach_attr(uint16_t start_handle, uint16_t end_handle,
                          bt_gatt_attr_func_t func, FAR void *user_data)
{
  size_t i;

  for (i = 0; i < g_attr_count; i++)
    {
      FAR const struct bt_gatt_attr_s *attr = &g_db[i];

      /* Check if attribute handle is within range */

      if (attr->handle < start_handle || attr->handle > end_handle)
        {
          continue;
        }

      if (func(attr, user_data) == BT_GATT_ITER_STOP)
        {
          break;
        }
    }
}

int bt_gatt_attr_read_ccc(FAR struct bt_conn_s *conn,
                          FAR const struct bt_gatt_attr_s *attr,
                          FAR void *buf, uint8_t len, uint16_t offset)
{
  FAR struct _bt_gatt_ccc_s *ccc = attr->user_data;
  uint16_t value;
  size_t i;

  for (i = 0; i < ccc->cfg_len; i++)
    {
      if (bt_addr_le_cmp(&ccc->cfg[i].peer, &conn->dst))
        {
          continue;
        }

      value = BT_HOST2LE16(ccc->cfg[i].value);
      break;
    }

  /* Default to disable if there is no cfg for the peer */

  if (i == ccc->cfg_len)
    {
      value = 0x0000;
    }

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,
                           sizeof(value));
}

static void gatt_ccc_changed(FAR struct _bt_gatt_ccc_s *ccc)
{
  uint16_t value = 0x0000;
  int i;

  for (i = 0; i < ccc->cfg_len; i++)
    {
      if (ccc->cfg[i].value > value)
        {
          value = ccc->cfg[i].value;
        }
    }

  wlinfo("ccc %p value 0x%04x\n", ccc, value);

  if (value != ccc->value)
    {
      ccc->value = value;
      ccc->cfg_changed(value);
    }
}

int bt_gatt_attr_write_ccc(FAR struct bt_conn_s *conn,
                           FAR const struct bt_gatt_attr_s *attr,
                           FAR const void *buf, uint8_t len,
                           uint16_t offset)
{
  FAR struct _bt_gatt_ccc_s *ccc = attr->user_data;
  FAR const uint16_t *data = buf;
  bool bonded;
  size_t i;

  if (len != sizeof(*data) || offset)
    {
      return -EINVAL;
    }

  if (bt_keys_get_addr(&conn->dst))
    {
      bonded = true;
    }
  else
    {
      bonded = false;
    }

  for (i = 0; i < ccc->cfg_len; i++)
    {
      /* Check for existing configuration */

      if (!bt_addr_le_cmp(&ccc->cfg[i].peer, &conn->dst))
        {
          break;
        }
    }

  if (i == ccc->cfg_len)
    {
      for (i = 0; i < ccc->cfg_len; i++)
        {
          /* Check for unused configuration */

          if (!ccc->cfg[i].valid)
            {
              bt_addr_le_copy(&ccc->cfg[i].peer, &conn->dst);

              /* Only set valid if bonded */

              ccc->cfg[i].valid = bonded;
              break;
            }
        }

      if (i == ccc->cfg_len)
        {
          wlwarn("No space to store CCC cfg\n");
          return -ENOMEM;
        }
    }

  ccc->cfg[i].value = BT_LE162HOST(*data);

  wlinfo("handle 0x%04x value %u\n", attr->handle, ccc->cfg[i].value);

  /* Update cfg if don't match */

  if (ccc->cfg[i].value != ccc->value)
    {
      gatt_ccc_changed(ccc);
    }

  return len;
}

int bt_gatt_attr_read_cep(FAR struct bt_conn_s *conn,
                          FAR const struct bt_gatt_attr_s *attr,
                          FAR void *buf, uint8_t len, uint16_t offset)
{
  FAR struct bt_gatt_cep_s *value = attr->user_data;
  uint16_t props = BT_HOST2LE16(value->properties);

  return bt_gatt_attr_read(conn, attr, buf, len, offset, &props,
                           sizeof(props));
}

static uint8_t notify_cb(FAR const struct bt_gatt_attr_s *attr,
                         FAR void *user_data)
{
  FAR struct notify_data_s *data = user_data;
  const struct bt_uuid_s uuid =
  {
    BT_UUID_16,
    {
      BT_UUID_GATT_CCC
    }
  };

  const struct bt_uuid_s chrc =
  {
    BT_UUID_16,
    {
      BT_UUID_GATT_CHRC
    }
  };

  FAR struct _bt_gatt_ccc_s *ccc;
  size_t i;

  if (bt_uuid_cmp(attr->uuid, &uuid))
    {
      /* Stop if we reach the next characteristic */

      if (!bt_uuid_cmp(attr->uuid, &chrc))
        {
          return BT_GATT_ITER_STOP;
        }

      return BT_GATT_ITER_CONTINUE;
    }

  /* Check attribute user_data must be of type struct _bt_gatt_ccc_s */

  if (attr->write != bt_gatt_attr_write_ccc)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  ccc = attr->user_data;

  /* Notify all peers configured */

  for (i = 0; i < ccc->cfg_len; i++)
    {
      FAR struct bt_conn_s *conn;
      FAR struct bt_buf_s *buf;
      FAR struct bt_att_notify_s *nfy;

      /* TODO: Handle indications */

      if (ccc->value != BT_GATT_CCC_NOTIFY)
        {
          continue;
        }

      conn = bt_conn_lookup_addr_le(&ccc->cfg[i].peer);
      if (!conn || conn->state != BT_CONN_CONNECTED)
        {
          continue;
        }

      buf = bt_att_create_pdu(conn, BT_ATT_OP_NOTIFY,
                              sizeof(*nfy) + data->len);
      if (!buf)
        {
          wlwarn("No buffer available to send notification");
          bt_conn_release(conn);
          return BT_GATT_ITER_STOP;
        }

      wlinfo("conn %p handle 0x%04x\n", conn, data->handle);

      nfy         = bt_buf_extend(buf, sizeof(*nfy));
      nfy->handle = BT_HOST2LE16(data->handle);

      bt_buf_extend(buf, data->len);
      memcpy(nfy->value, data->data, data->len);

      bt_l2cap_send(conn, BT_L2CAP_CID_ATT, buf);
      bt_conn_release(conn);
    }

  return BT_GATT_ITER_CONTINUE;
}

void bt_gatt_notify(uint16_t handle, FAR const void *data, size_t len)
{
  struct notify_data_s nfy;

  nfy.handle = handle;
  nfy.data   = data;
  nfy.len    = len;

  bt_gatt_foreach_attr(handle, 0xffff, notify_cb, &nfy);
}

static uint8_t connected_cb(FAR const struct bt_gatt_attr_s *attr,
                            FAR void *user_data)
{
  FAR struct bt_conn_s *conn = user_data;
  FAR struct _bt_gatt_ccc_s *ccc;
  size_t i;

  /* Check attribute user_data must be of type struct _bt_gatt_ccc_s */

  if (attr->write != bt_gatt_attr_write_ccc)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  ccc = attr->user_data;

  /* If already enabled skip */

  if (ccc->value)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  for (i = 0; i < ccc->cfg_len; i++)
    {
      /* Ignore configuration for different peer */

      if (bt_addr_le_cmp(&conn->dst, &ccc->cfg[i].peer))
        {
          continue;
        }

      if (ccc->cfg[i].value)
        {
          gatt_ccc_changed(ccc);
          return BT_GATT_ITER_CONTINUE;
        }
    }

  return BT_GATT_ITER_CONTINUE;
}

void bt_gatt_connected(FAR struct bt_conn_s *conn)
{
  wlinfo("conn %p\n", conn);
  bt_gatt_foreach_attr(0x0001, 0xffff, connected_cb, conn);
}

static uint8_t disconnected_cb(FAR const struct bt_gatt_attr_s *attr,
                               FAR void *user_data)
{
  FAR struct bt_conn_s *conn = user_data;
  FAR struct _bt_gatt_ccc_s *ccc;
  size_t i;

  /* Check attribute user_data must be of type struct _bt_gatt_ccc_s */

  if (attr->write != bt_gatt_attr_write_ccc)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  ccc = attr->user_data;

  /* If already disabled skip */

  if (!ccc->value)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  for (i = 0; i < ccc->cfg_len; i++)
    {
      /* Ignore configurations with disabled value */

      if (!ccc->cfg[i].value)
        {
          continue;
        }

      if (bt_addr_le_cmp(&conn->dst, &ccc->cfg[i].peer))
        {
          FAR struct bt_conn_s *tmp;

          /* Skip if there is another peer connected */

          tmp = bt_conn_lookup_addr_le(&ccc->cfg[i].peer);
          if (tmp && tmp->state == BT_CONN_CONNECTED)
            {
              bt_conn_release(tmp);
              return BT_GATT_ITER_CONTINUE;
            }
        }
    }

  /* Reset value while disconnected */

  memset(&ccc->value, 0, sizeof(ccc->value));
  ccc->cfg_changed(ccc->value);

  wlinfo("ccc %p reset\n", ccc);
  return BT_GATT_ITER_CONTINUE;
}

void bt_gatt_disconnected(FAR struct bt_conn_s *conn)
{
  wlinfo("conn %p\n", conn);
  bt_gatt_foreach_attr(0x0001, 0xffff, disconnected_cb, conn);
}

static void gatt_mtu_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                         FAR const void *pdu, uint16_t length,
                         FAR void *user_data)
{
  bt_gatt_rsp_func_t func = user_data;

  func(conn, err);
}

static int gatt_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                     bt_att_func_t func, FAR void *user_data,
                     bt_att_destroy_t destroy)
{
  int err;

  err = bt_att_send(conn, buf, func, user_data, destroy);
  if (err)
    {
      wlerr("ERROR: Error sending ATT PDU: %d\n", err);
      bt_buf_release(buf);
    }

  return err;
}

int bt_gatt_exchange_mtu(FAR struct bt_conn_s *conn,
                         bt_gatt_rsp_func_t func)
{
  FAR struct bt_att_exchange_mtu_req_s *req;
  FAR struct bt_buf_s *buf;
  uint16_t mtu;

  if (!conn || !func)
    {
      return -EINVAL;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_MTU_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  /* Select MTU based on the amount of room we have in bt_buf_s including one
   * extra byte for ATT header.
   */

  mtu = bt_buf_tailroom(buf) + 1;

  wlinfo("Client MTU %u\n", mtu);

  req      = bt_buf_extend(buf, sizeof(*req));
  req->mtu = BT_HOST2LE16(mtu);

  return gatt_send(conn, buf, gatt_mtu_rsp, func, NULL);
}

static void att_find_type_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                              FAR const void *pdu, uint16_t length,
                              FAR void *user_data)
{
  FAR const struct bt_att_find_type_rsp_s *rsp = pdu;
  FAR struct bt_gatt_discover_params_s *params = user_data;
  uint16_t end_handle = 0;
  uint16_t start_handle;
  uint8_t i;

  wlinfo("err 0x%02x\n", err);

  if (err)
    {
      goto done;
    }

  /* Parse attributes found */

  for (i = 0; length >= sizeof(rsp->list[i]);
       i++, length -= sizeof(rsp->list[i]))
    {
      FAR const struct bt_gatt_attr_s *attr;

      start_handle = BT_LE162HOST(rsp->list[i].start_handle);
      end_handle   = BT_LE162HOST(rsp->list[i].end_handle);

      wlinfo("start_handle 0x%04x end_handle 0x%04x\n", start_handle,
             end_handle);

      attr =
        (&(struct bt_gatt_attr_s)BT_GATT_PRIMARY_SERVICE
         (start_handle, params->uuid));

      if (params->func(attr, params) == BT_GATT_ITER_STOP)
        {
          goto done;
        }
    }

  /* Stop if could not parse the whole PDU */

  if (length > 0)
    {
      goto done;
    }

  /* Stop if over the range or the requests */

  if (end_handle >= params->end_handle)
    {
      goto done;
    }

  /* Continue for the last found handle */

  params->start_handle = end_handle;
  if (!bt_gatt_discover(conn, params))
    {
      return;
    }

done:
  if (params->destroy)
    {
      params->destroy(params);
    }
}

/****************************************************************************
 * Name: bt_gatt_discover
 *
 * Description:
 *   This function implements the SIOCBTDISCOVER ioctl command for the
 *   GATT discovery.
 *
 ****************************************************************************/

int bt_gatt_discover(FAR struct bt_conn_s *conn,
                     FAR struct bt_gatt_discover_params_s *params)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_find_type_req_s *req;
  FAR uint16_t *value;

  if (!conn || !params->uuid || !params->func || !params->start_handle ||
      !params->end_handle)
    {
      return -EINVAL;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_FIND_TYPE_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  req               = bt_buf_extend(buf, sizeof(*req));
  req->start_handle = BT_HOST2LE16(params->start_handle);
  req->end_handle   = BT_HOST2LE16(params->end_handle);
  req->type         = BT_HOST2LE16(BT_UUID_GATT_PRIMARY);

  wlinfo("uuid 0x%04x start_handle 0x%04x end_handle 0x%04x\n",
         params->uuid->u.u16, params->start_handle, params->end_handle);

  switch (params->uuid->type)
    {
      case BT_UUID_16:
        value  = bt_buf_extend(buf, sizeof(*value));
        *value = BT_HOST2LE16(params->uuid->u.u16);
        break;

      case BT_UUID_128:
        bt_buf_extend(buf, sizeof(params->uuid->u.u128));
        memcpy(req->value,
               params->uuid->u.u128,
               sizeof(params->uuid->u.u128));
        break;

      default:
        wlerr("ERROR: Unknown UUID type %u\n", params->uuid->type);
        bt_buf_release(buf);
        return -EINVAL;
    }

  return gatt_send(conn, buf, att_find_type_rsp, params, NULL);
}

static void att_read_type_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                              FAR const void *pdu, uint16_t length,
                              FAR void *user_data)
{
  FAR const struct bt_att_read_type_rsp_s *rsp = pdu;
  FAR struct bt_gatt_discover_params_s *params = user_data;
  struct bt_uuid_s uuid;
  struct bt_gatt_chrc_s value;
  uint16_t handle = 0;

  wlinfo("err 0x%02x\n", err);

  if (err)
    {
      goto done;
    }

  /* Data can be either in UUID16 or UUID128 */

  switch (rsp->len)
    {
      case 7:                    /* UUID16 */
        uuid.type = BT_UUID_16;
        break;

      case 21:                   /* UUID128 */
        uuid.type = BT_UUID_128;
        break;

      default:
        wlerr("ERROR: Invalid data len %u\n", rsp->len);
        goto done;
    }

  /* Parse characteristics found */

  for (length--, pdu = rsp->data; length >= rsp->len;
       length -= rsp->len, pdu += rsp->len)
    {
      FAR const struct bt_gatt_attr_s *attr;
      FAR const struct bt_att_data_s *data = pdu;
      FAR struct gatt_chrc_s *chrc = (FAR void *)data->value;

      handle = BT_LE162HOST(data->handle);

      /* Handle 0 is invalid */

      if (!handle)
        {
          goto done;
        }

      /* Convert characteristic data, bt_gatt_chrc and gatt_chrc_s have
       * different formats so the conversion have to be done field by field.
       */

      value.properties   = chrc->properties;
      value.value_handle = BT_LE162HOST(chrc->value_handle);
      value.uuid         = &uuid;

      switch (uuid.type)
        {
          case BT_UUID_16:
            uuid.u.u16 = BT_LE162HOST(chrc->u.uuid16);
            break;

          case BT_UUID_128:
            memcpy(uuid.u.u128, chrc->u.uuid, sizeof(chrc->u.uuid));
            break;
        }

      wlinfo("handle 0x%04x properties 0x%02x value_handle 0x%04x\n",
             handle, value.properties, value.value_handle);

      /* Skip if UUID is set but doesn't match */

      if (params->uuid && bt_uuid_cmp(&uuid, params->uuid))
        {
          continue;
        }

      attr = (&(struct bt_gatt_attr_s)BT_GATT_CHARACTERISTIC(handle,
                                                             &value));

      if (params->func(attr, params) == BT_GATT_ITER_STOP)
        {
          goto done;
        }
    }

  /* Stop if could not parse the whole PDU */

  if (length > 0)
    {
      goto done;
    }

  /* Next characteristic shall be after current value handle */

  params->start_handle = handle;
  if (params->start_handle < UINT16_MAX)
    {
      params->start_handle++;
    }

  /* Stop if over the requested range */

  if (params->start_handle >= params->end_handle)
    {
      goto done;
    }

  /* Continue to the next range */

  if (!bt_gatt_discover_characteristic(conn, params))
    {
      return;
    }

done:
  if (params->destroy)
    {
      params->destroy(params);
    }
}

/****************************************************************************
 * Name: bt_gatt_discover_characteristic
 *
 * Description:
 *   This function implements the SIOCBTDISCOVER ioctl command for the
 *   GATT discover characteristics type.
 *
 ****************************************************************************/

int bt_gatt_discover_characteristic(FAR struct bt_conn_s *conn,
                                    FAR struct bt_gatt_discover_params_s
                                               *params)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_type_req_s *req;
  FAR uint16_t *value;

  if (!conn || !params->func || !params->start_handle || !params->end_handle)
    {
      return -EINVAL;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_TYPE_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  req               = bt_buf_extend(buf, sizeof(*req));
  req->start_handle = BT_HOST2LE16(params->start_handle);
  req->end_handle   = BT_HOST2LE16(params->end_handle);

  value             = bt_buf_extend(buf, sizeof(*value));
  *value            = BT_HOST2LE16(BT_UUID_GATT_CHRC);

  wlinfo("start_handle 0x%04x end_handle 0x%04x\n", params->start_handle,
         params->end_handle);

  return gatt_send(conn, buf, att_read_type_rsp, params, NULL);
}

static void att_find_info_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                              FAR const void *pdu, uint16_t length,
                              FAR void *user_data)
{
  FAR const struct bt_att_find_info_rsp_s *rsp = pdu;
  FAR struct bt_gatt_discover_params_s *params = user_data;
  struct bt_uuid_s uuid;
  uint16_t handle = 0;
  uint8_t len;
  union
    {
      FAR const struct bt_att_info_16_s *i16;
      FAR const struct bt_att_info_128_s *i128;
    } info;

  wlinfo("err 0x%02x\n", err);

  if (err)
    {
      goto done;
    }

  /* Data can be either in UUID16 or UUID128 */

  switch (rsp->format)
    {
      case BT_ATT_INFO_16:
        uuid.type = BT_UUID_16;
        len = sizeof(info.i16);
        break;

      case BT_ATT_INFO_128:
        uuid.type = BT_UUID_128;
        len = sizeof(info.i128);
        break;

      default:
      wlerr("ERROR: Invalid format %u\n", rsp->format);
      goto done;
    }

  /* Parse descriptors found */

  for (length--, pdu = rsp->info; length >= len; length -= len, pdu += len)
    {
      FAR const struct bt_gatt_attr_s *attr;

      info.i16 = pdu;
      handle = BT_LE162HOST(info.i16->handle);

      switch (uuid.type)
        {
          case BT_UUID_16:
            uuid.u.u16 = BT_LE162HOST(info.i16->uuid);
            break;

          case BT_UUID_128:
            memcpy(uuid.u.u128, info.i128->uuid, sizeof(uuid.u.u128));
            break;
        }

      wlinfo("handle 0x%04x\n", handle);

      /* Skip if UUID is set but doesn't match */

      if (params->uuid && bt_uuid_cmp(&uuid, params->uuid))
        {
          continue;
        }

      attr =
        (&(struct bt_gatt_attr_s)BT_GATT_DESCRIPTOR
         (handle, &uuid, 0, NULL, NULL, NULL));

      if (params->func(attr, params) == BT_GATT_ITER_STOP)
        {
          goto done;
        }
    }

  /* Stop if could not parse the whole PDU */

  if (length > 0)
    {
      goto done;
    }

  /* Next characteristic shall be after current value handle */

  params->start_handle = handle;
  if (params->start_handle < UINT16_MAX)
    {
      params->start_handle++;
    }

  /* Stop if over the requested range */

  if (params->start_handle >= params->end_handle)
    {
      goto done;
    }

  /* Continue to the next range */

  if (!bt_gatt_discover_descriptor(conn, params))
    {
      return;
    }

done:
  if (params->destroy)
    {
      params->destroy(params);
    }
}

/****************************************************************************
 * Name: bt_gatt_discover_descriptor
 *
 * Description:
 *   This function implements the SIOCBTDISCOVER ioctl command for the
 *   GATT discover descriptor type.
 *
 ****************************************************************************/

int bt_gatt_discover_descriptor(FAR struct bt_conn_s *conn,
                                FAR struct bt_gatt_discover_params_s *params)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_find_info_req_s *req;

  if (!conn || !params->func || !params->start_handle || !params->end_handle)
    {
      return -EINVAL;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_FIND_INFO_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  req               = bt_buf_extend(buf, sizeof(*req));
  req->start_handle = BT_HOST2LE16(params->start_handle);
  req->end_handle   = BT_HOST2LE16(params->end_handle);

  wlinfo("start_handle 0x%04x end_handle 0x%04x\n", params->start_handle,
         params->end_handle);

  return gatt_send(conn, buf, att_find_info_rsp, params, NULL);
}

static void att_read_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                         FAR const void *pdu, uint16_t length,
                         FAR void *user_data)
{
  bt_gatt_read_func_t func = user_data;

  wlinfo("err 0x%02x\n", err);

  if (err)
    {
      func(conn, err, NULL, 0);
      return;
    }

  func(conn, 0, pdu, length);
}

static int gatt_read_blob(FAR struct bt_conn_s *conn, uint16_t handle,
                          uint16_t offset, bt_gatt_read_func_t func)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_blob_req_s *req;

  buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_BLOB_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  req         = bt_buf_extend(buf, sizeof(*req));
  req->handle = BT_HOST2LE16(handle);
  req->offset = BT_HOST2LE16(offset);

  wlinfo("handle 0x%04x offset 0x%04x\n", handle, offset);

  return gatt_send(conn, buf, att_read_rsp, func, NULL);
}

int bt_gatt_read(FAR struct bt_conn_s *conn, uint16_t handle,
                 uint16_t offset, bt_gatt_read_func_t func)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_req_s *req;

  if (!conn || !handle || !func)
    {
      return -EINVAL;
    }

  if (offset)
    {
      return gatt_read_blob(conn, handle, offset, func);
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_REQ, sizeof(*req));
  if (!buf)
    {
      return -ENOMEM;
    }

  req         = bt_buf_extend(buf, sizeof(*req));
  req->handle = BT_HOST2LE16(handle);

  wlinfo("handle 0x%04x\n", handle);

  return gatt_send(conn, buf, att_read_rsp, func, NULL);
}

static void att_write_rsp(FAR struct bt_conn_s *conn, uint8_t err,
                          FAR const void *pdu, uint16_t length,
                          FAR void *user_data)
{
  bt_gatt_rsp_func_t func = user_data;

  wlinfo("err 0x%02x\n", err);

  func(conn, err);
}

static int gatt_write_cmd(FAR struct bt_conn_s *conn, uint16_t handle,
                          FAR const void *data, uint16_t length)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_write_cmd_s *cmd;

  buf = bt_att_create_pdu(conn, BT_ATT_OP_WRITE_CMD, sizeof(*cmd) + length);
  if (!buf)
    {
      return -ENOMEM;
    }

  cmd         = bt_buf_extend(buf, sizeof(*cmd));
  cmd->handle = BT_HOST2LE16(handle);
  memcpy(cmd->value, data, length);
  bt_buf_extend(buf, length);

  wlinfo("handle 0x%04x length %u\n", handle, length);

  return gatt_send(conn, buf, NULL, NULL, NULL);
}

int bt_gatt_write(FAR struct bt_conn_s *conn, uint16_t handle,
                  FAR const void *data, uint16_t length,
                  bt_gatt_rsp_func_t func)
{
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_write_req_s *req;

  if (!conn || !handle)
    {
      return -EINVAL;
    }

  if (!func)
    {
      return gatt_write_cmd(conn, handle, data, length);
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_WRITE_REQ, sizeof(*req) + length);
  if (!buf)
    {
      return -ENOMEM;
    }

  req         = bt_buf_extend(buf, sizeof(*req));
  req->handle = BT_HOST2LE16(handle);
  memcpy(req->value, data, length);
  bt_buf_extend(buf, length);

  wlinfo("handle 0x%04x length %u\n", handle, length);

  return gatt_send(conn, buf, att_write_rsp, func, NULL);
}

void bt_gatt_cancel(FAR struct bt_conn_s *conn)
{
  bt_att_cancel(conn);
}

int bt_gatt_read_multiple(FAR struct bt_conn_s *conn,
                          FAR const uint16_t *handles, size_t count,
                          bt_gatt_read_func_t func)
{
  FAR struct bt_buf_s *buf;
  uint8_t i;

  if (!conn && conn->state != BT_CONN_CONNECTED)
    {
      return -ENOTCONN;
    }

  if (!handles || count < 2 || !func)
    {
      return -EINVAL;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_MULT_REQ,
                          count * sizeof(*handles));
  if (!buf)
    {
      return -ENOMEM;
    }

  for (i = 0; i < count; i++)
    {
      bt_buf_put_le16(buf, handles[i]);
    }

  return gatt_send(conn, buf, att_read_rsp, func, NULL);
}
