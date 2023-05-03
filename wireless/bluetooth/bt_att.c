/****************************************************************************
 * wireless/bluetooth/bt_att.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_core.h>
#include <nuttx/wireless/bluetooth/bt_uuid.h>
#include <nuttx/wireless/bluetooth/bt_gatt.h>

#include "bt_hcicore.h"
#include "bt_conn.h"
#include "bt_l2cap.h"
#include "bt_att.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_BLUETOOTH_DEBUG_ATT)
#  undef wlinfo
#  define wlinfo(fmt, ...)
#endif

#define BT_GATT_PERM_READ_MASK     (BT_GATT_PERM_READ | \
                                    BT_GATT_PERM_READ_ENCRYPT | \
                                    BT_GATT_PERM_READ_AUTHEN | \
                                    BT_GATT_PERM_AUTHOR)
#define BT_GATT_PERM_WRITE_MASK    (BT_GATT_PERM_WRITE | \
                                    BT_GATT_PERM_WRITE_ENCRYPT | \
                                    BT_GATT_PERM_WRITE_AUTHEN | \
                                    BT_GATT_PERM_AUTHOR)
#define BT_GATT_PERM_ENCRYPT_MASK  (BT_GATT_PERM_READ_ENCRYPT | \
                                    BT_GATT_PERM_WRITE_ENCRYPT)
#define BT_GATT_PERM_AUTHEN_MASK    (BT_GATT_PERM_READ_AUTHEN | \
                                    BT_GATT_PERM_WRITE_AUTHEN)
#define BT_ATT_OP_CMD_MASK         (BT_ATT_OP_WRITE_CMD & \
                                    BT_ATT_OP_SIGNED_WRITE_CMD)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ATT request context */

struct bt_att_req_s
{
  bt_att_func_t func;
  FAR void *user_data;
  bt_att_destroy_t destroy;
  uint8_t op;
};

/* ATT channel specific context */

struct bt_att_s
{
  /* The connection this context is associated with */

  FAR struct bt_conn_s *conn;
  uint16_t mtu;
  struct bt_att_req_s req;

  /* TODO: Allow more than one pending request */
};

struct find_info_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_find_info_rsp_s *rsp;
  union
  {
    FAR struct bt_att_info_16_s *info16;
    FAR struct bt_att_info_128_s *info128;
  } u;
};

struct find_type_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_handle_group_s *group;
  FAR const void *value;
  uint8_t value_len;
};

struct read_type_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_uuid_s *uuid;
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_type_rsp_s *rsp;
  FAR struct bt_att_data_s *item;
};

struct read_data_s
{
  FAR struct bt_conn_s *conn;
  uint16_t offset;
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_rsp_s *rsp;
  uint8_t err;
};

struct read_group_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_uuid_s *uuid;
  FAR struct bt_buf_s *buf;
  FAR struct bt_att_read_group_rsp_s *rsp;
  FAR struct bt_att_group_data_s *group;
};

struct write_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_buf_s *buf;
  uint8_t op;
  FAR const void *value;
  uint8_t len;
  uint16_t offset;
  uint8_t err;
};

struct flush_data_s
{
  FAR struct bt_conn_s *conn;
  FAR struct bt_buf_s *buf;
  uint8_t flags;
  uint8_t err;
};

struct handler_info_s
{
  uint8_t op;
  CODE uint8_t(*func)(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf);
  uint8_t expect_len;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    att_req_destroy(FAR struct bt_att_req_s *req);
static void    send_err_rsp(struct bt_conn_s *conn, uint8_t req,
                 uint16_t handle, uint8_t err);
static uint8_t att_mtu_req(struct bt_conn_s *conn, struct bt_buf_s *data);
static uint8_t att_handle_rsp(struct bt_conn_s *conn, void *pdu,
                 uint16_t len, uint8_t err);
static uint8_t att_mtu_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static bool    range_is_valid(uint16_t start, uint16_t end,
                 FAR uint16_t *err);
static uint8_t find_info_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_find_info_rsp(FAR struct bt_conn_s *conn,
                 uint16_t start_handle, uint16_t end_handle);
static uint8_t att_find_info_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t find_type_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_find_type_rsp(FAR struct bt_conn_s *conn,
                 FAR uint16_t start_handle, uint16_t end_handle,
                 FAR const void *value, uint8_t value_len);
static uint8_t att_find_type_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static bool uuid_create(FAR struct bt_uuid_s *uuid,
                 FAR struct bt_buf_s *data);
static uint8_t read_type_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_read_type_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_uuid_s *uuid, uint16_t start_handle,
                 uint16_t end_handle);
static uint8_t att_read_type_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t err_to_att(int err);
static uint8_t check_perm(FAR struct bt_conn_s *conn,
                 FAR const struct bt_gatt_attr_s *attr, uint8_t mask);
static uint8_t read_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_read_rsp(FAR struct bt_conn_s *conn, uint8_t op,
                 uint8_t rsp, uint16_t handle, uint16_t offset);
static uint8_t att_read_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_read_blob_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_read_mult_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t read_group_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_read_group_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_uuid_s *uuid, uint16_t start_handle,
                 uint16_t end_handle);
static uint8_t att_read_group_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t write_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_write_rsp(FAR struct bt_conn_s *conn, uint8_t op,
                 uint8_t rsp, uint16_t handle, uint16_t offset,
                 FAR const void *value, uint8_t len);
static uint8_t att_write_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_prepare_write_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t flush_cb(FAR const struct bt_gatt_attr_s *attr,
                 FAR void *user_data);
static uint8_t att_exec_write_rsp(FAR struct bt_conn_s *conn,
                 uint8_t flags);
static uint8_t att_exec_write_req(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_write_cmd(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_signed_write_cmd(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_error_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *data);
static uint8_t att_handle_find_info_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_find_type_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_read_type_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_read_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_read_blob_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_read_mult_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static uint8_t att_handle_write_rsp(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf);
static void bt_att_receive(FAR struct bt_conn_s *conn,
                 FAR struct bt_buf_s *buf, FAR void *context, uint16_t cid);
static void bt_att_connected(FAR struct bt_conn_s *conn, FAR void *context,
                 uint16_t cid);
static void bt_att_disconnected(FAR struct bt_conn_s *conn,
                 FAR void *context, uint16_t cid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bt_att_s g_bt_att_pool[CONFIG_BLUETOOTH_MAX_CONN];

static const struct bt_uuid_s g_primary_uuid =
{
  BT_UUID_16,
  {
    BT_UUID_GATT_PRIMARY
  }
};

static const struct bt_uuid_s g_secondary_uuid =
{
  BT_UUID_16,
  {
    BT_UUID_GATT_SECONDARY
  },
};

static const struct handler_info_s g_handlers[] =
{
  {
    BT_ATT_OP_ERROR_RSP,
    att_error_rsp,
    sizeof(struct bt_att_error_rsp_s)
  },
  {
    BT_ATT_OP_MTU_REQ,
    att_mtu_req,
    sizeof(struct bt_att_exchange_mtu_req_s)
  },
  {
    BT_ATT_OP_MTU_RSP,
    att_mtu_rsp,
    sizeof(struct bt_att_exchange_mtu_rsp_s)
  },
  {
    BT_ATT_OP_FIND_INFO_REQ,
    att_find_info_req,
    sizeof(struct bt_att_find_info_req_s)
  },
  {
    BT_ATT_OP_FIND_INFO_RSP,
    att_handle_find_info_rsp,
    sizeof(struct bt_att_find_info_rsp_s)
  },
  {
    BT_ATT_OP_FIND_TYPE_REQ,
    att_find_type_req,
    sizeof(struct bt_att_find_type_req_s)
  },
  {
    BT_ATT_OP_FIND_TYPE_RSP,
    att_handle_find_type_rsp,
    sizeof(struct bt_att_find_type_rsp_s)
  },
  {
    BT_ATT_OP_READ_TYPE_REQ,
    att_read_type_req,
    sizeof(struct bt_att_read_type_req_s)
  },
  {
    BT_ATT_OP_READ_TYPE_RSP,
    att_handle_read_type_rsp,
    sizeof(struct bt_att_read_type_rsp_s)
  },
  {
    BT_ATT_OP_READ_REQ,
    att_read_req,
    sizeof(struct bt_att_read_req_s)
  },
  {
    BT_ATT_OP_READ_RSP,
    att_handle_read_rsp,
    sizeof(struct bt_att_read_rsp_s)
  },
  {
    BT_ATT_OP_READ_BLOB_REQ,
    att_read_blob_req,
    sizeof(struct bt_att_read_blob_req_s)
  },
  {
    BT_ATT_OP_READ_BLOB_RSP,
    att_handle_read_blob_rsp,
    sizeof(struct bt_att_read_blob_rsp_s)
  },
  {
    BT_ATT_OP_READ_MULT_REQ,
    att_read_mult_req,
    BT_ATT_READ_MULT_MIN_LEN_REQ
  },
  {
    BT_ATT_OP_READ_MULT_RSP,
    att_handle_read_mult_rsp,
    sizeof(struct bt_att_read_mult_rsp_s)
  },
  {
    BT_ATT_OP_READ_GROUP_REQ,
    att_read_group_req,
    sizeof(struct bt_att_read_group_req_s)
  },
  {
    BT_ATT_OP_WRITE_REQ,
    att_write_req,
    sizeof(struct bt_att_write_req_s)
  },
  {
    BT_ATT_OP_WRITE_RSP,
    att_handle_write_rsp,
    0
  },
  {
    BT_ATT_OP_PREPARE_WRITE_REQ,
    att_prepare_write_req,
    sizeof(struct bt_att_prepare_write_req_s)
  },
  {
    BT_ATT_OP_EXEC_WRITE_REQ,
    att_exec_write_req,
    sizeof(struct bt_att_exec_write_req_s)
  },
  {
    BT_ATT_OP_WRITE_CMD,
    att_write_cmd,
    sizeof(struct bt_att_write_cmd_s)
  },
  {
    BT_ATT_OP_SIGNED_WRITE_CMD,
    att_signed_write_cmd,
    sizeof(struct bt_att_write_cmd_s) + sizeof(struct bt_att_signature_s)
  }
};

#define NHANDLERS (sizeof(g_handlers) / sizeof(struct handler_info_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void att_req_destroy(FAR struct bt_att_req_s *req)
{
  if (req->destroy)
    {
      req->destroy(req->user_data);
    }

  memset(req, 0, sizeof(*req));
}

static void send_err_rsp(struct bt_conn_s *conn,
                         uint8_t req,
                         uint16_t handle,
                         uint8_t err)
{
  struct bt_att_error_rsp_s *rsp;
  struct bt_buf_s *buf;

  /* Ignore opcode 0x00 */

  if (!req)
    {
      return;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_ERROR_RSP, sizeof(*rsp));
  if (!buf)
    {
      return;
    }

  rsp = bt_buf_extend(buf, sizeof(*rsp));
  rsp->request = req;
  rsp->handle = BT_HOST2LE16(handle);
  rsp->error = err;

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, buf);
}

static uint8_t att_mtu_req(struct bt_conn_s *conn, struct bt_buf_s *data)
{
  FAR struct bt_att_s *att = conn->att;
  struct bt_att_exchange_mtu_req_s *req;
  struct bt_att_exchange_mtu_rsp_s *rsp;
  struct bt_buf_s *buf;
  uint16_t maxmtu;
  uint16_t mtu;

  req = (void *)data->data;

  mtu = BT_LE162HOST(req->mtu);

  wlinfo("Client MTU %u\n", mtu);

  if (mtu > BT_ATT_MAX_LE_MTU || mtu < BT_ATT_DEFAULT_LE_MTU)
    {
      return BT_ATT_ERR_INVALID_PDU;
    }

  buf = bt_att_create_pdu(conn, BT_ATT_OP_MTU_RSP, sizeof(*rsp));
  if (!buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  /* Select MTU based on the amount of room we have in bt_buf_s including one
   * extra byte for ATT header.
   */

  maxmtu = bt_buf_tailroom(buf) + 1;
  if (mtu > maxmtu)
    {
      mtu = maxmtu;
    }

  wlinfo("Server MTU %u\n", mtu);

  att->mtu = mtu;

  rsp = bt_buf_extend(buf, sizeof(*rsp));
  rsp->mtu = BT_HOST2LE16(mtu);

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, buf);

  return 0;
}

static uint8_t att_handle_rsp(struct bt_conn_s *conn,
                              void *pdu,
                              uint16_t len,
                              uint8_t err)
{
  FAR struct bt_att_s *att = conn->att;
  struct bt_att_req_s req;

  if (!att->req.func)
    {
      return 0;
    }

  /* Reset request before callback so another request can be queued */

  memcpy(&req, &att->req, sizeof(req));
  att->req.func = NULL;

  req.func(conn, err, pdu, len, req.user_data);

  att_req_destroy(&req);
  return 0;
}

static uint8_t att_mtu_rsp(FAR struct bt_conn_s *conn,
                           FAR struct bt_buf_s *buf)
{
  FAR struct bt_att_s *att = conn->att;
  FAR struct bt_att_exchange_mtu_rsp_s *rsp;
  uint16_t maxmtu;
  uint16_t mtu;

  if (!att)
    {
      return 0;
    }

  rsp = (void *)buf->data;
  mtu = BT_LE162HOST(rsp->mtu);

  wlinfo("Server MTU %u\n", mtu);

  /* Check if MTU is within allowed range */

  if (mtu > BT_ATT_MAX_LE_MTU || mtu < BT_ATT_DEFAULT_LE_MTU)
    {
      return att_handle_rsp(conn, NULL, 0, BT_ATT_ERR_INVALID_PDU);
    }

  /* Clip MTU based on the maximum amount of data bt_buf_s can hold excluding
   * L2CAP, ACL and driver headers.
   */

  maxmtu = BLUETOOTH_MAX_FRAMELEN - (sizeof(struct bt_l2cap_hdr_s) +
                                     sizeof(struct bt_hci_acl_hdr_s) +
                                     g_btdev.btdev->head_reserve);
  if (mtu > maxmtu)
    {
      mtu = maxmtu;
    }

  att->mtu = mtu;
  return att_handle_rsp(conn, rsp, buf->len, 0);
}

static bool range_is_valid(uint16_t start, uint16_t end, FAR uint16_t *err)
{
  /* Handle 0 is invalid */

  if (!start || !end)
    {
      if (err)
        {
          *err = 0;
        }

      return false;
    }

  /* Check if range is valid */

  if (start > end)
    {
      if (err)
        {
          *err = start;
        }

      return false;
    }

  return true;
}

static uint8_t find_info_cb(FAR const struct bt_gatt_attr_s *attr,
                            FAR void *user_data)
{
  FAR struct find_info_data_s *data = user_data;
  FAR struct bt_att_s *att = data->conn->att;

  wlinfo("handle 0x%04x\n", attr->handle);

  /* Initialize rsp at first entry */

  if (!data->rsp)
    {
      data->rsp = bt_buf_extend(data->buf, sizeof(*data->rsp));
      data->rsp->format = (attr->uuid->type == BT_UUID_16) ?
        BT_ATT_INFO_16 : BT_ATT_INFO_128;
    }

  switch (data->rsp->format)
    {
      case BT_ATT_INFO_16:
        if (attr->uuid->type != BT_UUID_16)
          {
            return BT_GATT_ITER_STOP;
          }

        /* Fast forward to next item position */

        data->u.info16         = bt_buf_extend(data->buf,
                                               sizeof(*data->u.info16));
        data->u.info16->handle = BT_HOST2LE16(attr->handle);
        data->u.info16->uuid   = BT_HOST2LE16(attr->uuid->u.u16);

        return att->mtu - data->buf->len > sizeof(*data->u.info16) ?
          BT_GATT_ITER_CONTINUE : BT_GATT_ITER_STOP;

      case BT_ATT_INFO_128:
        if (attr->uuid->type != BT_UUID_128)
          {
            return BT_GATT_ITER_STOP;
          }

        /* Fast forward to next item position */

        data->u.info128         = bt_buf_extend(data->buf,
                                                sizeof(*data->u.info128));
        data->u.info128->handle = BT_HOST2LE16(attr->handle);
        memcpy(data->u.info128->uuid,
               attr->uuid->u.u128,
               sizeof(data->u.info128->uuid));

        return att->mtu - data->buf->len > sizeof(*data->u.info128) ?
          BT_GATT_ITER_CONTINUE : BT_GATT_ITER_STOP;
    }

  return BT_GATT_ITER_STOP;
}

static uint8_t att_find_info_rsp(FAR struct bt_conn_s *conn,
                                 uint16_t start_handle,
                                 uint16_t end_handle)
{
  struct find_info_data_s data;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_FIND_INFO_RSP, 0);
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn = conn;
  bt_gatt_foreach_attr(start_handle, end_handle, find_info_cb, &data);

  if (!data.rsp)
    {
      bt_buf_release(data.buf);

      /* Respond since handle is set */

      send_err_rsp(conn, BT_ATT_OP_FIND_INFO_REQ, start_handle,
                   BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
      return 0;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_find_info_req(FAR struct bt_conn_s *conn,
                                 FAR struct bt_buf_s *data)
{
  FAR struct bt_att_find_info_req_s *req;
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t err_handle;

  req = (void *)data->data;

  start_handle = BT_LE162HOST(req->start_handle);
  end_handle   = BT_LE162HOST(req->end_handle);

  wlinfo("start_handle 0x%04x end_handle 0x%04x\n",
         start_handle, end_handle);

  if (!range_is_valid(start_handle, end_handle, &err_handle))
    {
      send_err_rsp(conn, BT_ATT_OP_FIND_INFO_REQ, err_handle,
                   BT_ATT_ERR_INVALID_HANDLE);
      return 0;
    }

  return att_find_info_rsp(conn, start_handle, end_handle);
}

static uint8_t find_type_cb(FAR const struct bt_gatt_attr_s *attr,
                            FAR void *user_data)
{
  FAR struct find_type_data_s *data = user_data;
  FAR struct bt_att_s *att = data->conn->att;
  uint8_t uuid[16];
  int read;

  /* Skip if not a primary service */

  if (bt_uuid_cmp(attr->uuid, &g_primary_uuid))
    {
      if (data->group && attr->handle > data->group->end_handle)
        {
          data->group->end_handle = BT_HOST2LE16(attr->handle);
        }

      return BT_GATT_ITER_CONTINUE;
    }

  wlinfo("handle 0x%04x\n", attr->handle);

  /* Stop if there is no space left */

  if (att->mtu - data->buf->len < sizeof(*data->group))
    {
      return BT_GATT_ITER_STOP;
    }

  /* Read attribute value and store in the buffer */

  read = attr->read(data->conn, attr, uuid, sizeof(uuid), 0);
  if (read < 0)
    {
      /* TODO: Return an error if this fails */

      return BT_GATT_ITER_STOP;
    }

  /* Check if data matches */

  if (read != data->value_len || memcmp(data->value, uuid, read))
    {
      /* If a group exists stop otherwise continue */

      return data->group ? BT_GATT_ITER_STOP : BT_GATT_ITER_CONTINUE;
    }

  /* Fast forward to next item position */

  data->group               = bt_buf_extend(data->buf, sizeof(*data->group));
  data->group->start_handle = BT_HOST2LE16(attr->handle);
  data->group->end_handle   = BT_HOST2LE16(attr->handle);

  /* Continue to find the end_handle */

  return BT_GATT_ITER_CONTINUE;
}

static uint8_t att_find_type_rsp(FAR struct bt_conn_s *conn,
                                 FAR uint16_t start_handle,
                                 uint16_t end_handle,
                                 FAR const void *value,
                                 uint8_t value_len)
{
  struct find_type_data_s data;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_FIND_TYPE_RSP, 0);
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn      = conn;
  data.value     = value;
  data.value_len = value_len;

  bt_gatt_foreach_attr(start_handle, end_handle, find_type_cb, &data);

  if (!data.group)
    {
      bt_buf_release(data.buf);

      /* Respond since handle is set */

      send_err_rsp(conn, BT_ATT_OP_FIND_TYPE_REQ, start_handle,
                   BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
      return 0;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_find_type_req(FAR struct bt_conn_s *conn,
                                 FAR struct bt_buf_s *data)
{
  FAR struct bt_att_find_type_req_s *req;
  FAR uint8_t *value;
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t err_handle;
  uint16_t type;

  req = (FAR void *)data->data;

  start_handle = BT_LE162HOST(req->start_handle);
  end_handle   = BT_LE162HOST(req->end_handle);
  type         = BT_LE162HOST(req->type);
  value        = bt_buf_consume(data, sizeof(*req));

  wlinfo("start_handle 0x%04x end_handle 0x%04x type %u\n", start_handle,
         end_handle, type);

  if (!range_is_valid(start_handle, end_handle, &err_handle))
    {
      send_err_rsp(conn, BT_ATT_OP_FIND_TYPE_REQ, err_handle,
                   BT_ATT_ERR_INVALID_HANDLE);
      return 0;
    }

  /* The Attribute Protocol Find By Type Value Request shall be used with the
   * Attribute Type parameter set to the UUID for «Primary Service» and the
   * Attribute Value set to the 16-bit Bluetooth UUID or 128-bit UUID for the
   * specific primary service.
   */

  if (type != BT_UUID_GATT_PRIMARY)
    {
      send_err_rsp(conn, BT_ATT_OP_FIND_TYPE_REQ, start_handle,
                   BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
      return 0;
    }

  return att_find_type_rsp(conn, start_handle, end_handle, value, data->len);
}

static bool uuid_create(FAR struct bt_uuid_s *uuid,
                        FAR struct bt_buf_s *data)
{
  if (data->len > sizeof(uuid->u.u128))
    {
      return false;
    }

  switch (data->len)
    {
    case 2:
      uuid->type  = BT_UUID_16;
      uuid->u.u16 = bt_buf_get_le16(data);
      return true;

    case 16:
      uuid->type = BT_UUID_128;
      memcpy(uuid->u.u128, data->data, data->len);
      return true;
    }

  return false;
}

static uint8_t read_type_cb(FAR const struct bt_gatt_attr_s *attr,
                            FAR void *user_data)
{
  FAR struct read_type_data_s *data = user_data;
  FAR struct bt_att_s *att = data->conn->att;
  int read;

  /* Skip if doesn't match */

  if (bt_uuid_cmp(attr->uuid, data->uuid))
    {
      return BT_GATT_ITER_CONTINUE;
    }

  wlinfo("handle 0x%04x\n", attr->handle);

  /* Fast forward to next item position */

  data->item = bt_buf_extend(data->buf, sizeof(*data->item));
  data->item->handle = BT_HOST2LE16(attr->handle);

  /* Read attribute value and store in the buffer */

  read = attr->read(data->conn, attr, data->buf->data + data->buf->len,
                    att->mtu - data->buf->len, 0);
  if (read < 0)
    {
      /* TODO: Handle read errors */

      return BT_GATT_ITER_STOP;
    }

  if (!data->rsp->len)
    {
      /* Set len to be the first item found */

      data->rsp->len = read + sizeof(*data->item);
    }
  else if (data->rsp->len != read + sizeof(*data->item))
    {
      /* All items should have the same size */

      data->buf->len -= sizeof(*data->item);
      return BT_GATT_ITER_STOP;
    }

  bt_buf_extend(data->buf, read);

  /* Return true only if there are still space for more items */

  return att->mtu - data->buf->len > data->rsp->len ?
    BT_GATT_ITER_CONTINUE : BT_GATT_ITER_STOP;
}

static uint8_t att_read_type_rsp(FAR struct bt_conn_s *conn,
                                 FAR struct bt_uuid_s *uuid,
                                 uint16_t start_handle,
                                 uint16_t end_handle)
{
  struct read_type_data_s data;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_TYPE_RSP,
                               sizeof(*data.rsp));
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn     = conn;
  data.uuid     = uuid;
  data.rsp      = bt_buf_extend(data.buf, sizeof(*data.rsp));
  data.rsp->len = 0;

  bt_gatt_foreach_attr(start_handle, end_handle, read_type_cb, &data);

  if (!data.rsp->len)
    {
      bt_buf_release(data.buf);

      /* Response here since handle is set */

      send_err_rsp(conn, BT_ATT_OP_READ_TYPE_REQ, start_handle,
                   BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
      return 0;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_read_type_req(FAR struct bt_conn_s *conn,
                                 FAR struct bt_buf_s *data)
{
  FAR struct bt_att_read_type_req_s *req;
  struct bt_uuid_s uuid;
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t err_handle;

  /* Type can only be UUID16 or UUID128 */

  if (data->len != sizeof(*req) + sizeof(uuid.u.u16) &&
      data->len != sizeof(*req) + sizeof(uuid.u.u128))
    {
      return BT_ATT_ERR_INVALID_PDU;
    }

  req          = (FAR void *)data->data;
  start_handle = BT_LE162HOST(req->start_handle);
  end_handle   = BT_LE162HOST(req->end_handle);
  bt_buf_consume(data, sizeof(*req));

  if (!uuid_create(&uuid, data))
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  wlinfo("start_handle 0x%04x end_handle 0x%04x type %u\n",
         start_handle, end_handle, uuid.u.u16);

  if (!range_is_valid(start_handle, end_handle, &err_handle))
    {
      send_err_rsp(conn, BT_ATT_OP_READ_TYPE_REQ, err_handle,
                   BT_ATT_ERR_INVALID_HANDLE);
      return 0;
    }

  return att_read_type_rsp(conn, &uuid, start_handle, end_handle);
}

static uint8_t err_to_att(int err)
{
  wlinfo("%d", err);

  switch (err)
    {
    case -EINVAL:
      return BT_ATT_ERR_INVALID_OFFSET;

    case -EFBIG:
      return BT_ATT_ERR_INVALID_ATTRIBUTE_LEN;

    default:
      return BT_ATT_ERR_UNLIKELY;
    }
}

static uint8_t check_perm(FAR struct bt_conn_s *conn,
                          FAR const struct bt_gatt_attr_s *attr,
                          uint8_t mask)
{
  if ((mask & BT_GATT_PERM_READ) && !(attr->perm & BT_GATT_PERM_READ))
    {
      return BT_ATT_ERR_READ_NOT_PERMITTED;
    }

  if ((mask & BT_GATT_PERM_WRITE) && !(attr->perm & BT_GATT_PERM_WRITE))
    {
      return BT_ATT_ERR_READ_NOT_PERMITTED;
    }

  mask &= attr->perm;
  if (mask & BT_GATT_PERM_AUTHEN_MASK)
    {
      /* TODO: Check conn authentication */

      return BT_ATT_ERR_AUTHENTICATION;
    }

  if ((mask & BT_GATT_PERM_ENCRYPT_MASK) && !conn->encrypt)
    {
      return BT_ATT_ERR_INSUFFICIENT_ENCRYPTION;
    }

  if (mask & BT_GATT_PERM_AUTHOR)
    {
      return BT_ATT_ERR_AUTHORIZATION;
    }

  return 0;
}

static uint8_t read_cb(FAR const struct bt_gatt_attr_s *attr,
                       FAR void *user_data)
{
  FAR struct read_data_s *data = user_data;
  FAR struct bt_att_s *att = data->conn->att;
  int read;

  wlinfo("handle 0x%04x\n", attr->handle);

  data->rsp = bt_buf_extend(data->buf, sizeof(*data->rsp));

  if (!attr->read)
    {
      data->err = BT_ATT_ERR_READ_NOT_PERMITTED;
      return BT_GATT_ITER_STOP;
    }

  /* Check attribute permissions */

  data->err = check_perm(data->conn, attr, BT_GATT_PERM_READ_MASK);
  if (data->err)
    {
      return BT_GATT_ITER_STOP;
    }

  /* Read attribute value and store in the buffer */

  read = attr->read(data->conn, attr, data->buf->data + data->buf->len,
                    att->mtu - data->buf->len, data->offset);
  if (read < 0)
    {
      data->err = err_to_att(read);
      return BT_GATT_ITER_STOP;
    }

  bt_buf_extend(data->buf, read);
  return BT_GATT_ITER_CONTINUE;
}

static uint8_t att_read_rsp(FAR struct bt_conn_s *conn, uint8_t op,
                            uint8_t rsp, uint16_t handle, uint16_t offset)
{
  struct read_data_s data;

  if (!handle)
    {
      return BT_ATT_ERR_INVALID_HANDLE;
    }

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, rsp, 0);
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn = conn;
  data.offset = offset;

  bt_gatt_foreach_attr(handle, handle, read_cb, &data);

  /* In case of error discard data and respond with an error */

  if (data.err)
    {
      bt_buf_release(data.buf);

      /* Respond here since handle is set */

      send_err_rsp(conn, op, handle, data.err);
      return 0;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_read_req(FAR struct bt_conn_s *conn,
                            FAR struct bt_buf_s *data)
{
  FAR struct bt_att_read_req_s *req;
  uint16_t handle;

  req = (void *)data->data;

  handle = BT_LE162HOST(req->handle);

  wlinfo("handle 0x%04x\n", handle);

  return att_read_rsp(conn,
                      BT_ATT_OP_READ_REQ,
                      BT_ATT_OP_READ_RSP,
                      handle,
                      0);
}

static uint8_t att_read_blob_req(FAR struct bt_conn_s *conn,
                                 FAR struct bt_buf_s *data)
{
  FAR struct bt_att_read_blob_req_s *req;
  uint16_t handle;
  uint16_t offset;

  req = (FAR void *)data->data;

  handle = BT_LE162HOST(req->handle);
  offset = BT_LE162HOST(req->offset);

  wlinfo("handle 0x%04x offset %u\n", handle, offset);

  return att_read_rsp(conn, BT_ATT_OP_READ_BLOB_REQ,
                      BT_ATT_OP_READ_BLOB_RSP, handle, offset);
}

static uint8_t att_read_mult_req(FAR struct bt_conn_s *conn,
                                 FAR struct bt_buf_s *buf)
{
  struct read_data_s data;
  uint16_t handle;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_MULT_RSP, 0);
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn = conn;

  while (buf->len >= sizeof(uint16_t))
    {
      handle = bt_buf_get_le16(buf);

      wlinfo("handle 0x%04x\n", handle);

      bt_gatt_foreach_attr(handle, handle, read_cb, &data);

      /* Stop reading in case of error */

      if (data.err)
        {
          bt_buf_release(data.buf);

          /* Respond here since handle is set */

          send_err_rsp(conn, BT_ATT_OP_READ_MULT_REQ, handle, data.err);
          return 0;
        }
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);

  return 0;
}

static uint8_t read_group_cb(FAR const struct bt_gatt_attr_s *attr,
                             FAR void *user_data)
{
  FAR struct read_group_data_s *data = user_data;
  FAR struct bt_att_s *att = data->conn->att;
  int read;

  /* If UUID don't match update group end_handle */

  if (bt_uuid_cmp(attr->uuid, data->uuid))
    {
      if (data->group && attr->handle > data->group->end_handle)
        {
          data->group->end_handle = BT_HOST2LE16(attr->handle);
        }

      return BT_GATT_ITER_CONTINUE;
    }

  wlinfo("handle 0x%04x\n", attr->handle);

  /* Stop if there is no space left */

  if (data->rsp->len && att->mtu - data->buf->len < data->rsp->len)
    {
      return BT_GATT_ITER_STOP;
    }

  /* Fast forward to next group position */

  data->group               = bt_buf_extend(data->buf, sizeof(*data->group));

  /* Initialize group handle range */

  data->group->start_handle = BT_HOST2LE16(attr->handle);
  data->group->end_handle   = BT_HOST2LE16(attr->handle);

  /* Read attribute value and store in the buffer */

  read = attr->read(data->conn, attr, data->buf->data + data->buf->len,
                    att->mtu - data->buf->len, 0);
  if (read < 0)
    {
      /* TODO: Handle read errors */

      return BT_GATT_ITER_STOP;
    }

  if (!data->rsp->len)
    {
      /* Set len to be the first group found */

      data->rsp->len = read + sizeof(*data->group);
    }
  else if (data->rsp->len != read + sizeof(*data->group))
    {
      /* All groups entries should have the same size */

      data->buf->len -= sizeof(*data->group);
      return false;
    }

  bt_buf_extend(data->buf, read);

  /* Continue to find the end handle */

  return BT_GATT_ITER_CONTINUE;
}

static uint8_t att_read_group_rsp(FAR struct bt_conn_s *conn,
                                  FAR struct bt_uuid_s *uuid,
                                  uint16_t start_handle,
                                  uint16_t end_handle)
{
  struct read_group_data_s data;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_READ_GROUP_RSP,
                               sizeof(*data.rsp));
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn     = conn;
  data.uuid     = uuid;
  data.rsp      = bt_buf_extend(data.buf, sizeof(*data.rsp));
  data.rsp->len = 0;

  bt_gatt_foreach_attr(start_handle, end_handle, read_group_cb, &data);

  if (!data.rsp->len)
    {
      bt_buf_release(data.buf);

      /* Respond here since handle is set */

      send_err_rsp(conn, BT_ATT_OP_READ_GROUP_REQ, start_handle,
                   BT_ATT_ERR_ATTRIBUTE_NOT_FOUND);
      return 0;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_read_group_req(FAR struct bt_conn_s *conn,
                                  FAR struct bt_buf_s *data)
{
  FAR struct bt_att_read_group_req_s *req;
  struct bt_uuid_s uuid;
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t err_handle;

  /* Type can only be UUID16 or UUID128 */

  if (data->len != sizeof(*req) + sizeof(uuid.u.u16) &&
      data->len != sizeof(*req) + sizeof(uuid.u.u128))
    {
      return BT_ATT_ERR_INVALID_PDU;
    }

  req          = (FAR void *)data->data;
  start_handle = BT_LE162HOST(req->start_handle);
  end_handle   = BT_LE162HOST(req->end_handle);
  bt_buf_consume(data, sizeof(*req));

  if (!uuid_create(&uuid, data))
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  wlinfo("start_handle 0x%04x end_handle 0x%04x type %u\n",
         start_handle, end_handle, uuid.u.u16);

  if (!range_is_valid(start_handle, end_handle, &err_handle))
    {
      send_err_rsp(conn, BT_ATT_OP_READ_GROUP_REQ, err_handle,
                   BT_ATT_ERR_INVALID_HANDLE);
      return 0;
    }

  /* Core v4.2, Vol 3, sec 2.5.3 Attribute Grouping: Not all of the grouping
   * attributes can be used in the ATT Read By Group Type Request. The
   * «Primary Service» and «Secondary Service» grouping types may be used
   * in the Read By Group Type Request. The «Characteristic» grouping type
   * shall not be used in the ATT Read By Group Type Request.
   */

  if (bt_uuid_cmp(&uuid, &g_primary_uuid) &&
      bt_uuid_cmp(&uuid, &g_secondary_uuid))
    {
      send_err_rsp(conn, BT_ATT_OP_READ_GROUP_REQ, start_handle,
                   BT_ATT_ERR_UNSUPPORTED_GROUP_TYPE);
      return 0;
    }

  return att_read_group_rsp(conn, &uuid, start_handle, end_handle);
}

static uint8_t write_cb(FAR const struct bt_gatt_attr_s *attr,
                        FAR void *user_data)
{
  FAR struct write_data_s *data = user_data;
  int write;

  wlinfo("handle 0x%04x\n", attr->handle);

  /* Check for write support and flush support in case of prepare */

  if (!attr->write ||
      (data->op == BT_ATT_OP_PREPARE_WRITE_REQ && !attr->flush))
    {
      data->err = BT_ATT_ERR_WRITE_NOT_PERMITTED;
      return BT_GATT_ITER_STOP;
    }

  /* Check attribute permissions */

  data->err = check_perm(data->conn, attr, BT_GATT_PERM_WRITE_MASK);
  if (data->err)
    {
      return BT_GATT_ITER_STOP;
    }

  /* Read attribute value and store in the buffer */

  write = attr->write(data->conn,
                      attr,
                      data->value,
                      data->len,
                      data->offset);
  if (write < 0 || write != data->len)
    {
      data->err = err_to_att(write);
      return BT_GATT_ITER_STOP;
    }

  /* Flush in case of regular write operation */

  if (attr->flush && data->op != BT_ATT_OP_PREPARE_WRITE_REQ)
    {
      write = attr->flush(data->conn, attr, BT_GATT_FLUSH_SYNC);
      if (write < 0)
        {
          data->err = err_to_att(write);
          return BT_GATT_ITER_STOP;
        }
    }

  data->err = 0;

  return BT_GATT_ITER_CONTINUE;
}

static uint8_t att_write_rsp(FAR struct bt_conn_s *conn, uint8_t op,
                             uint8_t rsp, uint16_t handle,
                             uint16_t offset, FAR const void *value,
                             uint8_t len)
{
  struct write_data_s data;

  if (!handle)
    {
      return BT_ATT_ERR_INVALID_HANDLE;
    }

  memset(&data, 0, sizeof(data));

  /* Only allocate buf if required to respond */

  if (rsp)
    {
      data.buf = bt_att_create_pdu(conn, rsp, 0);
      if (!data.buf)
        {
          return BT_ATT_ERR_UNLIKELY;
        }
    }

  data.conn   = conn;
  data.op     = op;
  data.offset = offset;
  data.value  = value;
  data.len    = len;
  data.err    = BT_ATT_ERR_INVALID_HANDLE;

  bt_gatt_foreach_attr(handle, handle, write_cb, &data);

  /* In case of error discard data and respond with an error */

  if (data.err)
    {
      if (rsp)
        {
          bt_buf_release(data.buf);

          /* Respond here since handle is set */

          send_err_rsp(conn, op, handle, data.err);
        }

      return 0;
    }

  if (data.buf)
    {
      /* Add prepare write response */

      if (rsp == BT_ATT_OP_PREPARE_WRITE_RSP)
        {
          FAR struct bt_att_prepare_write_rsp_s *wrrsp;

          wrrsp         = bt_buf_extend(data.buf, sizeof(*wrrsp));
          wrrsp->handle = BT_HOST2LE16(handle);
          wrrsp->offset = BT_HOST2LE16(offset);
          bt_buf_extend(data.buf, len);
          memcpy(wrrsp->value, value, len);
        }

      bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
    }

  return 0;
}

static uint8_t att_write_req(FAR struct bt_conn_s *conn,
                             FAR struct bt_buf_s *data)
{
  FAR struct bt_att_write_req_s *req;
  uint16_t handle;

  req = (FAR void *)data->data;

  handle = BT_LE162HOST(req->handle);
  bt_buf_consume(data, sizeof(*req));

  wlinfo("handle 0x%04x\n", handle);

  return att_write_rsp(conn, BT_ATT_OP_WRITE_REQ, BT_ATT_OP_WRITE_RSP,
                       handle, 0, data->data, data->len);
}

static uint8_t att_prepare_write_req(FAR struct bt_conn_s *conn,
                                     FAR struct bt_buf_s *data)
{
  FAR struct bt_att_prepare_write_req_s *req;
  uint16_t handle;
  uint16_t offset;

  req    = (FAR void *)data->data;
  handle = BT_LE162HOST(req->handle);
  offset = BT_LE162HOST(req->offset);
  bt_buf_consume(data, sizeof(*req));

  wlinfo("handle 0x%04x offset %u\n", handle, offset);

  return att_write_rsp(conn, BT_ATT_OP_PREPARE_WRITE_REQ,
                       BT_ATT_OP_PREPARE_WRITE_RSP, handle, offset,
                       data->data, data->len);
}

typedef CODE uint8_t
  (*bt_gatt_attr_func_t)(FAR const struct bt_gatt_attr_s *attr,
                         FAR void *user_data);

static uint8_t flush_cb(FAR const struct bt_gatt_attr_s *attr,
                        FAR void *user_data)
{
  FAR struct flush_data_s *data = user_data;
  int err;

  /* If attribute cannot be flushed continue to next */

  if (!attr->flush)
    {
      return BT_GATT_ITER_CONTINUE;
    }

  wlinfo("handle 0x%04x flags 0x%02x\n", attr->handle, data->flags);

  /* Flush attribute any data cached to be written */

  err = attr->flush(data->conn, attr, data->flags);
  if (err < 0)
    {
      data->err = err_to_att(err);
      return BT_GATT_ITER_STOP;
    }

  data->err = 0;
  return BT_GATT_ITER_CONTINUE;
}

static uint8_t att_exec_write_rsp(FAR struct bt_conn_s *conn, uint8_t flags)
{
  struct flush_data_s data;

  memset(&data, 0, sizeof(data));

  data.buf = bt_att_create_pdu(conn, BT_ATT_OP_EXEC_WRITE_RSP, 0);
  if (!data.buf)
    {
      return BT_ATT_ERR_UNLIKELY;
    }

  data.conn = conn;
  data.flags = flags;

  /* Apply to the whole database */

  bt_gatt_foreach_attr(0x0000, 0xffff, flush_cb, &data);

  /* In case of error discard data */

  if (data.err)
    {
      bt_buf_release(data.buf);
      return data.err;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, data.buf);
  return 0;
}

static uint8_t att_exec_write_req(FAR struct bt_conn_s *conn,
                                  FAR struct bt_buf_s *data)
{
  FAR struct bt_att_exec_write_req_s *req;

  req = (FAR void *)data->data;

  wlinfo("flags 0x%02x\n", req->flags);

  return att_exec_write_rsp(conn, req->flags);
}

static uint8_t att_write_cmd(FAR struct bt_conn_s *conn,
                             FAR struct bt_buf_s *data)
{
  FAR struct bt_att_write_cmd_s *req;
  uint16_t handle;

  if (data->len < sizeof(*req))
    {
      /* Commands don't have any response */

      return 0;
    }

  req = (FAR void *)data->data;

  handle = BT_LE162HOST(req->handle);

  wlinfo("handle 0x%04x\n", handle);

  return att_write_rsp(conn, 0, 0, handle, 0, data->data, data->len);
}

static uint8_t att_signed_write_cmd(FAR struct bt_conn_s *conn,
                                    FAR struct bt_buf_s *data)
{
  FAR struct bt_att_signed_write_cmd_s *req;
  uint16_t handle;

  req = (FAR void *)data->data;

  handle = BT_LE162HOST(req->handle);
  bt_buf_consume(data, sizeof(*req));

  wlinfo("handle 0x%04x\n", handle);

  /* TODO: Validate signature */

  return att_write_rsp(conn, 0, 0, handle, 0, data->data,
                       data->len - sizeof(struct bt_att_signature_s));
}

static uint8_t att_error_rsp(FAR struct bt_conn_s *conn,
                             FAR struct bt_buf_s *data)
{
  FAR struct bt_att_s *att = conn->att;
  FAR struct bt_att_error_rsp_s *rsp;
  uint8_t err;

  rsp = (FAR void *)data->data;

  wlinfo("request 0x%02x handle 0x%04x error 0x%02x\n",
         rsp->request, BT_LE162HOST(rsp->handle), rsp->error);

  /* Match request with response */

  err = rsp->request == att->req.op ? rsp->error : BT_ATT_ERR_UNLIKELY;
  return att_handle_rsp(conn, NULL, 0, err);
}

static uint8_t att_handle_find_info_rsp(FAR struct bt_conn_s *conn,
                                        FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_find_type_rsp(FAR struct bt_conn_s *conn,
                                        FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_read_type_rsp(FAR struct bt_conn_s *conn,
                                        FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_read_rsp(FAR struct bt_conn_s *conn,
                                   FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_read_blob_rsp(FAR struct bt_conn_s *conn,
                                        FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_read_mult_rsp(FAR struct bt_conn_s *conn,
                                        FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static uint8_t att_handle_write_rsp(FAR struct bt_conn_s *conn,
                                    FAR struct bt_buf_s *buf)
{
  wlinfo("\n");

  return att_handle_rsp(conn, buf->data, buf->len, 0);
}

static void bt_att_receive(FAR struct bt_conn_s *conn,
                           FAR struct bt_buf_s *buf, FAR void *context,
                           uint16_t cid)
{
  FAR struct bt_att_hdr_s *hdr = (FAR void *)buf->data;
  uint8_t err = BT_ATT_ERR_NOT_SUPPORTED;
  size_t i;

  DEBUGASSERT(conn->att);

  if (buf->len < sizeof(*hdr))
    {
      wlerr("ERROR: Too small ATT PDU received\n");
      goto done;
    }

  wlinfo("Received ATT code 0x%02x len %u\n", hdr->code, buf->len);

  bt_buf_consume(buf, sizeof(*hdr));

  for (i = 0; i < NHANDLERS; i++)
    {
      if (hdr->code != g_handlers[i].op)
        {
          continue;
        }

      if (buf->len < g_handlers[i].expect_len)
        {
          wlerr("ERROR: Invalid len %u for code 0x%02x\n",
                buf->len, hdr->code);
          err = BT_ATT_ERR_INVALID_PDU;
          break;
        }

      err = g_handlers[i].func(conn, buf);
      break;
    }

  /* Commands don't have response */

  if ((hdr->code & BT_ATT_OP_CMD_MASK))
    {
      goto done;
    }

  if (err)
    {
      wlinfo("ATT error 0x%02x", err);
      send_err_rsp(conn, hdr->code, 0, err);
    }

done:
  bt_buf_release(buf);
}

static void bt_att_connected(FAR struct bt_conn_s *conn, FAR void *context,
                             uint16_t cid)
{
  int i;

  wlinfo("conn %p handle %u\n", conn, conn->handle);

  for (i = 0; i < CONFIG_BLUETOOTH_MAX_CONN; i++)
    {
      FAR struct bt_att_s *att = &g_bt_att_pool[i];

      if (!att->conn)
        {
          att->conn = conn;
          conn->att = att;
          att->mtu  = BT_ATT_DEFAULT_LE_MTU;
          bt_gatt_connected(conn);
          return;
        }
    }

  wlerr("ERROR: No available ATT context for conn %p\n", conn);
}

static void bt_att_disconnected(FAR struct bt_conn_s *conn,
                                FAR void *context, uint16_t cid)
{
  FAR struct bt_att_s *att = conn->att;

  if (!att)
    {
      return;
    }

  wlinfo("conn %p handle %u\n", conn, conn->handle);

  conn->att = NULL;
  memset(att, 0, sizeof(*att));
  bt_gatt_disconnected(conn);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_att_initialize(void)
{
  static struct bt_l2cap_chan_s chan =
  {
    .cid          = BT_L2CAP_CID_ATT,
    .receive      = bt_att_receive,
    .connected    = bt_att_connected,
    .disconnected = bt_att_disconnected,
  };

  memset(g_bt_att_pool, 0, sizeof(g_bt_att_pool));

  bt_l2cap_chan_register(&chan);
}

FAR struct bt_buf_s *bt_att_create_pdu(FAR struct bt_conn_s *conn,
                                       uint8_t op,
                                       size_t len)
{
  FAR struct bt_att_hdr_s *hdr;
  FAR struct bt_buf_s *buf;
  FAR FAR struct bt_att_s *att = conn->att;

  if (len + sizeof(op) > att->mtu)
    {
      wlwarn("ATT MTU exceeded, max %u, wanted %zu\n", att->mtu, len);
      return NULL;
    }

  buf = bt_l2cap_create_pdu(conn);
  if (!buf)
    {
      return NULL;
    }

  hdr = bt_buf_extend(buf, sizeof(*hdr));
  hdr->code = op;

  return buf;
}

int bt_att_send(FAR struct bt_conn_s *conn, FAR struct bt_buf_s *buf,
                bt_att_func_t func, FAR void *user_data,
                bt_att_destroy_t destroy)
{
  FAR struct bt_att_s *att;

  if (!conn)
    {
      return -EINVAL;
    }

  att = conn->att;
  if (!att)
    {
      return -ENOTCONN;
    }

  if (func)
    {
      FAR struct bt_att_hdr_s *hdr;

      /* Check if there is a request pending */

      if (att->req.func)
        {
          /* TODO: Allow more than one pending request */

          return -EBUSY;
        }

      hdr                = (void *)buf->data;
      att->req.op        = hdr->code;
      att->req.func      = func;
      att->req.user_data = user_data;
      att->req.destroy   = destroy;
    }

  bt_l2cap_send(conn, BT_L2CAP_CID_ATT, buf);
  return 0;
}

void bt_att_cancel(FAR struct bt_conn_s *conn)
{
  FAR struct bt_att_s *att;

  if (!conn)
    {
      return;
    }

  att = conn->att;
  if (!att)
    {
      return;
    }

  att_req_destroy(&att->req);
}
