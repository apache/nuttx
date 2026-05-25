/****************************************************************************
 * drivers/i2c/i2c_rpmsg_server.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/i2c/i2c_rpmsg.h>
#include <nuttx/kmalloc.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "i2c_rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_RPMSG_MAX_MSGS   8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_rpmsg_server_s
{
  FAR const char *name;
  FAR struct i2c_master_s *i2c;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_rpmsg_transfer_handler
 ****************************************************************************/

static int i2c_rpmsg_transfer_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len)
{
  FAR struct i2c_rpmsg_server_s *server = ept->priv;
  FAR struct i2c_rpmsg_transfer_s *req = data;
  FAR struct i2c_rpmsg_transfer_s *rsp;
  struct i2c_msg_s local[I2C_RPMSG_MAX_MSGS];
  FAR uint8_t *rxdata;
  FAR uint8_t *txdata;
  size_t reqoff = 0;
  size_t rspoff = 0;
  size_t datalen;
  size_t hdrlen;
  uint32_t space;
  uint32_t i;

  hdrlen = sizeof(*req) - sizeof(req->msg[0]);
  if (len < hdrlen || req->count == 0 || req->count > I2C_RPMSG_MAX_MSGS)
    {
      req->header.result = -EINVAL;
      return rpmsg_send(ept, req, len);
    }

  hdrlen += req->count * sizeof(req->msg[0]);
  if (len < hdrlen)
    {
      req->header.result = -EINVAL;
      return rpmsg_send(ept, req, len);
    }

  datalen = len - hdrlen;
  rxdata = (FAR uint8_t *)&req->msg[req->count];

  rsp = rpmsg_get_tx_payload_buffer(ept, &space, true);
  if (rsp == NULL)
    {
      req->header.result = -ENOMEM;
      return rpmsg_send(ept, req, len);
    }

  if (hdrlen > space)
    {
      rpmsg_release_tx_buffer(ept, rsp);
      req->header.result = -E2BIG;
      return rpmsg_send(ept, req, len);
    }

  memcpy(rsp, req, hdrlen);
  txdata = (FAR uint8_t *)&rsp->msg[rsp->count];

  for (i = 0; i < req->count; i++)
    {
      local[i].frequency = req->msg[i].frequency;
      local[i].addr = req->msg[i].addr;
      local[i].flags = req->msg[i].flags;
      local[i].length = req->msg[i].length;
      local[i].callback = NULL;
      local[i].arg = NULL;

      if ((req->msg[i].flags & I2C_M_READ) != 0)
        {
          /* Only reads own a response slot; only writes carry
           * request data.
           */

          if (req->msg[i].length > space - hdrlen - rspoff)
            {
              rpmsg_release_tx_buffer(ept, rsp);
              req->header.result = -E2BIG;
              return rpmsg_send(ept, req, len);
            }

          local[i].buffer = &txdata[rspoff];
          rspoff += req->msg[i].length;
        }
      else
        {
          if (req->msg[i].length > datalen - reqoff)
            {
              rpmsg_release_tx_buffer(ept, rsp);
              req->header.result = -EINVAL;
              return rpmsg_send(ept, req, len);
            }

          local[i].buffer = &rxdata[reqoff];
          reqoff += req->msg[i].length;
        }
    }

  rsp->header.result = I2C_TRANSFER(server->i2c, local, req->count);
  return rpmsg_send_nocopy(ept, rsp, hdrlen + rspoff);
}

/****************************************************************************
 * Name: i2c_rpmsg_simple_handler
 ****************************************************************************/

static int i2c_rpmsg_simple_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len)
{
  FAR struct i2c_rpmsg_server_s *server = ept->priv;
  FAR struct i2c_rpmsg_header_s *msg = data;

  if (len != sizeof(struct i2c_rpmsg_simple_s))
    {
      msg->result = -EINVAL;
      return rpmsg_send(ept, msg, len);
    }

  switch (msg->command)
    {
      case I2C_RPMSG_SETUP:
        msg->result = I2C_SETUP(server->i2c);
        break;

      case I2C_RPMSG_SHUTDOWN:
        msg->result = I2C_SHUTDOWN(server->i2c);
        break;

#ifdef CONFIG_I2C_RESET
      case I2C_RPMSG_RESET:
        msg->result = I2C_RESET(server->i2c);
        break;
#endif

      default:
        msg->result = -ENOTTY;
        break;
    }

  return rpmsg_send(ept, msg, len);
}

/****************************************************************************
 * Name: i2c_rpmsg_server_ept_cb
 ****************************************************************************/

static int i2c_rpmsg_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct i2c_rpmsg_header_s *msg = data;

  if (len < sizeof(*msg))
    {
      return -EINVAL;
    }

  switch (msg->command)
    {
      case I2C_RPMSG_TRANSFER:
        return i2c_rpmsg_transfer_handler(ept, data, len);

      case I2C_RPMSG_SETUP:
      case I2C_RPMSG_SHUTDOWN:
      case I2C_RPMSG_RESET:
        return i2c_rpmsg_simple_handler(ept, data, len);

      default:
        msg->result = -ENOTTY;
        return rpmsg_send(ept, msg, len);
    }
}

/****************************************************************************
 * Name: i2c_rpmsg_server_match
 ****************************************************************************/

static bool i2c_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv, FAR const char *name,
                                   uint32_t dest)
{
  FAR struct i2c_rpmsg_server_s *server = priv;
  char eptname[RPMSG_NAME_SIZE];

  snprintf(eptname, sizeof(eptname), I2C_RPMSG_EPT_FORMAT, server->name);
  return strcmp(name, eptname) == 0;
}

/****************************************************************************
 * Name: i2c_rpmsg_server_ept_release
 ****************************************************************************/

static void i2c_rpmsg_server_ept_release(FAR struct rpmsg_endpoint *ept)
{
  kmm_free(ept);
}

/****************************************************************************
 * Name: i2c_rpmsg_server_bind
 ****************************************************************************/

static void i2c_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct rpmsg_endpoint *ept;

  ept = kmm_zalloc(sizeof(*ept));
  if (ept == NULL)
    {
      return;
    }

  ept->priv = priv;
  ept->release_cb = i2c_rpmsg_server_ept_release;

  rpmsg_create_ept(ept, rdev, name, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   i2c_rpmsg_server_ept_cb, rpmsg_destroy_ept);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_rpmsg_server_initialize
 *
 * Description:
 *   Export a local I2C lower-half to remote RPMsg I2C clients.
 *
 * Input Parameters:
 *   name - RPMsg I2C service name.
 *   i2c  - Local I2C lower-half to export.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int i2c_rpmsg_server_initialize(FAR const char *name,
                                FAR struct i2c_master_s *i2c)
{
  FAR struct i2c_rpmsg_server_s *server;
  int ret;

  if (name == NULL || i2c == NULL)
    {
      return -EINVAL;
    }

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      return -ENOMEM;
    }

  server->name = name;
  server->i2c = i2c;

  ret = rpmsg_register_callback(server, NULL, NULL,
                                i2c_rpmsg_server_match,
                                i2c_rpmsg_server_bind);
  if (ret < 0)
    {
      kmm_free(server);
    }

  return ret;
}
