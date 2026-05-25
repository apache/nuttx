/****************************************************************************
 * drivers/i2c/i2c_rpmsg.c
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
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/semaphore.h>

#include "i2c_rpmsg.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_rpmsg_cookie_s
{
  FAR void *response;
  size_t response_len;
  int result;
  sem_t sem;
};

struct i2c_rpmsg_client_s
{
  struct i2c_master_s i2c;
  FAR const char *cpuname;
  FAR const char *name;
  sem_t sem;
  struct rpmsg_endpoint ept;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int i2c_rpmsg_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int i2c_rpmsg_reset(FAR struct i2c_master_s *dev);
#endif
static int i2c_rpmsg_setup(FAR struct i2c_master_s *dev);
static int i2c_rpmsg_shutdown(FAR struct i2c_master_s *dev);
static uint8_t i2c_rpmsg_status(FAR struct i2c_master_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_rpmsg_ops =
{
  i2c_rpmsg_transfer,
#ifdef CONFIG_I2C_RESET
  i2c_rpmsg_reset,
#endif
  i2c_rpmsg_setup,
  i2c_rpmsg_shutdown,
  i2c_rpmsg_status
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_rpmsg_wait_ready
 ****************************************************************************/

static int i2c_rpmsg_wait_ready(FAR struct rpmsg_endpoint *ept)
{
  if (!is_rpmsg_ept_ready(ept))
    {
      FAR struct i2c_rpmsg_client_s *priv =
        container_of(ept, struct i2c_rpmsg_client_s, ept);
      int ret;

      ret = rpmsg_wait(ept, &priv->sem);
      if (ret < 0)
        {
          return ret;
        }

      rpmsg_post(ept, &priv->sem);
    }

  return OK;
}

/****************************************************************************
 * Name: i2c_rpmsg_sendrecv
 ****************************************************************************/

static int i2c_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                              uint32_t command,
                              FAR struct i2c_rpmsg_header_s *msg,
                              size_t len)
{
  struct i2c_rpmsg_cookie_s cookie;
  int ret;

  cookie.response     = msg;
  cookie.response_len = len;
  cookie.result       = -ENXIO;

  ret = i2c_rpmsg_wait_ready(ept);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_init(&cookie.sem, 0, 0);

  msg->command = command;
  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send(ept, msg, len);
  if (ret >= 0)
    {
      ret = rpmsg_wait(ept, &cookie.sem);
      if (ret >= 0)
        {
          ret = cookie.result;
        }
    }

  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: i2c_rpmsg_sendrecv_nocopy
 ****************************************************************************/

static int i2c_rpmsg_sendrecv_nocopy(FAR struct rpmsg_endpoint *ept,
                                     uint32_t command,
                                     FAR struct i2c_rpmsg_header_s *msg,
                                     size_t len,
                                     FAR struct i2c_msg_s *msgs,
                                     int count)
{
  struct i2c_rpmsg_cookie_s cookie;
  int ret;

  cookie.response     = msgs;
  cookie.response_len = count;
  cookie.result       = -ENXIO;

  nxsem_init(&cookie.sem, 0, 0);

  msg->command = command;
  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send_nocopy(ept, msg, len);
  if (ret >= 0)
    {
      ret = rpmsg_wait(ept, &cookie.sem);
      if (ret >= 0)
        {
          ret = cookie.result;
        }
    }
  else
    {
      rpmsg_release_tx_buffer(ept, msg);
    }

  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: i2c_rpmsg_transfer_data
 ****************************************************************************/

static FAR uint8_t *
i2c_rpmsg_transfer_data(FAR struct i2c_rpmsg_transfer_s *msg)
{
  return (FAR uint8_t *)&msg->msg[msg->count];
}

/****************************************************************************
 * Name: i2c_rpmsg_transfer
 ****************************************************************************/

static int i2c_rpmsg_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct i2c_rpmsg_client_s *priv =
    (FAR struct i2c_rpmsg_client_s *)dev;
  FAR struct i2c_rpmsg_transfer_s *msg;
  FAR uint8_t *data;
  uint32_t space;
  size_t datalen = 0;
  size_t offset = 0;
  size_t msglen;
  int ret;
  int i;

  if (msgs == NULL || count <= 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      if (msgs[i].length < 0 ||
          (msgs[i].length > 0 && msgs[i].buffer == NULL))
        {
          return -EINVAL;
        }

      if ((msgs[i].flags & I2C_M_READ) == 0)
        {
          datalen += msgs[i].length;
        }
    }

  /* The request packs write payloads only and must fit a single rpmsg
   * payload buffer.  The server bounds the response against its own buffer.
   */

  msglen = sizeof(*msg) + (count - 1) * sizeof(msg->msg[0]) + datalen;

  ret = i2c_rpmsg_wait_ready(&priv->ept);
  if (ret < 0)
    {
      return ret;
    }

  msg = rpmsg_get_tx_payload_buffer(&priv->ept, &space, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  if (msglen > space)
    {
      rpmsg_release_tx_buffer(&priv->ept, msg);
      return -E2BIG;
    }

  msg->count = count;
  data = i2c_rpmsg_transfer_data(msg);

  for (i = 0; i < count; i++)
    {
      msg->msg[i].frequency = msgs[i].frequency;
      msg->msg[i].addr = msgs[i].addr;
      msg->msg[i].flags = msgs[i].flags;
      msg->msg[i].length = msgs[i].length;

      /* A write carries its payload; a read has no request slot. */

      if ((msgs[i].flags & I2C_M_READ) == 0 && msgs[i].length > 0)
        {
          memcpy(&data[offset], msgs[i].buffer, msgs[i].length);
          offset += msgs[i].length;
        }
    }

  return i2c_rpmsg_sendrecv_nocopy(&priv->ept, I2C_RPMSG_TRANSFER,
                                   &msg->header, msglen, msgs, count);
}

#ifdef CONFIG_I2C_RESET
/****************************************************************************
 * Name: i2c_rpmsg_reset
 ****************************************************************************/

static int i2c_rpmsg_reset(FAR struct i2c_master_s *dev)
{
  FAR struct i2c_rpmsg_client_s *priv =
    (FAR struct i2c_rpmsg_client_s *)dev;
  struct i2c_rpmsg_simple_s msg;

  return i2c_rpmsg_sendrecv(&priv->ept, I2C_RPMSG_RESET,
                            &msg.header, sizeof(msg));
}
#endif

/****************************************************************************
 * Name: i2c_rpmsg_setup
 ****************************************************************************/

static int i2c_rpmsg_setup(FAR struct i2c_master_s *dev)
{
  FAR struct i2c_rpmsg_client_s *priv =
    (FAR struct i2c_rpmsg_client_s *)dev;
  struct i2c_rpmsg_simple_s msg;

  return i2c_rpmsg_sendrecv(&priv->ept, I2C_RPMSG_SETUP,
                            &msg.header, sizeof(msg));
}

/****************************************************************************
 * Name: i2c_rpmsg_shutdown
 ****************************************************************************/

static int i2c_rpmsg_shutdown(FAR struct i2c_master_s *dev)
{
  FAR struct i2c_rpmsg_client_s *priv =
    (FAR struct i2c_rpmsg_client_s *)dev;
  struct i2c_rpmsg_simple_s msg;

  return i2c_rpmsg_sendrecv(&priv->ept, I2C_RPMSG_SHUTDOWN,
                            &msg.header, sizeof(msg));
}

/****************************************************************************
 * Name: i2c_rpmsg_status
 ****************************************************************************/

static uint8_t i2c_rpmsg_status(FAR struct i2c_master_s *dev)
{
  return I2C_STATUS_PRESENT;
}

/****************************************************************************
 * Name: i2c_rpmsg_client_ept_cb
 ****************************************************************************/

static int i2c_rpmsg_client_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv)
{
  FAR struct i2c_rpmsg_header_s *msg = data;
  FAR struct i2c_rpmsg_cookie_s *cookie =
    (FAR struct i2c_rpmsg_cookie_s *)(uintptr_t)msg->cookie;

  if (msg->command == I2C_RPMSG_TRANSFER)
    {
      FAR struct i2c_rpmsg_transfer_s *rsp = data;
      FAR struct i2c_msg_s *msgs = cookie->response;
      int count = cookie->response_len;
      FAR uint8_t *buf = NULL;
      size_t datalen = 0;
      size_t offset = 0;
      size_t hdrlen;
      int i;

      /* Validate the server-controlled response geometry against the
       * actual received rpmsg length before reading any payload, so a
       * mismatched or corrupted response cannot drive an out-of-bounds
       * read of the vring buffer.
       */

      hdrlen = sizeof(*rsp) - sizeof(rsp->msg[0]);
      if (len < hdrlen ||
          rsp->count > (len - hdrlen) / sizeof(rsp->msg[0]))
        {
          msg->result = -EINVAL;
        }
      else
        {
          hdrlen += rsp->count * sizeof(rsp->msg[0]);
          datalen = len - hdrlen;
          buf = i2c_rpmsg_transfer_data(rsp);
        }

      for (i = 0; msg->result >= 0 && i < count && i < rsp->count; i++)
        {
          if (rsp->msg[i].length < msgs[i].length)
            {
              msg->result = -EINVAL;
              break;
            }

          if ((msgs[i].flags & I2C_M_READ) != 0 && msgs[i].length > 0)
            {
              if (rsp->msg[i].length > datalen - offset)
                {
                  msg->result = -EINVAL;
                  break;
                }

              memcpy(msgs[i].buffer, &buf[offset], msgs[i].length);
              offset += rsp->msg[i].length;
            }
        }
    }
  else if (cookie->response != NULL)
    {
      memcpy(cookie->response, data, MIN(len, cookie->response_len));
    }

  cookie->result = msg->result;
  rpmsg_post(ept, &cookie->sem);

  return OK;
}

/****************************************************************************
 * Name: i2c_rpmsg_client_created
 ****************************************************************************/

static void i2c_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct i2c_rpmsg_client_s *client = priv;

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      char eptname[RPMSG_NAME_SIZE];

      snprintf(eptname, sizeof(eptname), I2C_RPMSG_EPT_FORMAT,
               client->name);

      client->ept.priv = client;
      rpmsg_create_ept(&client->ept, rdev, eptname, RPMSG_ADDR_ANY,
                       RPMSG_ADDR_ANY, i2c_rpmsg_client_ept_cb, NULL);
      rpmsg_post(&client->ept, &client->sem);
    }
}

/****************************************************************************
 * Name: i2c_rpmsg_client_destroy
 ****************************************************************************/

static void i2c_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv)
{
  FAR struct i2c_rpmsg_client_s *client = priv;

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_wait(&client->ept, &client->sem);
      rpmsg_destroy_ept(&client->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_rpmsg_initialize
 *
 * Description:
 *   Create an I2C client lower-half that forwards transfers to a remote
 *   RPMsg I2C server.
 *
 * Input Parameters:
 *   cpuname - Name of the remote CPU to connect to.
 *   name    - RPMsg I2C service name.
 *
 * Returned Value:
 *   A pointer to the I2C lower-half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *i2c_rpmsg_initialize(FAR const char *cpuname,
                                              FAR const char *name)
{
  FAR struct i2c_rpmsg_client_s *client;
  int ret;

  if (cpuname == NULL || name == NULL)
    {
      return NULL;
    }

  client = kmm_zalloc(sizeof(*client));
  if (client == NULL)
    {
      return NULL;
    }

  client->i2c.ops = &g_i2c_rpmsg_ops;
  client->cpuname = cpuname;
  client->name = name;

  nxsem_init(&client->sem, 0, 0);

  ret = rpmsg_register_callback(client, i2c_rpmsg_client_created,
                                i2c_rpmsg_client_destroy, NULL, NULL);
  if (ret < 0)
    {
      nxsem_destroy(&client->sem);
      kmm_free(client);
      return NULL;
    }

  return &client->i2c;
}
