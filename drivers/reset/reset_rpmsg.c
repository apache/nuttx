/****************************************************************************
 * drivers/reset/reset_rpmsg.c
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

#include <errno.h>
#include <debug.h>
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/reset/reset.h>
#include <nuttx/reset/reset-controller.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RESET_RPMSG_EPT_NAME    "rpmsg-reset"

#define RESET_RPMSG_ACQUIRE      0
#define RESET_RPMSG_RELEASE      1
#define RESET_RPMSG_RESET        2
#define RESET_RPMSG_ASSERT       3
#define RESET_RPMSG_DEASSERT     4
#define RESET_RPMSG_STATUS       5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct reset_rpmsg_cookie_s
{
  int32_t                        result;
  sem_t                          sem;
};

struct reset_rpmsg_server_s
{
  struct rpmsg_endpoint          ept;
  struct list_node               list;
  mutex_t                        lock;
};

struct reset_rpmsg_client_s
{
  struct rpmsg_endpoint          ept;
  struct list_node               node;
  sem_t                          sem;
  char                           cpuname[0];
};

begin_packed_struct struct reset_rpmsg_header_s
{
  uint32_t                       command : 31;
  uint32_t                       response : 1;
  int32_t                        result;
  uint64_t                       cookie;
} end_packed_struct;

begin_packed_struct struct rpmsg_reset_msg
{
  struct reset_rpmsg_header_s    header;
  int                            id;
  bool                           shared;
  bool                           acquired;
  char                           name[0];
} end_packed_struct;

struct reset_rpmsg_s
{
  FAR struct reset_control *rstc;
  struct list_node          node;
  unsigned int              count;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int reset_rpmsg_acquire_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

static int reset_rpmsg_release_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

static int reset_rpmsg_reset_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);

static int reset_rpmsg_assert_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);

static int reset_rpmsg_deassert_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);

static int reset_rpmsg_status_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);

static int reset_rpmsg_acquire(FAR struct reset_controller_dev *rcdev,
                               unsigned int id, bool shared,
                               bool acquired);

static int reset_rpmsg_release(FAR struct reset_controller_dev *rcdev,
                               unsigned int id);

static int reset_rpmsg_reset(FAR struct reset_controller_dev *rcdev,
                             unsigned int id);

static int reset_rpmsg_assert(FAR struct reset_controller_dev *rcdev,
                              unsigned int id);

static int reset_rpmsg_deassert(FAR struct reset_controller_dev *rcdev,
                                unsigned int id);

static int reset_rpmsg_status(FAR struct reset_controller_dev *rcdev,
                              unsigned int id);

static int reset_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                              FAR void *data, size_t len,
                              uint32_t src, FAR void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_reset_rpmsg_lock = NXMUTEX_INITIALIZER;
static struct list_node g_reset_rpmsg_client =
                        LIST_INITIAL_VALUE(g_reset_rpmsg_client);

static const rpmsg_ept_cb g_reset_rpmsg_handler[] =
{
  [RESET_RPMSG_ACQUIRE]  = reset_rpmsg_acquire_handler,
  [RESET_RPMSG_RELEASE]  = reset_rpmsg_release_handler,
  [RESET_RPMSG_RESET]    = reset_rpmsg_reset_handler,
  [RESET_RPMSG_ASSERT]   = reset_rpmsg_assert_handler,
  [RESET_RPMSG_DEASSERT] = reset_rpmsg_deassert_handler,
  [RESET_RPMSG_STATUS]   = reset_rpmsg_status_handler,
};

static const struct reset_control_ops g_reset_rpmsg_ops =
{
  .acquire  = reset_rpmsg_acquire,
  .release  = reset_rpmsg_release,
  .reset    = reset_rpmsg_reset,
  .assert   = reset_rpmsg_assert,
  .deassert = reset_rpmsg_deassert,
  .status   = reset_rpmsg_status,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void reset_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                       FAR void *priv)
{
  FAR struct reset_rpmsg_client_s *client = priv;

  if (client == NULL)
    {
      return;
    }

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_create_ept(&client->ept, rdev, RESET_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       reset_rpmsg_ept_cb, NULL);

      nxsem_post(&client->sem);
    }
}

static void reset_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                       FAR void *priv)
{
  FAR struct reset_rpmsg_client_s *client = priv;

  if (client == NULL)
    {
      return;
    }

  if (strcmp(client->cpuname, rpmsg_get_cpuname(rdev)) == 0)
    {
      nxsem_wait(&client->sem);
      rpmsg_destroy_ept(&client->ept);
    }
}

static FAR struct reset_rpmsg_client_s *
reset_rpmsg_get_client(FAR const char *name)
{
  FAR struct reset_rpmsg_client_s *client;
  FAR const char *slash = strchr(name, '/');

  if (slash == NULL)
    {
      return NULL;
    }

  nxmutex_lock(&g_reset_rpmsg_lock);

  list_for_every_entry(&g_reset_rpmsg_client, client,
                       struct reset_rpmsg_client_s, node)
    {
      if (strncmp(client->cpuname, name, slash - name) == 0)
        {
          goto out; /* Find the target, exit */
        }
    }

  client = kmm_zalloc(sizeof(*client) + slash - name + 1);
  if (client == NULL)
    {
      goto out;
    }

  memcpy(client->cpuname, name, slash - name);
  list_add_head(&g_reset_rpmsg_client, &client->node);
  nxmutex_unlock(&g_reset_rpmsg_lock);

  rpmsg_register_callback(client,
                          reset_rpmsg_client_created,
                          reset_rpmsg_client_destroy,
                          NULL,
                          NULL);
  return client;

out:
  nxmutex_unlock(&g_reset_rpmsg_lock);
  return client;
}

static struct rpmsg_endpoint *rpmsg_reset_get_ept(FAR const char **name)
{
  FAR struct reset_rpmsg_client_s *client;
  int ret = 0;

  client = reset_rpmsg_get_client(*name);
  if (client == NULL)
    {
      return NULL;
    }

  if (!is_rpmsg_ept_ready(&client->ept))
    {
      ret = nxsem_wait_uninterruptible(&client->sem);
      if (ret < 0)
        {
          return NULL;
        }

      nxsem_post(&client->sem);
    }

  *name += strlen(client->cpuname) + 1;
  return &client->ept;
}

static int reset_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                                uint32_t command,
                                FAR struct reset_rpmsg_header_s *msg,
                                int len)
{
  struct reset_rpmsg_cookie_s cookie =
    {
      0
    };

  int ret;

  nxsem_init(&cookie.sem, 0, 0);
  msg->command = command;
  msg->response = 0;
  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;
  ret = rpmsg_send_nocopy(ept, msg, len);
  if (ret >= 0)
    {
      ret = nxsem_wait_uninterruptible(&cookie.sem);
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

static int reset_rpmsg_acquire(FAR struct reset_controller_dev *rcdev,
                               unsigned int id, bool shared,
                               bool acquired)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  /* We have to pass name, id, shared, acquired for the remote
   * to create rstc, after that only need to pass name and id.
   */

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  msg->shared = shared;
  msg->acquired = acquired;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_ACQUIRE,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static int reset_rpmsg_release(FAR struct reset_controller_dev *rcdev,
                               unsigned int id)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_RELEASE,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static int reset_rpmsg_reset(FAR struct reset_controller_dev *rcdev,
                             unsigned int id)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_RESET,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static int reset_rpmsg_assert(FAR struct reset_controller_dev *rcdev,
                              unsigned int id)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_ASSERT,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static int reset_rpmsg_deassert(FAR struct reset_controller_dev *rcdev,
                                unsigned int id)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_DEASSERT,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static int reset_rpmsg_status(FAR struct reset_controller_dev *rcdev,
                              unsigned int id)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct rpmsg_reset_msg *msg;
  FAR const char *name = rcdev->name;
  uint32_t len;

  ept = rpmsg_reset_get_ept(&name);
  if (ept == NULL)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  strlcpy(msg->name, name, len - sizeof(*msg));
  msg->id = id;
  return reset_rpmsg_sendrecv(ept, RESET_RPMSG_STATUS,
                              (struct reset_rpmsg_header_s *)msg,
                              len);
}

static FAR struct reset_control *
reset_rpmsg_find(FAR struct rpmsg_endpoint *ept,
                 FAR const char *name, unsigned int id)
{
  FAR struct reset_rpmsg_server_s *server = ept->priv;
  FAR struct reset_rpmsg_s *reset;

  nxmutex_lock(&server->lock);
  list_for_every_entry(&server->list, reset,
                       struct reset_rpmsg_s, node)
    {
      if (strcmp(reset->rstc->rcdev->name, name) == 0)
        {
          if (reset->rstc->id == id)
            {
              nxmutex_unlock(&server->lock);
              return reset->rstc;
            }
        }
    }

  nxmutex_unlock(&server->lock);
  return NULL;
}

static int reset_rpmsg_acquire_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct reset_rpmsg_server_s *server = ept->priv;
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_control *rstc = NULL;
  FAR struct reset_rpmsg_s *reset;

  nxmutex_lock(&server->lock);
  list_for_every_entry(&server->list, reset,
                       struct reset_rpmsg_s, node)
    {
      if (strcmp(reset->rstc->rcdev->name, msg->name) == 0)
        {
          if (reset->rstc->id == msg->id)
            {
              reset->count++;
              nxmutex_unlock(&server->lock);
              rstc = reset->rstc;
              goto out;
            }
        }
    }

  reset = kmm_zalloc(sizeof(*reset));
  if (reset == NULL)
    {
      nxmutex_unlock(&server->lock);
      goto out;
    }

  reset->rstc = reset_control_get(msg->name, msg->id,
                                  msg->shared, msg->acquired);
  if (reset->rstc == NULL)
    {
      kmm_free(reset);
      nxmutex_unlock(&server->lock);
      goto out;
    }

  reset->count++;
  rstc = reset->rstc;
  list_add_head(&server->list, &reset->node);
  nxmutex_unlock(&server->lock);

out:
  msg->header.result = rstc ? 0 : -ENOENT;
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_release_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct reset_rpmsg_server_s *server = ept->priv;
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_rpmsg_s *reset;

  nxmutex_lock(&server->lock);
  list_for_every_entry(&server->list, reset,
                       struct reset_rpmsg_s, node)
    {
      if (strcmp(reset->rstc->rcdev->name, msg->name) == 0)
        {
          if (reset->rstc->id == msg->id)
            {
              msg->header.result = 0;
              if (--reset->count == 0)
                {
                  reset_control_put(reset->rstc);
                  list_delete(&reset->node);
                  kmm_free(reset);
                  break;
                }
            }
        }
    }

  nxmutex_unlock(&server->lock);
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_reset_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_control *rstc = reset_rpmsg_find(ept, msg->name, msg->id);

  msg->header.result = reset_control_reset(rstc);
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_assert_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_control *rstc = reset_rpmsg_find(ept, msg->name, msg->id);

  msg->header.result = reset_control_assert(rstc);
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_deassert_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_control *rstc = reset_rpmsg_find(ept, msg->name, msg->id);

  msg->header.result = reset_control_deassert(rstc);
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_status_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_reset_msg *msg = data;
  FAR struct reset_control *rstc = reset_rpmsg_find(ept, msg->name, msg->id);

  msg->header.result = reset_control_status(rstc);
  return rpmsg_send(ept, data, len);
}

static int reset_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                              FAR void *data, size_t len,
                              uint32_t src, FAR void *priv)
{
  FAR struct reset_rpmsg_header_s *header = data;
  uint32_t cmd = header->command;
  int ret = -EINVAL;

  struct reset_rpmsg_cookie_s *cookie =
         (struct reset_rpmsg_cookie_s *)(uintptr_t)header->cookie;

  if (cookie && header->response)
    {
      cookie->result = header->result;
      nxsem_post(&cookie->sem);
      ret = 0;
    }
  else if (cmd < nitems(g_reset_rpmsg_handler)
           && g_reset_rpmsg_handler[cmd])
    {
      header->response = 1;
      ret = g_reset_rpmsg_handler[cmd](ept, data, len, src, priv);
    }

  return ret;
}

static bool reset_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                     FAR void *priv,
                                     FAR const char *name,
                                     uint32_t dest)
{
  return strcmp(name, RESET_RPMSG_EPT_NAME) == 0;
}

static void reset_rpmsg_server_ept_release(FAR struct rpmsg_endpoint *ept,
                                           FAR void *priv)
{
  FAR struct reset_rpmsg_server_s *server = ept->priv;
  FAR struct reset_rpmsg_s *reset;
  FAR struct reset_rpmsg_s *tmp;

  list_for_every_entry_safe(&server->list, reset, tmp,
                            struct reset_rpmsg_s, node)
    {
      reset_control_put(reset->rstc);
      list_delete(&reset->node);
      kmm_free(reset);
    }

  nxmutex_destroy(&server->lock);
  kmm_free(server);
}

static void reset_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                    FAR void *priv,
                                    FAR const char *name,
                                    uint32_t dest)
{
  FAR struct reset_rpmsg_server_s *server;

  server = kmm_zalloc(sizeof(struct reset_rpmsg_server_s));
  if (server == NULL)
    {
      return;
    }

  server->ept.priv = server;
  server->ept.release_cb = reset_rpmsg_server_ept_release;
  list_initialize(&server->list);
  nxmutex_init(&server->lock);

  rpmsg_create_ept(&server->ept, rdev, name,
                   RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   reset_rpmsg_ept_cb,
                   rpmsg_destroy_ept);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_rpmsg_get
 *
 * Description:
 *
 * Input Parameters:
 *
 *   name - the name of the remote reset controller
 *
 * Returned Value:
 *
 *   Reset controller pointer
 *
 ****************************************************************************/

FAR struct reset_controller_dev *reset_rpmsg_get(FAR const char *name)
{
  FAR struct reset_controller_dev *rcdev;
  size_t len = strlen(name) + 1;
  int ret;

  rcdev = kmm_zalloc(sizeof(struct reset_controller_dev) + len);
  if (rcdev == NULL)
    {
      return NULL;
    }

  rcdev->name = (FAR char *)(rcdev + 1);
  memcpy((FAR char *)rcdev->name, name, len);
  rcdev->ops = &g_reset_rpmsg_ops;

  ret = reset_controller_register(rcdev);
  if (ret < 0)
    {
      kmm_free(rcdev);
      return NULL;
    }

  return rcdev;
}

/****************************************************************************
 * Name: reset_rpmsg_server_init
 *
 * Description:
 *
 *   Establish rpmsg channel for the operations of the
 *   remote reset controller
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int reset_rpmsg_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 reset_rpmsg_server_match,
                                 reset_rpmsg_server_bind);
}
