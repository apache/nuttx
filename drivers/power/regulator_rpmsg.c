/****************************************************************************
 * drivers/power/regulator_rpmsg.c
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
#include <semaphore.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/power/consumer.h>
#include <nuttx/rptun/openamp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define REGULATOR_RPMSG_EPT_NAME    "rpmsg-regulator"

#define REGULATOR_RPMSG_ENABLE      0
#define REGULATOR_RPMSG_DISABLE     1
#define REGULATOR_RPMSG_GET_VOLTAGE 2
#define REGULATOR_RPMSG_SET_VOLTAGE 3
#define REGULATOR_RPMSG_IS_ENABLED  4

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct regulator_rpmsg_header_s
{
  uint32_t                          command : 31;
  uint32_t                          response : 1;
  int32_t                           result;
  uint64_t                          cookie;
} end_packed_struct;

begin_packed_struct struct regulator_rpmsg_enable_s
{
  struct regulator_rpmsg_header_s   header;
  char                              name[0];
} end_packed_struct;

#define regulator_rpmsg_disable_s   regulator_rpmsg_enable_s
#define regulator_rpmsg_isenabled_s regulator_rpmsg_enable_s
#define regulator_rpmsg_getvol_s    regulator_rpmsg_enable_s

begin_packed_struct struct regulator_rpmsg_setvol_s
{
  struct regulator_rpmsg_header_s   header;
  int32_t                           min_uv;
  int32_t                           max_uv;
  char                              name[0];
} end_packed_struct;

struct regulator_rpmsg_cookie_s
{
  int32_t                           result;
  sem_t                             sem;
};

struct regulator_rpmsg_client_s
{
  struct rpmsg_endpoint             ept;
  FAR const char                   *cpuname;
  struct list_node                  node;
  sem_t                             sem;
};

struct regulator_rpmsg_server_s
{
  struct rpmsg_endpoint             ept;
  struct list_node                  regulator_list;
};

struct regulator_rpmsg_s
{
  FAR struct regulator_s           *regulator;
  struct list_node                  node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int regulator_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv_);

static int regulator_rpmsg_enable_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int regulator_rpmsg_disable_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv_);
static int regulator_rpmsg_getvol_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int regulator_rpmsg_setvol_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_);
static int regulator_rpmsg_isenabled_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len,
                                             uint32_t src, FAR void *priv_);

static void regulator_rpmsg_client_created(struct rpmsg_device *rdev,
                                           FAR void *priv_);
static void regulator_rpmsg_client_destroy(struct rpmsg_device *rdev,
                                           FAR void *priv_);

static void regulator_rpmsg_server_unbind(FAR struct rpmsg_endpoint *ept);
static void regulator_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_,
                                        FAR const char *name,
                                        uint32_t dest);

static int regulator_rpmsg_set_voltage(FAR struct regulator_dev_s *rdev,
                                       int min_uv, int max_uv,
                                       FAR unsigned *selector);
static int regulator_rpmsg_get_voltage(FAR struct regulator_dev_s *rdev);
static int regulator_rpmsg_enable(FAR struct regulator_dev_s *rdev);
static int regulator_rpmsg_disable(FAR struct regulator_dev_s *rdev);
static int regulator_rpmsg_is_enabled(FAR struct regulator_dev_s *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_regulator_rpmsg_lock          =  NXMUTEX_INITIALIZER;
static struct list_node g_regulator_rpmsg_priv =
          LIST_INITIAL_VALUE(g_regulator_rpmsg_priv);

static const rpmsg_ept_cb g_regulator_rpmsg_handler[] =
{
  [REGULATOR_RPMSG_ENABLE]      = regulator_rpmsg_enable_handler,
  [REGULATOR_RPMSG_DISABLE]     = regulator_rpmsg_disable_handler,
  [REGULATOR_RPMSG_GET_VOLTAGE] = regulator_rpmsg_getvol_handler,
  [REGULATOR_RPMSG_SET_VOLTAGE] = regulator_rpmsg_setvol_handler,
  [REGULATOR_RPMSG_IS_ENABLED]  = regulator_rpmsg_isenabled_handler,
};

static const struct regulator_ops_s g_regulator_rpmsg_ops =
{
  .set_voltage = regulator_rpmsg_set_voltage,
  .get_voltage = regulator_rpmsg_get_voltage,
  .enable      = regulator_rpmsg_enable,
  .disable     = regulator_rpmsg_disable,
  .is_enabled  = regulator_rpmsg_is_enabled,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct regulator_rpmsg_client_s *
regulator_rpmsg_get_priv(FAR const char *name)
{
  FAR struct regulator_rpmsg_client_s *priv;
  FAR const char *slash = strchr(name, '/');

  if (!slash)
    {
      return NULL;
    }

  nxmutex_lock(&g_regulator_rpmsg_lock);

  list_for_every_entry(&g_regulator_rpmsg_priv, priv,
                       struct regulator_rpmsg_client_s, node)
    {
      if (!strncmp(priv->cpuname, name, slash - name))
        {
          goto out; /* Find the target, exit */
        }
    }

  priv = kmm_zalloc(sizeof(struct regulator_rpmsg_client_s));
  if (!priv)
    {
      goto out;
    }

  priv->cpuname = strndup(name, slash - name);

  list_add_head(&g_regulator_rpmsg_priv, &priv->node);

  nxmutex_unlock(&g_regulator_rpmsg_lock);

  rpmsg_register_callback(priv,
                          regulator_rpmsg_client_created,
                          regulator_rpmsg_client_destroy,
                          NULL);

  return priv;

out:
  nxmutex_unlock(&g_regulator_rpmsg_lock);
  return priv;
}

static struct rpmsg_endpoint *regulator_rpmsg_get_ept(FAR const char **name)
{
  FAR struct regulator_rpmsg_client_s *priv;
  int ret = 0;

  priv = regulator_rpmsg_get_priv(*name);

  if (!priv)
    {
      return NULL;
    }

  if (!is_rpmsg_ept_ready(&priv->ept))
    {
      ret = nxsem_wait_uninterruptible(&priv->sem);
      if (ret < 0)
        {
          return NULL;
        }

      nxsem_post(&priv->sem);
    }

  *name += strlen(priv->cpuname) + 1;

  return &priv->ept;
}

static FAR struct regulator_s *
regulator_rpmsg_get_reg(FAR struct rpmsg_endpoint *ept, FAR const char *name)
{
  FAR struct regulator_rpmsg_server_s *priv = ept->priv;
  FAR struct list_node *regulator_list = &priv->regulator_list;
  FAR struct regulator_rpmsg_s *reg;

  list_for_every_entry(regulator_list, reg,
                       struct regulator_rpmsg_s, node)
    {
       if (reg && !strcmp(reg->regulator->rdev->desc->name, name))
         {
           return reg->regulator;
         }
    }

  reg = kmm_zalloc(sizeof(*reg));
  if (!reg)
    {
      return NULL;
    }

  reg->regulator = regulator_get(name);
  if (!reg->regulator)
    {
      kmm_free(reg);
      return NULL;
    }

  list_add_head(regulator_list, &reg->node);

  return reg->regulator;
}

static void regulator_rpmsg_client_created(struct rpmsg_device *rdev,
                                           FAR void *priv_)
{
  FAR struct regulator_rpmsg_client_s *priv = priv_;

  if (!priv)
    {
      return;
    }

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      priv->ept.priv = priv;

      rpmsg_create_ept(&priv->ept, rdev, REGULATOR_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       regulator_rpmsg_ept_cb, NULL);

      nxsem_post(&priv->sem);
    }
}

static void regulator_rpmsg_client_destroy(struct rpmsg_device *rdev,
                                           FAR void *priv_)
{
  FAR struct regulator_rpmsg_client_s *priv = priv_;

  if (!priv)
    {
      return;
    }

  nxsem_wait(&priv->sem);
  rpmsg_destroy_ept(&priv->ept);
}

static void regulator_rpmsg_server_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct regulator_rpmsg_server_s *priv = ept->priv;
  FAR struct regulator_rpmsg_s *reg;
  FAR struct regulator_rpmsg_s *tmp;

  list_for_every_entry_safe(&priv->regulator_list, reg, tmp,
                            struct regulator_rpmsg_s, node)
    {
      while (regulator_is_enabled(reg->regulator))
        {
          regulator_disable(reg->regulator);
        }

      regulator_put(reg->regulator);
      list_delete(&reg->node);
      kmm_free(reg);
    }

  rpmsg_destroy_ept(ept);

  kmm_free(priv);
}

static void regulator_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                        FAR void *priv_,
                                        FAR const char *name,
                                        uint32_t dest)
{
  FAR struct regulator_rpmsg_server_s *priv;

  if (!strcmp(name, REGULATOR_RPMSG_EPT_NAME))
    {
      priv = kmm_zalloc(sizeof(struct regulator_rpmsg_server_s));
      if (!priv)
        {
          return;
        }

      priv->ept.priv = priv;

      list_initialize(&priv->regulator_list);

      rpmsg_create_ept(&priv->ept, rdev, name,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       regulator_rpmsg_ept_cb,
                       regulator_rpmsg_server_unbind);
    }
}

static int regulator_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                                  FAR void *data, size_t len,
                                  uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_header_s *header = data;
  uint32_t cmd = header->command;
  int ret = -EINVAL;

  struct regulator_rpmsg_cookie_s *cookie =
              (struct regulator_rpmsg_cookie_s *)(uintptr_t)header->cookie;

  if (cookie && header->response)
    {
      cookie->result = header->result;
      nxsem_post(&cookie->sem);
      ret = 0;
    }
  else if (cmd < ARRAY_SIZE(g_regulator_rpmsg_handler)
           && g_regulator_rpmsg_handler[cmd])
    {
      header->response = 1;
      ret = g_regulator_rpmsg_handler[cmd](ept, data, len, src, priv_);
    }

  return ret;
}

static int regulator_rpmsg_enable_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_enable_s *msg = data;
  FAR struct regulator_s *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_enable(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_disable_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_disable_s *msg = data;
  FAR struct regulator_s *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_disable(regulator);
  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_getvol_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_getvol_s *msg = data;
  FAR struct regulator_s *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_get_voltage(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_setvol_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_setvol_s *msg = data;
  FAR struct regulator_s *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result =
    regulator_set_voltage(regulator, msg->min_uv, msg->max_uv);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_isenabled_handler(FAR struct rpmsg_endpoint *ept,
                                             FAR void *data, size_t len,
                                             uint32_t src, FAR void *priv_)
{
  FAR struct regulator_rpmsg_isenabled_s *msg = data;
  FAR struct regulator_s *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_is_enabled(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                                    uint32_t command,
                                    FAR struct regulator_rpmsg_header_s *msg,
                                    int len)
{
  struct regulator_rpmsg_cookie_s cookie =
    {
      0
    };

  int ret;

  nxsem_init(&cookie.sem, 0, 0);
  nxsem_set_protocol(&cookie.sem, SEM_PRIO_NONE);

  msg->command = command;
  msg->response = 0;
  msg->result = -ENXIO;
  msg->cookie = (uintptr_t)&cookie;

  ret = rpmsg_send_nocopy(ept, msg, len);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_wait_uninterruptible(&cookie.sem);
  if (ret < 0)
    {
      return ret;
    }

  return cookie.result;
}

static int regulator_rpmsg_enable(FAR struct regulator_dev_s *rdev)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct regulator_rpmsg_enable_s *msg;
  FAR const char *name = rdev->desc->name;
  uint32_t len;

  ept = regulator_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  strcpy(msg->name, name);
  return regulator_rpmsg_sendrecv(ept, REGULATOR_RPMSG_ENABLE,
                                 (struct regulator_rpmsg_header_s *)msg,
                                  len);
}

static int regulator_rpmsg_disable(FAR struct regulator_dev_s *rdev)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct regulator_rpmsg_disable_s *msg;
  FAR const char *name = rdev->desc->name;
  uint32_t len;

  ept = regulator_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  strcpy(msg->name, name);
  return regulator_rpmsg_sendrecv(ept, REGULATOR_RPMSG_DISABLE,
                                 (struct regulator_rpmsg_header_s *)msg,
                                 len);
}

static int regulator_rpmsg_set_voltage(FAR struct regulator_dev_s *rdev,
                                       int min_uv, int max_uv,
                                       FAR unsigned *selector)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct regulator_rpmsg_setvol_s *msg;
  FAR const char *name = rdev->desc->name;
  uint32_t len;

  ept = regulator_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  strcpy(msg->name, name);
  msg->min_uv = min_uv;
  msg->max_uv = max_uv;

  return regulator_rpmsg_sendrecv(ept, REGULATOR_RPMSG_SET_VOLTAGE,
                                 (struct regulator_rpmsg_header_s *)msg,
                                 len);
}

static int regulator_rpmsg_get_voltage(FAR struct regulator_dev_s *rdev)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct regulator_rpmsg_getvol_s *msg;
  FAR const char *name = rdev->desc->name;
  uint32_t len;

  ept = regulator_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  strcpy(msg->name, name);
  return regulator_rpmsg_sendrecv(ept, REGULATOR_RPMSG_GET_VOLTAGE,
                                 (struct regulator_rpmsg_header_s *)msg,
                                  len);
}

static int regulator_rpmsg_is_enabled(FAR struct regulator_dev_s *rdev)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct regulator_rpmsg_isenabled_s *msg;
  FAR const char *name = rdev->desc->name;
  uint32_t len;

  ept = regulator_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;
  msg = rpmsg_get_tx_payload_buffer(ept, &len, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  strcpy(msg->name, name);
  return regulator_rpmsg_sendrecv(ept, REGULATOR_RPMSG_IS_ENABLED,
                                 (struct regulator_rpmsg_header_s *)msg,
                                  len);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regulator_rpmsg_get
 *
 * Description:
 *
 * Input Parameters:
 *
 *   name - the name for register the rpmsg regulator dev
 *
 * Returned Value:
 *
 *   Regulator dev pointer
 *
 ****************************************************************************/

FAR struct regulator_dev_s *regulator_rpmsg_get(FAR const char *name)
{
  FAR struct regulator_desc_s *desc;
  FAR struct regulator_dev_s *dev;

  desc = kmm_zalloc(sizeof(struct regulator_desc_s));
  if (!desc)
    {
      return NULL;
    }

  desc->name = name;

  dev = regulator_register(desc, &g_regulator_rpmsg_ops, NULL);
  if (!dev)
    {
      kmm_free(desc);
    }

  return dev;
}

/****************************************************************************
 * Name: regulator_rpmsg_server_init
 *
 * Description:
 *
 *   Establish rpmsg channel for the operations of the remote regulator
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int regulator_rpmsg_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 regulator_rpmsg_server_bind);
}
