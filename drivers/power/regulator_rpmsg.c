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

struct regulator_rpmsg_priv_s
{
  struct rpmsg_endpoint             ept;
  const char                        *cpuname;
  struct list_node                  node;
  struct list_node                  regulator_list;
};

struct regulator_rpmsg_s
{
  struct regulator                  *regulator;
  struct list_node                  node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int regulator_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                                  size_t len, uint32_t src, void *priv_);

static int regulator_rpmsg_enable_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_);
static int regulator_rpmsg_disable_handler(struct rpmsg_endpoint *ept,
                                           void *data, size_t len,
                                           uint32_t src, void *priv_);
static int regulator_rpmsg_getvol_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_);
static int regulator_rpmsg_setvol_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_);
static int regulator_rpmsg_isenabled_handler(struct rpmsg_endpoint *ept,
                                             void *data, size_t len,
                                             uint32_t src, void *priv_);

static int regulator_rpmsg_set_voltage(struct regulator_dev *rdev,
                                       int min_uv, int max_uv,
                                       unsigned *selector);
static int regulator_rpmsg_get_voltage(struct regulator_dev *rdev);
static int regulator_rpmsg_enable(struct regulator_dev *rdev);
static int regulator_rpmsg_disable(struct regulator_dev *rdev);
static int regulator_rpmsg_is_enabled(struct regulator_dev *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_regulator_rpmsg_lock          =  MUTEX_INITIALIZER;
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

static const struct regulator_ops g_regulator_rpmsg_ops =
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

static struct regulator_rpmsg_priv_s *
regulator_rpmsg_get_priv(const char *name)
{
  struct regulator_rpmsg_priv_s *priv;

  nxmutex_lock(&g_regulator_rpmsg_lock);

  list_for_every_entry(&g_regulator_rpmsg_priv, priv,
                       struct regulator_rpmsg_priv_s, node)
    {
      size_t len = strlen(priv->cpuname);

      if (!strncmp(priv->cpuname, name, len) &&
         (name[len] == '/' || name[len] == 0))
        {
          goto out; /* Find the target, exit */
        }
    }

  priv = NULL;

out:
  nxmutex_unlock(&g_regulator_rpmsg_lock);
  return priv;
}

static struct rpmsg_endpoint *regulator_rpmsg_get_ept(const char **name)
{
  struct regulator_rpmsg_priv_s *priv;

  priv = regulator_rpmsg_get_priv(*name);
  if (priv == NULL)
    {
      return NULL;
    }

  *name += strlen(priv->cpuname) + 1;

  return &priv->ept;
}

static struct regulator *
regulator_rpmsg_get_reg(struct rpmsg_endpoint *ept, const char *name)
{
  struct regulator_rpmsg_priv_s *priv = ept->priv;
  struct list_node *regulator_list = &priv->regulator_list;
  struct regulator_rpmsg_s *reg;

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

static void regulator_rpmsg_device_created(struct rpmsg_device *rdev,
                                           void *priv_)
{
  struct regulator_rpmsg_priv_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct regulator_rpmsg_priv_s));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;
  priv->cpuname  = rpmsg_get_cpuname(rdev);

  list_initialize(&priv->regulator_list);
  nxmutex_lock(&g_regulator_rpmsg_lock);
  list_add_head(&g_regulator_rpmsg_priv, &priv->node);
  nxmutex_unlock(&g_regulator_rpmsg_lock);

  ret = rpmsg_create_ept(&priv->ept, rdev, REGULATOR_RPMSG_EPT_NAME,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         regulator_rpmsg_ept_cb, NULL);
  if (ret)
    {
      free(priv);
    }
}

static void regulator_rpmsg_device_destroy(struct rpmsg_device *rdev,
                                           void *priv_)
{
  struct regulator_rpmsg_priv_s *priv;
  struct regulator_rpmsg_s *reg;

  priv = regulator_rpmsg_get_priv(rpmsg_get_cpuname(rdev));

  if (!priv)
    {
      return;
    }

  list_for_every_entry(&priv->regulator_list, reg,
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

  nxmutex_lock(&g_regulator_rpmsg_lock);
  list_delete(&priv->node);
  nxmutex_unlock(&g_regulator_rpmsg_lock);

  rpmsg_destroy_ept(&priv->ept);
  kmm_free(priv);
}

static int regulator_rpmsg_ept_cb(struct rpmsg_endpoint *ept, void *data,
                                  size_t len, uint32_t src, void *priv_)
{
  struct regulator_rpmsg_header_s *header = data;
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

static int regulator_rpmsg_enable_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_)
{
  struct regulator_rpmsg_enable_s *msg = data;
  struct regulator *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_enable(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_disable_handler(struct rpmsg_endpoint *ept,
                                           void *data, size_t len,
                                           uint32_t src, void *priv_)
{
  struct regulator_rpmsg_disable_s *msg = data;
  struct regulator *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_disable(regulator);
  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_getvol_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_)
{
  struct regulator_rpmsg_getvol_s *msg = data;
  struct regulator *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_get_voltage(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_setvol_handler(struct rpmsg_endpoint *ept,
                                          void *data, size_t len,
                                          uint32_t src, void *priv_)
{
  struct regulator_rpmsg_setvol_s *msg = data;
  struct regulator *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result =
    regulator_set_voltage(regulator, msg->min_uv, msg->max_uv);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_isenabled_handler(struct rpmsg_endpoint *ept,
                                             void *data, size_t len,
                                             uint32_t src, void *priv_)
{
  struct regulator_rpmsg_isenabled_s *msg = data;
  struct regulator *regulator =
                        regulator_rpmsg_get_reg(ept, msg->name);

  msg->header.result = regulator_is_enabled(regulator);

  return rpmsg_send(ept, data, len);
}

static int regulator_rpmsg_sendrecv(struct rpmsg_endpoint *ept,
                                    uint32_t command,
                                    struct regulator_rpmsg_header_s *msg,
                                    int len)
{
  struct regulator_rpmsg_cookie_s cookie = {0};
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

static int regulator_rpmsg_enable(struct regulator_dev *rdev)
{
  struct rpmsg_endpoint *ept;
  struct regulator_rpmsg_enable_s *msg;
  const char *name = rdev->desc->name;
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

static int regulator_rpmsg_disable(struct regulator_dev *rdev)
{
  struct rpmsg_endpoint *ept;
  struct regulator_rpmsg_disable_s *msg;
  const char *name = rdev->desc->name;
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

static int regulator_rpmsg_set_voltage(struct regulator_dev *rdev,
                                       int min_uv, int max_uv,
                                       unsigned *selector)
{
  struct rpmsg_endpoint *ept;
  struct regulator_rpmsg_setvol_s *msg;
  const char *name = rdev->desc->name;
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

static int regulator_rpmsg_get_voltage(struct regulator_dev *rdev)
{
  struct rpmsg_endpoint *ept;
  struct regulator_rpmsg_getvol_s *msg;
  const char *name = rdev->desc->name;
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

static int regulator_rpmsg_is_enabled(struct regulator_dev *rdev)
{
  struct rpmsg_endpoint *ept;
  struct regulator_rpmsg_isenabled_s *msg;
  const char *name = rdev->desc->name;
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

struct regulator_dev *regulator_rpmsg_get(const char *name)
{
  struct regulator_desc *desc;

  desc = kmm_zalloc(sizeof(struct regulator_desc));
  if (!desc)
    {
      return NULL;
    }

  desc->name = name;

  return regulator_register(desc, &g_regulator_rpmsg_ops, NULL);
}

/****************************************************************************
 * Name: regulator_rpmsg_init
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

int regulator_rpmsg_init(void)
{
  return rpmsg_register_callback(NULL,
                                 regulator_rpmsg_device_created,
                                 regulator_rpmsg_device_destroy,
                                 NULL);
}
