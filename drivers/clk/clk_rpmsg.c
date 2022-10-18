/****************************************************************************
 * drivers/clk/clk_rpmsg.c
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

#include <string.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk_provider.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/rptun/openamp.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CLK_RPMSG_EPT_NAME          "rpmsg-clk"

#define CLK_RPMSG_ENABLE            0
#define CLK_RPMSG_DISABLE           1
#define CLK_RPMSG_SETRATE           2
#define CLK_RPMSG_SETPHASE          3
#define CLK_RPMSG_GETPHASE          4
#define CLK_RPMSG_GETRATE           5
#define CLK_RPMSG_ROUNDRATE         6
#define CLK_RPMSG_ISENABLED         7

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)             (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct clk_rpmsg_client_s
{
  struct rpmsg_endpoint     ept;
  struct list_node          node;
  FAR const char            *cpuname;
  sem_t                     sem;
};

struct clk_rpmsg_server_s
{
  struct rpmsg_endpoint     ept;
  struct list_node          clk_list;
};

struct clk_rpmsg_s
{
  FAR struct clk_s          *clk;
  uint32_t                  count;
  struct list_node          node;
};

struct clk_rpmsg_cookie_s
{
  sem_t                     sem;
  int64_t                   result;
};

begin_packed_struct struct clk_rpmsg_header_s
{
  uint32_t                  command;
  uint32_t                  response;
  int64_t                   result;
  uint64_t                  cookie;
} end_packed_struct;

begin_packed_struct struct clk_rpmsg_enable_s
{
  struct clk_rpmsg_header_s header;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_disable_s clk_rpmsg_enable_s
#define clk_rpmsg_isenabled_s clk_rpmsg_enable_s

begin_packed_struct struct clk_rpmsg_setrate_s
{
  struct clk_rpmsg_header_s header;
  uint64_t                  rate;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_getrate_s clk_rpmsg_enable_s
#define clk_rpmsg_roundrate_s clk_rpmsg_setrate_s

begin_packed_struct struct clk_rpmsg_setphase_s
{
  struct clk_rpmsg_header_s header;
  int32_t                   degrees;
  char                      name[0];
} end_packed_struct;

#define clk_rpmsg_getphase_s clk_rpmsg_enable_s

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR struct clk_rpmsg_client_s *
clk_rpmsg_get_priv(FAR const char *name);
static FAR struct rpmsg_endpoint *clk_rpmsg_get_ept(FAR const char **name);
static FAR struct clk_rpmsg_s *
clk_rpmsg_get_clk(FAR struct rpmsg_endpoint *ept,
                  FAR const char *name);

static int clk_rpmsg_enable_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv);
static int clk_rpmsg_disable_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int clk_rpmsg_getrate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int clk_rpmsg_roundrate_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);
static int clk_rpmsg_setrate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int clk_rpmsg_setphase_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int clk_rpmsg_getphase_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv);
static int clk_rpmsg_isenabled_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

static void clk_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_);
static void clk_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_);

static bool clk_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_,
                                   FAR const char *name,
                                   uint32_t dest);
static void clk_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_,
                                  FAR const char *name,
                                  uint32_t dest);
static void clk_rpmsg_server_unbind(FAR struct rpmsg_endpoint *ept);

static int clk_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len,
                            uint32_t src, FAR void *priv);

static int64_t clk_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                                  uint32_t command,
                                  FAR struct clk_rpmsg_header_s *msg,
                                  int32_t len);

static int clk_rpmsg_enable(FAR struct clk_s *clk);
static void clk_rpmsg_disable(FAR struct clk_s *clk);
static int clk_rpmsg_is_enabled(FAR struct clk_s *clk);
static uint32_t clk_rpmsg_round_rate(FAR struct clk_s *clk, uint32_t rate,
                                     FAR uint32_t *parent_rate);

static int clk_rpmsg_set_rate(FAR struct clk_s *clk,
                              uint32_t rate, uint32_t parent_rate);
static uint32_t clk_rpmsg_recalc_rate(FAR struct clk_s *clk,
                                      uint32_t parent_rate);
static int clk_rpmsg_get_phase(FAR struct clk_s *clk);
static int clk_rpmsg_set_phase(FAR struct clk_s *clk, int degrees);

/****************************************************************************
 * Private Datas
 ****************************************************************************/

static mutex_t g_clk_rpmsg_lock          = NXMUTEX_INITIALIZER;
static struct list_node g_clk_rpmsg_priv =
              LIST_INITIAL_VALUE(g_clk_rpmsg_priv);

static const rpmsg_ept_cb g_clk_rpmsg_handler[] =
{
  [CLK_RPMSG_ENABLE]    = clk_rpmsg_enable_handler,
  [CLK_RPMSG_DISABLE]   = clk_rpmsg_disable_handler,
  [CLK_RPMSG_SETRATE]   = clk_rpmsg_setrate_handler,
  [CLK_RPMSG_SETPHASE]  = clk_rpmsg_setphase_handler,
  [CLK_RPMSG_GETPHASE]  = clk_rpmsg_getphase_handler,
  [CLK_RPMSG_GETRATE]   = clk_rpmsg_getrate_handler,
  [CLK_RPMSG_ROUNDRATE] = clk_rpmsg_roundrate_handler,
  [CLK_RPMSG_ISENABLED] = clk_rpmsg_isenabled_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct clk_rpmsg_client_s *
clk_rpmsg_get_priv(FAR const char *name)
{
  FAR struct clk_rpmsg_client_s *priv;
  FAR const char *slash = strchr(name, '/');

  if (!slash)
    {
      return NULL;
    }

  nxmutex_lock(&g_clk_rpmsg_lock);

  list_for_every_entry(&g_clk_rpmsg_priv, priv,
                       struct clk_rpmsg_client_s, node)
    {
      if (!strncmp(priv->cpuname, name, slash - name))
        {
          goto out;
        }
    }

  priv = kmm_zalloc(sizeof(struct clk_rpmsg_client_s));
  if (!priv)
    {
      goto out;
    }

  priv->cpuname = strndup(name, slash - name);

  list_add_head(&g_clk_rpmsg_priv, &priv->node);

  nxmutex_unlock(&g_clk_rpmsg_lock);

  rpmsg_register_callback(priv,
                          clk_rpmsg_client_created,
                          clk_rpmsg_client_destroy,
                          NULL,
                          NULL);
  return priv;

out:
  nxmutex_unlock(&g_clk_rpmsg_lock);
  return priv;
}

static FAR struct rpmsg_endpoint *clk_rpmsg_get_ept(FAR const char **name)
{
  FAR struct clk_rpmsg_client_s *priv;

  priv = clk_rpmsg_get_priv(*name);
  if (priv == NULL)
    {
      return NULL;
    }

  *name += strlen(priv->cpuname) + 1;

  return &priv->ept;
}

static FAR struct clk_rpmsg_s *
clk_rpmsg_get_clk(FAR struct rpmsg_endpoint *ept, FAR const char *name)
{
  FAR struct clk_rpmsg_server_s *priv = ept->priv;
  FAR struct list_node *clk_list = &priv->clk_list;
  FAR struct clk_rpmsg_s *clkrp;

  list_for_every_entry(clk_list, clkrp, struct clk_rpmsg_s, node)
    {
      if (!strcmp(clk_get_name(clkrp->clk), name))
        {
          return clkrp;
        }
    }

  clkrp = kmm_zalloc(sizeof(*clkrp));
  if (!clkrp)
    {
      return NULL;
    }

  clkrp->clk = clk_get(name);
  if (!clkrp->clk)
    {
      kmm_free(clkrp);
      return NULL;
    }

  list_add_head(clk_list, &clkrp->node);

  return clkrp;
}

static int clk_rpmsg_enable_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_enable_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_enable(clkrp->clk);
      if (!msg->header.result)
        {
          clkrp->count++;
        }
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_disable_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_disable_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      clk_disable(clkrp->clk);
      clkrp->count--;
      msg->header.result = 0;
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_getrate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_getrate_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_get_rate(clkrp->clk);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int
clk_rpmsg_roundrate_handler(FAR struct rpmsg_endpoint *ept,
                            FAR void *data, size_t len,
                            uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_roundrate_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_round_rate(clkrp->clk, msg->rate);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_setrate_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_setrate_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_set_rate(clkrp->clk, msg->rate);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_setphase_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_setphase_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_set_phase(clkrp->clk, msg->degrees);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_getphase_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_getphase_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_get_phase(clkrp->clk);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int clk_rpmsg_isenabled_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_isenabled_s *msg = data;
  FAR struct clk_rpmsg_s *clkrp = clk_rpmsg_get_clk(ept, msg->name);

  if (clkrp)
    {
      msg->header.result = clk_is_enabled(clkrp->clk);
    }
  else
    {
      msg->header.result = -ENOENT;
    }

  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int64_t clk_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                                  uint32_t command,
                                  FAR struct clk_rpmsg_header_s *msg,
                                  int32_t len)
{
  struct clk_rpmsg_cookie_s cookie;
  int ret;

  msg->command  = command;
  msg->response = 0;
  msg->cookie   = (uintptr_t)&cookie;

  nxsem_init(&cookie.sem, 0, 0);
  cookie.result  = -EIO;

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

static bool clk_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_,
                                   FAR const char *name,
                                   uint32_t dest)
{
  return !strcmp(name, CLK_RPMSG_EPT_NAME);
}

static void clk_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_,
                                  FAR const char *name,
                                  uint32_t dest)
{
  FAR struct clk_rpmsg_server_s *priv;

  priv = kmm_zalloc(sizeof(struct clk_rpmsg_server_s));
  if (!priv)
    {
      return;
    }

  priv->ept.priv = priv;

  list_initialize(&priv->clk_list);

  rpmsg_create_ept(&priv->ept, rdev, name,
                   RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   clk_rpmsg_ept_cb,
                   clk_rpmsg_server_unbind);
}

static void clk_rpmsg_server_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR struct clk_rpmsg_server_s *priv = ept->priv;
  FAR struct clk_rpmsg_s *clkrp_tmp;
  FAR struct clk_rpmsg_s *clkrp;

  list_for_every_entry_safe(&priv->clk_list, clkrp, clkrp_tmp,
                            struct clk_rpmsg_s, node)
    {
      while (clkrp->count--)
        {
          clk_disable(clkrp->clk);
        }

      list_delete(&clkrp->node);
      kmm_free(clkrp);
    }

  rpmsg_destroy_ept(ept);

  kmm_free(priv);
}

static void clk_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_)
{
  struct clk_rpmsg_client_s *priv = priv_;

  if (!priv)
    {
      return;
    }

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      priv->ept.priv = priv;

      rpmsg_create_ept(&priv->ept, rdev, CLK_RPMSG_EPT_NAME,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       clk_rpmsg_ept_cb, NULL);

      nxsem_post(&priv->sem);
    }
}

static void clk_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_)
{
  struct clk_rpmsg_client_s *priv = priv_;

  if (!priv)
    {
      return;
    }

  nxsem_wait(&priv->sem);
  rpmsg_destroy_ept(&priv->ept);
}

static int clk_rpmsg_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                            size_t len, uint32_t src, FAR void *priv)
{
  FAR struct clk_rpmsg_header_s *hdr = data;
  uint32_t cmd = hdr->command;
  int ret = -EINVAL;

  if (hdr->response)
    {
      FAR struct clk_rpmsg_cookie_s *cookie =
      (struct clk_rpmsg_cookie_s *)(uintptr_t)hdr->cookie;
      if (cookie)
        {
          cookie->result = hdr->result;
          nxsem_post(&cookie->sem);
          ret = 0;
        }
    }
  else if (cmd < ARRAY_SIZE(g_clk_rpmsg_handler)
           && g_clk_rpmsg_handler[cmd])
    {
      hdr->response = 1;
      ret = g_clk_rpmsg_handler[cmd](ept, data, len, src, priv);
    }

  return ret;
}

static int clk_rpmsg_enable(FAR struct clk_s *clk)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_enable_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= size);

  strcpy(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_ENABLE,
                           (struct clk_rpmsg_header_s *)msg,
                            len);
}

static void clk_rpmsg_disable(FAR struct clk_s *clk)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_disable_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
    return;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return;
    }

  DEBUGASSERT(len <= size);

  strcpy(msg->name, name);

  clk_rpmsg_sendrecv(ept, CLK_RPMSG_DISABLE,
                    (struct clk_rpmsg_header_s *)msg, len);
}

static int clk_rpmsg_is_enabled(FAR struct clk_s *clk)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_enable_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= size);

  strcpy(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_ISENABLED,
                           (struct clk_rpmsg_header_s *)msg, len);
}

static uint32_t clk_rpmsg_round_rate(FAR struct clk_s *clk, uint32_t rate,
                                     FAR uint32_t *parent_rate)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_roundrate_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;
  int64_t ret;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return 0;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return 0;
    }

  DEBUGASSERT(len <= size);

  msg->rate = rate;
  strcpy(msg->name, name);

  ret = clk_rpmsg_sendrecv(ept, CLK_RPMSG_ROUNDRATE,
                          (struct clk_rpmsg_header_s *)msg, len);
  if (ret < 0)
    {
      return 0;
    }

  return ret;
}

static int clk_rpmsg_set_rate(FAR struct clk_s *clk, uint32_t rate,
                              uint32_t parent_rate)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_setrate_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= size);

  msg->rate = rate;
  strcpy(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_SETRATE,
                           (struct clk_rpmsg_header_s *)msg, len);
}

static uint32_t clk_rpmsg_recalc_rate(FAR struct clk_s *clk,
                                      uint32_t parent_rate)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_getrate_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;
  int64_t ret;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return 0;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return 0;
    }

  DEBUGASSERT(len <= size);

  strcpy(msg->name, name);

  ret = clk_rpmsg_sendrecv(ept, CLK_RPMSG_GETRATE,
                          (struct clk_rpmsg_header_s *)msg, len);
  if (ret < 0)
    {
      return 0;
    }

  return ret;
}

static int clk_rpmsg_get_phase(FAR struct clk_s *clk)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_getphase_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= size);

  strcpy(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_GETPHASE,
                           (struct clk_rpmsg_header_s *)msg, len);
}

static int clk_rpmsg_set_phase(FAR struct clk_s *clk, int degrees)
{
  FAR struct rpmsg_endpoint *ept;
  FAR struct clk_rpmsg_setphase_s *msg;
  FAR const char *name = clk->name;
  uint32_t size;
  uint32_t len;

  ept = clk_rpmsg_get_ept(&name);
  if (!ept)
    {
      return -ENODEV;
    }

  len = sizeof(*msg) + strlen(name) + 1;

  msg = rpmsg_get_tx_payload_buffer(ept, &size, true);
  if (!msg)
    {
      return -ENOMEM;
    }

  DEBUGASSERT(len <= size);

  msg->degrees = degrees;
  strcpy(msg->name, name);

  return clk_rpmsg_sendrecv(ept, CLK_RPMSG_SETPHASE,
                           (struct clk_rpmsg_header_s *)msg, len);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops_s g_clk_rpmsg_ops =
{
  .enable = clk_rpmsg_enable,
  .disable = clk_rpmsg_disable,
  .is_enabled = clk_rpmsg_is_enabled,
  .recalc_rate = clk_rpmsg_recalc_rate,
  .round_rate = clk_rpmsg_round_rate,
  .set_rate = clk_rpmsg_set_rate,
  .set_phase = clk_rpmsg_set_phase,
  .get_phase = clk_rpmsg_get_phase,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct clk_s *clk_register_rpmsg(FAR const char *name, uint8_t flags)
{
  if (strchr(name, '/') == NULL)
    {
      return NULL;
    }

  return clk_register(name, NULL, 0, flags | CLK_IS_CRITICAL,
                      &g_clk_rpmsg_ops, NULL, 0);
}

int clk_rpmsg_server_initialize(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 clk_rpmsg_server_match,
                                 clk_rpmsg_server_bind);
}
