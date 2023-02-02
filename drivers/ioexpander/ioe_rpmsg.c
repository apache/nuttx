/****************************************************************************
 * drivers/ioexpander/ioe_rpmsg.c
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

#include <sys/param.h>

#include <nuttx/ioexpander/ioe_rpmsg.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/rptun/openamp.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IOE_RPMSG_EPT_FORMAT   "rpmsg-ioe-%s"

#define IOE_RPMSG_DIRECTION    0
#define IOE_RPMSG_OPTION       1
#define IOE_RPMSG_WRITEPIN     2
#define IOE_RPMSG_READPIN      3
#define IOE_RPMSG_ATTACH       4
#define IOE_RPMSG_DETACH       5
#define IOE_RPMSG_IRQ          6

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ioe_rpmsg_cookie_s
{
  int32_t                      result;
  sem_t                        sem;
};

begin_packed_struct struct ioe_rpmsg_header_s
{
  uint32_t                     command;
  int32_t                      result;
  uint64_t                     cookie;
} end_packed_struct;

begin_packed_struct struct ioe_rpmsg_direction_s
{
  struct ioe_rpmsg_header_s    header;
  uint8_t                      pin;
  uint8_t                      val;
} end_packed_struct;

#define ioe_rpmsg_writepin_s ioe_rpmsg_direction_s

begin_packed_struct struct ioe_rpmsg_option_s
{
  struct ioe_rpmsg_header_s    header;
  uint64_t                     val;
  uint32_t                     opt;
  uint8_t                      pin;
} end_packed_struct;

begin_packed_struct struct ioe_rpmsg_readpin_s
{
  struct ioe_rpmsg_header_s    header;
  uint8_t                      pin;
} end_packed_struct;

#ifdef CONFIG_IOEXPANDER_INT_ENABLE

begin_packed_struct struct ioe_rpmsg_attach_s
{
  struct ioe_rpmsg_header_s    header;
  uint64_t                     pinset;
  uint64_t                     cbfunc;
  uint64_t                     cbarg;
} end_packed_struct;

#define ioe_rpmsg_irq_s ioe_rpmsg_attach_s

begin_packed_struct struct ioe_rpmsg_detach_s
{
  struct ioe_rpmsg_header_s    header;
  uint32_t                     cbidx;
} end_packed_struct;

struct ioe_rpmsg_cb_s
{
  uint64_t                     pendset;
  uint64_t                     cbfunc;
  uint64_t                     cbarg;
  FAR void                     *handler;
  FAR struct rpmsg_endpoint    *ept;
  struct work_s                work;
};
#endif

struct ioe_rpmsg_server_s
{
  FAR struct ioexpander_dev_s  *ioe;
  FAR const char               *name;
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  struct ioe_rpmsg_cb_s        cb[CONFIG_IOEXPANDER_RPMSG_INT_NCALLBACKS];
#endif
};

struct ioe_rpmsg_client_s
{
  struct ioexpander_dev_s      ioe;
  FAR const char               *cpuname;
  FAR const char               *name;
  sem_t                        sem;
  struct rpmsg_endpoint        ept;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ioe_rpmsg_client_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv_);
static int ioe_rpmsg_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv_);

static int ioe_rpmsg_direction_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_);
static int ioe_rpmsg_option_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_);
static int ioe_rpmsg_writepin_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_);
static int ioe_rpmsg_readpin_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static int ioe_rpmsg_attach_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_);
static int ioe_rpmsg_detach_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_);
static int ioe_rpmsg_irq_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv_);
#endif

static int ioe_rpmsg_direction(FAR struct ioexpander_dev_s *dev,
                               uint8_t pin, int dir);
static int ioe_rpmsg_option(FAR struct ioexpander_dev_s *dev,
                            uint8_t pin, int opt,
                            void *regval);
static int ioe_rpmsg_writepin(FAR struct ioexpander_dev_s *dev,
                              uint8_t pin, bool value);
static int ioe_rpmsg_readpin(FAR struct ioexpander_dev_s *dev,
                             uint8_t pin, FAR bool *value);
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static void *ioe_rpmsg_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset,
                              ioe_callback_t callback,
                              FAR void *arg);
static int ioe_rpmsg_detach(FAR struct ioexpander_dev_s *dev,
                            FAR void *handle);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_ioe_rpmsg_handler[] =
{
  [IOE_RPMSG_DIRECTION] = ioe_rpmsg_direction_handler,
  [IOE_RPMSG_OPTION]    = ioe_rpmsg_option_handler,
  [IOE_RPMSG_WRITEPIN]  = ioe_rpmsg_writepin_handler,
  [IOE_RPMSG_READPIN]   = ioe_rpmsg_readpin_handler,
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  [IOE_RPMSG_ATTACH]    = ioe_rpmsg_attach_handler,
  [IOE_RPMSG_DETACH]    = ioe_rpmsg_detach_handler,
#endif
};

static const struct ioexpander_ops_s g_ioe_rpmsg_ops =
{
  ioe_rpmsg_direction,
  ioe_rpmsg_option,
  ioe_rpmsg_writepin,
  ioe_rpmsg_readpin,
  ioe_rpmsg_readpin
#ifdef CONFIG_IOEXPANDER_MULTIPIN
  , NULL
  , NULL
  , NULL
#endif
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  , ioe_rpmsg_attach
  , ioe_rpmsg_detach
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ioe_rpmsg_wait_ready(FAR struct rpmsg_endpoint *ept)
{
  int ret = 0;

  if (!is_rpmsg_ept_ready(ept))
    {
      FAR struct ioe_rpmsg_client_s *priv =
        container_of(ept, struct ioe_rpmsg_client_s, ept);

      ret = rpmsg_wait(ept, &priv->sem);
      if (ret < 0)
        {
          return ret;
        }

      rpmsg_post(ept, &priv->sem);
    }

  return ret;
}

static int ioe_rpmsg_sendrecv(FAR struct rpmsg_endpoint *ept,
                              uint32_t command,
                              FAR struct ioe_rpmsg_header_s *msg,
                              int len)
{
  struct ioe_rpmsg_cookie_s cookie =
    {
      0
    };

  int ret;

  ret = ioe_rpmsg_wait_ready(ept);
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

static int ioe_rpmsg_direction(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                               int dir)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_direction_s msg;

  msg.pin = pin;
  msg.val = dir;

  return ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_DIRECTION,
                            (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
}

static int ioe_rpmsg_readpin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                             FAR bool *value)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_readpin_s msg;
  int ret;

  msg.pin = pin;

  ret = ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_READPIN,
                           (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
  if (ret >= 0)
    {
      *value = ret;
    }

  return ret;
}

static int ioe_rpmsg_writepin(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                              bool value)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_writepin_s msg;

  msg.pin = pin;
  msg.val = value;

  return ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_WRITEPIN,
                            (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
}

static int ioe_rpmsg_option(FAR struct ioexpander_dev_s *dev, uint8_t pin,
                            int opt, void *regval)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_option_s msg;

  msg.pin = pin;
  msg.opt = opt;
  msg.val = (uintptr_t)regval;

  return ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_OPTION,
                            (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE
static void *ioe_rpmsg_attach(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset,
                              ioe_callback_t callback,
                              FAR void *arg)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_attach_s msg;
  int ret;

  msg.pinset = pinset;
  msg.cbfunc = (uintptr_t)callback;
  msg.cbarg  = (uintptr_t)arg;

  ret = ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_ATTACH,
                           (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
  if (ret >= 0)
    {
      return (FAR void *)(ret + 1);
    }

  return NULL;
}

static int ioe_rpmsg_detach(FAR struct ioexpander_dev_s *dev,
                            FAR void *handle)
{
  FAR struct ioe_rpmsg_client_s *priv = (struct ioe_rpmsg_client_s *)dev;
  struct ioe_rpmsg_detach_s msg;

  msg.cbidx = (uintptr_t)handle - 1;

  return ioe_rpmsg_sendrecv(&priv->ept, IOE_RPMSG_DETACH,
                            (struct ioe_rpmsg_header_s *)&msg, sizeof(msg));
}
#endif

static int ioe_rpmsg_direction_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_direction_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;

  msg->header.result = IOEXP_SETDIRECTION(priv->ioe, msg->pin, msg->val);

  return rpmsg_send(ept, msg, len);
}

static int ioe_rpmsg_option_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_option_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;

  msg->header.result = IOEXP_SETOPTION(priv->ioe, msg->pin, msg->opt,
                                       (void *)(uintptr_t)msg->val);

  return rpmsg_send(ept, msg, len);
}

static int ioe_rpmsg_writepin_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_writepin_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;

  msg->header.result = IOEXP_WRITEPIN(priv->ioe, msg->pin, msg->val);

  return rpmsg_send(ept, msg, len);
}

static int ioe_rpmsg_readpin_handler(FAR struct rpmsg_endpoint *ept,
                                     FAR void *data, size_t len,
                                     uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_readpin_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;
  bool val;

  msg->header.result = IOEXP_READPIN(priv->ioe, msg->pin, &val);

  if (msg->header.result >= 0)
    {
      msg->header.result = val;
    }

  return rpmsg_send(ept, msg, len);
}

#ifdef CONFIG_IOEXPANDER_INT_ENABLE

static void ioe_rpmsg_irqworker(FAR void *priv_)
{
  FAR struct ioe_rpmsg_cb_s *cb = priv_;
  struct ioe_rpmsg_irq_s msg;

  msg.pinset = cb->pendset;
  msg.cbfunc = cb->cbfunc;
  msg.cbarg  = cb->cbarg;

  msg.header.command  = IOE_RPMSG_IRQ;
  msg.header.cookie   = 0;
  rpmsg_send(cb->ept, &msg, sizeof(msg));

  cb->pendset = 0;
}

static int ioe_rpmsg_irq_cb(FAR struct ioexpander_dev_s *dev,
                            ioe_pinset_t pinset, FAR void *priv_)
{
  FAR struct ioe_rpmsg_cb_s *cb = priv_;

  cb->pendset |= pinset;

  work_queue(HPWORK, &cb->work, ioe_rpmsg_irqworker, cb, 0);

  return OK;
}

static int ioe_rpmsg_attach_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_attach_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;
  int i;

  for (i = 0; i < CONFIG_IOEXPANDER_RPMSG_INT_NCALLBACKS; i++)
    {
      if (!priv->cb[i].handler)
        {
          priv->cb[i].handler = IOEP_ATTACH(priv->ioe, msg->pinset,
                                            ioe_rpmsg_irq_cb, &priv->cb[i]);

          if (priv->cb[i].handler)
            {
              priv->cb[i].cbfunc = msg->cbfunc;
              priv->cb[i].cbarg  = msg->cbarg;
              priv->cb[i].ept    = ept;

              msg->header.result = i;
            }

          break;
        }
    }

  return rpmsg_send(ept, msg, len);
}

static int ioe_rpmsg_detach_handler(FAR struct rpmsg_endpoint *ept,
                                    FAR void *data, size_t len,
                                    uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_detach_s *msg = data;
  FAR struct ioe_rpmsg_server_s *priv = priv_;

  msg->header.result = IOEP_DETACH(priv->ioe, priv->cb[msg->cbidx].handler);

  if (msg->header.result >= 0)
    {
      priv->cb[msg->cbidx].pendset = 0;
      priv->cb[msg->cbidx].cbfunc  = 0;
      priv->cb[msg->cbidx].cbarg   = 0;
      priv->cb[msg->cbidx].ept     = NULL;
    }

  return rpmsg_send(ept, msg, len);
}

static int ioe_rpmsg_irq_handler(FAR struct rpmsg_endpoint *ept,
                                 FAR void *data, size_t len,
                                 uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_irq_s *msg = data;
  FAR struct ioe_rpmsg_client_s *priv = priv_;

  ioe_callback_t cb = (ioe_callback_t)(uintptr_t)msg->cbfunc;

  cb(&priv->ioe, msg->pinset, (FAR void *)(uintptr_t)msg->cbarg);

  return 0;
}

#endif

static int ioe_rpmsg_client_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_header_s *msg = data;
  struct ioe_rpmsg_cookie_s *cookie =
              (struct ioe_rpmsg_cookie_s *)(uintptr_t)msg->cookie;

  if (cookie)
    {
      cookie->result = msg->result;
      rpmsg_post(ept, &cookie->sem);
    }
#ifdef CONFIG_IOEXPANDER_INT_ENABLE
  else if (msg->command == IOE_RPMSG_IRQ)
    {
      ioe_rpmsg_irq_handler(ept, data, len, src, priv_);
    }
#endif

  return 0;
}

static void ioe_rpmsg_client_created(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_)
{
  FAR struct ioe_rpmsg_client_s *priv = priv_;

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      char eptname[RPMSG_NAME_SIZE];
      snprintf(eptname, RPMSG_NAME_SIZE, IOE_RPMSG_EPT_FORMAT, priv->name);

      priv->ept.priv = priv;
      rpmsg_create_ept(&priv->ept, rdev, eptname, RPMSG_ADDR_ANY,
                       RPMSG_ADDR_ANY, ioe_rpmsg_client_ept_cb, NULL);

      rpmsg_post(&priv->ept, &priv->sem);
    }
}

static void ioe_rpmsg_client_destroy(FAR struct rpmsg_device *rdev,
                                     FAR void *priv_)
{
  FAR struct ioe_rpmsg_client_s *priv = priv_;

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      rpmsg_wait(&priv->ept, &priv->sem);
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int ioe_rpmsg_server_ept_cb(FAR struct rpmsg_endpoint *ept,
                                   FAR void *data, size_t len,
                                   uint32_t src, FAR void *priv_)
{
  FAR struct ioe_rpmsg_header_s *msg = data;
  uint32_t cmd = msg->command;

  if (cmd < nitems(g_ioe_rpmsg_handler) && g_ioe_rpmsg_handler[cmd])
    {
      return g_ioe_rpmsg_handler[cmd](ept, data, len, src, priv_);
    }

  return 0;
}

static void ioe_rpmsg_server_unbind(FAR struct rpmsg_endpoint *ept)
{
  rpmsg_destroy_ept(ept);

  kmm_free(ept);
}

static bool ioe_rpmsg_server_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv_,
                                   FAR const char *name,
                                   uint32_t dest)
{
  FAR struct ioe_rpmsg_server_s *priv = priv_;
  char eptname[RPMSG_NAME_SIZE];

  snprintf(eptname, RPMSG_NAME_SIZE, IOE_RPMSG_EPT_FORMAT, priv->name);

  return !strcmp(name, eptname);
}

static void ioe_rpmsg_server_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_,
                                  FAR const char *name,
                                  uint32_t dest)
{
  FAR struct ioe_rpmsg_server_s *priv = priv_;
  FAR struct rpmsg_endpoint *ept;

  ept = kmm_zalloc(sizeof(struct rpmsg_endpoint));
  if (!ept)
    {
      return;
    }

  ept->priv = priv;

  rpmsg_create_ept(ept, rdev, name, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                   ioe_rpmsg_server_ept_cb, ioe_rpmsg_server_unbind);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioe_rpmsg_server_initialize
 *
 * Description:
 *   Initialize IO expander rpmsg server
 *
 ****************************************************************************/

int ioe_rpmsg_server_initialize(FAR const char *name,
                                FAR struct ioexpander_dev_s *ioe)
{
  FAR struct ioe_rpmsg_server_s *priv;
  int ret;

  if (!name || !ioe)
    {
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ioe_rpmsg_server_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  priv->name = name;
  priv->ioe  = ioe;

  ret = rpmsg_register_callback(priv, NULL, NULL,
                                ioe_rpmsg_server_match,
                                ioe_rpmsg_server_bind);
  if (ret < 0)
    {
      kmm_free(priv);
    }

  return ret;
}

/****************************************************************************
 * Name: ioe_rpmsg_client_initialize
 *
 * Description:
 *   Initialize IO expander rpmsg client
 *
 ****************************************************************************/

FAR struct ioexpander_dev_s *
ioe_rpmsg_client_initialize(FAR const char *cpuname, FAR const char *name)
{
  FAR struct ioe_rpmsg_client_s *priv;
  int ret;

  if (!cpuname || !name)
    {
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct ioe_rpmsg_client_s));
  if (!priv)
    {
      return NULL;
    }

  priv->ioe.ops = &g_ioe_rpmsg_ops;
  priv->cpuname = cpuname;
  priv->name    = name;

  nxsem_init(&priv->sem, 0, 0);
  ret = rpmsg_register_callback(priv, ioe_rpmsg_client_created,
                                ioe_rpmsg_client_destroy, NULL, NULL);
  if (ret < 0)
    {
      kmm_free(priv);
      return NULL;
    }
  else
    {
      return &priv->ioe;
    }
}
