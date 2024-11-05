/****************************************************************************
 * drivers/rptun/rptun_secure.c
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

#include <nuttx/rptun/rptun.h>
#include <nuttx/rptun/rptun_secure.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rptun_secure_dev_s
{
  struct rptun_dev_s      rptun;
  rptun_callback_t        callback;
  FAR void               *arg;
  bool                    master;
  FAR struct rptun_rsc_s *rsc;
  char                    cpuname[RPMSG_NAME_SIZE + 1];
  int                     irq_event;
  int                     irq_trigger;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR const char *rptun_secure_get_cpuname(FAR struct rptun_dev_s *dev);
static struct
FAR rptun_rsc_s *rptun_secure_get_resource(FAR struct rptun_dev_s *dev);
static bool rptun_secure_is_autostart(FAR struct rptun_dev_s *dev);
static bool rptun_secure_is_master(FAR struct rptun_dev_s *dev);
static int rptun_secure_start(FAR struct rptun_dev_s *dev);
static int rptun_secure_stop(FAR struct rptun_dev_s *dev);
static int rptun_secure_notify(FAR struct rptun_dev_s *dev,
                               uint32_t notifyid);
static int rptun_secure_register_callback(FAR struct rptun_dev_s *dev,
                                          rptun_callback_t callback,
                                          FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rptun_secure_ops =
{
  .get_cpuname       = rptun_secure_get_cpuname,
  .get_resource      = rptun_secure_get_resource,
  .is_autostart      = rptun_secure_is_autostart,
  .is_master         = rptun_secure_is_master,
  .start             = rptun_secure_start,
  .stop              = rptun_secure_stop,
  .notify            = rptun_secure_notify,
  .register_callback = rptun_secure_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR const char *rptun_secure_get_cpuname(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_secure_dev_s *priv = (FAR struct rptun_secure_dev_s *)dev;
  return priv->cpuname;
}

static FAR struct rptun_rsc_s *
rptun_secure_get_resource(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_secure_dev_s *priv = (FAR struct rptun_secure_dev_s *)dev;
  return priv->rsc;
}

static bool rptun_secure_is_autostart(FAR struct rptun_dev_s *dev)
{
  return true;
}

static bool rptun_secure_is_master(FAR struct rptun_dev_s *dev)
{
  FAR struct rptun_secure_dev_s *priv = (FAR struct rptun_secure_dev_s *)dev;
  return priv->master;
}

static int rptun_secure_start(FAR struct rptun_dev_s *dev)
{
  return 0;
}

static int rptun_secure_stop(FAR struct rptun_dev_s *dev)
{
  return 0;
}

static int rptun_secure_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  FAR struct rptun_secure_dev_s *priv = (FAR struct rptun_secure_dev_s *)dev;
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  up_trigger_irq(priv->irq_trigger, cpuset);
  return 0;
}

static int rptun_secure_register_callback(FAR struct rptun_dev_s *dev,
                                          rptun_callback_t callback,
                                          FAR void *arg)
{
  FAR struct rptun_secure_dev_s *priv = (FAR struct rptun_secure_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;

  if (callback)
    {
      up_enable_irq(priv->irq_event);
    }
  else
    {
      up_disable_irq(priv->irq_event);
    }

  return 0;
}

/****************************************************************************
 * Name: rprun_secure_interrupt
 *
 * Description:
 *   This is the interrupt handler.
 *
 * Input Parameters:
 *   irq      - unused
 *   context  - context, unused
 *   arg      - private data pointer
 *
 * Returned Value:
 *   OK always
 *
 ****************************************************************************/

static int rprun_secure_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rptun_secure_dev_s *priv = arg;

  if (priv != NULL && priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rptun_secure_init(FAR const char *cpuname, bool master,
                      FAR struct rptun_rsc_s *rsc, int irq_event,
                      int irq_trigger)
{
  FAR struct rptun_secure_dev_s *dev;
  int ret;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->master = master;
  dev->irq_trigger = irq_trigger;
  dev->irq_event = irq_event;
  dev->rptun.ops = &g_rptun_secure_ops;
  dev->rsc = rsc;
  strlcpy(dev->cpuname, cpuname, sizeof(dev->cpuname));

  ret = irq_attach(dev->irq_event,
                   rprun_secure_interrupt, dev);
  if (ret < 0)
    {
      kmm_free(dev);
      return ret;
    }

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      irq_detach(dev->irq_event);
      kmm_free(dev);
      return ret;
    }

  return ret;
}
