/****************************************************************************
 * arch/sim/src/sim/sim_rpmsg_virtio.c
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

#include <string.h>

#include <nuttx/drivers/addrenv.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg_virtio_lite.h>
#include <nuttx/wdog.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_RPMSG_VIRTIO_WORK_DELAY   1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_rpmsg_virtio_shmem_s
{
  volatile uintptr_t             base;
  volatile unsigned int          seqs;
  volatile unsigned int          seqm;
  volatile unsigned int          boots;
  volatile unsigned int          bootm;
  struct rpmsg_virtio_lite_rsc_s rsc;
  char                           buf[0x10000];
};

struct sim_rpmsg_virtio_dev_s
{
  struct rpmsg_virtio_lite_s      dev;
  rpmsg_virtio_callback_t         callback;
  void                            *arg;
  int                             master;
  uint32_t                        seq;
  struct sim_rpmsg_virtio_shmem_s *shmem;
  struct simple_addrenv_s         addrenv[2];
  char                            cpuname[RPMSG_NAME_SIZE + 1];
  char                            shmemname[RPMSG_NAME_SIZE + 1];

  /* Wdog for transmit */

  struct wdog_s                   wdog;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *
sim_rpmsg_virtio_get_cpuname(struct rpmsg_virtio_lite_s *dev)
{
  struct sim_rpmsg_virtio_dev_s *priv =
    container_of(dev, struct sim_rpmsg_virtio_dev_s, dev);

  return priv->cpuname;
}

static struct rpmsg_virtio_lite_rsc_s *
sim_rpmsg_virtio_get_resource(struct rpmsg_virtio_lite_s *dev)
{
  struct sim_rpmsg_virtio_dev_s *priv =
    container_of(dev, struct sim_rpmsg_virtio_dev_s, dev);
  struct rpmsg_virtio_lite_rsc_s *rsc;
  struct rpmsg_virtio_lite_cmd_s *cmd;

  priv->shmem = host_allocshmem(priv->shmemname, sizeof(*priv->shmem));
  if (!priv->shmem)
    {
      return NULL;
    }

  rsc = &priv->shmem->rsc;
  cmd = RPMSG_VIRTIO_LITE_RSC2CMD(rsc);

  if (priv->master)
    {
      memset(priv->shmem, 0, sizeof(*priv->shmem));
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS |
                                      1 << VIRTIO_RPMSG_F_ACK;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = 2;
      rsc->rpmsg_vring0.da          = 0;
      rsc->rpmsg_vring0.align       = 8;
      rsc->rpmsg_vring0.num         = 8;
      rsc->rpmsg_vring1.da          = 0;
      rsc->rpmsg_vring1.align       = 8;
      rsc->rpmsg_vring1.num         = 8;
      rsc->config.r2h_buf_size      = 2048;
      rsc->config.h2r_buf_size      = 2048;
      cmd->cmd_slave                = 0;

      priv->shmem->base = (uintptr_t)priv->shmem;
    }
  else
    {
      /* Wait untils master is ready */

      while (priv->shmem->base == 0)
        {
          usleep(1000);
        }

      cmd->cmd_master       = 0;
      priv->addrenv[0].va   = (uintptr_t)priv->shmem;
      priv->addrenv[0].pa   = priv->shmem->base;
      priv->addrenv[0].size = sizeof(*priv->shmem);

      simple_addrenv_initialize(&priv->addrenv[0]);
    }

  return rsc;
}

static int sim_rpmsg_virtio_is_master(struct rpmsg_virtio_lite_s *dev)
{
  struct sim_rpmsg_virtio_dev_s *priv =
    container_of(dev, struct sim_rpmsg_virtio_dev_s, dev);

  return priv->master;
}

static int
sim_rpmsg_virtio_register_callback(struct rpmsg_virtio_lite_s *dev,
                                   rpmsg_virtio_callback_t callback,
                                   void *arg)
{
  struct sim_rpmsg_virtio_dev_s *priv =
    container_of(dev, struct sim_rpmsg_virtio_dev_s, dev);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

static void sim_rpmsg_virtio_work(wdparm_t arg)
{
  struct sim_rpmsg_virtio_dev_s *dev = (struct sim_rpmsg_virtio_dev_s *)arg;

  if (dev->shmem != NULL)
    {
      bool should_notify = false;

      if (dev->master && dev->seq != dev->shmem->seqs)
        {
          dev->seq = dev->shmem->seqs;
          should_notify = true;
        }
      else if (!dev->master && dev->seq != dev->shmem->seqm)
        {
          dev->seq = dev->shmem->seqm;
          should_notify = true;
        }

      if (should_notify && dev->callback != NULL)
        {
          dev->callback(dev->arg, RPMSG_VIRTIO_LITE_NOTIFY_ALL);
        }
    }

  wd_start(&dev->wdog, SIM_RPMSG_VIRTIO_WORK_DELAY,
           sim_rpmsg_virtio_work, (wdparm_t)dev);
}

static int sim_rpmsg_virtio_notify(struct rpmsg_virtio_lite_s *dev,
                                   uint32_t vqid)
{
  struct sim_rpmsg_virtio_dev_s *priv =
    container_of(dev, struct sim_rpmsg_virtio_dev_s, dev);

  if (priv->master)
    {
      priv->shmem->seqm++;
    }
  else
    {
      priv->shmem->seqs++;
    }

  return 0;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_virtio_lite_ops_s g_sim_rpmsg_virtio_ops =
{
  .get_cpuname       = sim_rpmsg_virtio_get_cpuname,
  .get_resource      = sim_rpmsg_virtio_get_resource,
  .is_master         = sim_rpmsg_virtio_is_master,
  .notify            = sim_rpmsg_virtio_notify,
  .register_callback = sim_rpmsg_virtio_register_callback,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_rpmsg_virtio_init(const char *shmemname, const char *cpuname,
                          bool master)
{
  struct sim_rpmsg_virtio_dev_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->master = master;
  priv->dev.ops = &g_sim_rpmsg_virtio_ops;
  strlcpy(priv->cpuname, cpuname, RPMSG_NAME_SIZE);
  strlcpy(priv->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rpmsg_virtio_lite_initialize(&priv->dev);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  return wd_start(&priv->wdog, 0, sim_rpmsg_virtio_work, (wdparm_t)priv);
}
