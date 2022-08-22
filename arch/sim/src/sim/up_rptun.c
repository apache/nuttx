/****************************************************************************
 * arch/sim/src/sim/up_rptun.c
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

#include <nuttx/drivers/addrenv.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/list.h>

#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_rptun_shmem_s
{
  volatile uintptr_t        base;
  volatile unsigned int     seqs;
  volatile unsigned int     seqm;
  struct rptun_rsc_s        rsc;
  char                      buf[0x10000];
};

struct sim_rptun_dev_s
{
  struct list_node          node;
  struct rptun_dev_s        rptun;
  rptun_callback_t          callback;
  void                     *arg;
  bool                      master;
  unsigned int              seq;
  struct sim_rptun_shmem_s *shmem;
  struct simple_addrenv_s   addrenv[2];
  char                      cpuname[RPMSG_NAME_SIZE + 1];
  char                      shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *sim_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);

  return priv->cpuname;
}

static const char *sim_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

static const struct rptun_addrenv_s *
sim_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

static struct rptun_rsc_s *
sim_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);

  if (priv->shmem)
    {
      return &priv->shmem->rsc;
    }

  while (priv->shmem == NULL)
    {
      priv->shmem = host_alloc_shmem(priv->shmemname, sizeof(*priv->shmem),
                                     priv->master);
      usleep(1000);

      /* Master isn't ready, sleep and try again */
    }

  if (priv->master)
    {
      struct rptun_rsc_s *rsc = &priv->shmem->rsc;

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                               rpmsg_vdev);
      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                    | 1 << VIRTIO_RPMSG_F_ACK
                                    | 1 << VIRTIO_RPMSG_F_BUFSZ;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = 2;
      rsc->rpmsg_vring0.align       = 8;
      rsc->rpmsg_vring0.num         = 8;
      rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg_vring1.align       = 8;
      rsc->rpmsg_vring1.num         = 8;
      rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->config.r2h_buf_size      = 0x800;
      rsc->config.h2r_buf_size      = 0x800;

      priv->shmem->base              = (uintptr_t)priv->shmem;
    }
  else
    {
      /* Wait untils master is ready */

      while (priv->shmem->base == 0)
        {
          usleep(1000);
        }

      priv->addrenv[0].va          = (uintptr_t)priv->shmem;
      priv->addrenv[0].pa          = priv->shmem->base;
      priv->addrenv[0].size        = sizeof(*priv->shmem);

      simple_addrenv_initialize(priv->addrenv);
    }

  return &priv->shmem->rsc;
}

static bool sim_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool sim_rptun_is_master(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);
  return priv->master;
}

static int sim_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

static int sim_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

static int sim_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);

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

static int sim_rptun_register_callback(struct rptun_dev_s *dev,
                                       rptun_callback_t callback,
                                       void *arg)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;
  return 0;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_sim_rptun_ops =
{
  .get_cpuname       = sim_rptun_get_cpuname,
  .get_firmware      = sim_rptun_get_firmware,
  .get_addrenv       = sim_rptun_get_addrenv,
  .get_resource      = sim_rptun_get_resource,
  .is_autostart      = sim_rptun_is_autostart,
  .is_master         = sim_rptun_is_master,
  .start             = sim_rptun_start,
  .stop              = sim_rptun_stop,
  .notify            = sim_rptun_notify,
  .register_callback = sim_rptun_register_callback,
};

static struct list_node g_dev_list = LIST_INITIAL_VALUE(g_dev_list);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_rptun_loop(void)
{
  struct sim_rptun_dev_s *dev;

  list_for_every_entry(&g_dev_list, dev,
                       struct sim_rptun_dev_s, node)
    {
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
              dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
            }
        }
    }
}

int up_rptun_init(const char *shmemname, const char *cpuname, bool master)
{
  struct sim_rptun_dev_s *dev;
  int ret;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->master = master;
  dev->rptun.ops = &g_sim_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);
  list_add_tail(&g_dev_list, &dev->node);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      list_delete(&dev->node);
      kmm_free(dev);
    }

  return ret;
}
