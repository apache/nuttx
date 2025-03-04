/****************************************************************************
 * arch/sim/src/sim/sim_rptun.c
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

#include <nuttx/nuttx.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/list.h>
#include <nuttx/wdog.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_RPTUN_WORK_DELAY 1

/* Status byte for master/slave to report progress */

#define SIM_RPTUN_STATUS_BOOT        0x01
#define SIM_RPTUN_STATUS_OK          0x02
#define SIM_RPTUN_STATUS_NEED_RESET  0x04

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_rptun_shmem_s
{
  volatile uint64_t         base;
  volatile uint32_t         seqs;
  volatile uint32_t         seqm;
  volatile uint32_t         boots;
  volatile uint32_t         bootm;
  struct rptun_rsc_s        rsc;
  char                      buf[0x10000];
};

struct sim_rptun_dev_s
{
  struct rptun_dev_s        rptun;
  rptun_callback_t          callback;
  void                     *arg;
  int                       master;
  uint32_t                  seq;
  struct sim_rptun_shmem_s *shmem;
  struct simple_addrenv_s   addrenv[2];
  struct rptun_addrenv_s    raddrenv[2];
  char                      cpuname[RPMSG_NAME_SIZE + 1];
  char                      shmemname[RPMSG_NAME_SIZE + 1];
  pid_t                     pid;

  /* Wdog for transmit */

  struct wdog_s             wdog;
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

static const struct rptun_addrenv_s *
sim_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);

  return &priv->raddrenv[0];
}

static struct rptun_rsc_s *
sim_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                                 struct sim_rptun_dev_s, rptun);
  struct rptun_cmd_s *cmd;

  priv->shmem = host_allocshmem(priv->shmemname,
                                sizeof(*priv->shmem));
  if (!priv->shmem)
    {
      return NULL;
    }

  cmd = RPTUN_RSC2CMD(&priv->shmem->rsc);

  priv->raddrenv[0].da   = 0;
  priv->raddrenv[0].size = sizeof(*priv->shmem);

  if (priv->master)
    {
      struct rptun_rsc_s *rsc = &priv->shmem->rsc;

      memset(priv->shmem->buf, 0, sizeof(priv->shmem->buf));
      memset(rsc, 0, sizeof(struct rptun_rsc_s));

      priv->raddrenv[0].pa          = (uintptr_t)priv->shmem;

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
      rsc->rpmsg_vring0.da          = 0;
      rsc->rpmsg_vring0.align       = 8;
      rsc->rpmsg_vring0.num         = 8;
      rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->rpmsg_vring1.da          = 0;
      rsc->rpmsg_vring1.align       = 8;
      rsc->rpmsg_vring1.num         = 8;
      rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
      rsc->config.r2h_buf_size      = 0x800;
      rsc->config.h2r_buf_size      = 0x800;

      cmd->cmd_slave                = 0;
      priv->shmem->base             = (uintptr_t)priv->shmem;

      /* The master notifies its slave when it starts again */

      if (priv->shmem->boots & SIM_RPTUN_STATUS_OK)
        {
          priv->shmem->boots = SIM_RPTUN_STATUS_NEED_RESET;
        }

      priv->shmem->bootm = SIM_RPTUN_STATUS_BOOT;
    }
  else
    {
      /* The slave notifies its master when it starts again */

      if (priv->shmem->boots & SIM_RPTUN_STATUS_OK)
        {
          priv->shmem->bootm = SIM_RPTUN_STATUS_NEED_RESET;
        }

      priv->shmem->boots = SIM_RPTUN_STATUS_BOOT;

      /* Wait untils master is ready */

      while (!(priv->shmem->bootm & SIM_RPTUN_STATUS_OK))
        {
          usleep(1000);
        }

      cmd->cmd_master       = 0;

      priv->raddrenv[0].pa  = (uintptr_t)priv->shmem->base;

      priv->shmem->boots    = SIM_RPTUN_STATUS_OK;

      priv->addrenv[0].va   = (uintptr_t)priv->shmem;
      priv->addrenv[0].pa   = priv->shmem->base;
      priv->addrenv[0].size = sizeof(*priv->shmem);

      simple_addrenv_initialize(&priv->addrenv[0]);
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
  struct sim_rptun_dev_s *priv = container_of(dev,
                            struct sim_rptun_dev_s, rptun);
  pid_t pid;

  if (priv->master & SIM_RPTUN_BOOT)
    {
      pid = host_posix_spawn(sim_rptun_get_cpuname(dev), NULL, NULL);
      if (pid < 0)
        {
          return pid;
        }

      priv->pid = pid;
    }

  /* Wait until slave has started */

  while (!(priv->shmem->boots & SIM_RPTUN_STATUS_BOOT))
    {
      usleep(1000);
    }

  priv->shmem->bootm = SIM_RPTUN_STATUS_OK;
  return 0;
}

static int sim_rptun_stop(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = container_of(dev,
                              struct sim_rptun_dev_s, rptun);
  struct rptun_cmd_s *cmd = RPTUN_RSC2CMD(&priv->shmem->rsc);

  /* Don't send RPTUN_CMD_STOP when slave recovery */

  if (priv->shmem->boots & SIM_RPTUN_STATUS_OK)
    {
      cmd->cmd_master = RPTUN_CMD(RPTUN_CMD_STOP, 0);
    }

  if ((priv->master & SIM_RPTUN_BOOT) && priv->pid > 0)
    {
      host_waitpid(priv->pid);
    }

  /* Master cleans shmem when both sides are about to exit */

  if (priv->shmem && (priv->shmem->boots & SIM_RPTUN_STATUS_OK))
    {
      host_freeshmem(priv->shmem);
      priv->shmem = NULL;
      host_unlinkshmem(priv->shmemname);
    }

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

static void sim_rptun_check_cmd(struct sim_rptun_dev_s *priv)
{
  struct rptun_cmd_s *rcmd = RPTUN_RSC2CMD(&priv->shmem->rsc);
  uint32_t cmd = priv->master ? rcmd->cmd_slave : rcmd->cmd_master;

  switch (RPTUN_GET_CMD(cmd))
    {
      case RPTUN_CMD_STOP:
        host_abort(RPTUN_GET_CMD_VAL(cmd));
        break;

      default:
        break;
    }
}

static void sim_rptun_check_reset(struct sim_rptun_dev_s *priv)
{
  if (priv->master &&
      (priv->shmem->bootm & SIM_RPTUN_STATUS_NEED_RESET))
    {
      priv->shmem->bootm = 0;
      rptun_boot(priv->cpuname);
    }
  else if (!priv->master &&
           (priv->shmem->boots & SIM_RPTUN_STATUS_NEED_RESET))
    {
      priv->shmem->boots = 0;
      rptun_boot(priv->cpuname);
    }
}

static void sim_rptun_work(wdparm_t arg)
{
  struct sim_rptun_dev_s *dev = (struct sim_rptun_dev_s *)arg;

  if (dev->shmem != NULL)
    {
      bool should_notify = false;

      sim_rptun_check_cmd(dev);

      /* Check if master/slave need to reset */

      sim_rptun_check_reset(dev);

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

  wd_start(&dev->wdog, SIM_RPTUN_WORK_DELAY, sim_rptun_work, (wdparm_t)dev);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_sim_rptun_ops =
{
  .get_cpuname       = sim_rptun_get_cpuname,
  .get_addrenv       = sim_rptun_get_addrenv,
  .get_resource      = sim_rptun_get_resource,
  .is_autostart      = sim_rptun_is_autostart,
  .is_master         = sim_rptun_is_master,
  .start             = sim_rptun_start,
  .stop              = sim_rptun_stop,
  .notify            = sim_rptun_notify,
  .register_callback = sim_rptun_register_callback,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_rptun_init(const char *shmemname, const char *cpuname, int master)
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
  strlcpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strlcpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      kmm_free(dev);
      return ret;
    }

  return wd_start(&dev->wdog, 0, sim_rptun_work, (wdparm_t)dev);
}
