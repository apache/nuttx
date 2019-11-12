/****************************************************************************
 * arch/sim/src/sim/up_rptun.c
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *   Author: Chao An <anchao@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SIM_RPTUN_MASTER
#define CONFIG_SIM_RPTUN_MASTER 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_rptun_shmem_s
{
  volatile uintptr_t        base;
#if CONFIG_SIM_RPTUN_MASTER
  volatile unsigned int     seqrx;
  volatile unsigned int     seqtx;
#else
  volatile unsigned int     seqtx;
  volatile unsigned int     seqrx;
#endif
  struct rptun_rsc_s        rsc;
  char                      buf[0x10000];
};

struct sim_rptun_dev_s
{
  struct rptun_dev_s        rptun;
  rptun_callback_t          callback;
  void                     *arg;
  unsigned int              seqrx;
  struct sim_rptun_shmem_s *shmem;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *sim_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  return CONFIG_SIM_RPTUN_MASTER ? "proxy" : "server";
}

static const char *sim_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

static const struct rptun_addrenv_s *sim_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

static struct rptun_rsc_s *sim_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct sim_rptun_dev_s *priv = (struct sim_rptun_dev_s *)dev;
  struct sim_rptun_shmem_s *shmem = priv->shmem;

  return &shmem->rsc;
}

static bool sim_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool sim_rptun_is_master(struct rptun_dev_s *dev)
{
  return CONFIG_SIM_RPTUN_MASTER;
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
  struct sim_rptun_dev_s *priv = (struct sim_rptun_dev_s *)dev;
  struct sim_rptun_shmem_s *shmem = priv->shmem;

  shmem->seqtx++;
  return 0;
}

static int sim_rptun_register_callback(struct rptun_dev_s *dev,
                                       rptun_callback_t callback, void *arg)
{
  struct sim_rptun_dev_s *priv = (struct sim_rptun_dev_s *)dev;

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

static struct sim_rptun_dev_s g_dev =
{
  .rptun.ops         = &g_sim_rptun_ops
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_rptun_loop(void)
{
  struct sim_rptun_shmem_s *shmem = g_dev.shmem;

  if (shmem != NULL && g_dev.seqrx != shmem->seqrx)
    {
      g_dev.seqrx = shmem->seqrx;
      if (g_dev.callback != NULL)
        {
          g_dev.callback(g_dev.arg, RPTUN_NOTIFY_ALL);
        }
    }
}

int up_rptun_init(void)
{
  int ret;

  g_dev.shmem = shmem_open("rptun-shmem",
                           sizeof(*g_dev.shmem),
                           CONFIG_SIM_RPTUN_MASTER);
  if (g_dev.shmem == NULL)
    {
      return -ENOMEM;
    }

  if (CONFIG_SIM_RPTUN_MASTER)
    {
      struct rptun_rsc_s *rsc = &g_dev.shmem->rsc;

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s, rpmsg_vdev);
      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                    | 1 << VIRTIO_RPMSG_F_BIND
                                    | 1 << VIRTIO_RPMSG_F_BUFSZ;
      rsc->rpmsg_vdev.num_of_vrings = 2;
      rsc->rpmsg_vring0.align       = 8;
      rsc->rpmsg_vring0.num         = 8;
      rsc->rpmsg_vring1.align       = 8;
      rsc->rpmsg_vring1.num         = 8;
      rsc->buf_size                 = 0x800;

      g_dev.shmem->base             = (uintptr_t)g_dev.shmem;
    }
  else
    {
      static struct simple_addrenv_s s_addrenv[2];

      /* Wait untils master is ready */

      while (g_dev.shmem->base == 0)
        {
          up_hostusleep(1000);
        }

      s_addrenv[0].va               = (uintptr_t)g_dev.shmem;
      s_addrenv[0].pa               = g_dev.shmem->base;
      s_addrenv[0].size             = sizeof(*g_dev.shmem);

      simple_addrenv_initialize(s_addrenv);
    }

  ret = rptun_initialize(&g_dev.rptun);
  if (ret < 0)
    {
      shmem_close(g_dev.shmem);
      return ret;
    }

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init("server");
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#endif

  return 0;
}

void rpmsg_serialinit(void)
{
#if CONFIG_SIM_RPTUN_MASTER
  uart_rpmsg_init("proxy", "proxy", 4096, false);
#else
  uart_rpmsg_init("server", "proxy", 4096, true);
#endif
}
