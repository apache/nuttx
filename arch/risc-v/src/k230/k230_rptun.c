/****************************************************************************
 * arch/risc-v/src/k230/k230_rptun.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>

#include <nuttx/rptun/rptun.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/list.h>

#include <arch/barriers.h>

#include <arch/board/board.h>
#include "hardware/k230_memorymap.h"
#include "riscv_internal.h"
#include "k230_hart.h"
#include "k230_ipi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rpinfo rpmsginfo
#define rpwarn rpmsgwarn
#define rperr  rpmsgerr

/* Vring config parameters taken from nrf53_rptun */

#define VRINGS                   2           /* Number of vrings        */
#define VRING_ALIGN              8           /* Vring alignment         */
#define VRING_NR                 8           /* Number of descriptors   */
#define VRING_SIZE               512         /* Size of one descriptor  */

/* The RPMSG default channel used with only one RPMSG channel */

#define VRING_SHMEM     (CONFIG_K230_RPTUN_SHM_ADDR) /* Vring addr      */
#define VRING0_NOTIFYID (RSC_NOTIFY_ID_ANY)          /* Vring0 id       */
#define VRING1_NOTIFYID (RSC_NOTIFY_ID_ANY)          /* Vring1 id       */

#define VRING_SHMEM_END (VRING_SHMEM + CONFIG_K230_RPTUN_SHM_SIZE)

/* Design notes:
 *
 * Though there are 16 IPI lines per K230 IPI device, we use only 1 IPI
 * line for each core to notify the peer. Later we will see if more lines
 * are really needed.
 *
 * For configurations, master and remote builds should use same IPI device
 * defined by CONFIG_K230_RPTUN_IPI_DEVN otherwise the IPI notifications
 * won't reach each other.
 */

#define RPTUN_IPI_DEVN         CONFIG_K230_RPTUN_IPI_DEV
#define RPTUN_IPI_LINE         0
#define RPTUN_IPI_LINE_MASK    (1 << RPTUN_IPI_LINE)

#if RPTUN_IPI_DEVN < 0 || RPTUN_IPI_DEVN > K230_IPI_DEVN_MAX
#error Invalid K230_RPTUN_IPI_DEV number
#endif

#ifdef CONFIG_K230_RPTUN_MASTER
#define RPTUN_IPI_ROLE        IPI_ROLE_MASTER
#else
#define RPTUN_IPI_ROLE        IPI_ROLE_REMOTE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k230_rptun_shmem_s
{
  volatile uintptr_t         base;
  struct rptun_rsc_s         rsc;
};

struct k230_rptun_dev_s
{
  struct rptun_dev_s         rptun;
  rptun_callback_t           callback;
  void                      *arg;
  bool                       master;
  struct k230_rptun_shmem_s *shmem;
  struct simple_addrenv_s    addrenv[VRINGS];
  char                       peername[RPMSG_NAME_SIZE + 1];
};

#define as_k230_rptun_dev(d) container_of(d, struct k230_rptun_dev_s, rptun)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rp_get_cpuname(struct rptun_dev_s *dev);
static struct rptun_rsc_s *rp_get_resource(struct rptun_dev_s *dev);
static bool rp_is_autostart(struct rptun_dev_s *dev);
static bool rp_is_master(struct rptun_dev_s *dev);
static int rp_start(struct rptun_dev_s *dev);
static int rp_stop(struct rptun_dev_s *dev);
static int rp_notify(struct rptun_dev_s *dev, uint32_t notifyid);
static int rp_set_callback(struct rptun_dev_s *, rptun_callback_t, void *);
static void k230_rptun_callback(uint16_t comb, void *args);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_k230_rptun_ops =
{
  .get_cpuname       = rp_get_cpuname,
  .get_resource      = rp_get_resource,
  .is_autostart      = rp_is_autostart,
  .is_master         = rp_is_master,
  .start             = rp_start,
  .stop              = rp_stop,
  .notify            = rp_notify,
  .register_callback = rp_set_callback,
};

#define SHMEM        (struct k230_rptun_shmem_s*)VRING_SHMEM
#define SHMEM_SIZE   sizeof(struct k230_rptun_shmem_s)
#define SHMEM_END    (VRING_SHMEM + SHMEM_SIZE)

static struct k230_rptun_dev_s    g_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *rp_get_cpuname(struct rptun_dev_s *dev)
{
  struct k230_rptun_dev_s *priv = as_k230_rptun_dev(dev);
  return priv->peername;
}

static struct rptun_rsc_s *rp_get_resource(struct rptun_dev_s *dev)
{
  struct k230_rptun_dev_s *priv = as_k230_rptun_dev(dev);
  struct rptun_rsc_s *rsc;

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

  priv->shmem = SHMEM;

  if (priv->master)
    {
      /* Perform initial setup */

      rsc = &priv->shmem->rsc;

      rsc->rsc_tbl_hdr.ver          = 1;
      rsc->rsc_tbl_hdr.num          = 1;
      rsc->rsc_tbl_hdr.reserved[0]  = 0;
      rsc->rsc_tbl_hdr.reserved[1]  = 0;
      rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                               rpmsg_vdev);

      rsc->rpmsg_vdev.type          = RSC_VDEV;
      rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
      rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                    | 1 << VIRTIO_RPMSG_F_ACK
                                    | 1 << VIRTIO_RPMSG_F_BUFSZ;
      rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
      rsc->rpmsg_vdev.num_of_vrings = VRINGS;

      rsc->rpmsg_vring0.align       = VRING_ALIGN;
      rsc->rpmsg_vring0.num         = VRING_NR;
      rsc->rpmsg_vring0.notifyid    = VRING0_NOTIFYID;
      rsc->rpmsg_vring1.align       = VRING_ALIGN;
      rsc->rpmsg_vring1.num         = VRING_NR;
      rsc->rpmsg_vring1.notifyid    = VRING1_NOTIFYID;
      rsc->config.r2h_buf_size      = VRING_SIZE;
      rsc->config.h2r_buf_size      = VRING_SIZE;

      priv->shmem->base             = (uintptr_t)priv->shmem;

      rpinfo("shmem:%lx, dev:%p\n", priv->shmem->base, dev);
    }
  else
    {
      /* TODO: use IPI later, polling now. */

      rpinfo("wait for shmem %p...\n", priv->shmem);
      while (priv->shmem->base == 0)
        {
          nxsig_usleep(100);
        }

      rpinfo("shmem:%lx, dev:%p\n", priv->shmem->base, dev);
    }

  return &priv->shmem->rsc;
}

static bool rp_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool rp_is_master(struct rptun_dev_s *dev)
{
  struct k230_rptun_dev_s *priv = as_k230_rptun_dev(dev);
  return priv->master;
}

static int rp_start(struct rptun_dev_s *dev)
{
  rpinfo("%p\n", dev);
#ifdef CONFIG_K230_RPTUN_MASTER
  k230_hart_big_boot(0x7000000);
#endif
  return 0;
}

static int rp_stop(struct rptun_dev_s *dev)
{
  rpinfo("%p\n", dev);
#ifdef CONFIG_K230_RPTUN_MASTER
  k230_hart_big_stop();
#endif
  return 0;
}

static int rp_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  UNUSED(dev);
  UNUSED(vqid);
  k230_ipi_notify(RPTUN_IPI_DEVN, RPTUN_IPI_LINE);
  return 0;
}

static int rp_set_callback(struct rptun_dev_s *dev, rptun_callback_t cb,
                           void *arg)
{
  struct k230_rptun_dev_s *priv = as_k230_rptun_dev(dev);

  priv->callback = cb;
  priv->arg      = arg;
  return 0;
}

static void k230_rptun_callback(uint16_t comb, void *args)
{
  UNUSED(comb);
  struct k230_rptun_dev_s *dev = args;
  if (dev->callback) dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int k230_rptun_init(const char *peername)
{
  struct k230_rptun_dev_s  *dev = &g_rptun_dev;
  int                       ret = OK;

  memset(dev, 0, sizeof(*dev));

#ifdef CONFIG_K230_RPTUN_MASTER
  /* master is responsible for initializing shmem */

  memset((void *)SHMEM, 0, SHMEM_SIZE);
  rpinfo("cleared %ld @ %p\n", SHMEM_SIZE, SHMEM);
  dev->master = true;
#else
  dev->master = false;
#endif

  ret = k230_ipi_init(RPTUN_IPI_DEVN, RPTUN_IPI_LINE_MASK, RPTUN_IPI_ROLE,
                      k230_rptun_callback, dev);
  if (ret < 0)
    {
      rperr("k230_ipi_init failed %d\n", ret);
      goto ipierr;
    }

  /* Configure device */

  dev->rptun.ops = &g_k230_rptun_ops;
  strncpy(dev->peername, peername, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      rperr("rptun_initialize failed %d!\n", ret);
      goto errout;
    }

  return 0;

errout:
  k230_ipi_finish(RPTUN_IPI_DEVN, RPTUN_IPI_LINE_MASK);

ipierr:
  return ret;
}
