/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_rptun.c
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
#include "hardware/qemu_rv_memorymap.h"
#include "riscv_internal.h"

#include "qemu_rv_rptun.h"

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

#define VRING_SHMEM     (CONFIG_QEMU_RPTUN_SHM_BASE) /* Vring addr      */
#define VRING0_NOTIFYID (RSC_NOTIFY_ID_ANY)          /* Vring0 id       */
#define VRING1_NOTIFYID (RSC_NOTIFY_ID_ANY)          /* Vring1 id       */

#define VRING_SHMEM_END (VRING_SHMEM + CONFIG_QEMU_RPTUN_SHM_SIZE)

/* Number of rptun peers */

#ifdef CONFIG_QEMU_RPTUN_MASTER
#define NUM_RPTUN_PEERS (CONFIG_QEMU_RPTUN_REMOTE_NUM)
#else
#define NUM_RPTUN_PEERS (1)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct qemu_rptun_shmem_s
{
  volatile uintptr_t         base;
  struct rptun_rsc_s         rsc;
};

struct qemu_rptun_dev_s
{
  struct rptun_dev_s         rptun;
  rptun_callback_t           callback;
  void                      *arg;
  bool                       master;
  struct qemu_rptun_shmem_s *shmem;
  struct simple_addrenv_s    addrenv[VRINGS];
  uintreg_t                  peeripi;
  char                       peername[RPMSG_NAME_SIZE + 1];
  uint8_t                    ndx;
};

#define as_qemu_rptun_dev(d) container_of(d, struct qemu_rptun_dev_s, rptun)

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rptun_ops =
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

#define SHMEM        (struct qemu_rptun_shmem_s*)VRING_SHMEM
#define SHMEM_SIZE   sizeof(struct qemu_rptun_shmem_s)
#define SHMEM_END    (VRING_SHMEM + SHMEM_SIZE)

static struct qemu_rptun_dev_s    g_rptun_devs[NUM_RPTUN_PEERS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int rp_init_ipi(void)
{
#ifdef CONFIG_QEMU_RPTUN_MASTER
  char *ptr = CONFIG_QEMU_RPTUN_REMOTE_IPIS;
  int i = 0;

  while (*ptr != '\0')
    {
      if (i < CONFIG_QEMU_RPTUN_REMOTE_NUM)
        {
#ifdef CONFIG_ARCH_RV64
          g_rptun_devs[i].peeripi = strtoll(ptr, &ptr, 0);
#else
          g_rptun_devs[i].peeripi = strtol(ptr, &ptr, 0);
#endif
          if (!g_rptun_devs[i].peeripi)
            {
              break;
            }
        }

      while (*ptr != '\0' && *ptr != ',')
        {
          ptr++;                     /* seek delimeter */
        }

      if (*ptr)
        {
          ptr++;                     /* skip delimeter */
        }

      i++;
    }

  return i;

#else

  g_rptun_devs[0].peeripi = CONFIG_QEMU_RPTUN_MASTER_IPI;
  return 1;

#endif
}

static const char *rp_get_cpuname(struct rptun_dev_s *dev)
{
  struct qemu_rptun_dev_s *priv = as_qemu_rptun_dev(dev);
  return priv->peername;
}

static struct rptun_rsc_s *rp_get_resource(struct rptun_dev_s *dev)
{
  struct qemu_rptun_dev_s *priv = as_qemu_rptun_dev(dev);
  struct rptun_rsc_s *rsc;

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

  priv->shmem = SHMEM + CONFIG_QEMU_RPTUN_SHM_SIZE * priv->ndx;

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
    }
  else
    {
      /* TODO: use IPI later, polling now. */

      rpinfo("wait for shmem %p...\n", priv->shmem);

      while (priv->shmem->base == 0)
        {
          nxsig_usleep(100);
        }
    }

  rpinfo("shmem:%p, dev:%p\n", (void *)priv->shmem->base, dev);
  return &priv->shmem->rsc;
}

static bool rp_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool rp_is_master(struct rptun_dev_s *dev)
{
  struct qemu_rptun_dev_s *priv = as_qemu_rptun_dev(dev);
  return priv->master;
}

static int rp_start(struct rptun_dev_s *dev)
{
  rpinfo("%p\n", dev);
  return 0;
}

static int rp_stop(struct rptun_dev_s *dev)
{
  rpinfo("%p\n", dev);
  return 0;
}

static int rp_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  struct qemu_rptun_dev_s *priv = as_qemu_rptun_dev(dev);
  UNUSED(vqid);
  putreg32(1, priv->peeripi);
  return 0;
}

static int rp_set_callback(struct rptun_dev_s *dev, rptun_callback_t cb,
                           void *arg)
{
  struct qemu_rptun_dev_s *priv = as_qemu_rptun_dev(dev);

  priv->callback = cb;
  priv->arg      = arg;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void qemu_rptun_ipi()
{
  for (int i = 0; i < NUM_RPTUN_PEERS ; i++)
    {
      if (g_rptun_devs[i].callback)
        {
          g_rptun_devs[i].callback(g_rptun_devs[i].arg, RPTUN_NOTIFY_ALL);
        }
    }
}

int qemu_rptun_init()
{
  struct qemu_rptun_dev_s  *dev;
  int                       ret;

  DEBUGASSERT(NUM_RPTUN_PEERS == rp_init_ipi());

  /* master is responsible for initializing shmem */

#ifdef CONFIG_QEMU_RPTUN_MASTER
  ret = CONFIG_QEMU_RPTUN_SHM_SIZE * NUM_RPTUN_PEERS;
  memset((void *)SHMEM, 0, ret);
  rpinfo("cleared %d @%p\n", ret, SHMEM);
#endif

  for (int i = 0; i < NUM_RPTUN_PEERS; i++)
    {
      dev         =  &g_rptun_devs[i];

#ifdef CONFIG_QEMU_RPTUN_MASTER
      dev->master = true;
      dev->ndx    = i;
      snprintf(dev->peername, sizeof(dev->peername), "remote%d", i + 1);
#else
      dev->master = false;
      strncpy(dev->peername, "master", sizeof(dev->peername) - 1);
#endif

      dev->rptun.ops = &g_rptun_ops;

      ret = rptun_initialize(&dev->rptun);
      if (ret < 0)
        {
          rperr("%s failed %d!\n", dev->peername, ret);
          return ret;
        }

      rpinfo("%s ok\n", dev->peername);
    }

  return OK;
}
