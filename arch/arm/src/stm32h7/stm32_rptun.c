/****************************************************************************
 * arch/arm/src/stm32h7/stm32_rptun.c
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

#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kthread.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/signal.h>

#include <nuttx/semaphore.h>

#include "arm_internal.h"

#include "stm32_hsem.h"
#include "stm32_dualcore.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
#  if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_OPENAMP_CACHE)
#    error CONFIG_OPENAMP_CACHE must be set
#  endif
#  if defined(CONFIG_ARMV7M_DCACHE) && !defined(CONFIG_ARM_MPU)
#    erro CONFIG_ARM_MPU must be enabled
#  endif
#endif

/* Vring configuration parameters */

#define VRINGS                   (2)           /* Number of vrings */
#define VRING_ALIGN              (8)           /* Vring alignment */
#define VRING_NR                 (8)           /* Number of descriptors */
#define VRING_SIZE               (512)         /* Size of one descriptor */

#ifdef CONFIG_STM32H7_SHMEM_SRAM3
/* Use 32kB of the SRAM3 as a shared memory */

#  define VRING_SHMEM            STM32_SRAM3_BASE
#else
#  error missing shmem SRAM configuration
#endif

#define VRING0_NOTIFYID          (RSC_NOTIFY_ID_ANY)  /* Vring0 id */
#define VRING1_NOTIFYID          (RSC_NOTIFY_ID_ANY)  /* Vring1 id */

/* HSEM configuration */

                                         /* 0 reserved for synchronisation */
#define RPTUN_HSEM_CHAN_MASTER_RX    (1) /* RX for master is ready */
#define RPTUN_HSEM_CHAN_SLAVE_RX     (2) /* RX for slave is ready */
#define RPTUN_HSEM_CHAN_SLAVE_RESET  (3)
#define RPTUN_HSEM_CHAN_SLAVE_PANIC  (4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* STM32 rptun sharred memory */

struct stm32_rptun_shmem_s
{
  volatile uintptr_t         base;
  struct rptun_rsc_s         rsc;
};

/* STM32 rptun device */

struct stm32_rptun_dev_s
{
  struct rptun_dev_s          rptun;
  rptun_callback_t            callback;
  void                       *arg;
  bool                        master;
  struct stm32_rptun_shmem_s *shmem;
  char                        cpuname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *stm32_rptun_get_cpuname(struct rptun_dev_s *dev);
static struct rptun_rsc_s *
stm32_rptun_get_resource(struct rptun_dev_s *dev);
static bool stm32_rptun_is_autostart(struct rptun_dev_s *dev);
static bool stm32_rptun_is_master(struct rptun_dev_s *dev);
static int stm32_rptun_start(struct rptun_dev_s *dev);
static int stm32_rptun_stop(struct rptun_dev_s *dev);
static int stm32_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int stm32_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg);

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
static void stm32_rptun_reset(struct rptun_dev_s *dev, int value);
static void stm32_rptun_panic(struct rptun_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_stm32_rptun_ops =
{
  .get_cpuname       = stm32_rptun_get_cpuname,
  .get_resource      = stm32_rptun_get_resource,
  .is_autostart      = stm32_rptun_is_autostart,
  .is_master         = stm32_rptun_is_master,
  .start             = stm32_rptun_start,
  .stop              = stm32_rptun_stop,
  .notify            = stm32_rptun_notify,
  .register_callback = stm32_rptun_register_callback,
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  .reset             = stm32_rptun_reset,
  .panic             = stm32_rptun_panic
#endif
};

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
/* Allocate shared memory on the CM7 core side */

static struct stm32_rptun_shmem_s g_shmem __attribute__((section(".shmem")));
#endif

struct stm32_rptun_dev_s          g_rptun_dev;
static sem_t                      g_stm32_rx_sig = SEM_INITIALIZER(0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rptun_get_cpuname
 ****************************************************************************/

static const char *stm32_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct stm32_rptun_dev_s *priv = container_of(dev,
                                 struct stm32_rptun_dev_s, rptun);

  return priv->cpuname;
}

/****************************************************************************
 * Name: stm32_rptun_get_resource
 ****************************************************************************/

static struct rptun_rsc_s *
stm32_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct stm32_rptun_dev_s *priv = container_of(dev,
                                   struct stm32_rptun_dev_s, rptun);
  struct rptun_rsc_s *rsc;

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  priv->shmem = &g_shmem;
#else
  priv->shmem = (struct stm32_rptun_shmem_s *)VRING_SHMEM;
#endif

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
      /* TODO: use HSEM */

      while (priv->shmem->base == 0)
        {
          nxsig_usleep(100);
        }
    }

  return &priv->shmem->rsc;
}

/****************************************************************************
 * Name: stm32_rptun_is_autostart
 ****************************************************************************/

static bool stm32_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: stm32_rptun_is_master
 ****************************************************************************/

static bool stm32_rptun_is_master(struct rptun_dev_s *dev)
{
  struct stm32_rptun_dev_s *priv = container_of(dev,
                                 struct stm32_rptun_dev_s, rptun);
  return priv->master;
}

/****************************************************************************
 * Name: stm32_rptun_start
 ****************************************************************************/

static int stm32_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_rptun_stop
 ****************************************************************************/

static int stm32_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_rptun_notify
 ****************************************************************************/

static int stm32_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  /* Notify slave that RX is ready */

  stm32_hsem_signal(RPTUN_HSEM_CHAN_SLAVE_RX);
#else
  /* Notify master that RX is ready */

  stm32_hsem_signal(RPTUN_HSEM_CHAN_MASTER_RX);
#endif

  return 0;
}

/****************************************************************************
 * Name: stm32_rptun_register_callback
 ****************************************************************************/

static int stm32_rptun_register_callback(struct rptun_dev_s *dev,
                                       rptun_callback_t callback,
                                       void *arg)
{
  struct stm32_rptun_dev_s *priv = container_of(dev,
                                 struct stm32_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
/****************************************************************************
 * Name: stm32_rptun_reset
 ****************************************************************************/

static void stm32_rptun_reset(struct rptun_dev_s *dev, int value)
{
  if (value == 0)
    {
      /* Soft reset */

      stm32_hsem_signal(RPTUN_HSEM_CHAN_SLAVE_RESET);
    }
}

/****************************************************************************
 * Name: stm32_rptun_panic
 ****************************************************************************/

static void stm32_rptun_panic(struct rptun_dev_s *dev)
{
  stm32_hsem_signal(RPTUN_HSEM_CHAN_SLAVE_PANIC);
}
#endif

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
/****************************************************************************
 * Name: stm32_hsem_master_callback
 ****************************************************************************/

static void stm32_hsem_master_callback(uint8_t id, void *arg)
{
  ipcinfo("Rptun HSEM master %d\n", id);

  switch (id)
    {
      case RPTUN_HSEM_CHAN_MASTER_RX:
        {
          nxsem_post(&g_stm32_rx_sig);
          break;
        }

      default:
        {
          DEBUGASSERT(0);
        }
    }
}

/****************************************************************************
 * Name: stm32_rptun_hsem_cm7
 ****************************************************************************/

static void stm32_rptun_hsem_cm7(struct stm32_rptun_dev_s *dev)
{
  DEBUGASSERT(dev);

  stm32_hsem_subscribe(RPTUN_HSEM_CHAN_MASTER_RX,
                       stm32_hsem_master_callback,
                       dev);
}
#else
/****************************************************************************
 * Name: stm32_hsem_slave_callback
 ****************************************************************************/

static void stm32_hsem_slave_callback(uint8_t id, void *arg)
{
  ipcinfo("Rptun HSEM slave %d\n", id);

  switch (id)
    {
      case RPTUN_HSEM_CHAN_SLAVE_RX:
        {
          nxsem_post(&g_stm32_rx_sig);
          break;
        }

      case RPTUN_HSEM_CHAN_SLAVE_RESET:
        {
          /* REVISIT: It's not possible to reset a single core.
           * What can we do here ?
           */

          break;
        }

      case RPTUN_HSEM_CHAN_SLAVE_PANIC:
        {
          PANIC();
        }

      default:
        {
          DEBUGASSERT(0);
        }
    }
}

/****************************************************************************
 * Name: stm32_rptun_hsem_cm4
 ****************************************************************************/

static void stm32_rptun_hsem_cm4(struct stm32_rptun_dev_s *dev)
{
  DEBUGASSERT(dev);

  stm32_hsem_subscribe(RPTUN_HSEM_CHAN_SLAVE_RX,
                       stm32_hsem_slave_callback,
                       dev);
  stm32_hsem_subscribe(RPTUN_HSEM_CHAN_SLAVE_RESET,
                       stm32_hsem_slave_callback,
                       dev);
  stm32_hsem_subscribe(RPTUN_HSEM_CHAN_SLAVE_PANIC,
                       stm32_hsem_slave_callback,
                       dev);
}
#endif

/****************************************************************************
 * Name: stm32_rptun_thread
 ****************************************************************************/

static int stm32_rptun_thread(int argc, char *argv[])
{
  struct stm32_rptun_dev_s *dev = &g_rptun_dev;

  while (1)
    {
      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }

      nxsem_wait(&g_stm32_rx_sig);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_rptun_init(const char *cpuname)
{
  struct stm32_rptun_dev_s *dev = &g_rptun_dev;
  int                       ret = OK;

  /* Initialize HSEM */

  stm32_hsem_init();

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  /* The CM7 core always master */

  memset(&g_shmem, 0, sizeof(struct stm32_rptun_shmem_s));
  dev->master = true;
#else
  dev->master = false;
#endif

  /* Configure HSEM */

#ifdef CONFIG_ARCH_CHIP_STM32H7_CORTEXM7
  stm32_rptun_hsem_cm7(dev);
#else
  stm32_rptun_hsem_cm4(dev);
#endif

  /* Configure device */

  dev->rptun.ops = &g_stm32_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      ipcerr("ERROR: rptun_initialize failed %d!\n", ret);
      goto errout;
    }

  /* Create rptun RX thread */

  ret = kthread_create("stm32-rptun", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, stm32_rptun_thread, NULL);
  if (ret < 0)
    {
      ipcerr("ERROR: kthread_create failed %d\n", ret);
    }

errout:
  return ret;
}
