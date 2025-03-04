/****************************************************************************
 * arch/arm/src/mx8mp/mx8mp_rptun.c
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

#include "mx8mp_rptun.h"
#include "mx8mp_ipc.h"
#include "mx8mp_rsctable.h"
#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kthread.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/signal.h>

#include <nuttx/semaphore.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Vring configuration parameters */

#define VRING_SHMEM       (0x550FF000) /* Vring shared memory start */

/* IPC configuration */

#define RPTUN_IPC_CHAN_MASTER_RX    (0) /* RX for master is ready */
#define RPTUN_IPC_CHAN_SLAVE_RX     (1) /* RX for slave is ready */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* MX8MP rptun sharred memory */

struct mx8mp_rptun_shmem_s
{
  struct rptun_rsc_s         rsc;
};

/* MX8MP rptun device */

struct mx8mp_rptun_dev_s
{
  struct rptun_dev_s          rptun;
  rptun_callback_t            callback;
  void                       *arg;
  struct mx8mp_rptun_shmem_s *shmem;
  char                        cpuname[RPMSG_NAME_SIZE + 1];
  char                        shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *mx8mp_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *mx8mp_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
mx8mp_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct rptun_rsc_s *
mx8mp_rptun_get_resource(struct rptun_dev_s *dev);
static bool mx8mp_rptun_is_autostart(struct rptun_dev_s *dev);
static bool mx8mp_rptun_is_master(struct rptun_dev_s *dev);
static int mx8mp_rptun_start(struct rptun_dev_s *dev);
static int mx8mp_rptun_stop(struct rptun_dev_s *dev);
static int mx8mp_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int mx8mp_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_mx8mp_rptun_ops =
{
  .get_cpuname       = mx8mp_rptun_get_cpuname,
  .get_firmware      = mx8mp_rptun_get_firmware,
  .get_addrenv       = mx8mp_rptun_get_addrenv,
  .get_resource      = mx8mp_rptun_get_resource,
  .is_autostart      = mx8mp_rptun_is_autostart,
  .is_master         = mx8mp_rptun_is_master,
  .start             = mx8mp_rptun_start,
  .stop              = mx8mp_rptun_stop,
  .notify            = mx8mp_rptun_notify,
  .register_callback = mx8mp_rptun_register_callback,
};

struct mx8mp_rptun_dev_s g_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_rptun_get_cpuname
 ****************************************************************************/

static const char *mx8mp_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct mx8mp_rptun_dev_s *priv = container_of(dev,
                                 struct mx8mp_rptun_dev_s, rptun);

  return priv->cpuname;
}

/****************************************************************************
 * Name: mx8mp_rptun_get_firmware
 ****************************************************************************/

static const char *mx8mp_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: mx8mp_rptun_get_addrenv
 ****************************************************************************/

static const struct rptun_addrenv_s *
mx8mp_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: mx8mp_rptun_get_resource
 ****************************************************************************/

static struct rptun_rsc_s *mx8mp_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct mx8mp_rptun_dev_s *priv
      = container_of(dev, struct mx8mp_rptun_dev_s, rptun);

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

  priv->shmem = (struct mx8mp_rptun_shmem_s *)VRING_SHMEM;
  if (priv->shmem->rsc.rsc_tbl_hdr.offset
      != g_mx8mp_rsc_table.rsc_tbl_hdr.offset)
    {
      mx8mp_copy_rsc_table();
    }

  return &priv->shmem->rsc;
}

/****************************************************************************
 * Name: mx8mp_rptun_is_autostart
 ****************************************************************************/

static bool mx8mp_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: mx8mp_rptun_is_master
 ****************************************************************************/

static bool mx8mp_rptun_is_master(struct rptun_dev_s *dev)
{
  return false;
}

/****************************************************************************
 * Name: mx8mp_rptun_start
 ****************************************************************************/

static int mx8mp_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: mx8mp_rptun_stop
 ****************************************************************************/

static int mx8mp_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: mx8mp_rptun_notify
 ****************************************************************************/

static int mx8mp_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  mx8mp_ipc_signal(RPTUN_IPC_CHAN_MASTER_RX);

  return 0;
}

/****************************************************************************
 * Name: mx8mp_rptun_register_callback
 ****************************************************************************/

static int mx8mp_rptun_register_callback(struct rptun_dev_s *dev,
                                       rptun_callback_t callback,
                                       void *arg)
{
  struct mx8mp_rptun_dev_s *priv = container_of(dev,
                                 struct mx8mp_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

/****************************************************************************
 * Name: mx8mp_ipc_slave_callback
 ****************************************************************************/

static void mx8mp_ipc_callback(int id, void *arg)
{
  ipcinfo("Rptun IPC interrupt %d\n", id);
  if (id == RPTUN_IPC_CHAN_SLAVE_RX)
    {
      struct mx8mp_rptun_dev_s *dev = &g_rptun_dev;

      up_invalidate_dcache(0x55000000, 0x55010000);
      up_invalidate_dcache(0x55400000, 0x55500000);
      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }
    }
  else
    {
      DEBUGASSERT(0);
    }

  __asm volatile("dsb 0xF" ::: "memory");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mx8mp_rptun_init(const char *shmemname, const char *cpuname)
{
  struct mx8mp_rptun_dev_s *dev = &g_rptun_dev;
  int                       ret = OK;

  /* Initialize IPC */

  mx8mp_ipc_init();

  /* Subscribe to IPC */

  mx8mp_ipc_subscribe(RPTUN_IPC_CHAN_SLAVE_RX,
                      mx8mp_ipc_callback,
                      dev);

  /* Configure device */

  dev->rptun.ops = &g_mx8mp_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      ipcerr("ERROR: rptun_initialize failed %d!\n", ret);
    }

  return ret;
}
