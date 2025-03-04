/****************************************************************************
 * arch/arm/src/imx9/imx9_rptun.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include "imx9_rptun.h"
#include "arm_internal.h"
#include "imx9_mu.h"
#include "imx9_rsctable.h"
#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VRING_SHMEM 0x88220000 /* Vring shared memory start */

#define RPMSG_MU_CHANNEL 1
#define MU_INSTANCE      7

#define MU_MSG_VQID_BITOFFSET 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* IMX9 rptun shared memory */

struct imx9_rptun_shmem_s
{
  struct rptun_rsc_s rsc;
};

/* IMX9 rptun device */

struct imx9_rptun_dev_s
{
  struct rptun_dev_s rptun;
  rptun_callback_t callback;
  void *mu;
  void *arg;
  struct imx9_rptun_shmem_s *shmem;
  char cpuname[RPMSG_NAME_SIZE + 1];
  char shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *imx9_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *imx9_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
imx9_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct rptun_rsc_s *imx9_rptun_get_resource(struct rptun_dev_s *dev);
static bool imx9_rptun_is_autostart(struct rptun_dev_s *dev);
static bool imx9_rptun_is_master(struct rptun_dev_s *dev);
static int imx9_rptun_start(struct rptun_dev_s *dev);
static int imx9_rptun_stop(struct rptun_dev_s *dev);
static int imx9_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int imx9_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_imx9_rptun_ops =
{
  .get_cpuname       = imx9_rptun_get_cpuname,
  .get_firmware      = imx9_rptun_get_firmware,
  .get_addrenv       = imx9_rptun_get_addrenv,
  .get_resource      = imx9_rptun_get_resource,
  .is_autostart      = imx9_rptun_is_autostart,
  .is_master         = imx9_rptun_is_master,
  .start             = imx9_rptun_start,
  .stop              = imx9_rptun_stop,
  .notify            = imx9_rptun_notify,
  .register_callback = imx9_rptun_register_callback,
};

struct imx9_rptun_dev_s g_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_rptun_get_cpuname
 ****************************************************************************/

static const char *imx9_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct imx9_rptun_dev_s *priv =
      container_of(dev, struct imx9_rptun_dev_s, rptun);

  return priv->cpuname;
}

/****************************************************************************
 * Name: imx9_rptun_get_firmware
 ****************************************************************************/

static const char *imx9_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: imx9_rptun_get_addrenv
 ****************************************************************************/

static const struct rptun_addrenv_s *
imx9_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

/****************************************************************************
 * Name: imx9_rptun_get_resource
 ****************************************************************************/

static struct rptun_rsc_s *imx9_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct imx9_rptun_dev_s *priv =
      container_of(dev, struct imx9_rptun_dev_s, rptun);

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc;
    }

  priv->shmem = (struct imx9_rptun_shmem_s *)VRING_SHMEM;
  if (priv->shmem->rsc.rsc_tbl_hdr.offset
      != g_imx9_rsc_table.rsc_tbl_hdr.offset)
    {
      imx9_rsctable_copy();
    }

  return &priv->shmem->rsc;
}

/****************************************************************************
 * Name: imx9_rptun_is_autostart
 ****************************************************************************/

static bool imx9_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: imx9_rptun_is_master
 ****************************************************************************/

static bool imx9_rptun_is_master(struct rptun_dev_s *dev)
{
  return false;
}

/****************************************************************************
 * Name: imx9_rptun_start
 ****************************************************************************/

static int imx9_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: imx9_rptun_stop
 ****************************************************************************/

static int imx9_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: imx9_rptun_notify
 ****************************************************************************/

static int imx9_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  struct imx9_rptun_dev_s *priv =
      container_of(dev, struct imx9_rptun_dev_s, rptun);

  ipcinfo("Rptun notify vqid=%ld\n", vqid);

  imx95_mu_send_msg(priv->mu, RPMSG_MU_CHANNEL,
                    vqid << MU_MSG_VQID_BITOFFSET);

  return 0;
}

/****************************************************************************
 * Name: imx9_rptun_register_callback
 ****************************************************************************/

static int imx9_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback, void *arg)
{
  struct imx9_rptun_dev_s *priv =
      container_of(dev, struct imx9_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

/****************************************************************************
 * Name: imx9_mu_callback
 ****************************************************************************/

static void imx9_mu_callback(int id, uint32_t msg, void *arg)
{
  if (id == RPMSG_MU_CHANNEL)
    {
      struct imx9_rptun_dev_s *dev = &g_rptun_dev;
      uint32_t vqid                = msg >> MU_MSG_VQID_BITOFFSET;

      ipcinfo("Rptun interrupt id=%d, vqid=%ld\n", id, vqid);

      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, vqid);
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

int imx9_rptun_init(const char *shmemname, const char *cpuname)
{
  struct imx9_rptun_dev_s *dev = &g_rptun_dev;
  int ret                      = OK;

  /* Subscribe to MU */

  dev->mu = imx95_mu_init(MU_INSTANCE);
  if (!dev->mu)
    {
      ipcerr("ERROR: cannot init mailbox %i!\n", MU_INSTANCE);
      return ret;
    }

  imx95_mu_subscribe_msg(dev->mu, (1 << RPMSG_MU_CHANNEL), imx9_mu_callback);

  /* Configure device */

  dev->rptun.ops = &g_imx9_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      ipcerr("ERROR: rptun_initialize failed %d!\n", ret);
    }

  return ret;
}
