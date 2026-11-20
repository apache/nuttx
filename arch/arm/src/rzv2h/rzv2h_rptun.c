/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_rptun.c
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

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include "r_mhu_b_ns.h"
#include "rzv2h_fsp_err.h"
#include "rzv2h_irq.h"
#include "rzv2h_rptun.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rzv2h_rptun_dev_s
{
  struct rptun_dev_s rptun;
  FAR const mhu_instance_t *mhu;
  FAR volatile const uint32_t *tx_status;
  FAR struct resource_table *resource;
  FAR const char *cpuname;
  rptun_callback_t callback;
  FAR void *callback_arg;
  mhu_callback_args_t mhu_callback_memory;
  struct work_s work;
  sem_t startup_sem;
  uint32_t notifyid;
  int irq;
  bool startup_complete;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR const char *
rzv2h_rptun_get_cpuname(FAR struct rptun_dev_s *dev);
static FAR struct resource_table *
rzv2h_rptun_get_rsc(FAR struct rptun_dev_s *dev);
static bool rzv2h_rptun_is_autostart(FAR struct rptun_dev_s *dev);
static bool rzv2h_rptun_is_master(FAR struct rptun_dev_s *dev);
static int rzv2h_rptun_notify(FAR struct rptun_dev_s *dev, uint32_t vqid);
static int rzv2h_rptun_register_callback(FAR struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         FAR void *arg);

static int rzv2h_rptun_interrupt(int irq, FAR void *context, FAR void *arg);
static void rzv2h_rptun_mhu_callback(FAR mhu_callback_args_t *args);
static void rzv2h_rptun_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rzv2h_rptun_ops =
{
  .get_cpuname       = rzv2h_rptun_get_cpuname,
  .get_firmware      = NULL, /* Firmware is already running on CR8. */
  .get_addrenv       = NULL, /* CA55 and CR8 use identity addresses. */
  .get_resource      = rzv2h_rptun_get_rsc,
  .is_autostart      = rzv2h_rptun_is_autostart,
  .is_master         = rzv2h_rptun_is_master,
  .config            = NULL, /* Linux userspace configures shared memory. */
  .start             = NULL, /* CR8 is already running. */
  .stop              = NULL, /* Linux owns the CR8 lifecycle. */
  .notify            = rzv2h_rptun_notify,
  .register_callback = rzv2h_rptun_register_callback,
  .reset             = NULL, /* Slave: Linux controls reset. */
};

static struct rzv2h_rptun_dev_s g_rzv2h_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rptun_get_cpuname
 ****************************************************************************/

static FAR const char *
rzv2h_rptun_get_cpuname(FAR struct rptun_dev_s *dev)
{
  FAR struct rzv2h_rptun_dev_s *priv =
    container_of(dev, struct rzv2h_rptun_dev_s, rptun);

  return priv->cpuname;
}

/****************************************************************************
 * Name: rzv2h_rptun_get_rsc
 ****************************************************************************/

static FAR struct resource_table *
rzv2h_rptun_get_rsc(FAR struct rptun_dev_s *dev)
{
  FAR struct rzv2h_rptun_dev_s *priv =
    container_of(dev, struct rzv2h_rptun_dev_s, rptun);

  return priv->resource;
}

/****************************************************************************
 * Name: rzv2h_rptun_is_autostart
 ****************************************************************************/

static bool rzv2h_rptun_is_autostart(FAR struct rptun_dev_s *dev)
{
  (void)dev;
  return true;
}

/****************************************************************************
 * Name: rzv2h_rptun_is_master
 ****************************************************************************/

static bool rzv2h_rptun_is_master(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return false; /* CR8 is slave/remote */
}

/****************************************************************************
 * Name: rzv2h_rptun_notify
 ****************************************************************************/

static int rzv2h_rptun_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  FAR struct rzv2h_rptun_dev_s *priv =
    container_of(dev, struct rzv2h_rptun_dev_s, rptun);
  fsp_err_t err;
  irqstate_t flags;
  int ret;

  /* Multi-OS v3.3 assigns the channel ID to both vrings.  Map NuttX's
   * internal all-queues notification to the selected transport value.
   */

  if (vqid == RPTUN_NOTIFY_ALL)
    {
      vqid = priv->notifyid;
    }
  else if (vqid != priv->notifyid)
    {
      return -EINVAL;
    }

  /* The shared-memory MPU window is non-cacheable.  Complete all preceding
   * memory accesses before the MHU kick becomes visible to Linux.
   */

  UP_DSB();

  /* R_MHU_B_NS_MsgSend() waits indefinitely while an earlier interrupt is
   * pending.  Linux closes the UIO transport immediately after the sample's
   * shutdown message, so that wait can otherwise block high-priority RPMsg
   * processing and starve the NuttX console.  A vring doorbell may be
   * coalesced safely: the pending interrupt already tells Linux to inspect
   * the shared vring.  Keep the readiness check and send atomic against
   * local callers.
   */

  flags = enter_critical_section();
  if (*priv->tx_status != 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  err = R_MHU_B_NS_MsgSend(priv->mhu->p_ctrl, vqid);
  leave_critical_section(flags);
  ret = rzv2h_fsp_err_to_errno(err);
  if (ret < 0)
    {
      rptunerr("MHU notification failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: rzv2h_rptun_register_callback
 ****************************************************************************/

static int rzv2h_rptun_register_callback(FAR struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         FAR void *arg)
{
  FAR struct rzv2h_rptun_dev_s *priv =
    container_of(dev, struct rzv2h_rptun_dev_s, rptun);
  bool startup_complete = false;
  irqstate_t flags;

  flags = enter_critical_section();

  if (callback == NULL)
    {
      up_disable_irq(priv->irq);
    }

  priv->callback = callback;
  priv->callback_arg = arg;

  if (callback != NULL)
    {
      up_enable_irq(priv->irq);

      if (!priv->startup_complete)
        {
          priv->startup_complete = true;
          startup_complete = true;
        }
    }

  leave_critical_section(flags);

  if (startup_complete)
    {
      nxsem_post(&priv->startup_sem);
    }

  if (callback == NULL)
    {
      work_cancel_sync(HPWORK, &priv->work);
    }

  return OK;
}

/****************************************************************************
 * Name: rzv2h_rptun_worker
 ****************************************************************************/

static void rzv2h_rptun_worker(FAR void *arg)
{
  FAR struct rzv2h_rptun_dev_s *priv = arg;
  rptun_callback_t callback;
  FAR void *callback_arg;
  irqstate_t flags;
  int ret;

  flags = enter_critical_section();
  callback = priv->callback;
  callback_arg = priv->callback_arg;
  leave_critical_section(flags);

  if (callback != NULL)
    {
      /* The shared regions are non-cacheable.  This barrier orders their
       * reads after the FSP acknowledgement of the MHU interrupt.
       */

      UP_DMB();
      ret = callback(callback_arg, priv->notifyid);
      if (ret < 0)
        {
          rptunerr("notification processing failed: %d\n", ret);
        }
    }

  flags = enter_critical_section();
  if (priv->callback != NULL)
    {
      up_enable_irq(priv->irq);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rzv2h_rptun_mhu_callback
 ****************************************************************************/

static void rzv2h_rptun_mhu_callback(FAR mhu_callback_args_t *args)
{
  FAR struct rzv2h_rptun_dev_s *priv;

  /* FSP has read and acknowledged the MHU message before this callback. */

  if (args == NULL || args->p_context == NULL)
    {
      return;
    }

  priv = (FAR struct rzv2h_rptun_dev_s *)args->p_context;
  if (args->msg == priv->notifyid && priv->callback != NULL)
    {
      /* OpenAMP can take locks and invoke endpoint callbacks, so defer it
       * out of the hardware interrupt.  Re-queuing coalesces identical
       * kicks; the worker drains the shared vring state.
       */

      up_disable_irq(priv->irq);
      if (work_queue(HPWORK, &priv->work, rzv2h_rptun_worker,
                     priv, 0) < 0)
        {
          up_enable_irq(priv->irq);
        }
    }
}

/****************************************************************************
 * Name: rzv2h_rptun_interrupt
 ****************************************************************************/

static int rzv2h_rptun_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rzv2h_rptun_dev_s *priv =
    (FAR struct rzv2h_rptun_dev_s *)arg;
  IRQn_Type fsp_irq = priv->mhu->p_cfg->rx_irq;

  DEBUGASSERT(irq == priv->irq);
  (void)context;

  /* NuttX owns the GIC vector table.  Let FSP perform all MHU-specific
   * status handling, acknowledgement, and callback dispatch.
   */

  R_MHU_B_NS_IsrSub((uint32_t)fsp_irq);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_rptun_initialize
 ****************************************************************************/

int rzv2h_rptun_initialize(
  FAR const struct rzv2h_rptun_config_s *config)
{
  FAR struct rzv2h_rptun_dev_s *priv = &g_rzv2h_rptun_dev;
  FAR const mhu_instance_t *mhu;
  irqstate_t flags;
  fsp_err_t err;
  int ret;

  if (config == NULL || config->mhu == NULL ||
      config->tx_status == NULL || config->resource == NULL ||
      config->cpuname == NULL || config->cpuname[0] == '\0')
    {
      return -EINVAL;
    }

  mhu = config->mhu;
  if (mhu->p_ctrl == NULL || mhu->p_cfg == NULL ||
      mhu->p_api == NULL || mhu->p_api->open == NULL ||
      mhu->p_api->close == NULL ||
      (int)mhu->p_cfg->rx_irq < 0 ||
      (uint32_t)mhu->p_cfg->rx_irq >= BSP_ICU_VECTOR_MAX_ENTRIES)
    {
      return -EINVAL;
    }

  if (priv->initialized)
    {
      return -EALREADY;
    }

  memset(priv, 0, sizeof(*priv));
  nxsem_init(&priv->startup_sem, 0, 0);
  priv->rptun.ops = &g_rzv2h_rptun_ops;
  priv->mhu = mhu;
  priv->tx_status = config->tx_status;
  priv->resource = config->resource;
  priv->cpuname = config->cpuname;
  priv->notifyid = config->notifyid;
  priv->irq = RZV2H_FSP_TO_GIC_IRQ(mhu->p_cfg->rx_irq);

  rptuninfo("resource=%p notify=%" PRIu32 " MHU channel=%" PRIu32
            " FSP IRQ=%d GIC IRQ=%d\n",
            priv->resource, priv->notifyid, mhu->p_cfg->channel,
            mhu->p_cfg->rx_irq, priv->irq);

  ret = up_set_irq_type(priv->irq, IRQ_HIGH_LEVEL);
  if (ret < 0)
    {
      return ret;
    }

  ret = irq_attach(priv->irq, rzv2h_rptun_interrupt, priv);
  if (ret < 0)
    {
      return ret;
    }

  /* FSP enables the receive IRQ during open().  Keep CPU interrupts masked
   * until callbackSet() has replaced the generated callback and context.
   */

  flags = enter_critical_section();
  err = mhu->p_api->open(mhu->p_ctrl, mhu->p_cfg);
  if (err != FSP_SUCCESS)
    {
      leave_critical_section(flags);
      ret = rzv2h_fsp_err_to_errno(err);
      goto err_detach;
    }

  err = R_MHU_B_NS_CallbackSet(mhu->p_ctrl, rzv2h_rptun_mhu_callback,
                               priv, &priv->mhu_callback_memory);

  /* open() enables the GIC interrupt.  Mask it before restoring the CPU
   * interrupt state; the generic RPTUN layer enables it by registering its
   * callback only after remoteproc and the resource table are ready.
   */

  up_disable_irq(priv->irq);
  leave_critical_section(flags);
  if (err != FSP_SUCCESS)
    {
      ret = rzv2h_fsp_err_to_errno(err);
      goto err_close;
    }

  ret = up_prioritize_irq(priv->irq, mhu->p_cfg->rx_ipl << 4);
  if (ret < 0)
    {
      goto err_close;
    }

  ret = rptun_initialize(&priv->rptun);
  if (ret < 0)
    {
      goto err_close;
    }

  /* rptun_initialize() starts a kernel thread and returns before that
   * thread has finished the remoteproc setup.  Wait until the receive
   * callback is registered so all startup diagnostics are queued before
   * board bring-up continues and NSH prints its banner.
   */

  ret = nxsem_tickwait_uninterruptible(
    &priv->startup_sem, MSEC2TICK(CONFIG_RPTUN_STATUS_TIMEOUT_MS));
  if (ret < 0)
    {
      rptunwarn("startup synchronization timed out: %d\n", ret);
    }

  priv->initialized = true;
  return OK;

err_close:
  up_disable_irq(priv->irq);
  mhu->p_api->close(mhu->p_ctrl);

err_detach:
  irq_detach(priv->irq);
  nxsem_destroy(&priv->startup_sem);
  memset(priv, 0, sizeof(*priv));
  return ret;
}
