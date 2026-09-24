/****************************************************************************
 * arch/arm/src/imxrt/imxrt_rptun.c
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

#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>

#include "arm_internal.h"
#include "imxrt_periphclks.h"
#include "imxrt_rptun.h"
#include "hardware/rt117x/imxrt117x_mu.h"
#include "hardware/rt117x/imxrt117x_src.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Kicks travel on MU channel 0 in both directions.  The word carries no
 * information: either side rescans every virtqueue on any kick.
 */

#define IMXRT_RPTUN_MU_CHAN   0
#define IMXRT_RPTUN_KICK      1

/* Wait for the M4 slice to leave reset: 1000 x 10 us. */

#define IMXRT_RPTUN_RESET_RETRIES 1000
#define IMXRT_RPTUN_RESET_DELAY   10

/* Once released, the M4 slice cannot be held in reset again: a software
 * reset is a pulse and the core restarts from CM4_INIT_VTOR.  To stop the
 * core we point its reset vector at a two-instruction park loop written
 * over the start of its image, then pulse the reset.
 *
 *   boot + 0: initial SP        (top of the CM4 system TCM)
 *   boot + 4: reset handler     (boot + 8, Thumb)
 *   boot + 8: wfi; b .-2        (0xBF30, 0xE7FD)
 */

#define IMXRT_RPTUN_PARK_SP       0x20020000
#define IMXRT_RPTUN_PARK_CODE     0xE7FDBF30

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imxrt_rptun_dev_s
{
  struct rptun_dev_s                 rptun;
  const struct imxrt_rptun_config_s *cfg;
  rptun_callback_t                   callback;
  void                              *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *imxrt_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *imxrt_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
imxrt_rptun_get_addrenv(struct rptun_dev_s *dev);
static bool imxrt_rptun_is_autostart(struct rptun_dev_s *dev);
static bool imxrt_rptun_is_master(struct rptun_dev_s *dev);
static int imxrt_rptun_start(struct rptun_dev_s *dev);
static int imxrt_rptun_stop(struct rptun_dev_s *dev);
static int imxrt_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int imxrt_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_imxrt_rptun_ops =
{
  .get_cpuname       = imxrt_rptun_get_cpuname,
  .get_firmware      = imxrt_rptun_get_firmware,
  .get_addrenv       = imxrt_rptun_get_addrenv,
  .is_autostart      = imxrt_rptun_is_autostart,
  .is_master         = imxrt_rptun_is_master,
  .start             = imxrt_rptun_start,
  .stop              = imxrt_rptun_stop,
  .notify            = imxrt_rptun_notify,
  .register_callback = imxrt_rptun_register_callback,
};

static struct imxrt_rptun_dev_s g_imxrt_rptun;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *imxrt_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);

  return priv->cfg->cpuname;
}

static const char *imxrt_rptun_get_firmware(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);

  return priv->cfg->firmware;
}

static const struct rptun_addrenv_s *
imxrt_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);

  return priv->cfg->addrenv;
}

static bool imxrt_rptun_is_autostart(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);

  return priv->cfg->autostart;
}

static bool imxrt_rptun_is_master(struct rptun_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: imxrt_rptun_start
 *
 * Description:
 *   Called by rptun after the ELF is loaded and the virtio device is
 *   DRIVER_OK.  Publish the boot vector and release the CM4.
 *
 ****************************************************************************/

static int imxrt_rptun_start(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);
  const struct rptun_addrenv_s *env;
  uint32_t vtor;
  int retries = IMXRT_RPTUN_RESET_RETRIES;

  /* The loader wrote the image through the (possibly cached) OCRAM_M4
   * alias; make sure the CM4 sees it before it starts fetching.
   */

  for (env = priv->cfg->addrenv; env != NULL && env->size != 0; env++)
    {
      up_clean_dcache(env->pa, env->pa + env->size);
    }

  putreg32(GPR0_CM4_INIT_VTOR_LOW(priv->cfg->boot_addr),
           IMXRT_IOMUXC_LPSR_GPR_GPR0);
  putreg32(GPR1_CM4_INIT_VTOR_HIGH(priv->cfg->boot_addr),
           IMXRT_IOMUXC_LPSR_GPR_GPR1);

  vtor = (getreg32(IMXRT_IOMUXC_LPSR_GPR_GPR1) & 0xffff) << 16 |
         (getreg32(IMXRT_IOMUXC_LPSR_GPR_GPR0) & 0xfff8);
  if (vtor != priv->cfg->boot_addr)
    {
      ipcerr("ERROR: CM4 VTOR readback %08" PRIx32 " != %08" PRIxPTR "\n",
             vtor, priv->cfg->boot_addr);
      return -EIO;
    }

  /* The slice stays in reset until the first release, so only re-reset a
   * core that is already running (restart, or released by a debugger).
   */

  if ((getreg32(IMXRT_SRC_SCR) & SRC_SCR_BT_RELEASE_M4) != 0)
    {
      putreg32(SRC_CTRL_M4CORE_SW_RESET, IMXRT_SRC_CTRL_M4CORE);
      while ((getreg32(IMXRT_SRC_STAT_M4CORE) &
              SRC_STAT_M4CORE_UNDER_RST) != 0)
        {
          if (--retries == 0)
            {
              ipcerr("ERROR: CM4 slice did not leave reset\n");
              return -ETIMEDOUT;
            }

          up_udelay(IMXRT_RPTUN_RESET_DELAY);
        }
    }

  modifyreg32(IMXRT_SRC_SCR, 0, SRC_SCR_BT_RELEASE_M4);

  ipcinfo("CM4 released, VTOR=%08" PRIxPTR "\n", priv->cfg->boot_addr);
  return OK;
}

static int imxrt_rptun_stop(struct rptun_dev_s *dev)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);
  uintptr_t boot = priv->cfg->boot_addr;

  /* Park the core: write the loop over the image start, then reset the
   * slice so it restarts into the loop.  The next start reloads the image.
   */

  putreg32(IMXRT_RPTUN_PARK_SP, boot);
  putreg32((boot + 8) | 1, boot + 4);
  putreg32(IMXRT_RPTUN_PARK_CODE, boot + 8);
  up_clean_dcache(boot, boot + 12);

  putreg32(SRC_CTRL_M4CORE_SW_RESET, IMXRT_SRC_CTRL_M4CORE);
  return OK;
}

/****************************************************************************
 * Name: imxrt_rptun_notify
 *
 * Description:
 *   Kick the CM4.  TEn clears on write and returns once the CM4 has read
 *   RRn; a full mailbox means a kick is already pending and the CM4 rescans
 *   everything when it takes it, so coalesce rather than spin: rptun kicks
 *   before the CM4 is even released from reset.
 *
 ****************************************************************************/

static int imxrt_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  irqstate_t flags = enter_critical_section();

  if ((getreg32(IMXRT_MUA_SR) & MU_SR_TE(IMXRT_RPTUN_MU_CHAN)) != 0)
    {
      putreg32(IMXRT_RPTUN_KICK, IMXRT_MUA_TR(IMXRT_RPTUN_MU_CHAN));
    }

  leave_critical_section(flags);
  return OK;
}

static int imxrt_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg)
{
  struct imxrt_rptun_dev_s *priv =
    container_of(dev, struct imxrt_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;
  return OK;
}

/****************************************************************************
 * Name: imxrt_rptun_mu_interrupt
 *
 * Description:
 *   The CM4 kicked.  Reading RRn clears RFn; rptun scans every virtqueue.
 *
 ****************************************************************************/

static int imxrt_rptun_mu_interrupt(int irq, void *context, void *arg)
{
  struct imxrt_rptun_dev_s *priv = arg;
  uint32_t sr = getreg32(IMXRT_MUA_SR);
  int chan;

  for (chan = 0; chan < IMXRT_MU_CHANNELS; chan++)
    {
      if ((sr & MU_SR_RF(chan)) != 0)
        {
          (void)getreg32(IMXRT_MUA_RR(chan));
        }
    }

  if ((sr & MU_SR_GIP_MASK) != 0)
    {
      putreg32(sr & MU_SR_GIP_MASK, IMXRT_MUA_SR);
    }

  if ((sr & MU_SR_RF(IMXRT_RPTUN_MU_CHAN)) != 0 && priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return OK;
}

/****************************************************************************
 * Name: imxrt_rptun_mu_init
 *
 * Description:
 *   Bring up MU-A without resetting the unit (MUR also resets the CM4
 *   side): clock both sides, drain the receive mailboxes, clear the
 *   general interrupt flags, then enable the channel 0 receive interrupt.
 *
 ****************************************************************************/

static int imxrt_rptun_mu_init(struct imxrt_rptun_dev_s *priv)
{
  int ret;
  int chan;

  imxrt_clockall_mu_a();
  imxrt_clockall_mu_b();

  putreg32(0, IMXRT_MUA_CR);
  for (chan = 0; chan < IMXRT_MU_CHANNELS; chan++)
    {
      (void)getreg32(IMXRT_MUA_RR(chan));
    }

  putreg32(MU_SR_GIP_MASK, IMXRT_MUA_SR);

  ret = irq_attach(IMXRT_IRQ_MU, imxrt_rptun_mu_interrupt, priv);
  if (ret < 0)
    {
      imxrt_clockoff_mu_b();
      imxrt_clockoff_mu_a();
      return ret;
    }

  putreg32(MU_CR_RIE(IMXRT_RPTUN_MU_CHAN), IMXRT_MUA_CR);
  up_enable_irq(IMXRT_IRQ_MU);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imxrt_rptun_init(const struct imxrt_rptun_config_s *cfg)
{
  struct imxrt_rptun_dev_s *priv = &g_imxrt_rptun;
  int ret;

  if (cfg == NULL || cfg->cpuname == NULL || cfg->firmware == NULL ||
      cfg->addrenv == NULL)
    {
      return -EINVAL;
    }

  if (priv->cfg != NULL)
    {
      return -EBUSY;
    }

  /* A CM4 lockup would otherwise reset the whole chip. The CM7 owns the
   * remote (stop/start), so keep the reset local to the CM4.
   */

  modifyreg32(IMXRT_SRC_SRMR, SRC_SRMR_M4LOCKUP_MASK,
              SRC_SRMR_M4LOCKUP_NONE);

  priv->cfg       = cfg;
  priv->rptun.ops = &g_imxrt_rptun_ops;

  ret = imxrt_rptun_mu_init(priv);
  if (ret < 0)
    {
      goto errout;
    }

  ret = rptun_initialize(&priv->rptun);
  if (ret < 0)
    {
      ipcerr("ERROR: rptun_initialize failed: %d\n", ret);
      up_disable_irq(IMXRT_IRQ_MU);
      irq_detach(IMXRT_IRQ_MU);
      imxrt_clockoff_mu_b();
      imxrt_clockoff_mu_a();
      goto errout;
    }

  return OK;

errout:
  priv->cfg = NULL;
  return ret;
}
