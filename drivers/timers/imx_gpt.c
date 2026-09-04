/****************************************************************************
 * drivers/timers/imx_gpt.c
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
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/imx_gpt.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_interrupt
 *
 * Description:
 *   The single interrupt service routine shared by all channels.  Reads the
 *   status register, dispatches to the per-channel handlers, then clears
 *   (write-1-to-clear) exactly the bits it serviced.
 *
 ****************************************************************************/

static int imx_gpt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct imx_gpt_dev_s *dev = (FAR struct imx_gpt_dev_s *)arg;
  irqstate_t flags;
  uint32_t status;
  uint32_t handled = 0;
#if defined(CONFIG_TIMER_IMX_GPT_OCU) || defined(CONFIG_TIMER_IMX_GPT_ICU)
  int ch;
#endif

  flags = spin_lock_irqsave(&dev->lock);

  /* Only act on sources that are both pending and enabled. */

  status = imx_gpt_getreg(dev, IMX_GPT_SR) & imx_gpt_getreg(dev, IMX_GPT_IR);

  /* OCU compare-match channels. */

#ifdef CONFIG_TIMER_IMX_GPT_OCU
  for (ch = 0; ch < IMX_GPT_NUM_OCU; ch++)
    {
      if ((status & GPT_SR_OF(ch)) != 0)
        {
          handled |= GPT_SR_OF(ch);
          if (dev->ocu[ch] != NULL)
            {
              imx_gpt_ocu_on_match(dev->ocu[ch]);
            }
        }
    }
#endif

#ifdef CONFIG_TIMER_IMX_GPT_ICU
  /* ICU edge-capture channels. */

  for (ch = 0; ch < IMX_GPT_NUM_ICU; ch++)
    {
      if ((status & GPT_SR_IF(ch)) != 0)
        {
          handled |= GPT_SR_IF(ch);
          if (dev->icu[ch] != NULL)
            {
              imx_gpt_icu_on_edge(dev->icu[ch]);
            }
        }
    }
#endif

  /* Counter roll-over accounting. */

  if ((status & GPT_SR_ROV) != 0)
    {
      handled |= GPT_SR_ROV;
      dev->rollover++;
    }

  /* Write-1-to-clear the serviced status bits. */

  if (handled != 0)
    {
      imx_gpt_putreg(dev, IMX_GPT_SR, handled);
    }

  spin_unlock_irqrestore(&dev->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: imx_gpt_hw_init
 *
 * Description:
 *   Software-reset the block and start the shared free-running counter:
 *   no prescaler divide, any non-zero CLKSRC (the QEMU virt model uses the
 *   qdev clock-in), free-run mode, ENMOD so the counter restarts from 0,
 *   and finally EN.  All per-channel OM bits stay disabled and IR is
 *   cleared; channels enable their own bits on start.
 *
 ****************************************************************************/

static void imx_gpt_hw_init(FAR struct imx_gpt_dev_s *dev)
{
  /* Software reset (self-clearing) and wait for it to complete. */

  imx_gpt_putreg(dev, IMX_GPT_CR, GPT_CR_SWR);
  while ((imx_gpt_getreg(dev, IMX_GPT_CR) & GPT_CR_SWR) != 0)
    {
    }

  /* Mask and clear all interrupt sources. */

  imx_gpt_putreg(dev, IMX_GPT_IR, 0);
  imx_gpt_putreg(dev, IMX_GPT_SR, 0x3f);

  /* No prescaler division (tick freq == input clock). */

  imx_gpt_putreg(dev, IMX_GPT_PR, 0);

  /* CLKSRC = 1 (peripheral clock), free-run, restart-from-zero, enable. */

  imx_gpt_putreg(dev, IMX_GPT_CR,
                 (1 << GPT_CR_CLKSRC_SHIFT) | GPT_CR_FRR |
                 GPT_CR_ENMOD | GPT_CR_EN);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx_gpt_initialize
 ****************************************************************************/

int imx_gpt_initialize(uintptr_t base, int irq, uint32_t freq,
                       int ocu_bus, int cap_bus)
{
  FAR struct imx_gpt_dev_s *dev;
#if defined(CONFIG_TIMER_IMX_GPT_OCU) || defined(CONFIG_TIMER_IMX_GPT_ICU)
  char devpath[20];
  int ch;
#endif
  int ret;

  DEBUGASSERT(base != 0);
#ifndef CONFIG_TIMER_IMX_GPT_OCU
  UNUSED(ocu_bus);
#endif
#ifndef CONFIG_TIMER_IMX_GPT_ICU
  UNUSED(cap_bus);
#endif

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->base = base;
  dev->irq  = irq;
  dev->freq = freq;
  spin_lock_init(&dev->lock);

  /* Bring up the shared counter before touching any channel. */

  imx_gpt_hw_init(dev);

  /* Register the OCU channels as /dev/ocu<ocu_bus + ch>. */

#ifdef CONFIG_TIMER_IMX_GPT_OCU
  for (ch = 0; ch < IMX_GPT_NUM_OCU; ch++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/ocu%d", ocu_bus + ch);
      dev->ocu[ch] = imx_gpt_ocu_register(dev, ch, devpath);
      if (dev->ocu[ch] == NULL)
        {
          tmrerr("ERROR: failed to register %s\n", devpath);
          ret = -ENODEV;
          goto err;
        }
    }
#endif

#ifdef CONFIG_TIMER_IMX_GPT_ICU
  /* Register the ICU channels as /dev/cap<cap_bus + ch>. */

  for (ch = 0; ch < IMX_GPT_NUM_ICU; ch++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/cap%d", cap_bus + ch);
      dev->icu[ch] = imx_gpt_icu_register(dev, ch, devpath);
      if (dev->icu[ch] == NULL)
        {
          tmrerr("ERROR: failed to register %s\n", devpath);
          ret = -ENODEV;
          goto err;
        }
    }
#endif

  /* Attach and enable the shared interrupt now that all channels exist. */

  ret = irq_attach(irq, imx_gpt_interrupt, dev);
  if (ret < 0)
    {
      tmrerr("ERROR: irq_attach failed: %d\n", ret);
      goto err;
    }

  up_enable_irq(irq);
  return OK;

err:
  kmm_free(dev);
  return ret;
}
