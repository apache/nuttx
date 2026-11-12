/****************************************************************************
 * arch/arm/src/kinetis/kinetis_adc.c
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
 * 16-bit Analog-to-Digital Converter (ADC)
 *
 * From the Reference Manual:
 *
 *  The 16-bit analog-to-digital converter (ADC) is a successive
 *  approximation ADC designed for operation within an integrated
 *  microcontroller system-on-chip.
 *
 * This file provides ADC driver for the Kinetis' 16-bit ADC.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>
#include <nuttx/analog/adc.h>   /* adc_dev_s, adc_ops_s, ... */
#include <nuttx/analog/ioctl.h> /* ANIOC_TRIGGER */
#include <nuttx/irq.h>          /* Enter and leave critical sections. */
#include <nuttx/wqueue.h>       /* struct work_s */

#include "arm_internal.h"             /* putreg... */
#include "hardware/kinetis_adc.h"     /* KINETIS_ADC... */
#include "hardware/kinetis_sim.h"     /* SIM_... */

#if defined(CONFIG_KINETIS_ADC0) || defined(CONFIG_KINETIS_ADC1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS 24

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kinetis_ad_priv_s
{
  int irq; /* What IRQ is used for the ADC device? */
  int rr;  /* Result register. */
  int scr; /* Status and Control Register. */

  uint8_t ch_count;                  /* Number of used channels. */
  uint8_t ch_list[ADC_MAX_CHANNELS]; /* List of used channels. */
  uint8_t ch_in_use;                 /* Actual channel being converted. */

  struct adc_callback_s const *cb;
  struct work_s work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  adc_bind(struct adc_dev_s *dev, struct adc_callback_s const *cb);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

static int  isr(int irq, void *context, void *arg);
static void work(void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adc_ops_s const g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

#if defined(CONFIG_KINETIS_ADC0)
static struct kinetis_ad_priv_s g_a0d_priv =
{
  .irq = KINETIS_IRQ_ADC0,
  .rr  = KINETIS_ADC0_RA,
  .scr = KINETIS_ADC0_SC1A,
};
static struct adc_dev_s g_adc0_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_a0d_priv,
};
#endif /* CONFIG_KINETIS_ADC0 */

#if defined(CONFIG_KINETIS_ADC1)
static struct kinetis_ad_priv_s g_a1d_priv =
{
  .irq = KINETIS_IRQ_ADC1,
  .rr  = KINETIS_ADC1_RA,
  .scr = KINETIS_ADC1_SC1A,
};
static struct adc_dev_s g_adc1_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_a1d_priv,
};
#endif /* CONFIG_KINETIS_ADC1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev, struct adc_callback_s const *cb)
{
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;

  DEBUGASSERT(NULL != priv);

  priv->cb = cb;
  return OK;
}

static void adc_reset(struct adc_dev_s *dev)
{
  uint32_t reg32;

  adc_shutdown(dev);

#if defined(CONFIG_KINETIS_ADC0)
  if (&g_adc0_dev == dev)
    {
      reg32 = getreg32(KINETIS_SIM_SCGC6);
      reg32 |= SIM_SCGC6_ADC0;
      putreg32(reg32, KINETIS_SIM_SCGC6);
    }
#endif

#if defined(CONFIG_KINETIS_ADC1)
  if (&g_adc1_dev == dev)
    {
      reg32 = getreg32(KINETIS_SIM_SCGC3);
      reg32 |= SIM_SCGC3_ADC1;
      putreg32(reg32, KINETIS_SIM_SCGC3);
    }
#endif
}

static int adc_setup(struct adc_dev_s *dev)
{
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  int ret = OK;

  priv->ch_in_use = 0; /* Start from channel at index 0. */

  ret = irq_attach(priv->irq, isr, dev);
  return ret;
}

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  uint32_t reg32;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

#if defined(CONFIG_KINETIS_ADC0)
  if (&g_adc0_dev == dev)
    {
      reg32 = getreg32(KINETIS_SIM_SCGC6);
      reg32 &= ~SIM_SCGC6_ADC0;
      putreg32(reg32, KINETIS_SIM_SCGC6);
    }
#endif

#if defined(CONFIG_KINETIS_ADC1)
  if (&g_adc1_dev == dev)
    {
      reg32 = getreg32(KINETIS_SIM_SCGC3);
      reg32 &= ~SIM_SCGC3_ADC1;
      putreg32(reg32, KINETIS_SIM_SCGC3);
    }
#endif
}

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  uint32_t reg32;

  if (enable)
    {
      reg32 = ADC_SC1_AIEN | priv->ch_list[priv->ch_in_use];
      putreg32(reg32, priv->scr);
      up_enable_irq(priv->irq);
    }
  else
    {
      up_disable_irq(priv->irq);
      putreg32(ADC_SC1_ADCH_DISABLED, priv->scr);
    }
}

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  int ret = OK;

  UNUSED(arg);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        adc_rxint(dev, true);
        break;

      case ANIOC_GET_NCHANNELS:
        ret = priv->ch_count;
        break;

      default:
        awarn("%d not implemented\n", cmd);
        ret = -ENOTTY;
    }

  return ret;
}

static int isr(int irq, void *context, void *arg)
{
  struct adc_dev_s *dev = (struct adc_dev_s *)arg;
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  int ret = OK;

  up_disable_irq(priv->irq);
  DEBUGVERIFY(work_queue(HPWORK, &priv->work, work, dev, 0));

  UNUSED(context);

  return ret;
}

static void work(void *arg)
{
  struct adc_dev_s *dev = (struct adc_dev_s *)arg;
  struct kinetis_ad_priv_s *priv = (struct kinetis_ad_priv_s *)dev->ad_priv;
  int32_t reg32;

  reg32 = getreg32(priv->scr) & ADC_SC1_ADCH_MASK;

  if (reg32 == priv->ch_list[priv->ch_in_use])
    {
      reg32 = getreg32(priv->rr);

      if (NULL != priv->cb)
        {
          priv->cb->au_receive(dev, priv->ch_list[priv->ch_in_use], reg32);
        }

      if (++priv->ch_in_use >= priv->ch_count)
        {
          priv->ch_in_use = 0;
        }
      else
        {
          adc_rxint(dev, true);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct adc_dev_s *kinetis_adc_initialize(
    int which, uint8_t const *ch_list, uint8_t ch_count)
{
  switch (which)
    {
      case 0:
#if defined(CONFIG_KINETIS_ADC0)
        g_a0d_priv.ch_count = ch_count;
        memcpy(g_a0d_priv.ch_list, ch_list, ch_count);
        return &g_adc0_dev;
#else
        awarn("CONFIG_KINETIS_ADC0 must be defined");
        return NULL;
#endif
        break;
      case 1:
#if defined(CONFIG_KINETIS_ADC1)
        g_a1d_priv.ch_count = ch_count;
        memcpy(g_a1d_priv.ch_list, ch_list, ch_count);
        return &g_adc1_dev;
#else
        awarn("CONFIG_KINETIS_ADC1 must be defined");
        return NULL;
#endif
        break;
      default:
        awarn("Bad device specified, must be 0 <= %d <= 1", which);
    }

  return NULL;
}

#endif /* defined(CONFIG_KINETIS_ADC0) || defined(CONFIG_KINETIS_ADC1) */
