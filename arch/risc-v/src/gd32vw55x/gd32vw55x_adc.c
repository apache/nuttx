/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_adc.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "gd32vw55x_gpio.h"
#include "gd32vw55x_adc.h"
#include "hardware/gd32vw55x_adc.h"
#include "hardware/gd32vw55x_rcu.h"

#ifdef CONFIG_GD32VW55X_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The ADC is clocked from PCLK2 (160 MHz in this port).  PCLK2/8 = 20 MHz
 * is used, which is within the range of every part of the family.
 */

#ifndef CONFIG_GD32VW55X_ADC_CLOCK
#  define CONFIG_GD32VW55X_ADC_CLOCK  ADC_CCTL_ADCCK_PCLK2_DIV8
#endif

/* Conversion resolution: 12 bits by default */

#ifndef CONFIG_GD32VW55X_ADC_RESOLUTION
#  define CONFIG_GD32VW55X_ADC_RESOLUTION  ADC_CTL0_DRES_12B
#endif

/* Sampling time used for every channel */

#ifndef CONFIG_GD32VW55X_ADC_SAMPLETIME
#  define CONFIG_GD32VW55X_ADC_SAMPLETIME  ADC_SAMPLETIME_55POINT5
#endif

/* Board pin configurations of the external analog inputs */

#ifndef GPIO_ADC_IN0
#  define GPIO_ADC_IN0  0
#endif
#ifndef GPIO_ADC_IN1
#  define GPIO_ADC_IN1  0
#endif
#ifndef GPIO_ADC_IN2
#  define GPIO_ADC_IN2  0
#endif
#ifndef GPIO_ADC_IN3
#  define GPIO_ADC_IN3  0
#endif
#ifndef GPIO_ADC_IN4
#  define GPIO_ADC_IN4  0
#endif
#ifndef GPIO_ADC_IN5
#  define GPIO_ADC_IN5  0
#endif
#ifndef GPIO_ADC_IN6
#  define GPIO_ADC_IN6  0
#endif
#ifndef GPIO_ADC_IN7
#  define GPIO_ADC_IN7  0
#endif
#ifndef GPIO_ADC_IN8
#  define GPIO_ADC_IN8  0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_adc_s
{
  const struct adc_callback_s *cb;  /* Upper half callback */
  uint8_t  irq;                     /* ADC interrupt number */
  uint8_t  nchannels;               /* Number of channels of the sequence */
  uint8_t  current;                 /* Rank of the running conversion */
  bool     enabled;                 /* The ADC is powered up */
  uint8_t  chanlist[GD32VW55X_ADC_MAX_SAMPLES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static void adc_set_sampletime(uint8_t channel, uint32_t sampletime);
static void adc_set_rank(uint8_t rank, uint8_t channel);
static void adc_configure_pin(uint8_t channel);
static int  adc_interrupt(int irq, void *context, void *arg);

/* ADC driver methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind       = adc_bind,
  .ao_reset      = adc_reset,
  .ao_setup      = adc_setup,
  .ao_shutdown   = adc_shutdown,
  .ao_rxint      = adc_rxint,
  .ao_ioctl      = adc_ioctl,
};

static struct gd32_adc_s g_adcpriv =
{
  .irq           = GD32VW55X_IRQ_ADC,
};

static struct adc_dev_s g_adcdev =
{
  .ad_ops        = &g_adcops,
  .ad_priv       = &g_adcpriv,
};

/* The GPIO configuration of every external analog input.  Channel 9 is the
 * temperature sensor and channel 10 is the internal reference voltage; they
 * have no pin.
 */

static const uint32_t g_adcpins[GD32VW55X_ADC_NCHANNELS] =
{
  GPIO_ADC_IN0, GPIO_ADC_IN1, GPIO_ADC_IN2, GPIO_ADC_IN3, GPIO_ADC_IN4,
  GPIO_ADC_IN5, GPIO_ADC_IN6, GPIO_ADC_IN7, GPIO_ADC_IN8, 0, 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_set_sampletime
 *
 * Description:
 *   Set the sampling time of one channel.  Each channel owns a 4-bit field:
 *   channels 0..7 are in SAMPT1 and channels 8..10 are in SAMPT0.
 *
 ****************************************************************************/

static void adc_set_sampletime(uint8_t channel, uint32_t sampletime)
{
  uint32_t regaddr;
  uint32_t shift;

  if (channel < 8)
    {
      regaddr = GD32VW55X_ADC_SAMPT1;
      shift   = channel * ADC_SAMPT_LENGTH;
    }
  else
    {
      regaddr = GD32VW55X_ADC_SAMPT0;
      shift   = (channel - 8) * ADC_SAMPT_LENGTH;
    }

  modifyreg32(regaddr, ADC_SAMPT_MASK << shift, sampletime << shift);
}

/****************************************************************************
 * Name: adc_set_rank
 *
 * Description:
 *   Put one channel at the given rank of the routine sequence.  Ranks 0..5
 *   are in RSQ2 and ranks 6..8 are in RSQ1.
 *
 ****************************************************************************/

static void adc_set_rank(uint8_t rank, uint8_t channel)
{
  uint32_t regaddr;
  uint32_t shift;

  if (rank < 6)
    {
      regaddr = GD32VW55X_ADC_RSQ2;
      shift   = rank * ADC_RSQ_LENGTH;
    }
  else
    {
      regaddr = GD32VW55X_ADC_RSQ1;
      shift   = (rank - 6) * ADC_RSQ_LENGTH;
    }

  modifyreg32(regaddr, ADC_RSQ_MASK << shift, (uint32_t)channel << shift);
}

/****************************************************************************
 * Name: adc_configure_pin
 *
 * Description:
 *   Configure the pin of one channel as an analog input.  The internal
 *   channels are enabled instead.
 *
 ****************************************************************************/

static void adc_configure_pin(uint8_t channel)
{
  if (channel >= GD32VW55X_ADC_NCHANNELS)
    {
      return;
    }

  /* Channel 9 (temperature sensor) and channel 10 (VREFINT) are internal */

  if (channel >= 9)
    {
      modifyreg32(GD32VW55X_ADC_CCTL, 0, ADC_CCTL_TSVREN);
    }
  else if (g_adcpins[channel] != 0)
    {
      gd32_gpio_config(g_adcpins[channel]);
    }
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler.  The EOC flag is set at the end of every routine
 *   conversion; the converted value is passed to the upper half driver.
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context, void *arg)
{
  struct adc_dev_s  *dev  = (struct adc_dev_s *)arg;
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;
  uint32_t stat;
  int32_t  data;

  stat = getreg32(GD32VW55X_ADC_STAT);

  /* Routine data register overflow: the conversions are being produced
   * faster than they are consumed.  Just clear the condition.
   */

  if ((stat & ADC_STAT_ROVF) != 0)
    {
      modifyreg32(GD32VW55X_ADC_STAT, ADC_STAT_ROVF, 0);
    }

  if ((stat & ADC_STAT_EOC) != 0)
    {
      /* Reading the data register clears the EOC flag */

      data = (int32_t)(getreg32(GD32VW55X_ADC_RDATA) & ADC_RDATA_MASK);

      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          priv->cb->au_receive(dev, priv->chanlist[priv->current], data);
        }

      /* Move to the next rank of the sequence */

      if (++priv->current >= priv->nchannels)
        {
          priv->current = 0;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper half driver callbacks to the lower half implementation.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.  This
 *   is called before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t regval;
  int i;

  flags = enter_critical_section();

  /* Enable the ADC clock and reset the peripheral */

  modifyreg32(GD32VW55X_RCU_APB2EN, 0, RCU_APB2EN_ADCEN);

  modifyreg32(GD32VW55X_RCU_APB2RST, 0, RCU_APB2RST_ADCRST);
  modifyreg32(GD32VW55X_RCU_APB2RST, RCU_APB2RST_ADCRST, 0);

  priv->enabled = false;
  priv->current = 0;

  /* ADC clock */

  modifyreg32(GD32VW55X_ADC_CCTL, ADC_CCTL_ADCCK_MASK,
              CONFIG_GD32VW55X_ADC_CLOCK);

  /* Scan mode and the requested resolution.  The EOC flag is raised at the
   * end of each conversion of the sequence, not only at the end of the
   * sequence.
   */

  regval = ADC_CTL0_SM | CONFIG_GD32VW55X_ADC_RESOLUTION;
  putreg32(regval, GD32VW55X_ADC_CTL0);

  /* Right aligned data, EOC set after each conversion, no external trigger
   * and no DMA.  The conversions are started by software.
   */

  putreg32(ADC_CTL1_EOCM, GD32VW55X_ADC_CTL1);

  /* No oversampling */

  putreg32(0, GD32VW55X_ADC_OVSAMPCTL);

  /* Configure the conversion sequence */

  for (i = 0; i < priv->nchannels; i++)
    {
      adc_configure_pin(priv->chanlist[i]);
      adc_set_sampletime(priv->chanlist[i],
                         CONFIG_GD32VW55X_ADC_SAMPLETIME);
      adc_set_rank(i, priv->chanlist[i]);
    }

  modifyreg32(GD32VW55X_ADC_RSQ0, ADC_RSQ0_RL_MASK,
              ADC_RSQ0_RL(priv->nchannels));

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC.  This method is called the first time that the ADC
 *   device is opened.  This setup includes configuring and attaching the
 *   ADC interrupt.  Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;
  int ret;

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, adc_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach IRQ %u: %d\n", priv->irq, ret);
      return ret;
    }

  /* Make sure that the interrupts are disabled and power up the ADC */

  modifyreg32(GD32VW55X_ADC_CTL0,
              ADC_CTL0_EOCIE | ADC_CTL0_ROVFIE | ADC_CTL0_WDEIE |
              ADC_CTL0_EOICIE, 0);

  modifyreg32(GD32VW55X_ADC_CTL1, 0, ADC_CTL1_ADCON);
  priv->enabled = true;
  priv->current = 0;

  up_enable_irq(priv->irq);

  return OK;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *
 ****************************************************************************/

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;

  /* Disable the ADC interrupts */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  modifyreg32(GD32VW55X_ADC_CTL0,
              ADC_CTL0_EOCIE | ADC_CTL0_ROVFIE, 0);

  /* Power down the ADC and turn its clock off */

  modifyreg32(GD32VW55X_ADC_CTL1, ADC_CTL1_ADCON, 0);
  modifyreg32(GD32VW55X_RCU_APB2EN, RCU_APB2EN_ADCEN, 0);

  priv->enabled = false;
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable the end of conversion interrupt.
 *
 ****************************************************************************/

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  if (enable)
    {
      modifyreg32(GD32VW55X_ADC_CTL0, 0,
                  ADC_CTL0_EOCIE | ADC_CTL0_ROVFIE);
    }
  else
    {
      modifyreg32(GD32VW55X_ADC_CTL0,
                  ADC_CTL0_EOCIE | ADC_CTL0_ROVFIE, 0);
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls are routed through this method.
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct gd32_adc_s *priv = (struct gd32_adc_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start the conversion of the whole routine sequence */

          irqstate_t flags = enter_critical_section();

          priv->current = 0;
          modifyreg32(GD32VW55X_ADC_CTL1, 0, ADC_CTL1_SWRCST);

          leave_critical_section(flags);
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          ret = priv->nchannels;
        }
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.  The GD32VW55x has a single ADC, so 'intf' must be
 *   zero.
 *
 * Input Parameters:
 *   intf       - Must be 0
 *   chanlist   - The list of channels (0..10) to convert, in the order of
 *                the conversion sequence
 *   nchannels  - Number of channels of chanlist, at most 9
 *
 * Returned Value:
 *   On success, a pointer to the ADC device structure is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

struct adc_dev_s *gd32_adc_initialize(int intf, const uint8_t *chanlist,
                                      int nchannels)
{
  struct gd32_adc_s *priv = &g_adcpriv;
  int i;

  ainfo("intf: %d nchannels: %d\n", intf, nchannels);

  if (intf != 0)
    {
      aerr("ERROR: No such ADC interface: %d\n", intf);
      return NULL;
    }

  if (chanlist == NULL || nchannels < 1 ||
      nchannels > GD32VW55X_ADC_MAX_SAMPLES)
    {
      aerr("ERROR: Invalid channel list of %d channels\n", nchannels);
      return NULL;
    }

  for (i = 0; i < nchannels; i++)
    {
      if (chanlist[i] >= GD32VW55X_ADC_NCHANNELS)
        {
          aerr("ERROR: No such ADC channel: %u\n", chanlist[i]);
          return NULL;
        }
    }

  memcpy(priv->chanlist, chanlist, nchannels);
  priv->nchannels = (uint8_t)nchannels;
  priv->current   = 0;

  return &g_adcdev;
}

#endif /* CONFIG_GD32VW55X_ADC */
