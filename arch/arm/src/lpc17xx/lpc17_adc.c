/************************************************************************************
 * arch/arm/src/lpc17xx/lpc17_adc.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *   History: 0.1 2011-08-05 initial version
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_adc.h"

#if defined(CONFIG_LPC17_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ADC0_MASK
#define CONFIG_ADC0_MASK     0x01
#endif
#ifndef CONFIG_ADC0_SPS
#define CONFIG_ADC0_SPS      1000
#endif
#ifndef CONFIG_ADC0_AVERAGE
#define CONFIG_ADC0_AVERAGE  200
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint8_t  mask;
  uint32_t sps;
  int      irq;
  int32_t  buf[8];
  uint8_t  count[8];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC methods */

static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
static int  adc_interrupt(int irq, void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

static struct up_dev_s g_adcpriv =
{
  .sps         = CONFIG_ADC0_SPS,
  .mask        = CONFIG_ADC0_MASK,
  .irq         = LPC17_IRQ_ADC,
};

static struct adc_dev_s g_adcdev =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t clkdiv;
  uint32_t regval;

  flags = irqsave();

  regval  = getreg32(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCADC;
  putreg32(regval, LPC17_SYSCON_PCONP);

  putreg32(ADC_CR_PDN, LPC17_ADC_CR);

  regval  = getreg32(LPC17_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_ADC_MASK;
  regval |= (SYSCON_PCLKSEL_CCLK8 << SYSCON_PCLKSEL0_ADC_SHIFT);
  putreg32(regval, LPC17_SYSCON_PCLKSEL0);

  clkdiv   = LPC17_CCLK / 8 / 65 / priv->sps;
  clkdiv <<= 8;
  clkdiv  &= 0xff00;
  putreg32(ADC_CR_PDN | ADC_CR_BURST | clkdiv | priv->mask, LPC17_ADC_CR);

  if ((priv->mask & 0x01) != 0)
    {
      lpc17_configgpio(GPIO_AD0p0);
    }
  if ((priv->mask & 0x02) != 0)
    {
      lpc17_configgpio(GPIO_AD0p1);
    }
  if ((priv->mask & 0x04) != 0)
    {
      lpc17_configgpio(GPIO_AD0p2);
    }
  if ((priv->mask & 0x08) != 0)
    {
      lpc17_configgpio(GPIO_AD0p3);
    }
  if ((priv->mask & 0x10) != 0)
    {
      lpc17_configgpio(GPIO_AD0p4);
    }
  if ((priv->mask & 0x20) != 0)
    {
      lpc17_configgpio(GPIO_AD0p5);
    }
  if ((priv->mask & 0x40) != 0)
    {
      lpc17_configgpio(GPIO_AD0p6);
    }
  if ((priv->mask & 0x80) != 0)
    {
      lpc17_configgpio(GPIO_AD0p7);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.  Interrupts
 *   are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;
  int i;

  int ret = irq_attach(priv->irq, adc_interrupt);
  if (ret == OK)
    {
      for (i = 0; i < 8; i++)
        {
          priv->buf[i]   = 0;
          priv->count[i] = 0;
        }

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */

  putreg32(0, LPC17_ADC_INTEN);
  up_disable_irq(priv->irq);

  /* Then detach the ADC interrupt handler. */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)dev->ad_priv;

  if (enable)
    {
#ifdef CONFIG_ADC_CHANLIST
      /* Trigger interrupt at the end of conversion on the last A/D channel
       * in the channel list.
       */

      putreg32(ADC_INTEN_CHAN(g_adc_chanlist[CONFIG_ADC_NCHANNELS - 1]),
               LPC17_ADC_INTEN);
#else
      /* Trigger interrupt using the global DONE flag. */

      putreg32(ADC_INTEN_GLOBAL, LPC17_ADC_INTEN);
#endif
    }
  else
    {
      putreg32(0, LPC17_ADC_INTEN);
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  /* No ioctl commands supported */

  return -ENOTTY;
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int adc_interrupt(int irq, void *context)
{
#ifdef CONFIG_ADC_CHANLIST

  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)g_adcdev.ad_priv;
  uint32_t regval;
  unsigned char ch;
  int32_t value;
  int i;

  regval = getreg32(LPC17_ADC_GDR);
  for (i = 0; i < CONFIG_ADC_NCHANNELS; i++)
    {
      ch     = g_adc_chanlist[i];
      regval = getreg32(LPC17_ADC_DR(ch));

      if (regval&ADC_DR_DONE)
        {
          priv->count[ch]++;
          priv->buf[ch] += regval & 0xfff0;

          if (priv->count[ch] >= CONFIG_ADC0_AVERAGE)
            {
              value           = priv->buf[ch] / priv->count[ch];
              value         <<= 15;
              adc_receive(&g_adcdev,ch,value);
              priv->buf[ch]   = 0;
              priv->count[ch] = 0;
            }
        }
    }

  return OK;

#else

  FAR struct up_dev_s *priv = (FAR struct up_dev_s *)g_adcdev.ad_priv;
  uint32_t regval;
  unsigned char ch;
  int32_t value;

  regval              = getreg32(LPC17_ADC_GDR);
  ch                  = (regval >> 24) & 0x07;
  priv->buf[ch]      += regval & 0xfff0;

  priv->count[ch]++;
  if (priv->count[ch] >= CONFIG_ADC0_AVERAGE)
    {
      value           = priv->buf[ch] / priv->count[ch];
      value         <<= 15;
      adc_receive(&g_adcdev,ch,value);
      priv->buf[ch]   = 0;
      priv->count[ch] = 0;
    }

  return OK;

#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *lpc17_adcinitialize(void)
{
  return &g_adcdev;
}

#endif /* CONFIG_LPC17_ADC */
