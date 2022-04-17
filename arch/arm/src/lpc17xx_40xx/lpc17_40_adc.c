/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_adc.c
 *
 *   Copyright (C) 2011 Li Zhuoyi. All rights reserved.
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2010, 2013, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/lpc17_40_syscon.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_adc.h"

#if defined(CONFIG_LPC17_40_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LPC17_40_ADC0_MASK
#  define CONFIG_LPC17_40_ADC0_MASK    0x01
#endif

#ifndef CONFIG_LPC17_40_ADC0_SPS
#  define CONFIG_LPC17_40_ADC0_SPS     1000
#endif

#ifndef CONFIG_LPC17_40_ADC0_AVERAGE
#  define CONFIG_LPC17_40_ADC0_AVERAGE 200
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const struct adc_callback_s *cb;
  uint8_t  mask;
  uint32_t sps;
  int      irq;
  int32_t  buf[8];
  uint8_t  count[8];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void adc_receive(struct up_dev_s *priv, uint8_t ch, int32_t data);

/* ADC methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static int  adc_interrupt(int irq, void *context, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

static struct up_dev_s g_adcpriv =
{
  .sps         = CONFIG_LPC17_40_ADC0_SPS,
  .mask        = CONFIG_LPC17_40_ADC0_MASK,
  .irq         = LPC17_40_IRQ_ADC,
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
 * Name: adc_receive
 *
 * Description:
 *   Provide received ADC dat to the upper-half driver.
 *
 ****************************************************************************/

static void adc_receive(struct up_dev_s *priv, uint8_t ch, int32_t data)
{
  /* Verify that the upper-half driver has bound its callback functions. */

  if (priv->cb != NULL)
    {
      /* Perform the data received callback */

      DEBUGASSERT(priv->cb->au_receive != NULL);
      priv->cb->au_receive(&g_adcdev, ch, data);
    }
}

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void adc_reset(struct adc_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t clkdiv;
  uint32_t regval;

  flags = enter_critical_section();

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCADC;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Power up before we access hardware */

  putreg32(ADC_CR_PDN, LPC17_40_ADC_CR);

#ifdef LPC176x
  /* PCLKSEL0 only exists in LPC176x family parts */

  regval  = getreg32(LPC17_40_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_ADC_MASK;
  regval |= (SYSCON_PCLKSEL_CCLK8 << SYSCON_PCLKSEL0_ADC_SHIFT);
  putreg32(regval, LPC17_40_SYSCON_PCLKSEL0);
#endif

#ifdef CONFIG_LPC17_40_ADC_BURSTMODE
  clkdiv   = LPC17_40_CCLK / 3 / 65 / priv->sps;

  /* putreg32(0x04, LPC17_40_ADC_INTEN);
   *                                   Enable only last channel interrupt
   */

  putreg32(0x100, LPC17_40_ADC_INTEN);     /* Enable only global interrupt */

  putreg32((priv->mask) |                  /* Select channels 0 to 7 on ADC0 */

  /*       (clkdiv) << 8) |           CLKDIV = divisor to make the samples
   *                                  per second conversion rate
   */

           ((32) << 8) |                        /* CLKDIV = divisor to make the faster
                                                 * conversion rate */
           (0 << 16) |                          /* BURST = 0, BURST capture all selected
                                                 * channels */
           (1 << 17) |                          /* Reserved bit = 0 */
           (1 << 21) |                          /* PDN = 1, normal operation */
           (1 << 26) | (0 << 25) | (0 << 24) |  /* START = at MAT0 signal */
           (1 << 27),                           /* EDGE = 1 (CAP/MAT signal rising
                                                 * trigger A/D conversion) */
           LPC17_40_ADC_CR);

#else /* CONFIG_LPC17_40_ADC_BURSTMODE */

  clkdiv   = LPC17_40_CCLK / 8 / 65 / priv->sps;
  clkdiv <<= 8;
  clkdiv  &= 0xff00;
  putreg32(ADC_CR_PDN | ADC_CR_BURST | clkdiv | priv->mask, LPC17_40_ADC_CR);

#endif /* CONFIG_LPC17_40_ADC_BURSTMODE */

  if ((priv->mask & 0x01) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p0);
    }

  if ((priv->mask & 0x02) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p1);
    }

  if ((priv->mask & 0x04) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p2);
    }

  if ((priv->mask & 0x08) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p3);
    }

  if ((priv->mask & 0x10) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p4);
    }

  if ((priv->mask & 0x20) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p5);
    }

  if ((priv->mask & 0x40) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p6);
    }

  if ((priv->mask & 0x80) != 0)
    {
      lpc17_40_configgpio(GPIO_AD0p7);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int adc_setup(struct adc_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;
  int i;

  int ret = irq_attach(priv->irq, adc_interrupt, NULL);
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

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */

  putreg32(0, LPC17_40_ADC_INTEN);
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

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->ad_priv;

  if (enable)
    {
#ifndef CONFIG_LPC17_40_ADC_BURSTMODE
#ifdef CONFIG_LPC17_40_ADC_CHANLIST
      /* Trigger interrupt at the end of conversion on the last A/D channel
       * in the channel list.
       */

      putreg32(ADC_INTEN_CHAN(
               g_adc_chanlist[CONFIG_LPC17_40_ADC_NCHANNELS - 1]),
               LPC17_40_ADC_INTEN);
#else
      /* Trigger interrupt using the global DONE flag. */

      putreg32(ADC_INTEN_GLOBAL, LPC17_40_ADC_INTEN);
#endif
#else /* CONFIG_LPC17_40_ADC_BURSTMODE */
      /* Enable only global interrupt */

      putreg32(0x100, LPC17_40_ADC_INTEN);
#endif /* CONFIG_LPC17_40_ADC_BURSTMODE */
    }
  else
    {
      putreg32(0, LPC17_40_ADC_INTEN);
    }
}

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *  All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
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

static int adc_interrupt(int irq, void *context, void *arg)
{
#ifndef CONFIG_LPC17_40_ADC_BURSTMODE
#ifdef CONFIG_LPC17_40_ADC_CHANLIST

  struct up_dev_s *priv = (struct up_dev_s *)g_adcdev.ad_priv;
  uint32_t regval;
  unsigned char ch;
  int32_t value;
  int i;

  regval = getreg32(LPC17_40_ADC_GDR);
  for (i = 0; i < CONFIG_LPC17_40_ADC_NCHANNELS; i++)
    {
      ch     = g_adc_chanlist[i];
      regval = getreg32(LPC17_40_ADC_DR(ch));

      if (regval&ADC_DR_DONE)
        {
          priv->count[ch]++;
          priv->buf[ch] += regval & 0xfff0;

          if (priv->count[ch] >= CONFIG_LPC17_40_ADC0_AVERAGE)
            {
              value           = priv->buf[ch] / priv->count[ch];
              value         <<= 15;
              adc_receive(priv, ch, value);
              priv->buf[ch]   = 0;
              priv->count[ch] = 0;
            }
        }
    }

  return OK;

#else

  struct up_dev_s *priv = (struct up_dev_s *)g_adcdev.ad_priv;
  uint32_t regval;
  unsigned char ch;
  int32_t value;

  regval               = getreg32(LPC17_40_ADC_GDR);
  ch                   = (regval >> 24) & 0x07;
  priv->buf[ch]       += regval & 0xfff0;

  priv->count[ch]++;
  if (priv->count[ch] >= CONFIG_LPC17_40_ADC0_AVERAGE)
    {
      value            = priv->buf[ch] / priv->count[ch];
      value          <<= 15;
      adc_receive(priv, ch, value);
      priv->buf[ch]    = 0;
      priv->count[ch]  = 0;
    }

  return OK;

#endif
#else /* CONFIG_LPC17_40_ADC_BURSTMODE */

  struct up_dev_s *priv = (struct up_dev_s *)g_adcdev.ad_priv;
  volatile uint32_t reg_val;
  volatile uint32_t reg_val2;
  volatile uint32_t reg_val3;

  /* Verify that an interrupt has actually occurred */

  reg_val2 = getreg32(LPC17_40_ADC_STAT);  /* Read ADSTAT will clear the interrupt flag */
  if ((reg_val2) & (1 << 16))
    {
      if ((priv->mask & 0x01) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR0);

#ifdef CONFIG_ADC_DIRECT_ACCESS
          /* Store the data value plus the status bits */

          adc0_buffer0[0] = reg_val;
          adc0_int_done = 1;
#else /* CONFIG_ADC_DIRECT_ACCESS */
#ifdef CONFIG_ADC_WORKER_THREAD
          /* Store the data value plus the status bits */

          adc0_buffer0[0] = reg_val;
          adc0_int_done = 1;

#else /* CONFIG_ADC_WORKER_THREAD */
      if ((reg_val) & (1 << 31))
        {
          adc_receive(priv, 0, (reg_val >> 4) & 0xfff);
        }

#endif /* CONFIG_ADC_WORKER_THREAD */
#endif /* CONFIG_ADC_DIRECT_ACCESS */
        }

      if ((priv->mask & 0x02) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR1);

#ifdef CONFIG_ADC_DIRECT_ACCESS
          /* Store the data value plus the status bits */

          adc1_buffer0[0] = reg_val;
          adc0_int_done = 1;

#else /* CONFIG_ADC_DIRECT_ACCESS */
#ifdef CONFIG_ADC_WORKER_THREAD
          /* Store the data value plus the status bits */

          adc1_buffer0[0] = reg_val;
          adc0_int_done = 1;

#else /* CONFIG_ADC_WORKER_THREAD */
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 1, (reg_val >> 4) & 0xfff);
            }

#endif /* CONFIG_ADC_WORKER_THREAD */
#endif /* CONFIG_ADC_DIRECT_ACCESS */
        }

      if ((priv->mask & 0x04) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR2);

#ifdef CONFIG_ADC_DIRECT_ACCESS
          /* Store the data value plus the status bits */

          adc2_buffer0[0] = reg_val;
          adc0_int_done = 1;

#else /* CONFIG_ADC_DIRECT_ACCESS */
#ifdef CONFIG_ADC_WORKER_THREAD
          /* Store the data value plus the status bits */

          adc2_buffer0[0] = reg_val;
          adc0_int_done = 1;

#else /* CONFIG_ADC_WORKER_THREAD */
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 2, (reg_val >> 4) & 0xfff);
            }

#endif /* CONFIG_ADC_WORKER_THREAD */
#endif /* CONFIG_ADC_DIRECT_ACCESS */
        }

      if ((priv->mask & 0x08) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR3);
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 3, (reg_val >> 4) & 0xfff);
            }
        }

      if ((priv->mask & 0x10) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR4);
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 4, (reg_val >> 4) & 0xfff);
            }
        }

      if ((priv->mask & 0x20) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR5);
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 5, (reg_val >> 4) & 0xfff);
            }
        }

      if ((priv->mask & 0x40) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR6);
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 6, (reg_val >> 4) & 0xfff);
            }
        }

      if ((priv->mask & 0x80) != 0)
        {
          reg_val = getreg32(LPC17_40_ADC_DR7);
          if ((reg_val) & (1 << 31))
            {
              adc_receive(priv, 7, (reg_val >> 4) & 0xfff);
            }
        }

#ifdef CONFIG_ADC_WORKER_THREAD
      if (adc0_int_done == 1)
        {
          work_queue(HPWORK, &priv->irqwork, adc_irqworker,
                     (void *)priv, 0);
        }

#endif /* CONFIG_ADC_WORKER_THREAD */
    }

  reg_val3 = getreg32(LPC17_40_ADC_GDR);        /* Read ADGDR clear the DONE and OVERRUN bits */
  putreg32((priv->mask) |                       /* Select channels 0 to 7 on ADC0 */
           (32 << 8) |                          /* CLKDIV = 16 */
           (0 << 16) |                          /* BURST = 1, BURST capture all selected channels */
           (1 << 17) |                          /* Reserved bit = 0 */
           (1 << 21) |                          /* PDN = 1, normal operation */
           (1 << 26) | (0 << 25) | (0 << 24) |  /* START = at MAT0 signal */
           (1 << 27),                           /* EDGE = 1 (CAP/MAT signal rising trigger A/D
                                                 * conversion) */
           LPC17_40_ADC_CR);

  /* lpc17_40_gpiowrite(LPCXPRESSO_GPIO0_21, 0);  Reset pin P0.21 */

  /* leave_critical_section(saved_state); */

  return OK;
#endif /* CONFIG_LPC17_40_ADC_BURSTMODE */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *lpc17_40_adcinitialize(void)
{
  return &g_adcdev;
}

#endif /* CONFIG_LPC17_40_ADC */
