/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_adc.c
 *
 *   Copyright(C) 2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the LPC17 version:
 *
 *   Copyright(C) 2011 Li Zhuoyi. All rights reserved.
 *   Copyright(C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *           Gregory Nutt
 *
 * This file is a part of NuttX:
 *
 *   Copyright(C) 2010-2012, 2015 Gregory Nutt. All rights reserved.
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>

#include "arm_internal.h"
#include "chip.h"

#include "lpc43_adc.h"
#include "lpc43_cgu.h"
#include "lpc43_scu.h"
#include "lpc43_ccu.h"
#include "lpc43_creg.h"

#include "hardware/lpc43_gima.h"
#include "hardware/lpc43_timer.h"

#include "lpc43_pinconfig.h"

/* board.h should be included last because it depends on the previous
 * inclusions and may need to modify other definitions.
 */

#include <arch/board/board.h>

#if defined(CONFIG_LPC43_ADC0) /* TODO ADC1 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LPC43_ADC0_MASK
#  define CONFIG_LPC43_ADC0_MASK 0x01
#endif
#ifndef CONFIG_LPC43_ADC0_FREQ
#  define CONFIG_LPC43_ADC0_FREQ 0
#endif

#define LPC43_ADC_MAX_FREQUENCY  4500000
#define LPC43_ADC_MIN_FREQUENCY  (BOARD_ABP3_FREQUENCY/256)

#if defined(CONFIG_LPC43_ADC0_USE_TIMER) && CONFIG_LPC43_ADC0_FREQ == 0
#  error "Set CONFIG_LPC43_ADC0_FREQ != 0 if CONFIG_LPC43_ADC0_USE_TIMER"
#endif

#ifndef CONFIG_LPC43_ADC0_USE_TIMER
#  if (CONFIG_LPC43_ADC0_FREQ != 0 &&(CONFIG_LPC43_ADC0_FREQ > LPC43_ADC_MAX_FREQUENCY || \
       CONFIG_LPC43_ADC0_FREQ < LPC43_ADC_MIN_FREQUENCY))
#    error "ADC0 sample rate can't be grater than LPC43_ADC_MAX_FREQUENCY or less than LPC43_ADC_MIN_FREQUENCY"
#  endif
#  define CONFIG_LPC43_ADC0_USE_TIMER 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  const struct adc_callback_s *cb;
  uint8_t  mask;
  uint8_t  mask_int;
  uint32_t freq;
  int      irq;
  bool     timer;
  bool     m_ch;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
  .freq        = CONFIG_LPC43_ADC0_FREQ,
  .mask        = CONFIG_LPC43_ADC0_MASK,
  .mask_int    = CONFIG_LPC43_ADC0_MASK,
  .irq         = LPC43M4_IRQ_ADC0,
  .timer       = CONFIG_LPC43_ADC0_USE_TIMER,
  .m_ch        = (CONFIG_LPC43_ADC0_MASK & (CONFIG_LPC43_ADC0_MASK - 1)) ?
                 true : false
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
  uint32_t regval;

  if (priv->m_ch) /* calc MSB */
    {
      priv->mask_int |= (priv->mask_int >> 1);
      priv->mask_int |= (priv->mask_int >> 2);
      priv->mask_int |= (priv->mask_int >> 4);
      priv->mask_int &= ~(priv->mask_int >> 1);
    }

  flags = enter_critical_section();

  /* Clock peripheral */

  regval  = getreg32(LPC43_CCU1_APB3_ADC0_CFG);
  regval |= CCU_CLK_CFG_RUN;
  putreg32(regval, LPC43_CCU1_APB3_ADC0_CFG);

  /* Calc config value */

  regval  = ADC_CR_PDN;
  regval |= priv->mask;

  if (priv->freq != 0)
    {
      if (priv->timer)
        {
          /* Start adc on timer */

          regval |= ADC_CR_START_CTOUT8;

          /* enable timer out in creg */

          uint32_t regval_timer = getreg32(LPC43_CREG6);
          regval_timer &= ~CREG6_CTOUTCTRL;
          putreg32(regval_timer, LPC43_CREG6);

          /* Enable synch timer 2 match 0 to adc */

          putreg32(GIMA_EDGE | GIMA_SYNCH | GIMA_ADC1_SELECT_T2MAT0,
                   LPC43_GIMA_ADCSTART1);

          /* Power on */

          regval_timer  = getreg32(LPC43_CCU1_M4_TIMER2_CFG);
          regval_timer |= CCU_CLK_CFG_RUN;
          putreg32(regval_timer, LPC43_CCU1_M4_TIMER2_CFG);

          putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_TCR_OFFSET); /* disable */
          putreg32(TMR_MCR_MR0R,
                   LPC43_TIMER2_BASE + LPC43_TMR_MCR_OFFSET);    /* reset on match only */
          putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_CCR_OFFSET); /* do not use capture */
          putreg32(TMR_EMR_EMC0_SET,
                   LPC43_TIMER2_BASE + LPC43_TMR_EMR_OFFSET);     /* external match */
          putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_CTCR_OFFSET); /* counter/timer mode */

          putreg32(LPC43_CCLK / priv->freq / 2 - 1,
                   LPC43_TIMER2_BASE + LPC43_TMR_PR_OFFSET); /* set clock, divide by 2 - bug in chip */

          putreg32(1, LPC43_TMR2_MR0); /* set match on 1 */
        }
      else
        {
          uint32_t clkdiv = BOARD_ABP3_FREQUENCY / priv->freq +
                           (BOARD_ABP3_FREQUENCY % priv->freq != 0) - 1;
          regval |= clkdiv << ADC_CR_CLKDIV_SHIFT;
        }
    }

  putreg32(regval, LPC43_ADC0_CR);

  /* Do pin configuration if defined */

#ifdef PINCONF_ADC0_C0
  if ((priv->mask & 0x01) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C0);
    }
#endif /* PINCONF_ADC0_C0 */

#ifdef PINCONF_ADC0_C1
  if ((priv->mask & 0x02) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C1);
    }
#endif /* PINCONF_ADC0_C1 */

#ifdef PINCONF_ADC0_C2
  if ((priv->mask & 0x04) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C2);
    }
#endif /* PINCONF_ADC0_C2 */

#ifdef PINCONF_ADC0_C3
  if ((priv->mask & 0x08) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C3);
    }
#endif /* PINCONF_ADC0_C3 */

#ifdef PINCONF_ADC0_C4
  if ((priv->mask & 0x10) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C4);
    }
#endif /* PINCONF_ADC0_C4 */

#ifdef PINCONF_ADC0_C5
  if ((priv->mask & 0x20) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C5);
    }
#endif /* PINCONF_ADC0_C5 */

#ifdef PINCONF_ADC0_C6
  if ((priv->mask & 0x40) != 0)
    {
      lpc43_pin_config(PINCONF_ADC0_C6);
    }
#endif /* PINCONF_ADC0_C6 */

#ifdef PINCONF_ADC0_C7
  if ((priv->mask & 0x80) != 0)
    {
      lpc43_configgpio(PINCONF_ADC0_C7);
    }
#endif /* PINCONF_ADC0_C7 */

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

  int ret = irq_attach(priv->irq, adc_interrupt, NULL);
  if (ret == OK)
    {
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

  if (priv->timer)
    {
      putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_TCR_OFFSET); /* disable the timer */
    }

  /* Disable ADC interrupts, both at the level of the ADC device and at the
   * level of the NVIC.
   */

  putreg32(0, LPC43_ADC0_INTEN);
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
      putreg32(priv->mask_int, LPC43_ADC0_INTEN);

      if (priv->timer)
        {
          putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_PC_OFFSET);           /* reset prescale counter */
          putreg32(0, LPC43_TIMER2_BASE + LPC43_TMR_TC_OFFSET);           /* reset timer counter */
          putreg32(TMR_TCR_EN, LPC43_TIMER2_BASE + LPC43_TMR_TCR_OFFSET); /* enable the timer */
        }
      else
        {
          uint32_t regval = getreg32(LPC43_ADC0_CR);

          if (priv->freq == 0 && !priv->m_ch)
            {
              regval |= ADC_CR_START_NOW;
            }
          else
            {
              regval |= ADC_CR_BURST;
            }

          putreg32(regval, LPC43_ADC0_CR);
        }
    }
  else
    {
      putreg32(0, LPC43_ADC0_INTEN);
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
  struct up_dev_s *priv = (struct up_dev_s *)g_adcdev.ad_priv;
  uint32_t regval;
  int i;

  if (priv->timer)
    {
      putreg32(TMR_EMR_EMC0_SET,
               LPC43_TIMER2_BASE + LPC43_TMR_EMR_OFFSET); /* put match to low */
    }
  else
    {
      if (priv->freq == 0 && !priv->m_ch) /* clear burst mode */
        {
          regval = getreg32(LPC43_ADC0_CR);
          regval &= ~ADC_CR_BURST;
          putreg32(regval, LPC43_ADC0_CR);
        }
    }

  /* Read data, clear interrupt by this */

  for (i = 0; i < 8; i++)
    {
      if (priv->mask & (1 << i))
        {
          int32_t data;

          regval = getreg32(LPC43_ADC0_DR(i));
          data   = (regval & ADC_DR_VVREF_MASK) >> ADC_DR_VVREF_SHIFT;

          /* Verify that the upper-half driver has bound its callback
           * functions
           */

          if (priv->cb != NULL)
            {
              /* Perform the data received callback */

              DEBUGASSERT(priv->cb->au_receive != NULL);
              priv->cb->au_receive(&g_adcdev, i, data);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_adcinitialize
 *
 * Description:
 *   Initialize the adc
 *
 * Returned Value:
 *   Valid can device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *lpc43_adcinitialize(void)
{
  return &g_adcdev;
}

#endif /* CONFIG_LPC43_ADC0 */
