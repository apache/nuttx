/****************************************************************************
 * arch/arm/src/common/stm32/stm32_pulsecount_m3m4_v1v2v3.c
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

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_pulsecount.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the pulsecount upper half driver.
 *
 * It implements support for both:
 *   1. STM32 TIMER IP version 1 - F0, F1, F2, F37x, F4, L0, L1
 *   2. STM32 TIMER IP version 2 - F3 (no F37x), F7, H7, L4, L4+
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer Definitions ********************************************************/

/* Pulsecount is supported by advanced timers only. */

#define TIMTYPE_ADVANCED  5
#define TIMTYPE_TIM1      TIMTYPE_ADVANCED
#define TIMTYPE_TIM8      TIMTYPE_ADVANCED

/* Advanced timer clock source, RCC EN offset, enable bit,
 * RCC RST offset, reset bit to use
 */

#  define TIMCLK_TIM1      STM32_APB2_TIM1_CLKIN
#  define TIMRCCEN_TIM1    STM32_RCC_APB2ENR
#  define TIMEN_TIM1       RCC_APB2ENR_TIM1EN
#  define TIMRCCRST_TIM1   STM32_RCC_APB2RSTR
#  define TIMRST_TIM1      RCC_APB2RSTR_TIM1RST
#  define TIMCLK_TIM8      STM32_APB2_TIM8_CLKIN
#  define TIMRCCEN_TIM8    STM32_RCC_APB2ENR
#  define TIMEN_TIM8       RCC_APB2ENR_TIM8EN
#  define TIMRCCRST_TIM8   STM32_RCC_APB2RSTR
#  define TIMRST_TIM8      RCC_APB2RSTR_TIM8RST

/* Default GPIO pins state */

#if defined(CONFIG_STM32_STM32F10XX)
#  define PINCFG_DEFAULT (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT)
#elif defined(CONFIG_STM32_STM32F20XX) ||         \
    defined(CONFIG_STM32_STM32F30XX) ||           \
    defined(CONFIG_STM32_STM32F33XX) ||           \
    defined(CONFIG_STM32_STM32F37XX) ||           \
    defined(CONFIG_STM32_STM32F4XXX) ||           \
    defined(CONFIG_STM32_STM32L15XX) ||           \
    defined(CONFIG_STM32_STM32G4XXX)
#  define PINCFG_DEFAULT (GPIO_INPUT | GPIO_FLOAT)
#else
#  error "Unrecognized STM32 chip"
#endif

#define PULSECOUNT_POL_NEG      1
#define PULSECOUNT_IDLE_ACTIVE  1

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define pulsecount_dumpgpio(p,m) stm32_dumpgpio(p,m)
#else
#  define pulsecount_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Pulsecount output configuration */

struct stm32_out_s
{
  uint8_t  in_use:1;
  uint8_t  pol:1;
  uint8_t  idle:1;
  uint8_t  _res:5;
  uint32_t pincfg;
};

/* Pulsecount channel configuration */

struct stm32_chan_s
{
  uint8_t            channel;
  struct stm32_out_s out1;
};

/* This structure represents the state of one pulsecount timer */

struct stm32_tim_s
{
  struct stm32_chan_s  channel;
  uint8_t              timid:5;
  uint8_t              timtype:3;
  uint8_t              t_dts:3;
  uint8_t              _res:5;
  uint8_t              irq;
  uint8_t              prev;
  uint8_t              curr;
  uint32_t             count;
  uint32_t             frequency;
  uint32_t             base;
  uint32_t             pclk;
  void                *handle;
};

struct stm32_pulsecount_s
{
  const struct pulsecount_ops_s *ops;
  struct stm32_tim_s *timer;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pulsecount_getreg(struct stm32_tim_s *priv, int offset);
static void pulsecount_putreg(struct stm32_tim_s *priv, int offset,
                              uint32_t value);
static void pulsecount_modifyreg(struct stm32_tim_s *priv, uint32_t offset,
                                 uint32_t clearbits, uint32_t setbits);

#ifdef CONFIG_DEBUG_TIMER_INFO
static void pulsecount_dumpregs(struct pulsecount_lowerhalf_s *dev,
                                const char *msg);
#else
#  define pulsecount_dumpregs(priv,msg)
#endif

/* Timer management */

static int pulsecount_ccr_update(struct pulsecount_lowerhalf_s *dev,
                                 uint8_t index, uint32_t ccr);
static int pulsecount_duty_update(struct pulsecount_lowerhalf_s *dev,
                                  uint8_t channel, ub16_t duty);
static int pulsecount_frequency_update(struct pulsecount_lowerhalf_s *dev,
                                       uint32_t frequency);
static int pulsecount_timer_configure(struct stm32_tim_s *priv);
static int pulsecount_channel_configure(struct pulsecount_lowerhalf_s *dev,
                                        uint8_t channel);
static int pulsecount_output_configure(struct stm32_tim_s *priv,
                                       struct stm32_chan_s *chan);
static int pulsecount_outputs_enable(struct pulsecount_lowerhalf_s *dev,
                                     uint16_t outputs, bool state);
static void pulsecount_moe_enable(struct pulsecount_lowerhalf_s *dev,
                                  bool enable);
static int pulsecount_configure(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_timer(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info);
static int pulsecount_interrupt(struct pulsecount_lowerhalf_s *dev);
#  ifdef CONFIG_STM32_TIM1_PULSECOUNT
static int pulsecount_tim1interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_STM32_TIM8_PULSECOUNT
static int pulsecount_tim8interrupt(int irq, void *context, void *arg);
#  endif
static uint8_t pulsecount_count(uint32_t count);

/* Pulsecount driver methods */

static int pulsecount_ll_setup(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_ll_shutdown(struct pulsecount_lowerhalf_s *dev);

static int pulsecount_ll_stop(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_ll_ioctl(struct pulsecount_lowerhalf_s *dev,
                               int cmd, unsigned long arg);

static int pulsecount_setup(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info,
                            void *handle);
static int pulsecount_stop(struct pulsecount_lowerhalf_s *dev);
static int pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                            int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_TIM1_PULSECOUNT

static struct stm32_tim_s g_pulsecount1dev =
{
  .channel =
  {
    .channel = CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL,
#if CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 1
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM1_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM1_CH1OUT,
    },
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 2
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM1_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM1_CH2OUT,
    },
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 3
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM1_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM1_CH3OUT,
    },
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 4
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM1_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM1_CH4OUT,
    },
#endif
  },
  .timid       = 1,
  .timtype     = TIMTYPE_TIM1,
  .t_dts       = CONFIG_STM32_TIM1_PULSECOUNT_TDTS,
  .irq         = STM32_IRQ_TIM1UP,
  .base        = STM32_TIM1_BASE,
  .pclk        = TIMCLK_TIM1,
};

#endif /* CONFIG_STM32_TIM1_PULSECOUNT */

#ifdef CONFIG_STM32_TIM8_PULSECOUNT

static struct stm32_tim_s g_pulsecount8dev =
{
  .channel =
  {
    .channel = CONFIG_STM32_TIM8_PULSECOUNT_CHANNEL,
#if CONFIG_STM32_TIM8_PULSECOUNT_CHANNEL == 1
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM8_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM8_CH1OUT,
    },
#elif CONFIG_STM32_TIM8_PULSECOUNT_CHANNEL == 2
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM8_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM8_CH2OUT,
    },
#elif CONFIG_STM32_TIM8_PULSECOUNT_CHANNEL == 3
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM8_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM8_CH3OUT,
    },
#elif CONFIG_STM32_TIM8_PULSECOUNT_CHANNEL == 4
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_PULSECOUNT_POL,
      .idle    = CONFIG_STM32_TIM8_PULSECOUNT_IDLE,
      .pincfg  = GPIO_TIM8_CH4OUT,
    },
#endif
  },
  .timid       = 8,
  .timtype     = TIMTYPE_TIM8,
  .t_dts       = CONFIG_STM32_TIM8_PULSECOUNT_TDTS,
  .irq         = STM32_IRQ_TIM8UP,
  .base        = STM32_TIM8_BASE,
  .pclk        = TIMCLK_TIM8,
};

#endif /* CONFIG_STM32_TIM8_PULSECOUNT */

static const struct pulsecount_ops_s g_pulsecountops =
{
  .setup       = pulsecount_setup,
  .shutdown    = pulsecount_shutdown,
  .start       = pulsecount_start,
  .stop        = pulsecount_stop,
  .ioctl       = pulsecount_ioctl,
};

#ifdef CONFIG_STM32_TIM1_PULSECOUNT
static struct stm32_pulsecount_s g_pulsecount1lower =
{
  .ops = &g_pulsecountops,
  .timer = &g_pulsecount1dev,
};
#endif

#ifdef CONFIG_STM32_TIM8_PULSECOUNT
static struct stm32_pulsecount_s g_pulsecount8lower =
{
  .ops = &g_pulsecountops,
  .timer = &g_pulsecount8dev,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pulsecount_reg_is_32bit
 ****************************************************************************/

static bool pulsecount_reg_is_32bit(uint8_t timtype, uint32_t offset)
{
  bool ret = false;

  if (timtype == TIMTYPE_ADVANCED)
    {
      if (offset == STM32_ATIM_CR2_OFFSET ||
          offset == STM32_ATIM_CCMR1_OFFSET ||
          offset == STM32_ATIM_CCMR2_OFFSET ||
          offset == STM32_ATIM_CCER_OFFSET ||
          offset == STM32_ATIM_BDTR_OFFSET)
        {
          ret = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pulsecount_getreg
 *
 * Description:
 *   Read the value of an pulsecount timer register
 *
 * Input Parameters:
 *   priv   - A reference to the pulsecount block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pulsecount_getreg(struct stm32_tim_s *priv, int offset)
{
  uint32_t retval = 0;

  if (pulsecount_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      retval = getreg32(priv->base + offset);
    }
  else
    {
      /* 16-bit register */

      retval = getreg16(priv->base + offset);
    }

  /* Return 32-bit value */

  return retval;
}

/****************************************************************************
 * Name: pulsecount_putreg
 *
 * Description:
 *   Read the value of an pulsecount timer register
 *
 * Input Parameters:
 *   priv   - A reference to the pulsecount block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pulsecount_putreg(struct stm32_tim_s *priv, int offset,
                              uint32_t value)
{
  if (pulsecount_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      putreg32(value, priv->base + offset);
    }
  else
    {
      /* 16-bit register */

      putreg16((uint16_t)value, priv->base + offset);
    }
}

/****************************************************************************
 * Name: pulsecount_modifyreg
 *
 * Description:
 *   Modify timer register (32-bit or 16-bit)
 *
 * Input Parameters:
 *   priv    - A reference to the pulsecount block status
 *   offset  - The offset to the register to read
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pulsecount_modifyreg(struct stm32_tim_s *priv, uint32_t offset,
                                 uint32_t clearbits, uint32_t setbits)
{
  if (pulsecount_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      modifyreg32(priv->base + offset, clearbits, setbits);
    }
  else
    {
      /* 16-bit register */

      modifyreg16(priv->base + offset, (uint16_t)clearbits,
                  (uint16_t)setbits);
    }
}

/****************************************************************************
 * Name: pulsecount_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
static void pulsecount_dumpregs(struct pulsecount_lowerhalf_s *dev,
                                const char *msg)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;

  _info("%s:\n", msg);
  _info("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          pulsecount_getreg(priv, STM32_GTIM_CR1_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CR2_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_SMCR_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_DIER_OFFSET));

  _info("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
          pulsecount_getreg(priv, STM32_GTIM_SR_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_EGR_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CCMR2_OFFSET));

  _info(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          pulsecount_getreg(priv, STM32_GTIM_CCER_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CNT_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_PSC_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_ARR_OFFSET));

  if (priv->timid == 1 || priv->timid == 8)
    {
      _info("  RCR: %04x BDTR: %04x\n",
          pulsecount_getreg(priv, STM32_ATIM_RCR_OFFSET),
          pulsecount_getreg(priv, STM32_ATIM_BDTR_OFFSET));
    }

  _info(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          pulsecount_getreg(priv, STM32_GTIM_CCR1_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CCR2_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CCR3_OFFSET),
          pulsecount_getreg(priv, STM32_GTIM_CCR4_OFFSET));

  _info("  DCR: %04x DMAR: %04x\n",
      pulsecount_getreg(priv, STM32_GTIM_DCR_OFFSET),
      pulsecount_getreg(priv, STM32_GTIM_DMAR_OFFSET));
}
#endif

/****************************************************************************
 * Name: pulsecount_ccr_update
 ****************************************************************************/

static int pulsecount_ccr_update(struct pulsecount_lowerhalf_s *dev,
                                 uint8_t index, uint32_t ccr)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t offset = 0;

  /* CCR channel indices are one-based to match timer channel numbers. */

  switch (index)
    {
      case 1:
        {
          offset = STM32_GTIM_CCR1_OFFSET;
          break;
        }

      case 2:
        {
          offset = STM32_GTIM_CCR2_OFFSET;
          break;
        }

      case 3:
        {
          offset = STM32_GTIM_CCR3_OFFSET;
          break;
        }

      case 4:
        {
          offset = STM32_GTIM_CCR4_OFFSET;
          break;
        }

      default:
        {
          _err("ERROR: No such CCR: %u\n", index);
          return -EINVAL;
        }
    }

  /* Update CCR register */

  pulsecount_putreg(priv, offset, ccr);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_duty_update
 *
 * Description:
 *   Try to change only channel duty
 *
 * Input Parameters:
 *   dev     - A reference to the lower half driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_duty_update(struct pulsecount_lowerhalf_s *dev,
                                  uint8_t channel, ub16_t duty)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t reload = 0;
  uint32_t ccr    = 0;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL);

  _info("TIM%u channel: %u duty: %08" PRIx32 "\n",
          priv->timid, channel, duty);

  /* Get the reload values */

  reload = pulsecount_getreg(priv, STM32_GTIM_ARR_OFFSET);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  _info("ccr: %" PRIu32 "\n", ccr);

  /* Write corresponding CCR register */

  return pulsecount_ccr_update(dev, channel, ccr);
}

/****************************************************************************
 * Name: pulsecount_frequency_update
 *
 * Description:
 *   Update a pulsecount timer frequency
 *
 ****************************************************************************/

static int pulsecount_frequency_update(struct pulsecount_lowerhalf_s *dev,
                                       uint32_t frequency)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t reload    = 0;
  uint32_t timclk    = 0;
  uint32_t prescaler = 0;

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register. If 'frequency' is the desired frequency, then
   *
   *   reload = timclk / frequency
   *   timclk = pclk / presc
   *
   * Or,
   *
   *   reload = pclk / presc / frequency
   *
   * There are many solutions to this, but the best solution will be the one
   * that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= presc  <= 65536
   *   1 <= reload <= 65535
   *
   * So presc = pclk / 65535 / frequency would be optimal.
   *
   * Example:
   *
   *  pclk      = 42 MHz
   *  frequency = 100 Hz
   *
   *  prescaler = 42,000,000 / 65,535 / 100
   *            = 6.4 (or 7 -- taking the ceiling always)
   *  timclk    = 42,000,000 / 7
   *            = 6,000,000
   *  reload    = 6,000,000 / 100
   *            = 60,000
   */

  prescaler = (priv->pclk / frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = priv->pclk / prescaler;

  reload = timclk / frequency;
  if (reload < 2)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }
  else
    {
      reload--;
    }

  _info("TIM%u PCLK: %" PRIu32" frequency: %" PRIu32
          " TIMCLK: %" PRIu32 " "
          "prescaler: %" PRIu32 " reload: %" PRIu32 "\n",
          priv->timid, priv->pclk, frequency, timclk, prescaler, reload);

  /* Set the reload and prescaler values */

  pulsecount_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);
  pulsecount_putreg(priv, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  return OK;
}

/****************************************************************************
 * Name: pulsecount_timer_configure
 *
 * Description:
 *   Initial configuration for pulsecount timer
 *
 ****************************************************************************/

static int pulsecount_timer_configure(struct stm32_tim_s *priv)
{
  uint16_t cr1 = 0;

  /* Set up the advanced timer CR1 register. */

  cr1 = pulsecount_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Pulsecount always uses edge-aligned up-counting mode. */

  cr1 &= ~(GTIM_CR1_DIR | GTIM_CR1_CMS_MASK);
  cr1 |= GTIM_CR1_EDGE;
  cr1 &= ~GTIM_CR1_CKD_MASK;
  cr1 |= priv->t_dts << GTIM_CR1_CKD_SHIFT;

  /* Enable ARR preload to preserve the previous pulsecount behavior. */

  cr1 |= GTIM_CR1_ARPE;

  /* Write CR1 */

  pulsecount_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_channel_configure
 *
 * Description:
 *   Configure pulsecount output compare for a channel
 *
 ****************************************************************************/

static int pulsecount_channel_configure(struct pulsecount_lowerhalf_s *dev,
                                        uint8_t channel)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t chanmode = 0;
  uint32_t ocmode   = 0;
  uint32_t ccmr     = 0;
  uint32_t offset   = 0;
  int      ret      = OK;

  /* Configure output compare mode */

  chanmode = GTIM_CCMR_MODE_PWM1;

  /* Get CCMR offset */

  switch (channel)
    {
      case 1:
      case 2:
        {
          offset = STM32_GTIM_CCMR1_OFFSET;
          break;
        }

      case 3:
      case 4:
        {
          offset = STM32_GTIM_CCMR2_OFFSET;
          break;
        }

      default:
        {
          _err("ERROR: No such channel: %u\n", channel);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Get current registers */

  ccmr = pulsecount_getreg(priv, offset);

  /* output compare configuration.
   * NOTE: The CCMRx registers are identical if the channels are outputs.
   */

  switch (channel)
    {
      /* Configure channel 1/3 */

      case 1:
      case 3:
        {
          ccmr &= ~(ATIM_CCMR1_CC1S_MASK | ATIM_CCMR1_OC1M_MASK |
                     ATIM_CCMR1_OC1PE);
          ocmode |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT);
          ocmode |= (chanmode << ATIM_CCMR1_OC1M_SHIFT);
          ocmode |= ATIM_CCMR1_OC1PE;
#ifdef HAVE_IP_TIMERS_V2
          ccmr &= ~(ATIM_CCMR1_OC1M);
#endif
          break;
        }

      /* Configure channel 2/4 */

      case 2:
      case 4:
        {
          ccmr &= ~(ATIM_CCMR1_CC2S_MASK | ATIM_CCMR1_OC2M_MASK |
                     ATIM_CCMR1_OC2PE);
          ocmode |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC2S_SHIFT);
          ocmode |= (chanmode << ATIM_CCMR1_OC2M_SHIFT);
          ocmode |= ATIM_CCMR1_OC2PE;
#ifdef HAVE_IP_TIMERS_V2
          ccmr &= ~(ATIM_CCMR1_OC2M);
#endif
          break;
        }
    }

  /* Set the selected output compare configuration */

  ccmr |= ocmode;

  /* Write CCMRx registers */

  pulsecount_putreg(priv, offset, ccmr);

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_output_configure
 *
 * Description:
 *   Configure pulsecount output for given channel
 *
 ****************************************************************************/

static int pulsecount_output_configure(struct stm32_tim_s *priv,
                                       struct stm32_chan_s *chan)
{
  uint32_t cr2  = 0;
  uint32_t ccer = 0;
  uint8_t  channel = 0;

  /* Get channel */

  channel = chan->channel;

  /* Get current registers state */

  cr2  = pulsecount_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccer = pulsecount_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* | OISx | IDLE | advanced timers | CR2 register
   * | CCxP | POL  | all pulsecount timers | CCER register
   */

  /* Configure output polarity (all pulsecount timers) */

  if (chan->out1.pol == PULSECOUNT_POL_NEG)
    {
      ccer |= (GTIM_CCER_CC1P << ((channel - 1) * 4));
    }
  else
    {
      ccer &= ~(GTIM_CCER_CC1P << ((channel - 1) * 4));
    }

  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      /* Configure output IDLE State */

      if (chan->out1.idle == PULSECOUNT_IDLE_ACTIVE)
        {
          cr2 |= (ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }
      else
        {
          cr2 &= ~(ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }
    }

  /* Write registers */

  pulsecount_modifyreg(priv, STM32_GTIM_CR2_OFFSET, 0, cr2);
  pulsecount_modifyreg(priv, STM32_GTIM_CCER_OFFSET, 0, ccer);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_outputs_enable
 *
 * Description:
 *   Enable/disable given timer pulsecount outputs.
 *
 *   NOTE: This is bulk operation - we can enable/disable many outputs
 *   at one time
 *
 * Input Parameters:
 *   dev     - A reference to the lower half driver state structure
 *   outputs - outputs to set (look at enum stm32_pulsecount_chan_e)
 *   state   - Enable/disable operation
 *
 ****************************************************************************/

static int pulsecount_outputs_enable(struct pulsecount_lowerhalf_s *dev,
                                     uint16_t outputs, bool state)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t ccer   = 0;
  uint32_t regval = 0;

  /* Get current register state */

  ccer = pulsecount_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* Get outputs configuration */

  regval |= ((outputs & (1 << 0))  ? GTIM_CCER_CC1E  : 0);
  regval |= ((outputs & (1 << 2))  ? GTIM_CCER_CC2E  : 0);
  regval |= ((outputs & (1 << 4))  ? GTIM_CCER_CC3E  : 0);
  regval |= ((outputs & (1 << 6))  ? GTIM_CCER_CC4E  : 0);

  if (state == true)
    {
      /* Enable outputs - set bits */

      ccer |= regval;
    }
  else
    {
      /* Disable outputs - reset bits */

      ccer &= ~regval;
    }

  /* Write register */

  pulsecount_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_moe_enable
 ****************************************************************************/

static void pulsecount_moe_enable(struct pulsecount_lowerhalf_s *dev,
                                  bool enable)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;

  if (enable)
    {
      pulsecount_modifyreg(priv, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }
  else
    {
      pulsecount_modifyreg(priv, STM32_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE, 0);
    }
}

/****************************************************************************
 * Name: pulsecount_outputs_from_channels
 *
 * Description:
 *   Get enabled outputs configuration from the pulsecount timer state
 *
 ****************************************************************************/

static uint16_t
pulsecount_outputs_from_channels(struct stm32_tim_s *priv, uint8_t selected)
{
  uint16_t outputs = 0;
  uint8_t  channel;

  channel = priv->channel.channel;

  if (channel != 0 && (selected == 0 || channel == selected) &&
      priv->channel.out1.in_use == 1)
    {
      outputs = (1 << ((channel - 1) * 2));
    }

  return outputs;
}

/****************************************************************************
 * Name: pulsecount_configure
 *
 * Description:
 *   Configure pulsecount timer in PULSECOUNT mode
 *
 ****************************************************************************/

static int pulsecount_configure(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint16_t outputs = 0;
  int     ret      = OK;

  /* NOTE: leave timer counter disabled and all outputs disabled! */

  /* Disable the timer until we get it configured */

  pulsecount_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);

  /* Get configured outputs */

  outputs = pulsecount_outputs_from_channels(priv, 0);

  /* Disable configured outputs before the timer is reconfigured. */

  ret = pulsecount_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initial timer configuration */

  ret = pulsecount_timer_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Disable software break (enable outputs) */

  pulsecount_moe_enable(dev, true);

  /* Configure timer channels */

  if (priv->channel.channel != 0)
    {
      pulsecount_channel_configure(dev, priv->channel.channel);
      pulsecount_output_configure(priv, &priv->channel);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * This split keeps pulsecount as the existing single-channel mode.
 *
 ****************************************************************************/

static int pulsecount_timer(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  ub16_t    duty    = 0;
  uint8_t   channel = 0;
  uint16_t  outputs = 0;
  int       ret     = OK;

  /* If we got here then the timer instance supports pulsecount output. */

  DEBUGASSERT(priv != NULL && info != NULL);

  _info("TIM%u channel: %u high: %" PRIu32 " ns low: %" PRIu32
          " ns count: %" PRIu32 "\n",
          priv->timid, priv->channel.channel, info->high_ns,
          info->low_ns, info->count);

  DEBUGASSERT(pulsecount_frequency(info) > 0);

  /* Channel specific setup */

  duty    = pulsecount_duty(info);
  channel = priv->channel.channel;

  /* Disable all interrupts and DMA requests, clear all pending status */

  pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Set timer frequency */

  ret = pulsecount_frequency_update(dev, pulsecount_frequency(info));
  if (ret < 0)
    {
      goto errout;
    }

  /* Update duty cycle */

  ret = pulsecount_duty_update(dev, channel, duty);
  if (ret < 0)
    {
      goto errout;
    }

  /* If a non-zero repetition count has been selected, then set the
   * repetition counter to the count-1 (pulsecount_start() has already
   * assured us that the count value is within range).
   */

  if (info->count > 0)
    {
      /* Save the remaining count and the number of counts that will have
       * elapsed on the first interrupt.
       */

      /* If the first interrupt occurs at the end end of the first
       * repetition count, then the count will be the same as the RCR
       * value.
       */

      priv->prev  = pulsecount_count(info->count);
      pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->prev - 1);

      /* Generate an update event to reload the prescaler.  This should
       * preload the RCR into active repetition counter.
       */

      pulsecount_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);

      /* Now set the value of the RCR that will be loaded on the next
       * update event.
       */

      priv->count = info->count;
      priv->curr  = pulsecount_count(info->count - priv->prev);
      pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->curr - 1);
    }

  /* Otherwise, just clear the repetition counter */

  else
    {
      /* Set the repetition counter to zero */

      pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);

      /* Generate an update event to reload the prescaler */

      pulsecount_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);
    }

  /* Get configured outputs */

  outputs = pulsecount_outputs_from_channels(priv, channel);

  /* Enable output */

  ret = pulsecount_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Setup update interrupt. If info->count is > 0, then we can
   * be assured that pulsecount_start() has already verified: (1) that
   * this is an advanced timer, and that (2) the repetition count is within
   * range.
   */

  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
      pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      pulsecount_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }

  pulsecount_dumpregs(dev, "After starting");

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_interrupt(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pulsecount_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pulsecount_putreg(priv, STM32_ATIM_SR_OFFSET, (regval & ~ATIM_SR_UIF));

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the master output to stop the output as
       * quickly as possible.
       */

      pulsecount_moe_enable(dev, false);

      /* Disable first interrupts, stop and reset the timer */

      pulsecount_ll_stop(dev);

      /* Then perform the callback into the upper half driver */

      pulsecount_expired(priv->handle);

      priv->handle = NULL;
      priv->count  = 0;
      priv->prev   = 0;
      priv->curr   = 0;
    }
  else
    {
      /* Decrement the count of pulses remaining using the number of
       * pulses generated since the last interrupt.
       */

      priv->count -= priv->prev;

      /* Set up the next RCR.  Set 'prev' to the value of the RCR that
       * was loaded when the update occurred (just before this interrupt)
       * and set 'curr' to the current value of the RCR register (which
       * will bet loaded on the next update event).
       */

      priv->prev = priv->curr;
      priv->curr = pulsecount_count(priv->count - priv->prev);
      pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output.
   */

  _info("Update interrupt SR: %04" PRIx16 " prev: %u curr: %u"
          " count: %" PRIu32 "\n",
          regval, (unsigned int)priv->prev, (unsigned int)priv->curr,
          priv->count);

  return OK;
}

/****************************************************************************
 * Name: pulsecount_tim1/8interrupt
 *
 * Description:
 *   Handle timer 1 and 8 interrupts.
 *
 * Input Parameters:
 *   Standard NuttX interrupt inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_TIM1_PULSECOUNT
static int pulsecount_tim1interrupt(int irq, void *context, void *arg)
{
  return pulsecount_interrupt((struct pulsecount_lowerhalf_s *)
                              &g_pulsecount1dev);
}
#endif /* CONFIG_STM32_TIM1_PULSECOUNT */

#ifdef CONFIG_STM32_TIM8_PULSECOUNT
static int pulsecount_tim8interrupt(int irq, void *context, void *arg)
{
  return pulsecount_interrupt((struct pulsecount_lowerhalf_s *)
                              &g_pulsecount8dev);
}
#endif /* CONFIG_STM32_TIM8_PULSECOUNT */

/****************************************************************************
 * Name: pulsecount_count
 *
 * Description:
 *   Pick an optimal pulse count to program the RCR.
 *
 * Input Parameters:
 *   count - The total count remaining
 *
 * Returned Value:
 *   The recommended pulse count
 *
 ****************************************************************************/

static uint8_t pulsecount_count(uint32_t count)
{
  /* Use the advanced-timer repetition counter limit. */

  /* The the remaining pulse count is less than or equal to the maximum, the
   * just return the count.
   */

  if (count <= ATIM_RCR_REP_MAX)
    {
      return (uint8_t)count;
    }

  /* Otherwise, we have to be careful.  We do not want a small number of
   * counts at the end because we might have trouble responding fast enough.
   * If the remaining count is less than 150% of the maximum, then return
   * half of the maximum.  In this case the final sequence will be between 64
   * and 128.
   */

  else if (count < (3 * ATIM_RCR_REP_MAX / 2))
    {
      return (uint8_t)((ATIM_RCR_REP_MAX + 1) >> 1);
    }

  /* Otherwise, return the maximum.  The final count will be 64 or more */

  else
    {
      return (uint8_t)ATIM_RCR_REP_MAX;
    }
}

/****************************************************************************
 * Name: pulsecount_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   priv - A reference to the pulsecount block status
 *   on   - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static int pulsecount_set_apb_clock(struct stm32_tim_s *priv, bool on)
{
  uint32_t en_bit  = 0;
  uint32_t regaddr = 0;
  int      ret     = OK;

  _info("timer %d clock enable: %d\n", priv->timid, on ? 1 : 0);

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM1_PULSECOUNT
      case 1:
        {
          regaddr = TIMRCCEN_TIM1;
          en_bit  = TIMEN_TIM1;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM8_PULSECOUNT
      case 8:
        {
          regaddr = TIMRCCEN_TIM8;
          en_bit  = TIMEN_TIM8;
          break;
        }
#endif

      default:
        {
          _err("ERROR: No such timer configured %d\n", priv->timid);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Enable/disable APB 1/2 clock for timer */

  _info("RCC_APBxENR base: %08" PRIx32 "  bits: %04" PRIx32 "\n",
          regaddr, en_bit);

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_ll_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int pulsecount_ll_setup(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t pincfg = 0;
  int      ret    = OK;

  _info("TIM%u\n", priv->timid);

  /* Enable APB1/2 clocking for timer. */

  ret = pulsecount_set_apb_clock(priv, true);
  if (ret < 0)
    {
      goto errout;
    }

  pulsecount_dumpregs(dev, "Initially");

  /* Configure the pulsecount output pins, but do not start the timer yet */

  if (priv->channel.out1.in_use == 1)
    {
      /* Do not configure the pin if pincfg is not specified.
       * This prevents overwriting the PA0 configuration if the
       * channel is used internally.
       */

      pincfg = priv->channel.out1.pincfg;
      if (pincfg != 0)
        {
          _info("pincfg: %08" PRIx32 "\n", pincfg);

          stm32_configgpio(pincfg);
          pulsecount_dumpgpio(pincfg, "pulsecount setup");
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_ll_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_ll_shutdown(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  uint32_t pincfg = 0;
  int      ret    = OK;

  _info("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  pulsecount_ll_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  ret = pulsecount_set_apb_clock(priv, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Then put the GPIO pins back to the default state */

  pincfg = priv->channel.out1.pincfg;
  if (pincfg != 0)
    {
      _info("pincfg: %08" PRIx32 "\n", pincfg);

      pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pincfg |= PINCFG_DEFAULT;

      stm32_configgpio(pincfg);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pulsecount_ll_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int pulsecount_ll_stop(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;
  irqstate_t flags    = 0;
  uint16_t   outputs  = 0;
  int        ret = OK;

  _info("TIM%u\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Stopped so frequency is zero */

  priv->frequency = 0;

  /* Disable further interrupts and stop the timer */

  pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Disable the timer and timer outputs */

  pulsecount_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
  outputs = pulsecount_outputs_from_channels(priv, 0);
  ret = pulsecount_outputs_enable(dev, outputs, false);

  /* Clear all channels */

  pulsecount_putreg(priv, STM32_GTIM_CCR1_OFFSET, 0);
  pulsecount_putreg(priv, STM32_GTIM_CCR2_OFFSET, 0);
  pulsecount_putreg(priv, STM32_GTIM_CCR3_OFFSET, 0);
  pulsecount_putreg(priv, STM32_GTIM_CCR4_OFFSET, 0);

  leave_critical_section(flags);

  pulsecount_dumpregs(dev, "After stop");

  return ret;
}

/****************************************************************************
 * Name: pulsecount_ll_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pulsecount_ll_ioctl(struct pulsecount_lowerhalf_s *dev, int cmd,
                               unsigned long arg)
{
#ifdef CONFIG_DEBUG_TIMER_INFO
  struct stm32_tim_s *priv = (struct stm32_tim_s *)dev;

  /* There are no platform-specific ioctl commands */

  _info("TIM%u\n", priv->timid);
#endif
  return -ENOTTY;
}

static int pulsecount_setup(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecount_s *pulse = (struct stm32_pulsecount_s *)dev;
  int ret;

  ret = pulsecount_ll_setup((struct pulsecount_lowerhalf_s *)pulse->timer);
  if (ret < 0)
    {
      return ret;
    }

  return pulsecount_configure((struct pulsecount_lowerhalf_s *)pulse->timer);
}

static int pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecount_s *pulse = (struct stm32_pulsecount_s *)dev;
  return pulsecount_ll_shutdown((struct pulsecount_lowerhalf_s *)
                                pulse->timer);
}

static int pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                            const struct pulsecount_info_s *info,
                            void *handle)
{
  struct stm32_pulsecount_s *pulse = (struct stm32_pulsecount_s *)dev;
  struct stm32_tim_s *priv = pulse->timer;

  /* Check if a pulsecount has been selected */

  if (info->count > 0)
    {
      /* Only the advanced timers (TIM1,8 can support the pulse counting)
       */

      if (priv->timtype != TIMTYPE_ADVANCED)
        {
          _err("ERROR: TIM%u cannot support pulse count: %" PRIu32 "\n",
                 priv->timid, info->count);
          return -EPERM;
        }
    }

  /* Save the handle */

  priv->handle = handle;

  /* Start the time */

  return pulsecount_timer((struct pulsecount_lowerhalf_s *)priv, info);
}

static int pulsecount_stop(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecount_s *pulse = (struct stm32_pulsecount_s *)dev;
  return pulsecount_ll_stop((struct pulsecount_lowerhalf_s *)pulse->timer);
}

static int pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                            int cmd, unsigned long arg)
{
  struct stm32_pulsecount_s *pulse = (struct stm32_pulsecount_s *)dev;
  return pulsecount_ll_ioctl((struct pulsecount_lowerhalf_s *)pulse->timer,
                             cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct pulsecount_lowerhalf_s *stm32_pulsecountinitialize(int timer)
{
  struct stm32_pulsecount_s *lower = NULL;

  _info("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_PULSECOUNT
      case 1:
        {
          lower = &g_pulsecount1lower;
          irq_attach(lower->timer->irq, pulsecount_tim1interrupt, NULL);
          up_disable_irq(lower->timer->irq);
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM8_PULSECOUNT
      case 8:
        {
          lower = &g_pulsecount8lower;
          irq_attach(lower->timer->irq, pulsecount_tim8interrupt, NULL);
          up_disable_irq(lower->timer->irq);
          break;
        }
#endif

      default:
        {
          _err("ERROR: TIM%d does not support pulse count\n", timer);
          return NULL;
        }
    }

  return (struct pulsecount_lowerhalf_s *)lower;
}
