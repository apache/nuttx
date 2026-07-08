/****************************************************************************
 * arch/arm/src/common/stm32/stm32_pulsecount_m0_v1.c
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pulsecount.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_pulsecount.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"

/* This module only supports pulse count on advanced timers. */

#ifdef CONFIG_STM32_TIM1_PULSECOUNT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pulse count is supported by advanced timers only. */

#define TIMTYPE_ADVANCED     4  /* Advanced timers:  TIM1 */
#define TIMTYPE_TIM1         TIMTYPE_ADVANCED

#define HAVE_IP_TIMERS_V2    1

/* CCMR2 */

#define HAVE_CCMR2           1

#define PULSECOUNT_TIM1_CLKIN STM32_TIM1_CLKIN

#if CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 1
#  define PULSECOUNT_TIM1_CHCFG GPIO_TIM1_CH1OUT
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 2
#  define PULSECOUNT_TIM1_CHCFG GPIO_TIM1_CH2OUT
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 3
#  define PULSECOUNT_TIM1_CHCFG GPIO_TIM1_CH3OUT
#elif CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL == 4
#  define PULSECOUNT_TIM1_CHCFG GPIO_TIM1_CH4OUT
#else
#  error Unsupported TIM1 pulse count channel
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
#  define pulsecount_dumpgpio(p,m)
#  warning "pulsecount_dumpgpio not implemented"
#else
#  define pulsecount_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_pulsecountchan_s
{
  uint8_t channel;
  uint32_t pincfg;
};

/* This structure represents the state of one pulsecount timer */

struct stm32_pulsecounttimer_s
{
  const struct pulsecount_ops_s *ops;
  struct stm32_pulsecountchan_s channel;
  uint8_t timid;
  uint8_t timtype;
  uint8_t irq;
  uint32_t prev;
  uint32_t curr;
  uint32_t count;
  uint32_t base;
  uint32_t pclk;
  void *handle;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t stm32pulsecount_getreg(struct stm32_pulsecounttimer_s *priv,
                                       int offset);
static void stm32pulsecount_putreg(struct stm32_pulsecounttimer_s *priv,
                                   int offset, uint32_t value);
static void stm32pulsecount_modifyreg(struct stm32_pulsecounttimer_s *priv,
                                      uint32_t offset, uint32_t clearbits,
                                      uint32_t setbits);

#ifdef CONFIG_DEBUG_TIMER_INFO
static void stm32pulsecount_dumpregs(struct stm32_pulsecounttimer_s *priv,
                              const char *msg);
#else
#  define stm32pulsecount_dumpregs(priv,msg)
#endif

/* Timer management */

static int
stm32pulsecount_output_configure(struct stm32_pulsecounttimer_s *priv,
                                 uint8_t channel);
static int stm32pulsecount_timer(struct stm32_pulsecounttimer_s *priv,
                                 const struct pulsecount_info_s *info);
static void stm32pulsecount_setapbclock(
  struct stm32_pulsecounttimer_s *priv, bool on);
static int stm32pulsecount_interrupt(struct stm32_pulsecounttimer_s *priv);
static int stm32pulsecount_tim1interrupt(int irq, void *context, void *arg);
static uint32_t stm32pulsecount_pulsecount(uint32_t count);

/* Pulsecount driver methods */

static int stm32pulsecount_setup(struct pulsecount_lowerhalf_s *dev);
static int stm32pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev);

static int stm32pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                                 const struct pulsecount_info_s *info,
                                 void *handle);

static int stm32pulsecount_stop(struct pulsecount_lowerhalf_s *dev);
static int stm32pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half pulsecount driver methods used by the upper
 * half driver.
 */

static const struct pulsecount_ops_s g_pulsecountops =
{
  .setup       = stm32pulsecount_setup,
  .shutdown    = stm32pulsecount_shutdown,
  .start       = stm32pulsecount_start,
  .stop        = stm32pulsecount_stop,
  .ioctl       = stm32pulsecount_ioctl,
};

#ifdef CONFIG_STM32_TIM1_PULSECOUNT
static struct stm32_pulsecounttimer_s g_pulsecount1dev =
{
  .ops         = &g_pulsecountops,
  .timid       = 1,
  .channel     =
  {
    .channel   = CONFIG_STM32_TIM1_PULSECOUNT_CHANNEL,
    .pincfg    = PULSECOUNT_TIM1_CHCFG,
  },
  .timtype     = TIMTYPE_TIM1,
  .irq         = STM32_IRQ_TIM1_BRK,
  .base        = STM32_TIM1_BASE,
  .pclk        = PULSECOUNT_TIM1_CLKIN,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32pulsecount_reg_is_32bit
 *
 * Description:
 *   Verify whether the timer register is 32bit or not.
 *
 * Input Parameters:
 *   timtype - The type of the timer. See the TIMTYPE_* definitions
 *   offset  - The offset to the register to read
 *
 * Returned Value:
 *   Return true for 32 bits register; false otherwise.
 *
 ****************************************************************************/

static bool stm32pulsecount_reg_is_32bit(uint8_t timtype, uint32_t offset)
{
  if (timtype == TIMTYPE_ADVANCED)
    {
      if (offset == STM32_ATIM_CR2_OFFSET   ||
          offset == STM32_ATIM_CCMR1_OFFSET ||
          offset == STM32_ATIM_CCMR2_OFFSET ||
          offset == STM32_ATIM_CCER_OFFSET  ||
          offset == STM32_ATIM_BDTR_OFFSET  ||
          offset == STM32_ATIM_DMAR_OFFSET  ||
          offset == STM32_ATIM_AF1_OFFSET   ||
          offset == STM32_ATIM_TISEL_OFFSET)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: stm32pulsecount_getreg
 *
 * Description:
 *   Read the value of an pulsecount timer register
 *
 * Input Parameters:
 *   priv   - A reference to the pulsecount timer status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t stm32pulsecount_getreg(
  struct stm32_pulsecounttimer_s *priv, int offset)
{
  uint32_t retval;

  if (stm32pulsecount_reg_is_32bit(priv->timtype, offset) == true)
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
 * Name: stm32pulsecount_putreg
 *
 * Description:
 *   Read the value of an pulsecount timer register
 *
 * Input Parameters:
 *   priv   - A reference to the pulsecount timer status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32pulsecount_putreg(struct stm32_pulsecounttimer_s *priv,
                                   int offset, uint32_t value)
{
  if (stm32pulsecount_reg_is_32bit(priv->timtype, offset) == true)
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
 * Name: stm32pulsecount_modifyreg
 *
 * Description:
 *   Modify timer register (32-bit or 16-bit)
 *
 * Input Parameters:
 *   priv    - A reference to the pulsecount timer status
 *   offset  - The offset to the register to read
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32pulsecount_modifyreg(struct stm32_pulsecounttimer_s *priv,
                               uint32_t offset, uint32_t clearbits,
                               uint32_t setbits)
{
  if (stm32pulsecount_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      modifyreg32(priv->base + offset, clearbits, setbits);
    }
  else
    {
      /* 16-bit register */

      modifyreg16(priv->base + offset, clearbits, setbits);
    }
}

/****************************************************************************
 * Name: stm32pulsecount_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the pulsecount timer status
 *   msg  - A message to be printed on the screen
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_TIMER_INFO
static void stm32pulsecount_dumpregs(struct stm32_pulsecounttimer_s *priv,
                              const char *msg)
{
  _info("%s:\n", msg);
  _info("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          stm32pulsecount_getreg(priv, STM32_GTIM_CR1_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CR2_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_SMCR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_DIER_OFFSET));
  _info("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
          stm32pulsecount_getreg(priv, STM32_GTIM_SR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_EGR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  _info(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          stm32pulsecount_getreg(priv, STM32_GTIM_CCER_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CNT_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_PSC_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_ARR_OFFSET));
  _info(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          stm32pulsecount_getreg(priv, STM32_GTIM_CCR1_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CCR2_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CCR3_OFFSET),
          stm32pulsecount_getreg(priv, STM32_GTIM_CCR4_OFFSET));
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      _info("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
          stm32pulsecount_getreg(priv, STM32_ATIM_RCR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_ATIM_BDTR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_ATIM_DCR_OFFSET),
          stm32pulsecount_getreg(priv, STM32_ATIM_DMAR_OFFSET));

          _info("  AF1: %04x TISEL: %04x\n",
          stm32pulsecount_getreg(priv, STM32_ATIM_AF1_OFFSET),
          stm32pulsecount_getreg(priv, STM32_ATIM_TISEL_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: stm32pulsecount_output_configure
 *
 * Description:
 *   Configure pulsecount output for given channel
 *
 * Input Parameters:
 *   priv - A reference to the pulsecount timer status
 *   channel - Timer output channel
 *
 * Returned Value:
 *   Zero on success;
 ****************************************************************************/

static int
stm32pulsecount_output_configure(struct stm32_pulsecounttimer_s *priv,
                                 uint8_t channel)
{
  uint32_t cr2;
  uint32_t ccer;

  /* Get current registers state */

  cr2  = stm32pulsecount_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccer = stm32pulsecount_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* Reset the output polarity level of all channels (selects high
   * polarity)
   */

  ccer &= ~(GTIM_CCER_CC1P << ((channel - 1) * 4));

  /* Enable the output state of the selected channels */

  ccer |= (GTIM_CCER_CC1E << ((channel - 1) * 4));

  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      cr2 &= ~(ATIM_CR2_OIS1 << ((channel - 1) * 2));
    }

  stm32pulsecount_modifyreg(priv, STM32_GTIM_CR2_OFFSET, 0, cr2);
  stm32pulsecount_modifyreg(priv, STM32_GTIM_CCER_OFFSET, 0, ccer);

  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stm32pulsecount_timer(struct stm32_pulsecounttimer_s *priv,
                          const struct pulsecount_info_s *info)
{
  /* Calculated values */

  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint32_t ccr;
  ub16_t duty;
  uint32_t chanmode = GTIM_CCMR_MODE_PWM1;
  uint8_t channel;

  /* Register contents */

  uint32_t cr1;
  uint32_t ccmr1;
#if defined(HAVE_CCMR2)
  uint32_t ccmr2;
  uint32_t ocmode2;
#endif

  /* New timer register bit settings */

  uint32_t ocmode1;

  DEBUGASSERT(priv != NULL && info != NULL);

  ccmr1 = stm32pulsecount_getreg(priv, STM32_GTIM_CCMR1_OFFSET);

#if defined(HAVE_CCMR2)
  ccmr2 = stm32pulsecount_getreg(priv, STM32_GTIM_CCMR2_OFFSET);
#endif

  _info("TIM%u channel: %u high: %" PRIu32 " ns low: %" PRIu32
          " ns count: %" PRIu32 "\n",
          priv->timid, priv->channel.channel, info->high_ns,
          info->low_ns, info->count);

  DEBUGASSERT(pulsecount_frequency(info) > 0);

  /* Disable all interrupts and DMA requests, clear all pending status */

  stm32pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stm32pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register.  If 'frequency' is the desired frequency, then
   *
   *   reload = timclk / frequency
   *   timclk = pclk / presc
   *
   * Or,
   *
   *   reload = pclk / presc / frequency
   *
   * There are many solutions to this, but the best solution will be the
   * one that has the largest reload value and the smallest prescaler value.
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

  prescaler = (priv->pclk / pulsecount_frequency(info) + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = priv->pclk / prescaler;

  reload = timclk / pulsecount_frequency(info);

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

  _info("TIM%u PCLK: %" PRIu32 " frequency: %" PRIu32 " "
          "TIMCLK: %" PRIu32 " prescaler: %" PRIu32
          " reload: %" PRIu32 "\n",
          priv->timid, priv->pclk, pulsecount_frequency(info), timclk,
          prescaler, reload);

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = stm32pulsecount_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Disable the timer until we get it configured */

  cr1 &= ~GTIM_CR1_CEN;

  cr1 &= ~(GTIM_CR1_DIR | GTIM_CR1_CMS_MASK);

  cr1 |= GTIM_CR1_EDGE;

  /* Set the clock division to zero for all (but the basic timers, but there
   * should be no basic timers in this context
   */

  cr1 &= ~GTIM_CR1_CKD_MASK;
  stm32pulsecount_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Set the reload and prescaler values */

  stm32pulsecount_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);
  stm32pulsecount_putreg(priv, STM32_GTIM_PSC_OFFSET, (prescaler - 1));

  /* Set the advanced timer's repetition counter */

  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repetition counter to the count-1.  stm32pulsecount_start() has
       * already assured us that the count value is within range.
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

          priv->prev  = stm32pulsecount_pulsecount(info->count);
          stm32pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET,
                                 priv->prev - 1);

          /* Generate an update event to reload the prescaler.  This should
           * preload the RCR into active repetition counter.
           */

          stm32pulsecount_putreg(priv, STM32_ATIM_EGR_OFFSET, ATIM_EGR_UG);

          /* Now set the value of the RCR that will be loaded on the next
           * update event.
           */

          priv->count = info->count;
          priv->curr  = stm32pulsecount_pulsecount(info->count
                                            - priv->prev);
          stm32pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET,
                                 priv->curr - 1);
        }

      /* Otherwise, just clear the repetition counter */

      else
        {
          /* Set the repetition counter to zero */

          stm32pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);

          /* Generate an update event to reload the prescaler */

          stm32pulsecount_putreg(priv, STM32_ATIM_EGR_OFFSET, ATIM_EGR_UG);
        }
    }

  /* Handle channel specific setup */

  ocmode1   = 0;
#if defined(HAVE_CCMR2)
  ocmode2   = 0;
#endif

  duty = pulsecount_duty(info);
  channel = priv->channel.channel;

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  _info("ccr: %" PRIu32 "\n", ccr);

  switch (channel)
    {
      case 1:
        ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC1S_SHIFT) |
                    (chanmode << GTIM_CCMR1_OC1M_SHIFT) |
                    GTIM_CCMR1_OC1PE;
        stm32pulsecount_putreg(priv, STM32_GTIM_CCR1_OFFSET, ccr);
        ccmr1 &= ~(GTIM_CCMR1_CC1S_MASK | GTIM_CCMR1_OC1M_MASK |
                   GTIM_CCMR1_OC1PE | GTIM_CCMR1_OC1M);
        break;

      case 2:
        ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC2S_SHIFT) |
                    (chanmode << GTIM_CCMR1_OC2M_SHIFT) |
                    GTIM_CCMR1_OC2PE;
        stm32pulsecount_putreg(priv, STM32_GTIM_CCR2_OFFSET, ccr);
        ccmr1 &= ~(GTIM_CCMR1_CC2S_MASK | GTIM_CCMR1_OC2M_MASK |
                   GTIM_CCMR1_OC2PE | GTIM_CCMR1_OC2M);
        break;

      case 3:
        ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC3S_SHIFT) |
                    (chanmode << ATIM_CCMR2_OC3M_SHIFT) |
                    ATIM_CCMR2_OC3PE;
        stm32pulsecount_putreg(priv, STM32_ATIM_CCR3_OFFSET, ccr);
        ccmr2 &= ~(ATIM_CCMR2_CC3S_MASK | ATIM_CCMR2_OC3M_MASK |
                   ATIM_CCMR2_OC3PE | ATIM_CCMR2_OC3M);
        break;

      case 4:
        ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC4S_SHIFT) |
                    (chanmode << ATIM_CCMR2_OC4M_SHIFT) |
                    ATIM_CCMR2_OC4PE;
        stm32pulsecount_putreg(priv, STM32_ATIM_CCR4_OFFSET, ccr);
        ccmr2 &= ~(ATIM_CCMR2_CC4S_MASK | ATIM_CCMR2_OC4M_MASK |
                   ATIM_CCMR2_OC4PE | ATIM_CCMR2_OC4M);
        break;

      default:
        _err("ERROR: No such channel: %u\n", channel);
        return -EINVAL;
    }

  stm32pulsecount_output_configure(priv, channel);

  ccmr1 |= ocmode1;
#if defined(HAVE_CCMR2)
  ccmr2 |= ocmode2;
#endif

  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      uint32_t bdtr;

      /* Get current register state */

      bdtr  = stm32pulsecount_getreg(priv, STM32_ATIM_BDTR_OFFSET);

      bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);
      bdtr |= ATIM_BDTR_MOE;

      stm32pulsecount_putreg(priv, STM32_ATIM_BDTR_OFFSET, bdtr);
    }

  /* Save the modified register values */

  putreg32(ccmr1, priv->base + STM32_GTIM_CCMR1_OFFSET);
#if defined(HAVE_CCMR2)
  putreg32(ccmr2, priv->base + STM32_ATIM_CCMR2_OFFSET);
#endif

  /* Set the ARR Preload Bit */

  cr1 = stm32pulsecount_getreg(priv, STM32_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_ARPE;
  stm32pulsecount_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Setup update interrupt. If info->count is > 0, then we can
   * be assured that stm32pulsecount_start() has already verified: (1) that
   * this is an advanced timer, and that (2) the repetition count is within
   * range.
   */

  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      stm32pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
      stm32pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      cr1 |= GTIM_CR1_CEN;
      stm32pulsecount_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }
  else
    {
      /* Just enable the timer, leaving all interrupts disabled */

      cr1 |= GTIM_CR1_CEN;
      stm32pulsecount_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);
    }

  stm32pulsecount_dumpregs(priv, "After starting");
  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the lower half pulsecount driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 ****************************************************************************/

static int stm32pulsecount_interrupt(struct stm32_pulsecounttimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = stm32pulsecount_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  stm32pulsecount_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = stm32pulsecount_getreg(priv, STM32_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      stm32pulsecount_putreg(priv, STM32_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrupts, stop and reset the timer */

      stm32pulsecount_stop((struct pulsecount_lowerhalf_s *)priv);

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
      priv->curr = stm32pulsecount_pulsecount(priv->count - priv->prev);
      stm32pulsecount_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output
   */

  _info("Update interrupt SR: %04x prev: %" PRIu32 " curr: %" PRIu32
          " count: %" PRIu32 "\n",
          regval, priv->prev, priv->curr, priv->count);

  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_tim1interrupt
 *
 * Description:
 *   Handle timer 1 interrupts.
 *
 * Input Parameters:
 *   Standard NuttX interrupt inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stm32pulsecount_tim1interrupt(int irq, void *context, void *arg)
{
  return stm32pulsecount_interrupt(&g_pulsecount1dev);
}

/****************************************************************************
 * Name: stm32pulsecount_pulsecount
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

static uint32_t stm32pulsecount_pulsecount(uint32_t count)
{
  /* The the remaining pulse count is less than or equal to the maximum, the
   * just return the count.
   */

  if (count <= ATIM_RCR_REP_MAX)
    {
      return count;
    }

  /* Otherwise, we have to be careful.  We do not want a small number of
   * counts at the end because we might have trouble responding fast enough.
   * If the remaining count is less than 150% of the maximum, then return
   * half of the maximum.  In this case the final sequence will be between 64
   * and 128.
   */

  else if (count < (3 * ATIM_RCR_REP_MAX / 2))
    {
      return (ATIM_RCR_REP_MAX + 1) >> 1;
    }

  /* Otherwise, return the maximum.  The final count will be 64 or more */

  else
    {
      return ATIM_RCR_REP_MAX;
    }
}

/****************************************************************************
 * Name: stm32pulsecount_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32pulsecount_setapbclock(
  struct stm32_pulsecounttimer_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM1_PULSECOUNT
      case 1:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM1EN;
        break;
#endif
      default:
        return;
    }

  /* Enable/disable APB 1/2 clock for timer */

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }
}

/****************************************************************************
 * Name: stm32pulsecount_setup
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

static int stm32pulsecount_setup(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecounttimer_s *priv =
    (struct stm32_pulsecounttimer_s *)dev;
  uint32_t pincfg;

  _info("TIM%u\n", priv->timid);
  stm32pulsecount_dumpregs(priv, "Initially");

  /* Enable APB1/2 clocking for timer. */

  stm32pulsecount_setapbclock(priv, true);

  /* Configure the pulsecount output pins, but do not start the timer yet */

  pincfg = priv->channel.pincfg;
  if (pincfg != 0)
    {
      _info("pincfg: %08" PRIx32 "\n", pincfg);

      stm32_configgpio(pincfg);
    }

  pulsecount_dumpgpio(pincfg, "pulsecount setup");

  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_shutdown
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

static int stm32pulsecount_shutdown(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecounttimer_s *priv =
    (struct stm32_pulsecounttimer_s *)dev;
  uint32_t pincfg;

  _info("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  stm32pulsecount_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  stm32pulsecount_setapbclock(priv, false);

  /* Then put the GPIO pins back to the default state */

  pincfg = priv->channel.pincfg;
  if (pincfg != 0)
    {
      _info("pincfg: %08" PRIx32 "\n", pincfg);

      pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pincfg |= GPIO_INPUT | GPIO_FLOAT;

      stm32_configgpio(pincfg);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half pulsecount driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stm32pulsecount_start(struct pulsecount_lowerhalf_s *dev,
                                 const struct pulsecount_info_s *info,
                                 void *handle)
{
  struct stm32_pulsecounttimer_s *priv =
    (struct stm32_pulsecounttimer_s *)dev;

  /* Check if a pulsecount has been selected */

  if (info->count > 0)
    {
      /* Only the advanced timers (TIM1,8 can support the pulse counting) */

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

  return stm32pulsecount_timer(priv, info);
}

/****************************************************************************
 * Name: stm32pulsecount_stop
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

static int stm32pulsecount_stop(struct pulsecount_lowerhalf_s *dev)
{
  struct stm32_pulsecounttimer_s *priv =
    (struct stm32_pulsecounttimer_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  _info("TIM%u\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Disable further interrupts and stop the timer */

  stm32pulsecount_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stm32pulsecount_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Determine which timer to reset */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM1_PULSECOUNT
      case 1:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM1RST;
        break;
#endif

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where stm32pulsecount_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  _info("regaddr: %08" PRIx32 " resetbit: %08" PRIx32 "\n",
          regaddr, resetbit);
  stm32pulsecount_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: stm32pulsecount_ioctl
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

static int stm32pulsecount_ioctl(struct pulsecount_lowerhalf_s *dev, int cmd,
                                 unsigned long arg)
{
#ifdef CONFIG_DEBUG_TIMER_INFO
  struct stm32_pulsecounttimer_s *priv =
    (struct stm32_pulsecounttimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  _info("TIM%u\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pulsecountinitialize
 *
 * Description:
 *   Initialize one timer for use with the upper-level pulsecount driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family.  This pulsecount driver
 *     supports TIM1 only on STM32F0/L0/G0.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half pulsecount driver is
 *   returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct pulsecount_lowerhalf_s *stm32_pulsecountinitialize(int timer)
{
  struct stm32_pulsecounttimer_s *lower;

  _info("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_PULSECOUNT
      case 1:
        lower = &g_pulsecount1dev;

        /* Attach but disable the TIM1 update interrupt */

        irq_attach(lower->irq, stm32pulsecount_tim1interrupt, NULL);
        up_disable_irq(lower->irq);
        break;
#endif

      default:
        _err("ERROR: No such timer configured\n");
        return NULL;
    }

  return (struct pulsecount_lowerhalf_s *)lower;
}

#endif /* CONFIG_STM32_TIMx_PULSECOUNT */
