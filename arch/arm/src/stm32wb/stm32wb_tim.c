/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_tim.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb.h"
#include "stm32wb_tim.h"
#include "stm32wb_gpio.h"
#include "stm32wb_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.
 * Such special purposes include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32WB_TIMn is defined then the CONFIG_STM32WB_TIMn_PWM may also
 *   be defined to indicate that the timer is intended to be used for pulsed
 *   output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32WB_TIMn is
 *   defined then CONFIG_STM32WB_TIMn_ADC may also be defined to indicate
 *   that timer "n" is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32WB_TIMn is defined then
 *   CONFIG_STM32WB_TIMn_QE may also be defined to indicate that timer "n"
 *   is intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32WB_TIM1_PWM) || defined (CONFIG_STM32WB_TIM1_ADC) || \
    defined(CONFIG_STM32WB_TIM1_QE)
#  undef CONFIG_STM32WB_TIM1
#endif

#if defined(CONFIG_STM32WB_TIM2_PWM) || defined (CONFIG_STM32WB_TIM2_ADC) || \
    defined(CONFIG_STM32WB_TIM2_QE)
#  undef CONFIG_STM32WB_TIM2
#endif

#if defined(CONFIG_STM32WB_TIM16_PWM) || defined (CONFIG_STM32WB_TIM16_ADC) || \
    defined(CONFIG_STM32WB_TIM16_QE)
#  undef CONFIG_STM32WB_TIM16
#endif

#if defined(CONFIG_STM32WB_TIM17_PWM) || defined (CONFIG_STM32WB_TIM17_ADC) || \
    defined(CONFIG_STM32WB_TIM17_QE)
#  undef CONFIG_STM32WB_TIM17
#endif

#if defined(CONFIG_STM32WB_TIM1)
#  if defined(GPIO_TIM1_CH1OUT) || defined(GPIO_TIM1_CH2OUT) || \
      defined(GPIO_TIM1_CH3OUT) || defined(GPIO_TIM1_CH4OUT)
#    define HAVE_TIM1_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32WB_TIM2)
#  if defined(GPIO_TIM2_CH1OUT) || defined(GPIO_TIM2_CH2OUT) || \
      defined(GPIO_TIM2_CH3OUT) || defined(GPIO_TIM2_CH4OUT)
#    define HAVE_TIM2_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32WB_TIM16)
#  if defined(GPIO_TIM16_CH1OUT) || defined(GPIO_TIM16_CH2OUT) || \
      defined(GPIO_TIM16_CH3OUT) || defined(GPIO_TIM16_CH4OUT)
#    define HAVE_TIM16_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32WB_TIM17)
#  if defined(GPIO_TIM17_CH1OUT) || defined(GPIO_TIM17_CH2OUT) || \
      defined(GPIO_TIM17_CH3OUT) || defined(GPIO_TIM17_CH4OUT)
#    define HAVE_TIM17_GPIOCONFIG 1
#endif
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32WB_TIM1) || defined(CONFIG_STM32WB_TIM2) || \
    defined(CONFIG_STM32WB_TIM16) || defined(CONFIG_STM32WB_TIM17)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct stm32wb_tim_priv_s
{
  const struct stm32wb_tim_ops_s *ops;
  enum stm32wb_tim_mode_e mode;
  uint32_t base;                      /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Register helpers */

static inline uint16_t stm32wb_getreg16(struct stm32wb_tim_dev_s *dev,
                                        uint8_t offset);
static inline void stm32wb_putreg16(struct stm32wb_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value);
static inline void stm32wb_modifyreg16(struct stm32wb_tim_dev_s *dev,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits);
static inline uint32_t stm32wb_getreg32(struct stm32wb_tim_dev_s *dev,
                                        uint8_t offset);
static inline void stm32wb_putreg32(struct stm32wb_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value);

/* Timer helpers */

static void stm32wb_tim_reload_counter(struct stm32wb_tim_dev_s *dev);
static void stm32wb_tim_enable(struct stm32wb_tim_dev_s *dev);
static void stm32wb_tim_disable(struct stm32wb_tim_dev_s *dev);
static void stm32wb_tim_reset(struct stm32wb_tim_dev_s *dev);
#if defined(HAVE_TIM1_GPIOCONFIG) || defined(HAVE_TIM2_GPIOCONFIG) || \
    defined(HAVE_TIM16_GPIOCONFIG) || defined(HAVE_TIM17_GPIOCONFIG)
static void stm32wb_tim_gpioconfig(uint32_t cfg,
                                   enum stm32wb_tim_channel_e mode);
#endif
static void stm32wb_tim_dumpregs(struct stm32wb_tim_dev_s *dev);

/* Timer methods */

static int stm32wb_tim_setmode(struct stm32wb_tim_dev_s *dev,
                               enum stm32wb_tim_mode_e mode);
static int stm32wb_tim_setfreq(struct stm32wb_tim_dev_s *dev, uint32_t freq);
static int stm32wb_tim_setclock(struct stm32wb_tim_dev_s *dev,
                                uint32_t freq);
static uint32_t  stm32wb_tim_getclock(struct stm32wb_tim_dev_s *dev);
static void stm32wb_tim_setperiod(struct stm32wb_tim_dev_s *dev,
                                  uint32_t period);
static uint32_t stm32wb_tim_getperiod(struct stm32wb_tim_dev_s *dev);
static uint32_t stm32wb_tim_getcounter(struct stm32wb_tim_dev_s *dev);
static uint32_t stm32wb_tim_getwidth(struct stm32wb_tim_dev_s *dev);
static int stm32wb_tim_setchannel(struct stm32wb_tim_dev_s *dev,
                                  uint8_t channel,
                                  enum stm32wb_tim_channel_e mode);
static int stm32wb_tim_setcompare(struct stm32wb_tim_dev_s *dev,
                                  uint8_t channel, uint32_t compare);
static uint32_t stm32wb_tim_getcapture(struct stm32wb_tim_dev_s *dev,
                                       uint8_t channel);
static int stm32wb_tim_setisr(struct stm32wb_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source);
static void stm32wb_tim_enableint(struct stm32wb_tim_dev_s *dev, int source);
static void stm32wb_tim_disableint(struct stm32wb_tim_dev_s *dev,
                                   int source);
static void stm32wb_tim_ackint(struct stm32wb_tim_dev_s *dev, int source);
static int stm32wb_tim_checkint(struct stm32wb_tim_dev_s *dev, int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32wb_tim_ops_s stm32wb_tim_ops =
{
  .enable     = stm32wb_tim_enable,
  .disable    = stm32wb_tim_disable,
  .setmode    = stm32wb_tim_setmode,
  .setfreq    = stm32wb_tim_setfreq,
  .setclock   = stm32wb_tim_setclock,
  .getclock   = stm32wb_tim_getclock,
  .setperiod  = stm32wb_tim_setperiod,
  .getperiod  = stm32wb_tim_getperiod,
  .getcounter = stm32wb_tim_getcounter,
  .getwidth   = stm32wb_tim_getwidth,
  .setchannel = stm32wb_tim_setchannel,
  .setcompare = stm32wb_tim_setcompare,
  .getcapture = stm32wb_tim_getcapture,
  .setisr     = stm32wb_tim_setisr,
  .enableint  = stm32wb_tim_enableint,
  .disableint = stm32wb_tim_disableint,
  .ackint     = stm32wb_tim_ackint,
  .checkint   = stm32wb_tim_checkint,
  .dump_regs  = stm32wb_tim_dumpregs,
};

#ifdef CONFIG_STM32WB_TIM1
struct stm32wb_tim_priv_s stm32wb_tim1_priv =
{
  .ops        = &stm32wb_tim_ops,
  .mode       = STM32WB_TIM_MODE_UNUSED,
  .base       = STM32WB_TIM1_BASE,
};
#endif
#ifdef CONFIG_STM32WB_TIM2
struct stm32wb_tim_priv_s stm32wb_tim2_priv =
{
  .ops        = &stm32wb_tim_ops,
  .mode       = STM32WB_TIM_MODE_UNUSED,
  .base       = STM32WB_TIM2_BASE,
};
#endif

#ifdef CONFIG_STM32WB_TIM16
struct stm32wb_tim_priv_s stm32wb_tim16_priv =
{
  .ops        = &stm32wb_tim_ops,
  .mode       = STM32WB_TIM_MODE_UNUSED,
  .base       = STM32WB_TIM16_BASE,
};
#endif

#ifdef CONFIG_STM32WB_TIM17
struct stm32wb_tim_priv_s stm32wb_tim17_priv =
{
  .ops        = &stm32wb_tim_ops,
  .mode       = STM32WB_TIM_MODE_UNUSED,
  .base       = STM32WB_TIM17_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t stm32wb_getreg16(struct stm32wb_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg16(((struct stm32wb_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32wb_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32wb_putreg16(struct stm32wb_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct stm32wb_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32wb_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32wb_modifyreg16(struct stm32wb_tim_dev_s *dev,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits)
{
  modifyreg16(((struct stm32wb_tim_priv_s *)dev)->base + offset, clearbits,
              setbits);
}

/****************************************************************************
 * Name: stm32wb_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset. This applies only for the STM32WB
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2.
 *
 ****************************************************************************/

static inline uint32_t stm32wb_getreg32(struct stm32wb_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg32(((struct stm32wb_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32wb_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset. This applies only for the STM32WB
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2.
 *
 ****************************************************************************/

static inline void stm32wb_putreg32(struct stm32wb_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct stm32wb_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32wb_tim_reload_counter
 ****************************************************************************/

static void stm32wb_tim_reload_counter(struct stm32wb_tim_dev_s *dev)
{
  uint16_t val = stm32wb_getreg16(dev, STM32WB_TIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  stm32wb_putreg16(dev, STM32WB_TIM_EGR_OFFSET, val);
}

/****************************************************************************
 * Name: stm32wb_tim_enable
 ****************************************************************************/

static void stm32wb_tim_enable(struct stm32wb_tim_dev_s *dev)
{
  uint16_t val = stm32wb_getreg16(dev, STM32WB_TIM_CR1_OFFSET);

  val |= GTIM_CR1_CEN;
  stm32wb_tim_reload_counter(dev);
  stm32wb_putreg16(dev, STM32WB_TIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32wb_tim_disable
 ****************************************************************************/

static void stm32wb_tim_disable(struct stm32wb_tim_dev_s *dev)
{
  uint16_t val = stm32wb_getreg16(dev, STM32WB_TIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  stm32wb_putreg16(dev, STM32WB_TIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32wb_tim_reset
 *
 * Description:
 *   Reset timer into system default state, but do not affect output/input
 *   pins
 *
 ****************************************************************************/

static void stm32wb_tim_reset(struct stm32wb_tim_dev_s *dev)
{
  ((struct stm32wb_tim_priv_s *)dev)->mode = STM32WB_TIM_MODE_DISABLED;
  stm32wb_tim_disable(dev);
}

/****************************************************************************
 * Name: stm32wb_tim_gpioconfig
 ****************************************************************************/

#if defined(HAVE_TIM1_GPIOCONFIG) || defined(HAVE_TIM2_GPIOCONFIG) || \
    defined(HAVE_TIM16_GPIOCONFIG) || defined(HAVE_TIM17_GPIOCONFIG)
static void stm32wb_tim_gpioconfig(uint32_t cfg,
                                   enum stm32wb_tim_channel_e mode)
{
  if (mode & STM32WB_TIM_CH_MODE_MASK)
    {
      stm32wb_configgpio(cfg);
    }
  else
    {
      stm32wb_unconfiggpio(cfg);
    }
}
#endif

/****************************************************************************
 * Name: stm32wb_tim_dumpregs
 ****************************************************************************/

static void stm32wb_tim_dumpregs(struct stm32wb_tim_dev_s *dev)
{
  struct stm32wb_tim_priv_s *priv = (struct stm32wb_tim_priv_s *)dev;

  ainfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          stm32wb_getreg16(dev, STM32WB_TIM_CR1_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CR2_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_SMCR_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_DIER_OFFSET)
        );
  ainfo("   SR: %04x EGR:  0000 CCMR1: %04x CCMR2: %04x\n",
          stm32wb_getreg16(dev, STM32WB_TIM_SR_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CCMR1_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CCMR2_OFFSET)
        );
  ainfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          stm32wb_getreg16(dev, STM32WB_TIM_CCER_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CNT_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_PSC_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_ARR_OFFSET)
        );
  ainfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          stm32wb_getreg16(dev, STM32WB_TIM_CCR1_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CCR2_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CCR3_OFFSET),
          stm32wb_getreg16(dev, STM32WB_TIM_CCR4_OFFSET)
        );

  if (priv->base == STM32WB_TIM1_BASE)
    {
      ainfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
            stm32wb_getreg16(dev, STM32WB_TIM_RCR_OFFSET),
            stm32wb_getreg16(dev, STM32WB_TIM_BDTR_OFFSET),
            stm32wb_getreg16(dev, STM32WB_TIM_DCR_OFFSET),
            stm32wb_getreg16(dev, STM32WB_TIM_DMAR_OFFSET));
    }
  else
    {
      ainfo("  DCR: %04x DMAR: %04x\n",
            stm32wb_getreg16(dev, STM32WB_TIM_DCR_OFFSET),
            stm32wb_getreg16(dev, STM32WB_TIM_DMAR_OFFSET));
    }
}

/****************************************************************************
 * Name: stm32wb_tim_setmode
 ****************************************************************************/

static int stm32wb_tim_setmode(struct stm32wb_tim_dev_s *dev,
                               enum stm32wb_tim_mode_e mode)
{
  uint16_t val;

  DEBUGASSERT(dev != NULL);

  /* The modes DOWN and UPDOWN are not supported on TIM16 and TIM17. */

#if defined(CONFIG_STM32WB_TIM16) || defined(CONFIG_STM32WB_TIM17)
  if ((mode == STM32WB_TIM_MODE_DOWN || mode == STM32WB_TIM_MODE_UPDOWN))
    {
#if defined(CONFIG_STM32WB_TIM16)
      if (((struct stm32wb_tim_priv_s *)dev)->base == STM32WB_TIM16_BASE)
        {
          return -EINVAL;
        }
#endif

#if defined(CONFIG_STM32WB_TIM17)
      if (((struct stm32wb_tim_priv_s *)dev)->base == STM32WB_TIM17_BASE)
        {
          return -EINVAL;
        }
#endif
    }
#endif

  /* Decode operational modes */

  switch (mode & STM32WB_TIM_MODE_MASK)
    {
      case STM32WB_TIM_MODE_DISABLED:
        val = 0;
        break;

      case STM32WB_TIM_MODE_UP:
        val = GTIM_CR1_CEN | GTIM_CR1_ARPE;
        break;

#if defined(CONFIG_STM32WB_TIM1) || defined(CONFIG_STM32WB_TIM2)
      case STM32WB_TIM_MODE_DOWN:
        val = GTIM_CR1_CEN | GTIM_CR1_ARPE | TIM_1_2_CR1_DIR;
        break;

      case STM32WB_TIM_MODE_UPDOWN:
        val = GTIM_CR1_CEN | GTIM_CR1_ARPE | TIM_1_2_CR1_CMS_CNTR1;

        /* Our default:
         * Interrupts are generated on compare, when counting down
         */

        break;
#endif

      case STM32WB_TIM_MODE_PULSE:
        val = GTIM_CR1_CEN | GTIM_CR1_ARPE | GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  stm32wb_tim_reload_counter(dev);
  stm32wb_putreg16(dev, STM32WB_TIM_CR1_OFFSET, val);

#ifdef CONFIG_STM32WB_TIM1
  /* Advanced registers require Main Output Enable */

  if (((struct stm32wb_tim_priv_s *)dev)->base == STM32WB_TIM1_BASE)
    {
      stm32wb_modifyreg16(dev, STM32WB_TIM_BDTR_OFFSET, 0, TIM1_BDTR_MOE);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32wb_tim_setfreq
 ****************************************************************************/

static int stm32wb_tim_setfreq(struct stm32wb_tim_dev_s *dev, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;
  uint32_t reload;
  uint32_t timclk;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32wb_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        freqin = BOARD_TIM16_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        freqin = BOARD_TIM17_FREQUENCY;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   *
   * Calculate optimal values for the timer prescaler and for the timer
   * reload register.  If freq is the desired frequency, then
   *
   *   reload = timclk / freq
   *   reload = (pclck / prescaler) / freq
   *
   * There are many solutions to do this, but the best solution will be the
   * one that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= prescaler  <= 65536
   *   1 <= reload <= 65535
   *
   * So ( prescaler = pclck / 65535 / freq ) would be optimal.
   */

  prescaler = (freqin / freq + 65534) / 65535;

  /* We need to decrement value for '1', but only if that will not to
   * cause underflow.
   */

  if (prescaler < 1)
    {
      awarn("WARNING: Prescaler underflowed.\n");
      prescaler = 1;
    }

  /* Check for overflow as well. */

  if (prescaler > 65536)
    {
      awarn("WARNING: Prescaler overflowed.\n");
      prescaler = 65536;
    }

  timclk = freqin / prescaler;

  reload = timclk / freq;
  if (reload < 1)
    {
      awarn("WARNING: Reload value underflowed.\n");
      reload = 1;
    }
  else if (reload > 65535)
    {
      awarn("WARNING: Reload value overflowed.\n");
      reload = 65535;
    }

  /* Set the reload and prescaler values */

  stm32wb_putreg16(dev, STM32WB_TIM_PSC_OFFSET, prescaler - 1);
  stm32wb_putreg16(dev, STM32WB_TIM_ARR_OFFSET, reload);

  return (timclk / reload);
}

/****************************************************************************
 * Name: stm32wb_tim_setclock
 ****************************************************************************/

static int stm32wb_tim_setclock(struct stm32wb_tim_dev_s *dev, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32wb_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        freqin = BOARD_TIM16_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        freqin = BOARD_TIM17_FREQUENCY;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0xffff)
    {
      prescaler = 0xffff;
    }

  stm32wb_putreg16(dev, STM32WB_TIM_PSC_OFFSET, prescaler);

  return prescaler;
}

/****************************************************************************
 * Name: stm32wb_tim_getclock
 ****************************************************************************/

static uint32_t stm32wb_tim_getclock(struct stm32wb_tim_dev_s *dev)
{
  uint32_t freqin;
  uint32_t clock;
  DEBUGASSERT(dev != NULL);

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        freqin = BOARD_TIM16_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        freqin = BOARD_TIM17_FREQUENCY;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* From chip datasheet, at page 1179. */

  clock = freqin / (stm32wb_getreg16(dev, STM32WB_TIM_PSC_OFFSET) + 1);
  return clock;
}

/****************************************************************************
 * Name: stm32wb_tim_setperiod
 ****************************************************************************/

static void stm32wb_tim_setperiod(struct stm32wb_tim_dev_s *dev,
                                  uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  stm32wb_putreg32(dev, STM32WB_TIM_ARR_OFFSET, period);
}

/****************************************************************************
 * Name: stm32wb_tim_getperiod
 ****************************************************************************/

static uint32_t stm32wb_tim_getperiod (struct stm32wb_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return stm32wb_getreg32 (dev, STM32WB_TIM_ARR_OFFSET);
}

/****************************************************************************
 * Name: stm32wb_tim_getcounter
 ****************************************************************************/

static uint32_t stm32wb_tim_getcounter(struct stm32wb_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  uint32_t counter = stm32wb_getreg32(dev, STM32WB_TIM_CNT_OFFSET);

  /* TIM2 is a 32-bit timer. */

#if defined(CONFIG_STM32WB_TIM2)
  if (((struct stm32wb_tim_priv_s *)dev)->base == STM32WB_TIM2_BASE)
    {
      return counter;
    }
#endif

  return counter & 0x0000ffff;
}

/****************************************************************************
 * Name: stm32wb_tim_getwidth
 ****************************************************************************/

static uint32_t stm32wb_tim_getwidth(struct stm32wb_tim_dev_s *dev)
{
  /* Only TIM2 is a 32-bit timer. */

#if defined(CONFIG_STM32WB_TIM2)
  if (((struct stm32wb_tim_priv_s *)dev)->base == STM32WB_TIM2_BASE)
    {
      return 32;
    }
#endif

  /* All others are 16-bit timers. */

  return 16;
}

/****************************************************************************
 * Name: stm32wb_tim_setchannel
 ****************************************************************************/

static int stm32wb_tim_setchannel(struct stm32wb_tim_dev_s *dev,
                                  uint8_t channel,
                                  enum stm32wb_tim_channel_e mode)
{
  uint16_t ccmr_orig = 0;
  uint16_t ccmr_val = 0;
  uint16_t ccer_val;
  uint8_t ccmr_offset = STM32WB_TIM_CCMR1_OFFSET;

  DEBUGASSERT(dev != NULL);

  /* Further we use range as 0..3; if channel=0 it will also overflow here */

  if (--channel > 3)
    {
      return -EINVAL;
    }

  /* Assume that channel is disabled and polarity is active high */

  ccer_val = stm32wb_getreg16(dev, STM32WB_TIM_CCER_OFFSET);
  ccer_val &= ~(GTIM_CCER_CCXE(channel) | GTIM_CCER_CCXP(channel));

  /* Decode configuration */

  switch (mode & STM32WB_TIM_CH_MODE_MASK)
    {
      case STM32WB_TIM_CH_DISABLED:
        break;

      case STM32WB_TIM_CH_OUTPWM:
        ccmr_val = GTIM_CCMR_OCXM_PWM1(channel) | GTIM_CCMR_OCXPE(channel);
        ccer_val |= GTIM_CCER_CCXE(channel);
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity */

  if (mode & STM32WB_TIM_CH_POLARITY_NEG)
    {
      ccer_val |= GTIM_CCER_CCXP(channel);
    }

  if (channel > 1)
    {
      ccmr_offset = STM32WB_TIM_CCMR2_OFFSET;
    }

  ccmr_orig  = stm32wb_getreg16(dev, ccmr_offset);
  ccmr_orig &= ~(GTIM_CCMR_OCXM_MASK(channel) | GTIM_CCMR_OCXPE(channel));
  ccmr_orig |= ccmr_val;
  stm32wb_putreg16(dev, ccmr_offset, ccmr_orig);
  stm32wb_putreg16(dev, STM32WB_TIM_CCER_OFFSET, ccer_val);

  /* set GPIO */

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM1_CH1OUT)
            case 0:
              stm32wb_tim_gpioconfig(GPIO_TIM1_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH2OUT)
            case 1:
              stm32wb_tim_gpioconfig(GPIO_TIM1_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH3OUT)
            case 2:
              stm32wb_tim_gpioconfig(GPIO_TIM1_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH4OUT)
            case 3:
              stm32wb_tim_gpioconfig(GPIO_TIM1_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM2_CH1OUT)
            case 0:
              stm32wb_tim_gpioconfig(GPIO_TIM2_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH2OUT)
            case 1:
              stm32wb_tim_gpioconfig(GPIO_TIM2_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH3OUT)
            case 2:
              stm32wb_tim_gpioconfig(GPIO_TIM2_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH4OUT)
            case 3:
              stm32wb_tim_gpioconfig(GPIO_TIM2_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM16_CH1OUT)
            case 0:
              stm32wb_tim_gpioconfig(GPIO_TIM16_CH1OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM17_CH1OUT)
            case 0:
              stm32wb_tim_gpioconfig(GPIO_TIM17_CH1OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32wb_tim_setcompare
 ****************************************************************************/

static int stm32wb_tim_setcompare(struct stm32wb_tim_dev_s *dev,
                                  uint8_t channel, uint32_t compare)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        stm32wb_putreg32(dev, STM32WB_TIM_CCR1_OFFSET, compare);
        break;

      case 2:
        stm32wb_putreg32(dev, STM32WB_TIM_CCR2_OFFSET, compare);
        break;

      case 3:
        stm32wb_putreg32(dev, STM32WB_TIM_CCR3_OFFSET, compare);
        break;

      case 4:
        stm32wb_putreg32(dev, STM32WB_TIM_CCR4_OFFSET, compare);
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32wb_tim_getcapture
 ****************************************************************************/

static uint32_t stm32wb_tim_getcapture(struct stm32wb_tim_dev_s *dev,
                                       uint8_t channel)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        return stm32wb_getreg32(dev, STM32WB_TIM_CCR1_OFFSET);

      case 2:
        return stm32wb_getreg32(dev, STM32WB_TIM_CCR2_OFFSET);

      case 3:
        return stm32wb_getreg32(dev, STM32WB_TIM_CCR3_OFFSET);

      case 4:
        return stm32wb_getreg32(dev, STM32WB_TIM_CCR4_OFFSET);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: stm32wb_tim_setisr
 ****************************************************************************/

static int stm32wb_tim_setisr(struct stm32wb_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        vectorno = STM32WB_IRQ_TIM1UP;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        vectorno = STM32WB_IRQ_TIM2;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        vectorno = STM32WB_IRQ_TIM16;
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        vectorno = STM32WB_IRQ_TIM17;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

  return OK;
}

/****************************************************************************
 * Name: stm32wb_tim_enableint
 ****************************************************************************/

static void stm32wb_tim_enableint(struct stm32wb_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32wb_modifyreg16(dev, STM32WB_TIM_DIER_OFFSET, 0, source);
}

/****************************************************************************
 * Name: stm32wb_tim_disableint
 ****************************************************************************/

static void stm32wb_tim_disableint(struct stm32wb_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32wb_modifyreg16(dev, STM32WB_TIM_DIER_OFFSET, source, 0);
}

/****************************************************************************
 * Name: stm32wb_tim_ackint
 ****************************************************************************/

static void stm32wb_tim_ackint(struct stm32wb_tim_dev_s *dev, int source)
{
  stm32wb_putreg16(dev, STM32WB_TIM_SR_OFFSET, ~source);
}

/****************************************************************************
 * Name: stm32wb_tim_checkint
 ****************************************************************************/

static int stm32wb_tim_checkint(struct stm32wb_tim_dev_s *dev, int source)
{
  uint16_t regval = stm32wb_getreg16(dev, STM32WB_TIM_SR_OFFSET);
  return (regval & source) ? 1 : 0;
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32wb_tim_init
 ****************************************************************************/

struct stm32wb_tim_dev_s *stm32wb_tim_init(int timer)
{
  struct stm32wb_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_STM32WB_TIM1
      case 1:
        dev = (struct stm32wb_tim_dev_s *)&stm32wb_tim1_priv;
        modifyreg32(STM32WB_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case 2:
        dev = (struct stm32wb_tim_dev_s *)&stm32wb_tim2_priv;
        modifyreg32(STM32WB_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM2EN);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case 16:
        dev = (struct stm32wb_tim_dev_s *)&stm32wb_tim16_priv;
        modifyreg32(STM32WB_RCC_APB2ENR, 0, RCC_APB2ENR_TIM16EN);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case 17:
        dev = (struct stm32wb_tim_dev_s *)&stm32wb_tim17_priv;
        modifyreg32(STM32WB_RCC_APB2ENR, 0, RCC_APB2ENR_TIM17EN);
        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct stm32wb_tim_priv_s *)dev)->mode != STM32WB_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  stm32wb_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: stm32wb_tim_deinit
 *
 * TODO: Detach interrupts, and close down all TIM Channels
 *
 ****************************************************************************/

int stm32wb_tim_deinit(struct stm32wb_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct stm32wb_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32WB_TIM1
      case STM32WB_TIM1_BASE:
        modifyreg32(STM32WB_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM2
      case STM32WB_TIM2_BASE:
        modifyreg32(STM32WB_RCC_APB1ENR1, RCC_APB1ENR1_TIM2EN, 0);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM16
      case STM32WB_TIM16_BASE:
        modifyreg32(STM32WB_RCC_APB2ENR, RCC_APB2ENR_TIM16EN, 0);
        break;
#endif

#ifdef CONFIG_STM32WB_TIM17
      case STM32WB_TIM17_BASE:
        modifyreg32(STM32WB_RCC_APB2ENR, RCC_APB2ENR_TIM17EN, 0);
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct stm32wb_tim_priv_s *)dev)->mode = STM32WB_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32WB_TIM1 || ... || TIM17) */
