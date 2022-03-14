/****************************************************************************
 * arch/arm/src/stm32u5/stm32_tim.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32U5_TIMn is defined then the CONFIG_STM32U5_TIMn_PWM may also
 *   be defined to indicate that the timer is intended to be used for pulsed
 *   output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32U5_TIMn is
 *   defined then CONFIG_STM32U5_TIMn_ADC may also be defined to indicate
 *   that timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32U5_TIMn is defined then
 *   CONFIG_STM32U5_TIMn_DAC may also be defined to indicate that timer "n"
 *   is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32U5_TIMn is defined then
 *   CONFIG_STM32U5_TIMn_QE may also be defined to indicate that timer "n"
 *   is intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32U5_TIM1_PWM) || defined (CONFIG_STM32U5_TIM1_ADC) || \
    defined(CONFIG_STM32U5_TIM1_DAC) || defined(CONFIG_STM32U5_TIM1_QE)
#  undef CONFIG_STM32U5_TIM1
#endif

#if defined(CONFIG_STM32U5_TIM2_PWM) || defined (CONFIG_STM32U5_TIM2_ADC) || \
    defined(CONFIG_STM32U5_TIM2_DAC) || defined(CONFIG_STM32U5_TIM2_QE)
#  undef CONFIG_STM32U5_TIM2
#endif

#if defined(CONFIG_STM32U5_TIM3_PWM) || defined (CONFIG_STM32U5_TIM3_ADC) || \
    defined(CONFIG_STM32U5_TIM3_DAC) || defined(CONFIG_STM32U5_TIM3_QE)
#  undef CONFIG_STM32U5_TIM3
#endif

#if defined(CONFIG_STM32U5_TIM4_PWM) || defined (CONFIG_STM32U5_TIM4_ADC) || \
    defined(CONFIG_STM32U5_TIM4_DAC) || defined(CONFIG_STM32U5_TIM4_QE)
#  undef CONFIG_STM32U5_TIM4
#endif

#if defined(CONFIG_STM32U5_TIM5_PWM) || defined (CONFIG_STM32U5_TIM5_ADC) || \
    defined(CONFIG_STM32U5_TIM5_DAC) || defined(CONFIG_STM32U5_TIM5_QE)
#  undef CONFIG_STM32U5_TIM5
#endif

#if defined(CONFIG_STM32U5_TIM6_PWM) || defined (CONFIG_STM32U5_TIM6_ADC) || \
    defined(CONFIG_STM32U5_TIM6_DAC) || defined(CONFIG_STM32U5_TIM6_QE)
#  undef CONFIG_STM32U5_TIM6
#endif

#if defined(CONFIG_STM32U5_TIM7_PWM) || defined (CONFIG_STM32U5_TIM7_ADC) || \
    defined(CONFIG_STM32U5_TIM7_DAC) || defined(CONFIG_STM32U5_TIM7_QE)
#  undef CONFIG_STM32U5_TIM7
#endif

#if defined(CONFIG_STM32U5_TIM8_PWM) || defined (CONFIG_STM32U5_TIM8_ADC) || \
    defined(CONFIG_STM32U5_TIM8_DAC) || defined(CONFIG_STM32U5_TIM8_QE)
#  undef CONFIG_STM32U5_TIM8
#endif

#if defined(CONFIG_STM32U5_TIM15_PWM) || defined (CONFIG_STM32U5_TIM15_ADC) || \
    defined(CONFIG_STM32U5_TIM15_DAC) || defined(CONFIG_STM32U5_TIM15_QE)
#  undef CONFIG_STM32U5_TIM15
#endif

#if defined(CONFIG_STM32U5_TIM16_PWM) || defined (CONFIG_STM32U5_TIM16_ADC) || \
    defined(CONFIG_STM32U5_TIM16_DAC) || defined(CONFIG_STM32U5_TIM16_QE)
#  undef CONFIG_STM32U5_TIM16
#endif

#if defined(CONFIG_STM32U5_TIM17_PWM) || defined (CONFIG_STM32U5_TIM17_ADC) || \
    defined(CONFIG_STM32U5_TIM17_DAC) || defined(CONFIG_STM32U5_TIM17_QE)
#  undef CONFIG_STM32U5_TIM17
#endif

#if defined(CONFIG_STM32U5_TIM1)
#  if defined(GPIO_TIM1_CH1OUT) ||defined(GPIO_TIM1_CH2OUT)||\
      defined(GPIO_TIM1_CH3OUT) ||defined(GPIO_TIM1_CH4OUT)
#    define HAVE_TIM1_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM2)
#  if defined(GPIO_TIM2_CH1OUT) ||defined(GPIO_TIM2_CH2OUT)||\
      defined(GPIO_TIM2_CH3OUT) ||defined(GPIO_TIM2_CH4OUT)
#    define HAVE_TIM2_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM3)
#  if defined(GPIO_TIM3_CH1OUT) ||defined(GPIO_TIM3_CH2OUT)||\
      defined(GPIO_TIM3_CH3OUT) ||defined(GPIO_TIM3_CH4OUT)
#    define HAVE_TIM3_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM4)
#  if defined(GPIO_TIM4_CH1OUT) ||defined(GPIO_TIM4_CH2OUT)||\
      defined(GPIO_TIM4_CH3OUT) ||defined(GPIO_TIM4_CH4OUT)
#    define HAVE_TIM4_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM5)
#  if defined(GPIO_TIM5_CH1OUT) ||defined(GPIO_TIM5_CH2OUT)||\
      defined(GPIO_TIM5_CH3OUT) ||defined(GPIO_TIM5_CH4OUT)
#    define HAVE_TIM5_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM8)
#  if defined(GPIO_TIM8_CH1OUT) ||defined(GPIO_TIM8_CH2OUT)||\
      defined(GPIO_TIM8_CH3OUT) ||defined(GPIO_TIM8_CH4OUT)
#    define HAVE_TIM8_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM15)
#  if defined(GPIO_TIM15_CH1OUT) ||defined(GPIO_TIM15_CH2OUT)||\
      defined(GPIO_TIM15_CH3OUT) ||defined(GPIO_TIM15_CH4OUT)
#    define HAVE_TIM15_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM16)
#  if defined(GPIO_TIM16_CH1OUT) ||defined(GPIO_TIM16_CH2OUT)||\
      defined(GPIO_TIM16_CH3OUT) ||defined(GPIO_TIM16_CH4OUT)
#    define HAVE_TIM16_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32U5_TIM17)
#  if defined(GPIO_TIM17_CH1OUT) ||defined(GPIO_TIM17_CH2OUT)||\
      defined(GPIO_TIM17_CH3OUT) ||defined(GPIO_TIM17_CH4OUT)
#    define HAVE_TIM17_GPIOCONFIG 1
#endif
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32U5_TIM1)  || defined(CONFIG_STM32U5_TIM2)  || \
    defined(CONFIG_STM32U5_TIM3)  || defined(CONFIG_STM32U5_TIM4)  || \
    defined(CONFIG_STM32U5_TIM5)  || defined(CONFIG_STM32U5_TIM6)  || \
    defined(CONFIG_STM32U5_TIM7)  || defined(CONFIG_STM32U5_TIM8)  || \
    defined(CONFIG_STM32U5_TIM15) || defined(CONFIG_STM32U5_TIM16) || \
    defined(CONFIG_STM32U5_TIM17)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct stm32_tim_priv_s
{
  FAR const struct stm32_tim_ops_s *ops;
  enum stm32_tim_mode_e mode;
  uint32_t base;                      /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Register helpers */

static inline uint16_t stm32_getreg16(FAR struct stm32_tim_dev_s *dev,
                                        uint8_t offset);
static inline void stm32_putreg16(FAR struct stm32_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value);
static inline void stm32_modifyreg16(FAR struct stm32_tim_dev_s *dev,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits);
static inline uint32_t stm32_getreg32(FAR struct stm32_tim_dev_s *dev,
                                        uint8_t offset);
static inline void stm32_putreg32(FAR struct stm32_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value);

/* Timer helpers */

static void stm32_tim_reload_counter(FAR struct stm32_tim_dev_s *dev);
static void stm32_tim_enable(FAR struct stm32_tim_dev_s *dev);
static void stm32_tim_disable(FAR struct stm32_tim_dev_s *dev);
static void stm32_tim_reset(FAR struct stm32_tim_dev_s *dev);
#if defined(HAVE_TIM1_GPIOCONFIG) || defined(HAVE_TIM2_GPIOCONFIG) || \
    defined(HAVE_TIM3_GPIOCONFIG) || defined(HAVE_TIM4_GPIOCONFIG) || \
    defined(HAVE_TIM5_GPIOCONFIG) || defined(HAVE_TIM8_GPIOCONFIG) || \
    defined(HAVE_TIM15_GPIOCONFIG) || defined(HAVE_TIM16_GPIOCONFIG) || \
    defined(HAVE_TIM17_GPIOCONFIG)
static void stm32_tim_gpioconfig(uint32_t cfg,
                                   enum stm32_tim_channel_e mode);
#endif

/* Timer methods */

static int stm32_tim_setmode(FAR struct stm32_tim_dev_s *dev,
                               enum stm32_tim_mode_e mode);
static int stm32_tim_setclock(FAR struct stm32_tim_dev_s *dev,
                                uint32_t freq);
static uint32_t  stm32_tim_getclock(FAR struct stm32_tim_dev_s *dev);
static void stm32_tim_setperiod(FAR struct stm32_tim_dev_s *dev,
                                  uint32_t period);
static uint32_t stm32_tim_getperiod(FAR struct stm32_tim_dev_s *dev);
static uint32_t stm32_tim_getcounter(FAR struct stm32_tim_dev_s *dev);
static int stm32_tim_setchannel(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel,
                                  enum stm32_tim_channel_e mode);
static int stm32_tim_setcompare(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel, uint32_t compare);
static int stm32_tim_getcapture(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel);
static int stm32_tim_setisr(FAR struct stm32_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source);
static void stm32_tim_enableint(FAR struct stm32_tim_dev_s *dev,
                                  int source);
static void stm32_tim_disableint(FAR struct stm32_tim_dev_s *dev,
                                   int source);
static void stm32_tim_ackint(FAR struct stm32_tim_dev_s *dev,
                               int source);
static int stm32_tim_checkint(FAR struct stm32_tim_dev_s *dev,
                                int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_tim_ops_s stm32_tim_ops =
{
  .setmode    = stm32_tim_setmode,
  .setclock   = stm32_tim_setclock,
  .getclock   = stm32_tim_getclock,
  .setperiod  = stm32_tim_setperiod,
  .getperiod  = stm32_tim_getperiod,
  .getcounter = stm32_tim_getcounter,
  .setchannel = stm32_tim_setchannel,
  .setcompare = stm32_tim_setcompare,
  .getcapture = stm32_tim_getcapture,
  .setisr     = stm32_tim_setisr,
  .enableint  = stm32_tim_enableint,
  .disableint = stm32_tim_disableint,
  .ackint     = stm32_tim_ackint,
  .checkint   = stm32_tim_checkint,
};

#ifdef CONFIG_STM32U5_TIM1
struct stm32_tim_priv_s stm32_tim1_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM1_BASE,
};
#endif
#ifdef CONFIG_STM32U5_TIM2
struct stm32_tim_priv_s stm32_tim2_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM2_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM3
struct stm32_tim_priv_s stm32_tim3_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM3_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM4
struct stm32_tim_priv_s stm32_tim4_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM4_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM5
struct stm32_tim_priv_s stm32_tim5_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM5_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM6
struct stm32_tim_priv_s stm32_tim6_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM6_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM7
struct stm32_tim_priv_s stm32_tim7_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM7_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM8
struct stm32_tim_priv_s stm32_tim8_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM8_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM15
struct stm32_tim_priv_s stm32_tim15_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM15_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM16
struct stm32_tim_priv_s stm32_tim16_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM16_BASE,
};
#endif

#ifdef CONFIG_STM32U5_TIM17
struct stm32_tim_priv_s stm32_tim17_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32U5_TIM_MODE_UNUSED,
  .base       = STM32U5_TIM17_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t stm32_getreg16(FAR struct stm32_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg16(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_putreg16(FAR struct stm32_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void stm32_modifyreg16(FAR struct stm32_tim_dev_s *dev,
                                       uint8_t offset, uint16_t clearbits,
                                       uint16_t setbits)
{
  modifyreg16(((struct stm32_tim_priv_s *)dev)->base + offset, clearbits,
              setbits);
}

/****************************************************************************
 * Name: stm32_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset.  This applies only for the
 *   STM32U5 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline uint32_t stm32_getreg32(FAR struct stm32_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg32(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset.  This applies only for the
 *   STM32U5 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline void stm32_putreg32(FAR struct stm32_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: stm32_tim_reload_counter
 ****************************************************************************/

static void stm32_tim_reload_counter(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32U5_GTIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  stm32_putreg16(dev, STM32U5_GTIM_EGR_OFFSET, val);
}

/****************************************************************************
 * Name: stm32_tim_enable
 ****************************************************************************/

static void stm32_tim_enable(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32U5_GTIM_CR1_OFFSET);
  val |= GTIM_CR1_CEN;
  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32U5_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32_tim_disable
 ****************************************************************************/

static void stm32_tim_disable(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32U5_GTIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  stm32_putreg16(dev, STM32U5_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32_tim_reset
 *
 * Description:
 *   Reset timer into system default state, but do not affect output/input
 *   pins
 *
 ****************************************************************************/

static void stm32_tim_reset(FAR struct stm32_tim_dev_s *dev)
{
  ((struct stm32_tim_priv_s *)dev)->mode = STM32U5_TIM_MODE_DISABLED;
  stm32_tim_disable(dev);
}

/****************************************************************************
 * Name: stm32_tim_gpioconfig
 ****************************************************************************/

#if defined(HAVE_TIM1_GPIOCONFIG) || defined(HAVE_TIM2_GPIOCONFIG) || \
    defined(HAVE_TIM3_GPIOCONFIG) || defined(HAVE_TIM4_GPIOCONFIG) || \
    defined(HAVE_TIM5_GPIOCONFIG) || defined(HAVE_TIM8_GPIOCONFIG) || \
    defined(HAVE_TIM15_GPIOCONFIG) || defined(HAVE_TIM16_GPIOCONFIG) || \
    defined(HAVE_TIM17_GPIOCONFIG)
static void stm32_tim_gpioconfig(uint32_t cfg,
                                   enum stm32_tim_channel_e mode)
{
  /* TODO: Add support for input capture and bipolar dual outputs for TIM8 */

  if (mode & STM32U5_TIM_CH_MODE_MASK)
    {
      stm32_configgpio(cfg);
    }
  else
    {
      stm32_unconfiggpio(cfg);
    }
}
#endif

/****************************************************************************
 * Name: stm32_tim_setmode
 ****************************************************************************/

static int stm32_tim_setmode(FAR struct stm32_tim_dev_s *dev,
                               enum stm32_tim_mode_e mode)
{
  uint16_t val = GTIM_CR1_CEN | GTIM_CR1_ARPE;

  DEBUGASSERT(dev != NULL);

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

#if STM32U5_NBTIM > 0
  if (((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM6_BASE
#endif
#if STM32U5_NBTIM > 1
      ||  ((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM7_BASE
#endif
#if STM32U5_NBTIM > 0
  )
    {
      return -EINVAL;
    }
#endif

  /* Decode operational modes */

  switch (mode & STM32U5_TIM_MODE_MASK)
    {
      case STM32U5_TIM_MODE_DISABLED:
        val = 0;
        break;

      case STM32U5_TIM_MODE_DOWN:
        val |= GTIM_CR1_DIR;

      case STM32U5_TIM_MODE_UP:
        break;

      case STM32U5_TIM_MODE_UPDOWN:
        val |= GTIM_CR1_CENTER1;

        /* Our default: Interrupts are generated on compare, when counting
         * down
         */

        break;

      case STM32U5_TIM_MODE_PULSE:
        val |= GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32U5_GTIM_CR1_OFFSET, val);

#if STM32U5_NATIM > 0
  /* Advanced registers require Main Output Enable */

    if (((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM1_BASE ||
        ((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM8_BASE)
      {
        stm32_modifyreg16(dev, STM32U5_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
      }
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_tim_setclock
 ****************************************************************************/

static int stm32_tim_setclock(FAR struct stm32_tim_dev_s *dev,
                                uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      stm32_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM1
      case STM32U5_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM3
      case STM32U5_TIM3_BASE:
        freqin = BOARD_TIM3_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM4
      case STM32U5_TIM4_BASE:
        freqin = BOARD_TIM4_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
        freqin = BOARD_TIM5_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM6
      case STM32U5_TIM6_BASE:
        freqin = BOARD_TIM6_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM7
      case STM32U5_TIM7_BASE:
        freqin = BOARD_TIM7_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM8
      case STM32U5_TIM8_BASE:
        freqin = BOARD_TIM8_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM15
      case STM32U5_TIM15_BASE:
        freqin = BOARD_TIM15_FREQUENCY;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM16
      case STM32U5_TIM16_BASE:
        freqin = BOARD_TIM16_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM17
      case STM32U5_TIM17_BASE:
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

  stm32_putreg16(dev, STM32U5_GTIM_PSC_OFFSET, prescaler);
  stm32_tim_enable(dev);

  return prescaler;
}

/****************************************************************************
 * Name: stm32_tim_getclock
 ****************************************************************************/

static uint32_t stm32_tim_getclock(FAR struct stm32_tim_dev_s *dev)
{
  uint32_t freqin;
  uint32_t clock;
  DEBUGASSERT(dev != NULL);

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM1
      case STM32U5_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM3
      case STM32U5_TIM3_BASE:
        freqin = BOARD_TIM3_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM4
      case STM32U5_TIM4_BASE:
        freqin = BOARD_TIM4_FREQUENCY;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
        freqin = BOARD_TIM5_FREQUENCY;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM6
      case STM32U5_TIM6_BASE:
        freqin = BOARD_TIM6_FREQUENCY;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM7
      case STM32U5_TIM7_BASE:
        freqin = BOARD_TIM7_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM8
      case STM32U5_TIM8_BASE:
        freqin = BOARD_TIM8_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM15
      case STM32U5_TIM15_BASE:
        freqin = BOARD_TIM15_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM16
      case STM32U5_TIM16_BASE:
        freqin = BOARD_TIM16_FREQUENCY;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM17
      case STM32U5_TIM17_BASE:
        freqin = BOARD_TIM17_FREQUENCY;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* From chip datasheet, at page 1179. */

  clock = freqin / (stm32_getreg16(dev, STM32U5_GTIM_PSC_OFFSET) + 1);
  return clock;
}

/****************************************************************************
 * Name: stm32_tim_setperiod
 ****************************************************************************/

static void stm32_tim_setperiod(FAR struct stm32_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  stm32_putreg32(dev, STM32U5_GTIM_ARR_OFFSET, period);
}

/****************************************************************************
 * Name: stm32_tim_getperiod
 ****************************************************************************/

static uint32_t stm32_tim_getperiod (FAR struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return stm32_getreg32 (dev, STM32U5_GTIM_ARR_OFFSET);
}

/****************************************************************************
 * Name: stm32_tim_getcounter
 ****************************************************************************/

static uint32_t stm32_tim_getcounter(FAR struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  uint32_t counter = stm32_getreg32(dev, STM32U5_GTIM_CNT_OFFSET);

  /* In datasheet page 988, there is a useless bit named UIFCPY in TIMx_CNT.
   * reset it it result when not TIM2 or TIM5.
   */

#if defined(CONFIG_STM32U5_TIM2) || defined(CONFIG_STM32U5_TIM5)
  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
#endif
#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
#endif
        return counter;

      default:
        return counter & 0x0000ffff;
    }
#else
  return counter & 0x0000ffff;
#endif
}

/****************************************************************************
 * Name: stm32_tim_setchannel
 ****************************************************************************/

static int stm32_tim_setchannel(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel,
                                  enum stm32_tim_channel_e mode)
{
  uint16_t ccmr_orig   = 0;
  uint16_t ccmr_val    = 0;
  uint16_t ccmr_mask   = 0xff;
  uint16_t ccer_val;
  uint8_t  ccmr_offset = STM32U5_GTIM_CCMR1_OFFSET;

  DEBUGASSERT(dev != NULL);

  /* Further we use range as 0..3; if channel=0 it will also overflow here */

  if (--channel > 4)
    {
      return -EINVAL;
    }

  /* Assume that channel is disabled and polarity is active high */

  ccer_val = stm32_getreg16(dev, STM32U5_GTIM_CCER_OFFSET);
  ccer_val &= ~((GTIM_CCER_CC1P | GTIM_CCER_CC1E) <<
                GTIM_CCER_CCXBASE(channel));

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

#if STM32U5_NBTIM > 0
  if (((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM6_BASE
#endif
#if STM32U5_NBTIM > 1
      || ((struct stm32_tim_priv_s *)dev)->base == STM32U5_TIM7_BASE
#endif
#if STM32U5_NBTIM > 0
     )
    {
      return -EINVAL;
    }
#endif

  /* Decode configuration */

  switch (mode & STM32U5_TIM_CH_MODE_MASK)
    {
      case STM32U5_TIM_CH_DISABLED:
        break;

      case STM32U5_TIM_CH_OUTPWM:
        ccmr_val  = (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) +
                    GTIM_CCMR1_OC1PE;
        ccer_val |= GTIM_CCER_CC1E << GTIM_CCER_CCXBASE(channel);
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity */

  if (mode & STM32U5_TIM_CH_POLARITY_NEG)
    {
      ccer_val |= GTIM_CCER_CC1P << GTIM_CCER_CCXBASE(channel);
    }

  /* Define its position (shift) and get register offset */

  if (channel & 1)
    {
      ccmr_val  <<= 8;
      ccmr_mask <<= 8;
    }

  if (channel > 1)
    {
      ccmr_offset = STM32U5_GTIM_CCMR2_OFFSET;
    }

  ccmr_orig  = stm32_getreg16(dev, ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  stm32_putreg16(dev, ccmr_offset, ccmr_orig);
  stm32_putreg16(dev, STM32U5_GTIM_CCER_OFFSET, ccer_val);

  /* set GPIO */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM1
      case STM32U5_TIM1_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM1_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM1_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM1_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM1_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM1_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM1_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM2_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM2_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM2_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM2_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM2_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM2_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM3
      case STM32U5_TIM3_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM3_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM3_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM3_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM3_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM3_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM3_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM3_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM3_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM4
      case STM32U5_TIM4_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM4_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM4_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM4_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM4_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM4_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM4_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM4_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM4_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM5_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM5_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM5_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM5_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM5_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM5_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM5_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM5_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM8
      case STM32U5_TIM8_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM8_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM8_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM8_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM8_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM8_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM8_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM8_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM8_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM15
      case STM32U5_TIM15_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM15_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM15_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM15_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM15_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM15_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM15_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM15_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM15_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM16
      case STM32U5_TIM16_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM16_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM16_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM16_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM16_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM16_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM16_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM16_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM16_CH4OUT, mode);
              break;
#endif

            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32U5_TIM17
      case STM32U5_TIM17_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM17_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM17_CH1OUT, mode);
              break;
#endif

#if defined(GPIO_TIM17_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM17_CH2OUT, mode);
              break;
#endif

#if defined(GPIO_TIM17_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM17_CH3OUT, mode);
              break;
#endif

#if defined(GPIO_TIM17_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM17_CH4OUT, mode);
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
 * Name: stm32_tim_setcompare
 ****************************************************************************/

static int stm32_tim_setcompare(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel, uint32_t compare)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        stm32_putreg32(dev, STM32U5_GTIM_CCR1_OFFSET, compare);
        break;

      case 2:
        stm32_putreg32(dev, STM32U5_GTIM_CCR2_OFFSET, compare);
        break;

      case 3:
        stm32_putreg32(dev, STM32U5_GTIM_CCR3_OFFSET, compare);
        break;

      case 4:
        stm32_putreg32(dev, STM32U5_GTIM_CCR4_OFFSET, compare);
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_tim_getcapture
 ****************************************************************************/

static int stm32_tim_getcapture(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t channel)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        return stm32_getreg32(dev, STM32U5_GTIM_CCR1_OFFSET);

      case 2:
        return stm32_getreg32(dev, STM32U5_GTIM_CCR2_OFFSET);

      case 3:
        return stm32_getreg32(dev, STM32U5_GTIM_CCR3_OFFSET);

      case 4:
        return stm32_getreg32(dev, STM32U5_GTIM_CCR4_OFFSET);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: stm32_tim_setisr
 ****************************************************************************/

static int stm32_tim_setisr(FAR struct stm32_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM1
      case STM32U5_TIM1_BASE:
        vectorno = STM32U5_IRQ_TIM1UP;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
        vectorno = STM32U5_IRQ_TIM2;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM3
      case STM32U5_TIM3_BASE:
        vectorno = STM32U5_IRQ_TIM3;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM4
      case STM32U5_TIM4_BASE:
        vectorno = STM32U5_IRQ_TIM4;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
        vectorno = STM32U5_IRQ_TIM5;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM6
      case STM32U5_TIM6_BASE:
        vectorno = STM32U5_IRQ_TIM6;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM7
      case STM32U5_TIM7_BASE:
        vectorno = STM32U5_IRQ_TIM7;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM8
      case STM32U5_TIM8_BASE:
        vectorno = STM32U5_IRQ_TIM8UP;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM15
      case STM32U5_TIM15_BASE:
        vectorno = STM32U5_IRQ_TIM15;
        break;
#endif
#ifdef CONFIG_STM32U5_TIM16
      case STM32U5_TIM16_BASE:
        vectorno = STM32U5_IRQ_TIM16;
        break;
#endif

#ifdef CONFIG_STM32U5_TIM17
      case STM32U5_TIM17_BASE:
        vectorno = STM32U5_IRQ_TIM17;
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
 * Name: stm32_tim_enableint
 ****************************************************************************/

static void stm32_tim_enableint(FAR struct stm32_tim_dev_s *dev,
                                  int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32U5_GTIM_DIER_OFFSET, 0, GTIM_DIER_UIE);
}

/****************************************************************************
 * Name: stm32_tim_disableint
 ****************************************************************************/

static void stm32_tim_disableint(FAR struct stm32_tim_dev_s *dev,
                                   int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32U5_GTIM_DIER_OFFSET, GTIM_DIER_UIE, 0);
}

/****************************************************************************
 * Name: stm32_tim_ackint
 ****************************************************************************/

static void stm32_tim_ackint(FAR struct stm32_tim_dev_s *dev, int source)
{
  stm32_putreg16(dev, STM32U5_GTIM_SR_OFFSET, ~GTIM_SR_UIF);
}

/****************************************************************************
 * Name: stm32_tim_checkint
 ****************************************************************************/

static int stm32_tim_checkint(FAR struct stm32_tim_dev_s *dev,
                                int source)
{
  uint16_t regval = stm32_getreg16(dev, STM32U5_GTIM_SR_OFFSET);
  return (regval & GTIM_SR_UIF) ? 1 : 0;
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tim_init
 ****************************************************************************/

FAR struct stm32_tim_dev_s *stm32_tim_init(int timer)
{
  struct stm32_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_STM32U5_TIM1
      case 1:
        dev = (struct stm32_tim_dev_s *)&stm32_tim1_priv;
        modifyreg32(STM32U5_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM2
      case 2:
        dev = (struct stm32_tim_dev_s *)&stm32_tim2_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM2EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM3
      case 3:
        dev = (struct stm32_tim_dev_s *)&stm32_tim3_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM3EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM4
      case 4:
        dev = (struct stm32_tim_dev_s *)&stm32_tim4_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM4EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM5
      case 5:
        dev = (struct stm32_tim_dev_s *)&stm32_tim5_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM5EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM6
      case 6:
        dev = (struct stm32_tim_dev_s *)&stm32_tim6_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM6EN);
        break;
#endif
#ifdef CONFIG_STM32U5_TIM7
      case 7:
        dev = (struct stm32_tim_dev_s *)&stm32_tim7_priv;
        modifyreg32(STM32U5_RCC_APB1ENR1, 0, RCC_APB1ENR1_TIM7EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM8
      case 8:
        dev = (struct stm32_tim_dev_s *)&stm32_tim8_priv;
        modifyreg32(STM32U5_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM15
      case 15:
        dev = (struct stm32_tim_dev_s *)&stm32_tim15_priv;
        modifyreg32(STM32U5_RCC_APB2ENR, 0, RCC_APB2ENR_TIM15EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM16
      case 16:
        dev = (struct stm32_tim_dev_s *)&stm32_tim16_priv;
        modifyreg32(STM32U5_RCC_APB2ENR, 0, RCC_APB2ENR_TIM16EN);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM17
      case 17:
        dev = (struct stm32_tim_dev_s *)&stm32_tim17_priv;
        modifyreg32(STM32U5_RCC_APB2ENR, 0, RCC_APB2ENR_TIM17EN);
        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct stm32_tim_priv_s *)dev)->mode != STM32U5_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  stm32_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: stm32_tim_deinit
 *
 * TODO: Detach interrupts, and close down all TIM Channels
 *
 ****************************************************************************/

int stm32_tim_deinit(FAR struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32U5_TIM1
      case STM32U5_TIM1_BASE:
        modifyreg32(STM32U5_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM2
      case STM32U5_TIM2_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM2EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM3
      case STM32U5_TIM3_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM3EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM4
      case STM32U5_TIM4_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM4EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM5
      case STM32U5_TIM5_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM5EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM6
      case STM32U5_TIM6_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM6EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM7
      case STM32U5_TIM7_BASE:
        modifyreg32(STM32U5_RCC_APB1ENR1, RCC_APB1ENR1_TIM7EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM8
      case STM32U5_TIM8_BASE:
        modifyreg32(STM32U5_RCC_APB2ENR, RCC_APB2ENR_TIM8EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM15
      case STM32U5_TIM15_BASE:
        modifyreg32(STM32U5_RCC_APB2ENR, RCC_APB2ENR_TIM15EN, 0);
        break;
#endif
#ifdef CONFIG_STM32U5_TIM16
      case STM32U5_TIM16_BASE:
        modifyreg32(STM32U5_RCC_APB2ENR, RCC_APB2ENR_TIM16EN, 0);
        break;
#endif

#ifdef CONFIG_STM32U5_TIM17
      case STM32U5_TIM17_BASE:
        modifyreg32(STM32U5_RCC_APB2ENR, RCC_APB2ENR_TIM17EN, 0);
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct stm32_tim_priv_s *)dev)->mode = STM32U5_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32U5_TIM1 || ... || TIM17) */
