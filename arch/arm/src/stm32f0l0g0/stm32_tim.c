/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_tim.c
 *
 *   Copyright (C) 2019 Fundação CERTI. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_rcc.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32F0L0G0_TIMn is defined then the CONFIG_STM32F0L0G0_TIMn_PWM
 *   may also be defined to indicate that the timer is intended to be used
 *   for pulsed output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32F0L0G0_TIMn is
 *   defined then CONFIG_STM32F0L0G0_TIMn_ADC may also be defined to indicate
 *   that timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32F0L0G0_TIMn is defined
 *   then CONFIG_STM32F0L0G0_TIMn_DAC may also be defined to indicate that
 *   timer "n" is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32F0L0G0_TIMn is defined then
 *   CONFIG_STM32F0L0G0_TIMn_QE may also be defined to indicate that timer
 *   "n" is intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM1_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM1_DAC) || defined(CONFIG_STM32F0L0G0_TIM1_QE)
#  undef CONFIG_STM32F0L0G0_TIM1
#endif

#if defined(CONFIG_STM32F0L0G0_TIM2_PWM) || defined(CONFIG_STM32F0L0G0_TIM2_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM2_DAC) || defined(CONFIG_STM32F0L0G0_TIM2_QE)
#  undef CONFIG_STM32F0L0G0_TIM2
#endif

#if defined(CONFIG_STM32F0L0G0_TIM3_PWM) || defined(CONFIG_STM32F0L0G0_TIM3_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM3_DAC) || defined(CONFIG_STM32F0L0G0_TIM3_QE)
#  undef CONFIG_STM32F0L0G0_TIM3
#endif

#if defined(CONFIG_STM32F0L0G0_TIM4_PWM) || defined(CONFIG_STM32F0L0G0_TIM4_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM4_DAC) || defined(CONFIG_STM32F0L0G0_TIM4_QE)
#  undef CONFIG_STM32F0L0G0_TIM4
#endif

#if defined(CONFIG_STM32F0L0G0_TIM5_PWM) || defined(CONFIG_STM32F0L0G0_TIM5_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM5_DAC) || defined(CONFIG_STM32F0L0G0_TIM5_QE)
#  undef CONFIG_STM32F0L0G0_TIM5
#endif

#if defined(CONFIG_STM32F0L0G0_TIM6_PWM) || defined(CONFIG_STM32F0L0G0_TIM6_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM6_DAC) || defined(CONFIG_STM32F0L0G0_TIM6_QE)
#  undef CONFIG_STM32F0L0G0_TIM6
#endif

#if defined(CONFIG_STM32F0L0G0_TIM7_PWM) || defined(CONFIG_STM32F0L0G0_TIM7_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM7_DAC) || defined(CONFIG_STM32F0L0G0_TIM7_QE)
#  undef CONFIG_STM32F0L0G0_TIM7
#endif

#if defined(CONFIG_STM32F0L0G0_TIM8_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM8_DAC) || defined(CONFIG_STM32F0L0G0_TIM8_QE)
#  undef CONFIG_STM32F0L0G0_TIM8
#endif

#if defined(CONFIG_STM32F0L0G0_TIM12_PWM) || defined(CONFIG_STM32F0L0G0_TIM12_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM12_DAC) || defined(CONFIG_STM32F0L0G0_TIM12_QE)
#  undef CONFIG_STM32F0L0G0_TIM12
#endif

#if defined(CONFIG_STM32F0L0G0_TIM13_PWM) || defined(CONFIG_STM32F0L0G0_TIM13_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM13_DAC) || defined(CONFIG_STM32F0L0G0_TIM13_QE)
#  undef CONFIG_STM32F0L0G0_TIM13
#endif

#if defined(CONFIG_STM32F0L0G0_TIM14_PWM) || defined(CONFIG_STM32F0L0G0_TIM14_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM14_DAC) || defined(CONFIG_STM32F0L0G0_TIM14_QE)
#  undef CONFIG_STM32F0L0G0_TIM14
#endif

#if defined(CONFIG_STM32F0L0G0_TIM15_PWM) || defined(CONFIG_STM32F0L0G0_TIM15_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM15_DAC) || defined(CONFIG_STM32F0L0G0_TIM15_QE)
#  undef CONFIG_STM32F0L0G0_TIM15
#endif

#if defined(CONFIG_STM32F0L0G0_TIM16_PWM) || defined(CONFIG_STM32F0L0G0_TIM16_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM16_DAC) || defined(CONFIG_STM32F0L0G0_TIM16_QE)
#  undef CONFIG_STM32F0L0G0_TIM16
#endif

#if defined(CONFIG_STM32F0L0G0_TIM17_PWM) || defined(CONFIG_STM32F0L0G0_TIM17_ADC) || \
    defined(CONFIG_STM32F0L0G0_TIM17_DAC) || defined(CONFIG_STM32F0L0G0_TIM17_QE)
#  undef CONFIG_STM32F0L0G0_TIM17
#endif

#if defined(CONFIG_STM32F0L0G0_TIM1)
#  if defined(GPIO_TIM1_CH1OUT) || defined(GPIO_TIM1_CH2OUT) || \
      defined(GPIO_TIM1_CH3OUT) || defined(GPIO_TIM1_CH4OUT) || \
      defined(GPIO_TIM1_CH5OUT) || defined(GPIO_TIM1_CH6OUT)
#    define HAVE_TIM1_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM2)
#  if defined(GPIO_TIM2_CH1OUT) || defined(GPIO_TIM2_CH2OUT) || \
      defined(GPIO_TIM2_CH3OUT) || defined(GPIO_TIM2_CH4OUT)
#    define HAVE_TIM2_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM3)
#  if defined(GPIO_TIM3_CH1OUT) || defined(GPIO_TIM3_CH2OUT) || \
      defined(GPIO_TIM3_CH3OUT) || defined(GPIO_TIM3_CH4OUT)
#    define HAVE_TIM3_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM4)
#  if defined(GPIO_TIM4_CH1OUT) || defined(GPIO_TIM4_CH2OUT) || \
      defined(GPIO_TIM4_CH3OUT) || defined(GPIO_TIM4_CH4OUT)
#    define HAVE_TIM4_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM5)
#  if defined(GPIO_TIM5_CH1OUT) || defined(GPIO_TIM5_CH2OUT) || \
      defined(GPIO_TIM5_CH3OUT) || defined(GPIO_TIM5_CH4OUT)
#    define HAVE_TIM5_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM8)
#  if defined(GPIO_TIM8_CH1OUT) || defined(GPIO_TIM8_CH2OUT) || \
      defined(GPIO_TIM8_CH3OUT) || defined(GPIO_TIM8_CH4OUT) || \
      defined(GPIO_TIM8_CH5OUT) || defined(GPIO_TIM8_CH6OUT)
#    define HAVE_TIM8_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM12)
#  if defined(GPIO_TIM12_CH1OUT) || defined(GPIO_TIM12_CH2OUT)
#    define HAVE_TIM12_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM13)
#  if defined(GPIO_TIM13_CH1OUT)
#    define HAVE_TIM13_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM14)
#  if defined(GPIO_TIM14_CH1OUT)
#    define HAVE_TIM14_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM15)
#  if defined(GPIO_TIM15_CH1OUT) || defined(GPIO_TIM15_CH2OUT)
#    define HAVE_TIM15_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM16)
#  if defined(GPIO_TIM16_CH1OUT)
#    define HAVE_TIM16_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_STM32F0L0G0_TIM17)
#  if defined(GPIO_TIM17_CH1OUT)
#    define HAVE_TIM17_GPIOCONFIG 1
#  endif
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32F0L0G0_TIM1)  || defined(CONFIG_STM32F0L0G0_TIM2)  || \
    defined(CONFIG_STM32F0L0G0_TIM3)  || defined(CONFIG_STM32F0L0G0_TIM4)  || \
    defined(CONFIG_STM32F0L0G0_TIM5)  || defined(CONFIG_STM32F0L0G0_TIM6)  || \
    defined(CONFIG_STM32F0L0G0_TIM7)  || defined(CONFIG_STM32F0L0G0_TIM8)  || \
    defined(CONFIG_STM32F0L0G0_TIM12) || defined(CONFIG_STM32F0L0G0_TIM13) || \
    defined(CONFIG_STM32F0L0G0_TIM14) || defined(CONFIG_STM32F0L0G0_TIM15) || \
    defined(CONFIG_STM32F0L0G0_TIM16) || defined(CONFIG_STM32F0L0G0_TIM17)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct stm32_tim_priv_s
{
  const struct stm32_tim_ops_s *ops;
  stm32_tim_mode_t        mode;
  uint32_t                base;   /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Timer methods */

static int  stm32_tim_setmode(FAR struct stm32_tim_dev_s *dev,
                              stm32_tim_mode_t mode);
static int  stm32_tim_setclock(FAR struct stm32_tim_dev_s *dev,
                               uint32_t freq);
static uint32_t stm32_tim_getclock(FAR struct stm32_tim_dev_s *dev);
static void stm32_tim_setperiod(FAR struct stm32_tim_dev_s *dev,
                                uint32_t period);
static uint32_t stm32_tim_getperiod(FAR struct stm32_tim_dev_s *dev);
static uint32_t stm32_tim_getcounter(FAR struct stm32_tim_dev_s *dev);
static int  stm32_tim_setchannel(FAR struct stm32_tim_dev_s *dev,
                                 uint8_t channel,
                                 stm32_tim_channel_t mode);
static int  stm32_tim_setcompare(FAR struct stm32_tim_dev_s *dev,
                                 uint8_t channel,
                                 uint32_t compare);
static int  stm32_tim_getcapture(FAR struct stm32_tim_dev_s *dev,
                                 uint8_t channel);
static int  stm32_tim_setisr(FAR struct stm32_tim_dev_s *dev, xcpt_t handler,
                             void *arg, int source);
static void stm32_tim_enableint(FAR struct stm32_tim_dev_s *dev, int source);
static void stm32_tim_disableint(FAR struct stm32_tim_dev_s *dev,
                                 int source);
static void stm32_tim_ackint(FAR struct stm32_tim_dev_s *dev, int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct stm32_tim_ops_s stm32_tim_ops =
{
  .setmode        = &stm32_tim_setmode,
  .setclock       = &stm32_tim_setclock,
  .getclock       = &stm32_tim_getclock,
  .setperiod      = &stm32_tim_setperiod,
  .getperiod      = &stm32_tim_getperiod,
  .getcounter     = &stm32_tim_getcounter,
  .setchannel     = &stm32_tim_setchannel,
  .setcompare     = &stm32_tim_setcompare,
  .getcapture     = &stm32_tim_getcapture,
  .setisr         = &stm32_tim_setisr,
  .enableint      = &stm32_tim_enableint,
  .disableint     = &stm32_tim_disableint,
  .ackint         = &stm32_tim_ackint
};

#ifdef CONFIG_STM32F0L0G0_TIM1
struct stm32_tim_priv_s stm32_tim1_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM1_BASE,
};
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
struct stm32_tim_priv_s stm32_tim2_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM2_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3
struct stm32_tim_priv_s stm32_tim3_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM3_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM4
struct stm32_tim_priv_s stm32_tim4_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM4_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM5
struct stm32_tim_priv_s stm32_tim5_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM5_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM6
struct stm32_tim_priv_s stm32_tim6_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM6_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM7
struct stm32_tim_priv_s stm32_tim7_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM7_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM8
struct stm32_tim_priv_s stm32_tim8_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM8_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM12
struct stm32_tim_priv_s stm32_tim12_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM12_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM13
struct stm32_tim_priv_s stm32_tim13_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM13_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14
struct stm32_tim_priv_s stm32_tim14_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM14_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
struct stm32_tim_priv_s stm32_tim15_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM15_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
struct stm32_tim_priv_s stm32_tim16_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM16_BASE,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
struct stm32_tim_priv_s stm32_tim17_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM17_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(FAR struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg16(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(FAR struct stm32_tim_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct stm32_tim_priv_s *)dev)->base + offset, clearbits,
              setbits);
}

/* Get a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline uint32_t stm32_getreg32(FAR struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Put a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline void stm32_putreg32(FAR struct stm32_tim_dev_s *dev,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

static void stm32_tim_reload_counter(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  stm32_putreg16(dev, STM32_GTIM_EGR_OFFSET, val);
}

static void stm32_tim_enable(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);
  val |= GTIM_CR1_CEN;
  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

static void stm32_tim_disable(FAR struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

/* Reset timer into system default state, but do not affect output/input
 * pins
 */

static void stm32_tim_reset(FAR struct stm32_tim_dev_s *dev)
{
  ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_DISABLED;
  stm32_tim_disable(dev);
}

#if defined(HAVE_TIM1_GPIOCONFIG) || defined(HAVE_TIM2_GPIOCONFIG) || \
    defined(HAVE_TIM3_GPIOCONFIG) || defined(HAVE_TIM4_GPIOCONFIG) || \
    defined(HAVE_TIM5_GPIOCONFIG) || defined(HAVE_TIM6_GPIOCONFIG) || \
    defined(HAVE_TIM7_GPIOCONFIG) || defined(HAVE_TIM8_GPIOCONFIG) || \
    defined(HAVE_TIM12_GPIOCONFIG) || defined(HAVE_TIM13_GPIOCONFIG) || \
    defined(HAVE_TIM14_GPIOCONFIG) || defined(HAVE_TIM15_GPIOCONFIG) || \
    defined(HAVE_TIM16_GPIOCONFIG) || defined(HAVE_TIM17_GPIOCONFIG)
static void stm32_tim_gpioconfig(uint32_t cfg, stm32_tim_channel_t mode)
{
  /* TODO: Add support for input capture and bipolar dual outputs for TIM8 */

  if (mode & STM32_TIM_CH_MODE_MASK)
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
 * Basic Functions
 ****************************************************************************/

static int stm32_tim_setclock(FAR struct stm32_tim_dev_s *dev, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  tmrinfo("Set clock=%" PRId32 "\n", freq);

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
#ifdef CONFIG_STM32F0L0G0_TIM1
      case STM32_TIM1_BASE:
        freqin = STM32_APB2_TIM1_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case STM32_TIM2_BASE:
        freqin = STM32_APB1_TIM2_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case STM32_TIM3_BASE:
        freqin = STM32_APB1_TIM3_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case STM32_TIM6_BASE:
        freqin = STM32_APB1_TIM6_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case STM32_TIM7_BASE:
        freqin = STM32_APB1_TIM7_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case STM32_TIM14_BASE:
        freqin = STM32_APB2_TIM14_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case STM32_TIM15_BASE:
        freqin = STM32_APB2_TIM15_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case STM32_TIM16_BASE:
        freqin = STM32_APB2_TIM16_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case STM32_TIM17_BASE:
        freqin = STM32_APB2_TIM17_CLKIN;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;
  tmrinfo("  timer freq=%" PRId32 "\n", freqin);
  tmrinfo("  prescaler=%d\n", prescaler);

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

  tmrinfo("  prescaler (adjusted)=%d\n", prescaler);

  /* PSC_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_putreg16(dev, STM32_GTIM_PSC_OFFSET, prescaler);
  stm32_tim_enable(dev);

  return prescaler;
}

static uint32_t stm32_tim_getclock(FAR struct stm32_tim_dev_s *dev)
{
  uint32_t freqin;
  uint32_t clock;
  uint32_t prescaler;
  DEBUGASSERT(dev != NULL);

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case STM32_TIM1_BASE:
        freqin = STM32_APB2_TIM1_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case STM32_TIM2_BASE:
        freqin = STM32_APB1_TIM2_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case STM32_TIM3_BASE:
        freqin = STM32_APB1_TIM3_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case STM32_TIM6_BASE:
        freqin = STM32_APB1_TIM6_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case STM32_TIM7_BASE:
        freqin = STM32_APB1_TIM7_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case STM32_TIM14_BASE:
        freqin = STM32_APB2_TIM14_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case STM32_TIM15_BASE:
        freqin = STM32_APB2_TIM15_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case STM32_TIM16_BASE:
        freqin = STM32_APB2_TIM16_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case STM32_TIM17_BASE:
        freqin = STM32_APB2_TIM17_CLKIN;
        break;
#endif
      default:
        return -EINVAL;
    }

  prescaler = stm32_getreg16(dev, STM32_GTIM_PSC_OFFSET);
  clock = freqin / (prescaler + 1);
  return clock;
}

static void stm32_tim_setperiod(FAR struct stm32_tim_dev_s *dev,
                                uint32_t period)
{
  tmrinfo("Set period=%" PRId32 "\n", period);
  DEBUGASSERT(dev != NULL);

  /* ARR_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_putreg32(dev, STM32_GTIM_ARR_OFFSET, period);
}

static uint32_t stm32_tim_getperiod (FAR struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return stm32_getreg32 (dev, STM32_GTIM_ARR_OFFSET);
}

static uint32_t stm32_tim_getcounter(FAR struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  /* According to STM32G0x0 datasheet, TIMx_CNT registers are 32-bits but
   * CNT field is 16-bits [15:0].
   * TIM 1, 3, 6-7, 14-17
   */

  /* In datasheet page 988, there is a useless bit named UIFCPY in TIMx_CNT.
   * reset it it result when not TIM2 or TIM5.
   */

  uint32_t counter = stm32_getreg32(dev, STM32_GTIM_CNT_OFFSET);
  counter &= 0xffff;
  return counter;
}

static int stm32_tim_setisr(FAR struct stm32_tim_dev_s *dev,
                            xcpt_t handler, void *arg, int source)
{
  int vectorno;

  tmrinfo("Set ISR\n");

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case STM32_TIM1_BASE:
        vectorno = STM32_IRQ_TIM1_BRK;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case STM32_TIM2_BASE:
        vectorno = STM32_IRQ_TIM2;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case STM32_TIM3_BASE:
        vectorno = STM32_IRQ_TIM3;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case STM32_TIM6_BASE:
        vectorno = STM32_IRQ_TIM6;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case STM32_TIM7_BASE:
        vectorno = STM32_IRQ_TIM7;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM13
      case STM32_TIM13_BASE:
        vectorno = STM32_IRQ_TIM13;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case STM32_TIM14_BASE:
        vectorno = STM32_IRQ_TIM14;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case STM32_TIM15_BASE:
        vectorno = STM32_IRQ_TIM15;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case STM32_TIM16_BASE:
        vectorno = STM32_IRQ_TIM16;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case STM32_TIM17_BASE:
        vectorno = STM32_IRQ_TIM17;
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

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

static void stm32_tim_enableint(FAR struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);

  /* DIER_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, 0, source);
}

static void stm32_tim_disableint(FAR struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);

  /* DIER_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, source, 0);
}

static void stm32_tim_ackint(FAR struct stm32_tim_dev_s *dev, int source)
{
  /* SR_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_putreg16(dev, STM32_GTIM_SR_OFFSET, ~source);
}

/****************************************************************************
 * General Functions
 ****************************************************************************/

static int stm32_tim_setmode(FAR struct stm32_tim_dev_s *dev,
                             stm32_tim_mode_t mode)
{
  tmrinfo("Set mode=%d\n", mode);
  uint16_t val = GTIM_CR1_CEN | GTIM_CR1_ARPE;

  DEBUGASSERT(dev != NULL);

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

  if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM6_BASE || \
      ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM7_BASE)
    {
      return -EINVAL;
    }

  /* Decode operational modes */

  switch (mode & STM32_TIM_MODE_MASK)
    {
      case STM32_TIM_MODE_DISABLED:
        val = 0;
        break;

      case STM32_TIM_MODE_DOWN:
        val |= GTIM_CR1_DIR;

      case STM32_TIM_MODE_UP:
        break;

      case STM32_TIM_MODE_UPDOWN:
        val |= GTIM_CR1_CENTER1;

        /* Our default: Interrupts are generated on compare, when counting
         * down
         */

        break;

      case STM32_TIM_MODE_PULSE:
        val |= GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  stm32_tim_reload_counter(dev);

  /* CR1_OFFSET is the same for ATIM, BTIM or GTIM */

  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);

  /* Advanced registers require Main Output Enable */
#if defined(CONFIG_STM32F0L0G0_TIM1) || defined(CONFIG_STM32F0L0G0_TIM8)
  if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM1_BASE
#  if defined(CONFIG_STM32F0L0G0_TIM8)
      || ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM8_BASE
#  endif
  )
      {
         stm32_modifyreg16(dev, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
      }
#endif /* CONFIG_STM32F0L0G0_TIM1 || CONFIG_STM32F0L0G0_TIM8 */

  return OK;
}

static int stm32_tim_setchannel(FAR struct stm32_tim_dev_s *dev,
                                uint8_t channel, stm32_tim_channel_t mode)
{
  uint16_t ccmr_orig   = 0;
  uint16_t ccmr_val    = 0;
  uint16_t ccmr_mask   = 0xff;

  /* CCER_OFFSET and CCMR1_OFFSET are the same for ATIM and GTIM */

  uint16_t ccer_val    = stm32_getreg16(dev, STM32_GTIM_CCER_OFFSET);
  uint8_t  ccmr_offset = STM32_GTIM_CCMR1_OFFSET;

  DEBUGASSERT(dev != NULL);

  /* Further we use range as 0..3; if channel=0 it will also overflow here */

  if (--channel > 4)
    {
      return -EINVAL;
    }

  /* Assume that channel is disabled and polarity is active high */

  ccer_val &= ~((GTIM_CCER_CC1P | GTIM_CCER_CC1E) <<
                GTIM_CCER_CCXBASE(channel));

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

  if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM6_BASE || \
      ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM7_BASE)
    {
      return -EINVAL;
    }

  /* Decode configuration */

  switch (mode & STM32_TIM_CH_MODE_MASK)
    {
      case STM32_TIM_CH_DISABLED:
        break;

      case STM32_TIM_CH_OUTPWM:
        ccmr_val  = (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) +
                    GTIM_CCMR1_OC1PE;
        ccer_val |= GTIM_CCER_CC1E << GTIM_CCER_CCXBASE(channel);
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity */

  if (mode & STM32_TIM_CH_POLARITY_NEG)
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
      ccmr_offset = STM32_GTIM_CCMR2_OFFSET;
    }

  ccmr_orig  = stm32_getreg16(dev, ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  stm32_putreg16(dev, ccmr_offset, ccmr_orig);
  stm32_putreg16(dev, STM32_GTIM_CCER_OFFSET, ccer_val);

  /* set GPIO */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case STM32_TIM1_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM1_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM1_CH1OUT, mode); break;
#  endif
#  if defined(GPIO_TIM1_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM1_CH2OUT, mode); break;
#  endif
#  if defined(GPIO_TIM1_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM1_CH3OUT, mode); break;
#  endif
#  if defined(GPIO_TIM1_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM1_CH4OUT, mode); break;
#  endif
#  if defined(GPIO_TIM1_CH5OUT)
            case 4:
              stm32_tim_gpioconfig(GPIO_TIM1_CH5OUT, mode); break;
#  endif
#  if defined(GPIO_TIM1_CH6OUT)
            case 5:
              stm32_tim_gpioconfig(GPIO_TIM1_CH6OUT, mode); break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2
      case STM32_TIM2_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM2_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM2_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM2_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM2_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM2_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM2_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM2_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM2_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3
      case STM32_TIM3_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM3_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM3_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM3_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM3_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM3_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM3_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM3_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM3_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM13
      case STM32_TIM13_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM13_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM13_CH1OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14
      case STM32_TIM14_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM14_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM14_CH1OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
      case STM32_TIM15_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM15_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM15_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM15_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM15_CH2OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
      case STM32_TIM16_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM16_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM16_CH1OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
      case STM32_TIM17_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM17_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM17_CH1OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
    }

  return OK;
}

static int stm32_tim_setcompare(FAR struct stm32_tim_dev_s *dev,
                                uint8_t channel, uint32_t compare)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        stm32_putreg32(dev, STM32_GTIM_CCR1_OFFSET, compare);
        break;

      case 2:
        stm32_putreg32(dev, STM32_GTIM_CCR2_OFFSET, compare);
        break;

      case 3:
        stm32_putreg32(dev, STM32_GTIM_CCR3_OFFSET, compare);
        break;

      case 4:
        stm32_putreg32(dev, STM32_GTIM_CCR4_OFFSET, compare);
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int stm32_tim_getcapture(FAR struct stm32_tim_dev_s *dev,
                                uint8_t channel)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        return stm32_getreg32(dev, STM32_GTIM_CCR1_OFFSET);

      case 2:
        return stm32_getreg32(dev, STM32_GTIM_CCR2_OFFSET);

      case 3:
        return stm32_getreg32(dev, STM32_GTIM_CCR3_OFFSET);

      case 4:
        return stm32_getreg32(dev, STM32_GTIM_CCR4_OFFSET);
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct stm32_tim_dev_s *stm32_tim_init(int timer)
{
  struct stm32_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case 1:
        dev = (struct stm32_tim_dev_s *)&stm32_tim1_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case 2:
        dev = (struct stm32_tim_dev_s *)&stm32_tim2_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM2EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case 3:
        dev = (struct stm32_tim_dev_s *)&stm32_tim3_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM4
      case 4:
        dev = (struct stm32_tim_dev_s *)&stm32_tim4_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM4EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM5
      case 5:
        dev = (struct stm32_tim_dev_s *)&stm32_tim5_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM5EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case 6:
        dev = (struct stm32_tim_dev_s *)&stm32_tim6_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM6EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case 7:
        dev = (struct stm32_tim_dev_s *)&stm32_tim7_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM7EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM8
      case 8:
        dev = (struct stm32_tim_dev_s *)&stm32_tim8_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM12
      case 12:
        dev = (struct stm32_tim_dev_s *)&stm32_tim12_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM12EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM13
      case 13:
        dev = (struct stm32_tim_dev_s *)&stm32_tim13_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM13EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case 14:
        dev = (struct stm32_tim_dev_s *)&stm32_tim14_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM14EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case 15:
        dev = (struct stm32_tim_dev_s *)&stm32_tim15_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM15EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case 16:
        dev = (struct stm32_tim_dev_s *)&stm32_tim16_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM16EN);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case 17:
        dev = (struct stm32_tim_dev_s *)&stm32_tim17_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM17EN);
        break;
#endif
      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct stm32_tim_priv_s *)dev)->mode != STM32_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  stm32_tim_reset(dev);

  return dev;
}

/* TODO: Detach interrupts, and close down all TIM Channels */

int stm32_tim_deinit(FAR struct stm32_tim_dev_s * dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1
      case STM32_TIM1_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2
      case STM32_TIM2_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM2EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3
      case STM32_TIM3_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM4
      case STM32_TIM4_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM4EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM5
      case STM32_TIM5_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM5EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM6
      case STM32_TIM6_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM6EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM7
      case STM32_TIM7_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM7EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM8
      case STM32_TIM8_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM8EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM12
      case STM32_TIM12_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM12EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM13
      case STM32_TIM13_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM13EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14
      case STM32_TIM14_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM14EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15
      case STM32_TIM15_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM15EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16
      case STM32_TIM16_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM16EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17
      case STM32_TIM17_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM17EN, 0);
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32F0L0G0_TIM1 || ... || TIM17) */
