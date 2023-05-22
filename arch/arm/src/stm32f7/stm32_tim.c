/****************************************************************************
 * arch/arm/src/stm32f7/stm32_tim.c
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

/* Timer devices may be used for different purposes.
 *  Such special purposes include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_STM32F7_TIMn is defined then the CONFIG_STM32F7_TIMn_PWM may also
 *   be defined to indicate that the timer is intended to be used for pulsed
 *   output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32F7_TIMn is
 *   defined then CONFIG_STM32F7_TIMn_ADC may also be defined to indicate
 *   that timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32F7_TIMn is defined
 *   then CONFIG_STM32F7_TIMn_DAC may also be defined to indicate that timer
 *   "n" is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_STM32F7_TIMn is defined then
 *   CONFIG_STM32F7_TIMn_QE may also be defined to indicate that timer
 *   "n" is intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32F7_TIM1_PWM) || defined (CONFIG_STM32F7_TIM1_ADC) || \
    defined(CONFIG_STM32F7_TIM1_DAC) || defined(CONFIG_STM32F7_TIM1_QE)
#  undef CONFIG_STM32F7_TIM1
#endif
#if defined(CONFIG_STM32F7_TIM2_PWM) || defined (CONFIG_STM32F7_TIM2_ADC) || \
    defined(CONFIG_STM32F7_TIM2_DAC) || defined(CONFIG_STM32F7_TIM2_QE)
#  undef CONFIG_STM32F7_TIM2
#endif
#if defined(CONFIG_STM32F7_TIM3_PWM) || defined (CONFIG_STM32F7_TIM3_ADC) || \
    defined(CONFIG_STM32F7_TIM3_DAC) || defined(CONFIG_STM32F7_TIM3_QE)
#  undef CONFIG_STM32F7_TIM3
#endif
#if defined(CONFIG_STM32F7_TIM4_PWM) || defined (CONFIG_STM32F7_TIM4_ADC) || \
    defined(CONFIG_STM32F7_TIM4_DAC) || defined(CONFIG_STM32F7_TIM4_QE)
#  undef CONFIG_STM32F7_TIM4
#endif
#if defined(CONFIG_STM32F7_TIM5_PWM) || defined (CONFIG_STM32F7_TIM5_ADC) || \
    defined(CONFIG_STM32F7_TIM5_DAC) || defined(CONFIG_STM32F7_TIM5_QE)
#  undef CONFIG_STM32F7_TIM5
#endif
#if defined(CONFIG_STM32F7_TIM6_PWM) || defined (CONFIG_STM32F7_TIM6_ADC) || \
    defined(CONFIG_STM32F7_TIM6_DAC) || defined(CONFIG_STM32F7_TIM6_QE)
#  undef CONFIG_STM32F7_TIM6
#endif
#if defined(CONFIG_STM32F7_TIM7_PWM) || defined (CONFIG_STM32F7_TIM7_ADC) || \
    defined(CONFIG_STM32F7_TIM7_DAC) || defined(CONFIG_STM32F7_TIM7_QE)
#  undef CONFIG_STM32F7_TIM7
#endif
#if defined(CONFIG_STM32F7_TIM8_PWM) || defined (CONFIG_STM32F7_TIM8_ADC) || \
    defined(CONFIG_STM32F7_TIM8_DAC) || defined(CONFIG_STM32F7_TIM8_QE)
#  undef CONFIG_STM32F7_TIM8
#endif
#if defined(CONFIG_STM32F7_TIM9_PWM) || defined (CONFIG_STM32F7_TIM9_ADC) || \
    defined(CONFIG_STM32F7_TIM9_DAC) || defined(CONFIG_STM32F7_TIM9_QE)
#  undef CONFIG_STM32F7_TIM9
#endif
#if defined(CONFIG_STM32F7_TIM10_PWM) || defined (CONFIG_STM32F7_TIM10_ADC) || \
    defined(CONFIG_STM32F7_TIM10_DAC) || defined(CONFIG_STM32F7_TIM10_QE)
#  undef CONFIG_STM32F7_TIM10
#endif
#if defined(CONFIG_STM32F7_TIM11_PWM) || defined (CONFIG_STM32F7_TIM11_ADC) || \
    defined(CONFIG_STM32F7_TIM11_DAC) || defined(CONFIG_STM32F7_TIM11_QE)
#  undef CONFIG_STM32F7_TIM11
#endif
#if defined(CONFIG_STM32F7_TIM12_PWM) || defined (CONFIG_STM32F7_TIM12_ADC) || \
    defined(CONFIG_STM32F7_TIM12_DAC) || defined(CONFIG_STM32F7_TIM12_QE)
#  undef CONFIG_STM32F7_TIM12
#endif
#if defined(CONFIG_STM32F7_TIM13_PWM) || defined (CONFIG_STM32F7_TIM13_ADC) || \
    defined(CONFIG_STM32F7_TIM13_DAC) || defined(CONFIG_STM32F7_TIM13_QE)
#  undef CONFIG_STM32F7_TIM13
#endif
#if defined(CONFIG_STM32F7_TIM14_PWM) || defined (CONFIG_STM32F7_TIM14_ADC) || \
    defined(CONFIG_STM32F7_TIM14_DAC) || defined(CONFIG_STM32F7_TIM14_QE)
#  undef CONFIG_STM32F7_TIM14
#endif

#if defined(CONFIG_STM32F7_TIM1)
#  if defined(GPIO_TIM1_CH1OUT) ||defined(GPIO_TIM1_CH2OUT)||\
      defined(GPIO_TIM1_CH3OUT) ||defined(GPIO_TIM1_CH4OUT)
#    define HAVE_TIM1_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM2)
#  if defined(GPIO_TIM2_CH1OUT) ||defined(GPIO_TIM2_CH2OUT)||\
      defined(GPIO_TIM2_CH3OUT) ||defined(GPIO_TIM2_CH4OUT)
#    define HAVE_TIM2_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM3)
#  if defined(GPIO_TIM3_CH1OUT) ||defined(GPIO_TIM3_CH2OUT)||\
      defined(GPIO_TIM3_CH3OUT) ||defined(GPIO_TIM3_CH4OUT)
#    define HAVE_TIM3_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM4)
#  if defined(GPIO_TIM4_CH1OUT) ||defined(GPIO_TIM4_CH2OUT)||\
      defined(GPIO_TIM4_CH3OUT) ||defined(GPIO_TIM4_CH4OUT)
#    define HAVE_TIM4_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM5)
#  if defined(GPIO_TIM5_CH1OUT) ||defined(GPIO_TIM5_CH2OUT)||\
      defined(GPIO_TIM5_CH3OUT) ||defined(GPIO_TIM5_CH4OUT)
#    define HAVE_TIM5_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM8)
#  if defined(GPIO_TIM8_CH1OUT) ||defined(GPIO_TIM8_CH2OUT)||\
      defined(GPIO_TIM8_CH3OUT) ||defined(GPIO_TIM8_CH4OUT)
#    define HAVE_TIM8_GPIOCONFIG 1
#endif
#endif
#if defined(CONFIG_STM32F7_TIM9)
#  if defined(GPIO_TIM9_CH1OUT) ||defined(GPIO_TIM9_CH2OUT)||\
      defined(GPIO_TIM9_CH3OUT) ||defined(GPIO_TIM9_CH4OUT)
#    define HAVE_TIM9_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM10)
#  if defined(GPIO_TIM10_CH1OUT) ||defined(GPIO_TIM10_CH2OUT)||\
      defined(GPIO_TIM10_CH3OUT) ||defined(GPIO_TIM10_CH4OUT)
#    define HAVE_TIM10_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM11)
#  if defined(GPIO_TIM11_CH1OUT) ||defined(GPIO_TIM11_CH2OUT)||\
      defined(GPIO_TIM11_CH3OUT) ||defined(GPIO_TIM11_CH4OUT)
#    define HAVE_TIM11_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM12)
#  if defined(GPIO_TIM12_CH1OUT) ||defined(GPIO_TIM12_CH2OUT)||\
      defined(GPIO_TIM12_CH3OUT) ||defined(GPIO_TIM12_CH4OUT)
#    define HAVE_TIM12_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM13)
#  if defined(GPIO_TIM13_CH1OUT) ||defined(GPIO_TIM13_CH2OUT)||\
      defined(GPIO_TIM13_CH3OUT) ||defined(GPIO_TIM13_CH4OUT)
#    define HAVE_TIM13_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_STM32F7_TIM14)
#  if defined(GPIO_TIM14_CH1OUT) ||defined(GPIO_TIM14_CH2OUT)||\
      defined(GPIO_TIM14_CH3OUT) ||defined(GPIO_TIM14_CH4OUT)
#    define HAVE_TIM14_GPIOCONFIG 1
#endif
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32F7_TIM1)  || defined(CONFIG_STM32F7_TIM2)  || \
    defined(CONFIG_STM32F7_TIM3)  || defined(CONFIG_STM32F7_TIM4)  || \
    defined(CONFIG_STM32F7_TIM5)  || defined(CONFIG_STM32F7_TIM6)  || \
    defined(CONFIG_STM32F7_TIM7)  || defined(CONFIG_STM32F7_TIM8)  || \
    defined(CONFIG_STM32F7_TIM9)  || defined(CONFIG_STM32F7_TIM10) || \
    defined(CONFIG_STM32F7_TIM11) || defined(CONFIG_STM32F7_TIM12) || \
    defined(CONFIG_STM32F7_TIM13) || defined(CONFIG_STM32F7_TIM14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct stm32_tim_priv_s
{
  struct stm32_tim_ops_s *ops;
  stm32_tim_mode_t        mode;
  uint32_t                base;   /* TIMn base address */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg16(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(struct stm32_tim_dev_s *dev,
                                  uint8_t offset,
                                  uint16_t value)
{
  putreg16(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(struct stm32_tim_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct stm32_tim_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/* Get a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline uint32_t stm32_getreg32(struct stm32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(((struct stm32_tim_priv_s *)dev)->base + offset);
}

/* Put a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline void stm32_putreg32(struct stm32_tim_dev_s *dev,
                                  uint8_t offset,
                                  uint32_t value)
{
  putreg32(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

static void stm32_tim_reload_counter(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  stm32_putreg16(dev, STM32_GTIM_EGR_OFFSET, val);
}

static void stm32_tim_enable(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);
  val |= GTIM_CR1_CEN;
  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

static void stm32_tim_disable(struct stm32_tim_dev_s *dev)
{
  uint16_t val = stm32_getreg16(dev, STM32_GTIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: stm32_tim_getwidth
 ****************************************************************************/

static int stm32_tim_getwidth(struct stm32_tim_dev_s *dev)
{
  /* Only TIM2 and TIM5 timers may be 32-bits in width
   *
   * Reference Table 2 of en.DM00042534.pdf
   */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
      /* TIM2 is 32-bits on all except F10x, L0x, and L1x lines */

#if defined(CONFIG_STM32F7_TIM2)
      case STM32_TIM2_BASE:
        return 32;
#endif

      /* TIM5 is 32-bits on all except F10x lines */

#if defined(CONFIG_STM32F7_TIM5)
      case STM32_TIM5_BASE:
        return 32;
#endif

      /* All others are 16-bit times */

      default:
        return 16;
    }
}

/****************************************************************************
 * Name: stm32_tim_getcounter
 ****************************************************************************/

static uint32_t stm32_tim_getcounter(struct stm32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return stm32_tim_getwidth(dev) > 16 ?
    stm32_getreg32(dev, STM32_GTIM_CNT_OFFSET) :
    (uint32_t)stm32_getreg16(dev, STM32_GTIM_CNT_OFFSET);
}

/****************************************************************************
 * Name: stm32_tim_setcounter
 ****************************************************************************/

static void stm32_tim_setcounter(struct stm32_tim_dev_s *dev,
                                 uint32_t count)
{
  DEBUGASSERT(dev != NULL);

  if (stm32_tim_getwidth(dev) > 16)
    {
      stm32_putreg32(dev, STM32_GTIM_CNT_OFFSET, count);
    }
  else
    {
      stm32_putreg16(dev, STM32_GTIM_CNT_OFFSET, (uint16_t)count);
    }
}

/* Reset timer into system default state,
 * but do not affect output/input pins
 */

static void stm32_tim_reset(struct stm32_tim_dev_s *dev)
{
  ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_DISABLED;
  stm32_tim_disable(dev);
}

#if defined(HAVE_TIM1_GPIOCONFIG)||defined(HAVE_TIM2_GPIOCONFIG)||\
    defined(HAVE_TIM3_GPIOCONFIG)||defined(HAVE_TIM4_GPIOCONFIG)||\
    defined(HAVE_TIM5_GPIOCONFIG)||defined(HAVE_TIM8_GPIOCONFIG)
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

static int stm32_tim_setclock(struct stm32_tim_dev_s *dev, uint32_t freq)
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
#ifdef CONFIG_STM32F7_TIM1
      case STM32_TIM1_BASE:
        freqin = STM32_APB2_TIM1_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2
      case STM32_TIM2_BASE:
        freqin = STM32_APB1_TIM2_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3
      case STM32_TIM3_BASE:
        freqin = STM32_APB1_TIM3_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4
      case STM32_TIM4_BASE:
        freqin = STM32_APB1_TIM4_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5
      case STM32_TIM5_BASE:
        freqin = STM32_APB1_TIM5_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM6
      case STM32_TIM6_BASE:
        freqin = STM32_APB1_TIM6_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM7
      case STM32_TIM7_BASE:
        freqin = STM32_APB1_TIM7_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8
      case STM32_TIM8_BASE:
        freqin = STM32_APB2_TIM8_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9
      case STM32_TIM9_BASE:
        freqin = STM32_APB2_TIM9_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10
      case STM32_TIM10_BASE:
        freqin = STM32_APB2_TIM10_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11
      case STM32_TIM11_BASE:
        freqin = STM32_APB2_TIM11_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12
      case STM32_TIM12_BASE:
        freqin = STM32_APB1_TIM12_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13
      case STM32_TIM13_BASE:
        freqin = STM32_APB1_TIM13_CLKIN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14
      case STM32_TIM14_BASE:
        freqin = STM32_APB1_TIM14_CLKIN;
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

  stm32_putreg16(dev, STM32_GTIM_PSC_OFFSET, prescaler);
  stm32_tim_enable(dev);

  return prescaler;
}

static void stm32_tim_setperiod(struct stm32_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  stm32_putreg32(dev, STM32_GTIM_ARR_OFFSET, period);
}

static int stm32_tim_setisr(struct stm32_tim_dev_s *dev,
                            xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F7_TIM1
      case STM32_TIM1_BASE:
        vectorno = STM32_IRQ_TIM1UP;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2
      case STM32_TIM2_BASE:
        vectorno = STM32_IRQ_TIM2;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3
      case STM32_TIM3_BASE:
        vectorno = STM32_IRQ_TIM3;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4
      case STM32_TIM4_BASE:
        vectorno = STM32_IRQ_TIM4;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5
      case STM32_TIM5_BASE:
        vectorno = STM32_IRQ_TIM5;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM6
      case STM32_TIM6_BASE:
        vectorno = STM32_IRQ_TIM6;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM7
      case STM32_TIM7_BASE:
        vectorno = STM32_IRQ_TIM7;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8
      case STM32_TIM8_BASE:
        vectorno = STM32_IRQ_TIM8UP;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9
      case STM32_TIM9_BASE:
        vectorno = STM32_IRQ_TIM9;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10
      case STM32_TIM10_BASE:
        vectorno = STM32_IRQ_TIM10;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11
      case STM32_TIM11_BASE:
        vectorno = STM32_IRQ_TIM11;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12
      case STM32_TIM12_BASE:
        vectorno = STM32_IRQ_TIM12;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13
      case STM32_TIM13_BASE:
        vectorno = STM32_IRQ_TIM13;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14
      case STM32_TIM14_BASE:
        vectorno = STM32_IRQ_TIM14;
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

static void stm32_tim_enableint(struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, 0, GTIM_DIER_UIE);
}

static void stm32_tim_disableint(struct stm32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  stm32_modifyreg16(dev, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE, 0);
}

static int stm32_tim_checkint(struct stm32_tim_dev_s *dev, int source)
{
  uint16_t regval = stm32_getreg16(dev, STM32_GTIM_SR_OFFSET);
  return (regval & source) ? 1 : 0;
}

static void stm32_tim_ackint(struct stm32_tim_dev_s *dev, int source)
{
  stm32_putreg16(dev, STM32_GTIM_SR_OFFSET, ~source);
}

/****************************************************************************
 * General Functions
 ****************************************************************************/

static int stm32_tim_setmode(struct stm32_tim_dev_s *dev,
                             stm32_tim_mode_t mode)
{
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

        /* Our default:
         * Interrupts are generated on compare, when counting down
         */

        val |= GTIM_CR1_CENTER1;
        break;

      case STM32_TIM_MODE_PULSE:
        val |= GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  stm32_tim_reload_counter(dev);
  stm32_putreg16(dev, STM32_GTIM_CR1_OFFSET, val);

  /* Advanced registers require Main Output Enable */

    if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM1_BASE ||
        ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM8_BASE)
      {
        stm32_modifyreg16(dev, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
      }

  return OK;
}

static int stm32_tim_setchannel(struct stm32_tim_dev_s *dev,
                                uint8_t channel,
                                stm32_tim_channel_t mode)
{
  uint16_t ccmr_orig   = 0;
  uint16_t ccmr_val    = 0;
  uint16_t ccmr_mask   = 0xff;
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

      case STM32_TIM_CH_OUTTOGGLE:
          ccmr_val  = (GTIM_CCMR_MODE_OCREFTOG << GTIM_CCMR1_OC1M_SHIFT) +
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
#ifdef CONFIG_STM32F7_TIM1
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
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2
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
#ifdef CONFIG_STM32F7_TIM3
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
#ifdef CONFIG_STM32F7_TIM4
      case STM32_TIM4_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM4_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM4_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM4_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM4_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM4_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM4_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM4_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM4_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5
      case STM32_TIM5_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM5_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM5_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM5_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM5_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM5_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM5_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM5_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM5_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8
      case STM32_TIM8_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM8_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM8_CH1OUT, mode); break;
#  endif
#  if defined(GPIO_TIM8_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM8_CH2OUT, mode); break;
#  endif
#  if defined(GPIO_TIM8_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM8_CH3OUT, mode); break;
#  endif
#  if defined(GPIO_TIM8_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM8_CH4OUT, mode); break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#  ifdef CONFIG_STM32F7_TIM9
      case STM32_TIM9_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM9_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM9_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM9_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM9_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM9_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM9_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM9_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM9_CH4OUT, mode);
              break;
#  endif
      default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10
      case STM32_TIM10_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM10_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM10_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM10_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM10_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM10_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM10_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM10_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM10_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11
      case STM32_TIM11_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM11_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM11_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM11_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM11_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM11_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM11_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM11_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM11_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12
      case STM32_TIM12_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM12_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM12_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM12_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM12_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM12_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM12_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM12_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM12_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13
      case STM32_TIM13_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM13_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM13_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM13_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM13_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM13_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM13_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM13_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM13_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14
      case STM32_TIM14_BASE:
        switch (channel)
          {
#  if defined(GPIO_TIM14_CH1OUT)
            case 0:
              stm32_tim_gpioconfig(GPIO_TIM14_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM14_CH2OUT)
            case 1:
              stm32_tim_gpioconfig(GPIO_TIM14_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM14_CH3OUT)
            case 2:
              stm32_tim_gpioconfig(GPIO_TIM14_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_TIM14_CH4OUT)
            case 3:
              stm32_tim_gpioconfig(GPIO_TIM14_CH4OUT, mode);
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

static int stm32_tim_setcompare(struct stm32_tim_dev_s *dev,
                                uint8_t channel,
                                uint32_t compare)
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

static int stm32_tim_getcapture(struct stm32_tim_dev_s *dev,
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
 * Advanced Functions
 ****************************************************************************/

/* TODO: Advanced functions for the STM32_ATIM */

/****************************************************************************
 * Device Structures, Instantiation
 ****************************************************************************/

struct stm32_tim_ops_s stm32_tim_ops =
{
  .setmode    = stm32_tim_setmode,
  .setclock   = stm32_tim_setclock,
  .setperiod  = stm32_tim_setperiod,
  .getcounter = stm32_tim_getcounter,
  .setcounter = stm32_tim_setcounter,
  .getwidth   = stm32_tim_getwidth,
  .setchannel = stm32_tim_setchannel,
  .setcompare = stm32_tim_setcompare,
  .getcapture = stm32_tim_getcapture,
  .setisr     = stm32_tim_setisr,
  .enableint  = stm32_tim_enableint,
  .disableint = stm32_tim_disableint,
  .ackint     = stm32_tim_ackint,
  .checkint   = stm32_tim_checkint,
};

#ifdef CONFIG_STM32F7_TIM1
struct stm32_tim_priv_s stm32_tim1_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM1_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM2
struct stm32_tim_priv_s stm32_tim2_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM2_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM3
struct stm32_tim_priv_s stm32_tim3_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM3_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM4
struct stm32_tim_priv_s stm32_tim4_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM4_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM5
struct stm32_tim_priv_s stm32_tim5_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM5_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM6
struct stm32_tim_priv_s stm32_tim6_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM6_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM7
struct stm32_tim_priv_s stm32_tim7_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM7_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM8
struct stm32_tim_priv_s stm32_tim8_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM8_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM9
struct stm32_tim_priv_s stm32_tim9_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM9_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM10
struct stm32_tim_priv_s stm32_tim10_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM10_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM11
struct stm32_tim_priv_s stm32_tim11_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM11_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM12
struct stm32_tim_priv_s stm32_tim12_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM12_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM13
struct stm32_tim_priv_s stm32_tim13_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM13_BASE,
};
#endif

#ifdef CONFIG_STM32F7_TIM14
struct stm32_tim_priv_s stm32_tim14_priv =
{
  .ops        = &stm32_tim_ops,
  .mode       = STM32_TIM_MODE_UNUSED,
  .base       = STM32_TIM14_BASE,
};
#endif

/****************************************************************************
 * Public Function - Initialization
 ****************************************************************************/

struct stm32_tim_dev_s *stm32_tim_init(int timer)
{
  struct stm32_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_STM32F7_TIM1
      case 1:
        dev = (struct stm32_tim_dev_s *)&stm32_tim1_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2
      case 2:
        dev = (struct stm32_tim_dev_s *)&stm32_tim2_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM2EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3
      case 3:
        dev = (struct stm32_tim_dev_s *)&stm32_tim3_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4
      case 4:
        dev = (struct stm32_tim_dev_s *)&stm32_tim4_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM4EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5
      case 5:
        dev = (struct stm32_tim_dev_s *)&stm32_tim5_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM5EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM6
      case 6:
        dev = (struct stm32_tim_dev_s *)&stm32_tim6_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM6EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM7
      case 7:
        dev = (struct stm32_tim_dev_s *)&stm32_tim7_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM7EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8
      case 8:
        dev = (struct stm32_tim_dev_s *)&stm32_tim8_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9
      case 9:
        dev = (struct stm32_tim_dev_s *)&stm32_tim9_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM9EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10
      case 10:
        dev = (struct stm32_tim_dev_s *)&stm32_tim10_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM10EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11
      case 11:
        dev = (struct stm32_tim_dev_s *)&stm32_tim11_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM11EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12
      case 12:
        dev = (struct stm32_tim_dev_s *)&stm32_tim12_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM12EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13
      case 13:
        dev = (struct stm32_tim_dev_s *)&stm32_tim13_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM13EN);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14
      case 14:
        dev = (struct stm32_tim_dev_s *)&stm32_tim14_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM14EN);
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

int stm32_tim_deinit(struct stm32_tim_dev_s * dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct stm32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_STM32F7_TIM1
      case STM32_TIM1_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2
      case STM32_TIM2_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM2EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3
      case STM32_TIM3_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4
      case STM32_TIM4_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM4EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5
      case STM32_TIM5_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM5EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM6
      case STM32_TIM6_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM6EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM7
      case STM32_TIM7_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM7EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8
      case STM32_TIM8_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM8EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9
      case STM32_TIM9_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM9EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10
      case STM32_TIM10_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM10EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11
      case STM32_TIM11_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM11EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12
      case STM32_TIM12_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM12EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13
      case STM32_TIM13_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM13EN, 0);
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14
      case STM32_TIM14_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM14EN, 0);
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_STM32F7_TIM1 || ... || TIM8) */
