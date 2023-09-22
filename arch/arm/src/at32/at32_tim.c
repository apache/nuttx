/****************************************************************************
 * arch/arm/src/at32/at32_tim.c
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
#include "at32.h"
#include "at32_gpio.h"
#include "at32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_AT32_TIMn is defined then the CONFIG_AT32_TIMn_PWM may also be
 *   defined to indicate that the timer is intended to be used for pulsed
 *   output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_AT32_TIMn is
 *   defined then CONFIG_AT32_TIMn_ADC may also be defined to indicate that
 *   timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_AT32_TIMn is defined then
 *   CONFIG_AT32_TIMn_DAC may also be defined to indicate that timer "n" is
 *   intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_AT32_TIMn is defined then
 *   CONFIG_AT32_TIMn_QE may also be defined to indicate that timer "n" is
 *   intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_AT32_TIM1_PWM) || defined (CONFIG_AT32_TIM1_ADC) || \
    defined(CONFIG_AT32_TIM1_DAC) || defined(CONFIG_AT32_TIM1_QE) || \
    defined(CONFIG_AT32_TIM1_CAP)
#  undef CONFIG_AT32_TIM1
#endif
#if defined(CONFIG_AT32_TIM2_PWM) || defined (CONFIG_AT32_TIM2_ADC) || \
    defined(CONFIG_AT32_TIM2_DAC) || defined(CONFIG_AT32_TIM2_QE) || \
    defined(CONFIG_AT32_TIM2_CAP)
#  undef CONFIG_AT32_TIM2
#endif
#if defined(CONFIG_AT32_TIM3_PWM) || defined (CONFIG_AT32_TIM3_ADC) || \
    defined(CONFIG_AT32_TIM3_DAC) || defined(CONFIG_AT32_TIM3_QE) || \
    defined(CONFIG_AT32_TIM3_CAP)
#  undef CONFIG_AT32_TIM3
#endif
#if defined(CONFIG_AT32_TIM4_PWM) || defined (CONFIG_AT32_TIM4_ADC) || \
    defined(CONFIG_AT32_TIM4_DAC) || defined(CONFIG_AT32_TIM4_QE) || \
    defined(CONFIG_AT32_TIM4_CAP)
#  undef CONFIG_AT32_TIM4
#endif
#if defined(CONFIG_AT32_TIM5_PWM) || defined (CONFIG_AT32_TIM5_ADC) || \
    defined(CONFIG_AT32_TIM5_DAC) || defined(CONFIG_AT32_TIM5_QE) || \
    defined(CONFIG_AT32_TIM5_CAP)
#  undef CONFIG_AT32_TIM5
#endif
#if defined(CONFIG_AT32_TIM6_PWM) || defined (CONFIG_AT32_TIM6_ADC) || \
    defined(CONFIG_AT32_TIM6_DAC) || defined(CONFIG_AT32_TIM6_QE)
#  undef CONFIG_AT32_TIM6
#endif
#if defined(CONFIG_AT32_TIM7_PWM) || defined (CONFIG_AT32_TIM7_ADC) || \
    defined(CONFIG_AT32_TIM7_DAC) || defined(CONFIG_AT32_TIM7_QE)
#  undef CONFIG_AT32_TIM7
#endif
#if defined(CONFIG_AT32_TIM8_PWM) || defined (CONFIG_AT32_TIM8_ADC) || \
    defined(CONFIG_AT32_TIM8_DAC) || defined(CONFIG_AT32_TIM8_QE) || \
    defined(CONFIG_AT32_TIM8_CAP)
#  undef CONFIG_AT32_TIM8
#endif
#if defined(CONFIG_AT32_TIM9_PWM) || defined (CONFIG_AT32_TIM9_ADC) || \
    defined(CONFIG_AT32_TIM9_DAC) || defined(CONFIG_AT32_TIM9_QE) || \
    defined(CONFIG_AT32_TIM9_CAP)
#  undef CONFIG_AT32_TIM9
#endif
#if defined(CONFIG_AT32_TIM10_PWM) || defined (CONFIG_AT32_TIM10_ADC) || \
    defined(CONFIG_AT32_TIM10_DAC) || defined(CONFIG_AT32_TIM10_QE) || \
    defined(CONFIG_AT32_TIM10_CAP)
#  undef CONFIG_AT32_TIM10
#endif
#if defined(CONFIG_AT32_TIM11_PWM) || defined (CONFIG_AT32_TIM11_ADC) || \
    defined(CONFIG_AT32_TIM11_DAC) || defined(CONFIG_AT32_TIM11_QE) || \
    defined(CONFIG_AT32_TIM11_CAP)
#  undef CONFIG_AT32_TIM11
#endif
#if defined(CONFIG_AT32_TIM12_PWM) || defined (CONFIG_AT32_TIM12_ADC) || \
    defined(CONFIG_AT32_TIM12_DAC) || defined(CONFIG_AT32_TIM12_QE) || \
    defined(CONFIG_AT32_TIM12_CAP)
#  undef CONFIG_AT32_TIM12
#endif
#if defined(CONFIG_AT32_TIM13_PWM) || defined (CONFIG_AT32_TIM13_ADC) || \
    defined(CONFIG_AT32_TIM13_DAC) || defined(CONFIG_AT32_TIM13_QE) || \
    defined(CONFIG_AT32_TIM13_CAP)
#  undef CONFIG_AT32_TIM13
#endif
#if defined(CONFIG_AT32_TIM14_PWM) || defined (CONFIG_AT32_TIM14_ADC) || \
    defined(CONFIG_AT32_TIM14_DAC) || defined(CONFIG_AT32_TIM14_QE) || \
    defined(CONFIG_AT32_TIM14_CAP)
#  undef CONFIG_AT32_TIM14
#endif
#if defined(CONFIG_AT32_TIM20_PWM) || defined (CONFIG_AT32_TIM20_ADC) || \
    defined(CONFIG_AT32_TIM20_DAC) || defined(CONFIG_AT32_TIM20_QE) || \
    defined(CONFIG_AT32_TIM20_CAP)
#  undef CONFIG_AT32_TIM20
#endif

#undef HAVE_TIM_GPIOCONFIG
#if defined(CONFIG_AT32_TIM1)
#  if defined(GPIO_TIM1_CH1OUT) ||defined(GPIO_TIM1_CH2OUT)||\
      defined(GPIO_TIM1_CH3OUT) ||defined(GPIO_TIM1_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM1_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM2)
#  if defined(GPIO_TIM2_CH1OUT) ||defined(GPIO_TIM2_CH2OUT)||\
      defined(GPIO_TIM2_CH3OUT) ||defined(GPIO_TIM2_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM2_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM3)
#  if defined(GPIO_TIM3_CH1OUT) ||defined(GPIO_TIM3_CH2OUT)||\
      defined(GPIO_TIM3_CH3OUT) ||defined(GPIO_TIM3_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM3_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM4)
#  if defined(GPIO_TIM4_CH1OUT) ||defined(GPIO_TIM4_CH2OUT)||\
      defined(GPIO_TIM4_CH3OUT) ||defined(GPIO_TIM4_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM4_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM5)
#  if defined(GPIO_TIM5_CH1OUT) ||defined(GPIO_TIM5_CH2OUT)||\
      defined(GPIO_TIM5_CH3OUT) ||defined(GPIO_TIM5_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM5_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM8)
#  if defined(GPIO_TIM8_CH1OUT) ||defined(GPIO_TIM8_CH2OUT)||\
      defined(GPIO_TIM8_CH3OUT) ||defined(GPIO_TIM8_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM8_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM9)
#  if defined(GPIO_TIM9_CH1OUT) ||defined(GPIO_TIM9_CH2OUT)||\
      defined(GPIO_TIM9_CH3OUT) ||defined(GPIO_TIM9_CH4OUT)
#    define HAVE_TIM9_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM10)
#  if defined(GPIO_TIM10_CH1OUT) ||defined(GPIO_TIM10_CH2OUT)||\
      defined(GPIO_TIM10_CH3OUT) ||defined(GPIO_TIM10_CH4OUT)
#    define HAVE_TIM10_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM11)
#  if defined(GPIO_TIM11_CH1OUT) ||defined(GPIO_TIM11_CH2OUT)||\
      defined(GPIO_TIM11_CH3OUT) ||defined(GPIO_TIM11_CH4OUT)
#    define HAVE_TIM11_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM12)
#  if defined(GPIO_TIM12_CH1OUT) ||defined(GPIO_TIM12_CH2OUT)||\
      defined(GPIO_TIM12_CH3OUT) ||defined(GPIO_TIM12_CH4OUT)
#    define HAVE_TIM12_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM13)
#  if defined(GPIO_TIM13_CH1OUT) ||defined(GPIO_TIM13_CH2OUT)||\
      defined(GPIO_TIM13_CH3OUT) ||defined(GPIO_TIM13_CH4OUT)
#    define HAVE_TIM13_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM14)
#  if defined(GPIO_TIM14_CH1OUT) ||defined(GPIO_TIM14_CH2OUT)||\
      defined(GPIO_TIM14_CH3OUT) ||defined(GPIO_TIM14_CH4OUT)
#    define HAVE_TIM14_GPIOCONFIG 1
#endif
#endif

#if defined(CONFIG_AT32_TIM20)
#  if defined(GPIO_TIM20_CH1OUT) ||defined(GPIO_TIM20_CH2OUT)||\
      defined(GPIO_TIM20_CH3OUT) ||defined(GPIO_TIM20_CH4OUT)
#    undef  HAVE_TIM_GPIOCONFIG
#    define HAVE_TIM_GPIOCONFIG  1
#    define HAVE_TIM20_GPIOCONFIG 1
#endif
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_AT32_TIM1)  || defined(CONFIG_AT32_TIM2)  || \
    defined(CONFIG_AT32_TIM3)  || defined(CONFIG_AT32_TIM4)  || \
    defined(CONFIG_AT32_TIM5)  || defined(CONFIG_AT32_TIM6)  || \
    defined(CONFIG_AT32_TIM7)  || defined(CONFIG_AT32_TIM8)  || \
    defined(CONFIG_AT32_TIM9)  || defined(CONFIG_AT32_TIM10) || \
    defined(CONFIG_AT32_TIM11) || defined(CONFIG_AT32_TIM12) || \
    defined(CONFIG_AT32_TIM13) || defined(CONFIG_AT32_TIM14) || \
    defined(CONFIG_AT32_TIM20) 

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct at32_tim_priv_s
{
  const struct at32_tim_ops_s *ops;
  at32_tim_mode_t mode;
  uint32_t base;                      /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Register helpers */

static inline uint16_t at32_getreg16(struct at32_tim_dev_s *dev,
                                      uint8_t offset);
static inline void at32_putreg16(struct at32_tim_dev_s *dev,
                                  uint8_t offset, uint16_t value);
static inline void at32_modifyreg16(struct at32_tim_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits);
static inline uint32_t at32_getreg32(struct at32_tim_dev_s *dev,
                                      uint8_t offset);
static inline void at32_putreg32(struct at32_tim_dev_s *dev,
                                  uint8_t offset, uint32_t value);

/* Timer helpers */

static void at32_tim_reload_counter(struct at32_tim_dev_s *dev);
static void at32_tim_enable(struct at32_tim_dev_s *dev);
static void at32_tim_disable(struct at32_tim_dev_s *dev);
static void at32_tim_reset(struct at32_tim_dev_s *dev);

#ifdef HAVE_TIM_GPIOCONFIG
static void at32_tim_gpioconfig(uint32_t cfg, at32_tim_channel_t mode);
#endif

/* Timer methods */

static int  at32_tim_setmode(struct at32_tim_dev_s *dev,
                              at32_tim_mode_t mode);
static int  at32_tim_setclock(struct at32_tim_dev_s *dev,
                               uint32_t freq);
static void at32_tim_setperiod(struct at32_tim_dev_s *dev,
                                uint32_t period);
static uint32_t at32_tim_getcounter(struct at32_tim_dev_s *dev);
static void at32_tim_setcounter(struct at32_tim_dev_s *dev,
                                 uint32_t count);
static int  at32_tim_getwidth(struct at32_tim_dev_s *dev);
static int  at32_tim_setchannel(struct at32_tim_dev_s *dev,
                                 uint8_t channel, at32_tim_channel_t mode);
static int  at32_tim_setcompare(struct at32_tim_dev_s *dev,
                                 uint8_t channel, uint32_t compare);
static int  at32_tim_getcapture(struct at32_tim_dev_s *dev,
                                 uint8_t channel);
static int  at32_tim_setisr(struct at32_tim_dev_s *dev,
                             xcpt_t handler, void *arg, int source);
static void at32_tim_enableint(struct at32_tim_dev_s *dev,
                                int source);
static void at32_tim_disableint(struct at32_tim_dev_s *dev,
                                 int source);
static void at32_tim_ackint(struct at32_tim_dev_s *dev, int source);
static int  at32_tim_checkint(struct at32_tim_dev_s *dev, int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at32_tim_ops_s at32_tim_ops =
{
  .setmode    = at32_tim_setmode,
  .setclock   = at32_tim_setclock,
  .setperiod  = at32_tim_setperiod,
  .getcounter = at32_tim_getcounter,
  .setcounter = at32_tim_setcounter,
  .getwidth   = at32_tim_getwidth,
  .setchannel = at32_tim_setchannel,
  .setcompare = at32_tim_setcompare,
  .getcapture = at32_tim_getcapture,
  .setisr     = at32_tim_setisr,
  .enableint  = at32_tim_enableint,
  .disableint = at32_tim_disableint,
  .ackint     = at32_tim_ackint,
  .checkint   = at32_tim_checkint,
};

#ifdef CONFIG_AT32_TIM1
struct at32_tim_priv_s at32_tim1_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR1_BASE,
};
#endif
#ifdef CONFIG_AT32_TIM2
struct at32_tim_priv_s at32_tim2_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR2_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM3
struct at32_tim_priv_s at32_tim3_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR3_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM4
struct at32_tim_priv_s at32_tim4_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR4_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM5
struct at32_tim_priv_s at32_tim5_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR5_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM6
struct at32_tim_priv_s at32_tim6_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR6_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM7
struct at32_tim_priv_s at32_tim7_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR7_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM8
struct at32_tim_priv_s at32_tim8_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR8_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM9
struct at32_tim_priv_s at32_tim9_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR9_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM10
struct at32_tim_priv_s at32_tim10_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR10_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM11
struct at32_tim_priv_s at32_tim11_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR11_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM12
struct at32_tim_priv_s at32_tim12_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR12_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM13
struct at32_tim_priv_s at32_tim13_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR13_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM14
struct at32_tim_priv_s at32_tim14_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR14_BASE,
};
#endif

#ifdef CONFIG_AT32_TIM20
struct at32_tim_priv_s at32_tim20_priv =
{
  .ops        = &at32_tim_ops,
  .mode       = AT32_TIM_MODE_UNUSED,
  .base       = AT32_TMR20_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t at32_getreg16(struct at32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg16(((struct at32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: at32_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void at32_putreg16(struct at32_tim_dev_s *dev,
                                  uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct at32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: at32_modifyreg16
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void at32_modifyreg16(struct at32_tim_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct at32_tim_priv_s *)dev)->base + offset,
              clearbits, setbits);
}

/****************************************************************************
 * Name: at32_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset.  This applies only for the AT32
 *   F4 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline uint32_t at32_getreg32(struct at32_tim_dev_s *dev,
                                      uint8_t offset)
{
  return getreg32(((struct at32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: at32_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset.  This applies only for the AT32
 *   F4 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline void at32_putreg32(struct at32_tim_dev_s *dev,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct at32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: at32_tim_reload_counter
 ****************************************************************************/

static void at32_tim_reload_counter(struct at32_tim_dev_s *dev)
{
  uint16_t val = at32_getreg16(dev, AT32_GTIM_EGR_OFFSET);
  val |= GTIM_EGR_UG;
  at32_putreg16(dev, AT32_GTIM_EGR_OFFSET, val);
}

/****************************************************************************
 * Name: at32_tim_enable
 ****************************************************************************/

static void at32_tim_enable(struct at32_tim_dev_s *dev)
{
  uint16_t val = at32_getreg16(dev, AT32_GTIM_CR1_OFFSET);
  val |= GTIM_CR1_CEN;
  at32_tim_reload_counter(dev);
  at32_putreg16(dev, AT32_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: at32_tim_disable
 ****************************************************************************/

static void at32_tim_disable(struct at32_tim_dev_s *dev)
{
  uint16_t val = at32_getreg16(dev, AT32_GTIM_CR1_OFFSET);
  val &= ~GTIM_CR1_CEN;
  at32_putreg16(dev, AT32_GTIM_CR1_OFFSET, val);
}

/****************************************************************************
 * Name: at32_tim_reset
 *
 * Description:
 *   Reset timer into system default state, but do not affect output/input
 *   pins
 *
 ****************************************************************************/

static void at32_tim_reset(struct at32_tim_dev_s *dev)
{
  ((struct at32_tim_priv_s *)dev)->mode = AT32_TIM_MODE_DISABLED;
  at32_tim_disable(dev);
}

/****************************************************************************
 * Name: at32_tim_gpioconfig
 ****************************************************************************/

#ifdef HAVE_TIM_GPIOCONFIG
static void at32_tim_gpioconfig(uint32_t cfg, at32_tim_channel_t mode)
{
  /* TODO: Add support for input capture and bipolar dual outputs for TIM8 */

  if (mode & AT32_TIM_CH_MODE_MASK)
    {
      at32_configgpio(cfg);
    }
  else
    {
      at32_unconfiggpio(cfg);
    }
}
#endif

/****************************************************************************
 * Name: at32_tim_setmode
 ****************************************************************************/

static int at32_tim_setmode(struct at32_tim_dev_s *dev,
                             at32_tim_mode_t mode)
{
  uint16_t val = GTIM_CR1_CEN | GTIM_CR1_ARPE;

  DEBUGASSERT(dev != NULL);

  /* This function is not supported on basic timers. To enable or
   * disable it, simply set its clock to valid frequency or zero.
   */

#if AT32_NBTIM > 0
  if (((struct at32_tim_priv_s *)dev)->base == AT32_TMR6_BASE
#endif
#if AT32_NBTIM > 1
      ||  ((struct at32_tim_priv_s *)dev)->base == AT32_TMR7_BASE
#endif
#if AT32_NBTIM > 0
  )
    {
      return -EINVAL;
    }
#endif

  /* Decode operational modes */

  switch (mode & AT32_TIM_MODE_MASK)
    {
      case AT32_TIM_MODE_DISABLED:
        val = 0;
        break;

      case AT32_TIM_MODE_DOWN:
        val |= GTIM_CR1_DIR;

      case AT32_TIM_MODE_UP:
        break;

      case AT32_TIM_MODE_UPDOWN:
        /* Our default:
         * Interrupts are generated on compare, when counting down
         */

        val |= GTIM_CR1_CENTER1;
        break;

      case AT32_TIM_MODE_PULSE:
        val |= GTIM_CR1_OPM;
        break;

      default:
        return -EINVAL;
    }

  at32_tim_reload_counter(dev);
  at32_putreg16(dev, AT32_GTIM_CR1_OFFSET, val);

#if AT32_NATIM > 0
  /* Advanced registers require Main Output Enable */

    if (((struct at32_tim_priv_s *)dev)->base == AT32_TMR1_BASE
#ifdef AT32_TMR8_BASE
        || ((struct at32_tim_priv_s *)dev)->base == AT32_TMR8_BASE
#endif
#ifdef AT32_TMR20_BASE
        || ((struct at32_tim_priv_s *)dev)->base == AT32_TMR20_BASE
#endif
      )
      {
        at32_modifyreg16(dev, AT32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
      }
#endif

  return OK;
}

/****************************************************************************
 * Name: at32_tim_setclock
 ****************************************************************************/

static int at32_tim_setclock(struct at32_tim_dev_s *dev, uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      at32_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct at32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_AT32_TIM1
      case AT32_TMR1_BASE:
        freqin = AT32_APB2_TIM1_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case AT32_TMR2_BASE:
        freqin = AT32_APB1_TIM2_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case AT32_TMR3_BASE:
        freqin = AT32_APB1_TIM3_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case AT32_TMR4_BASE:
        freqin = AT32_APB1_TIM4_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case AT32_TMR5_BASE:
        freqin = AT32_APB1_TIM5_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM6
      case AT32_TMR6_BASE:
        freqin = AT32_APB1_TIM6_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM7
      case AT32_TMR7_BASE:
        freqin = AT32_APB1_TIM7_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case AT32_TMR8_BASE:
        freqin = AT32_APB2_TIM8_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case AT32_TMR9_BASE:
        freqin = AT32_APB2_TIM9_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case AT32_TMR10_BASE:
        freqin = AT32_APB2_TIM10_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case AT32_TMR11_BASE:
        freqin = AT32_APB2_TIM11_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case AT32_TMR12_BASE:
        freqin = AT32_APB1_TIM12_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case AT32_TMR13_BASE:
        freqin = AT32_APB1_TIM13_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case AT32_TMR14_BASE:
        freqin = AT32_APB1_TIM14_CLKIN;
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case AT32_TMR20_BASE:
        freqin = AT32_APB2_TIM20_CLKIN;
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

  at32_putreg16(dev, AT32_GTIM_PSC_OFFSET, prescaler);
  at32_tim_enable(dev);

  return prescaler;
}

/****************************************************************************
 * Name: at32_tim_setperiod
 ****************************************************************************/

static void at32_tim_setperiod(struct at32_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  at32_putreg32(dev, AT32_GTIM_ARR_OFFSET, period);
}

/****************************************************************************
 * Name: at32_tim_getcounter
 ****************************************************************************/

static uint32_t at32_tim_getcounter(struct at32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return at32_tim_getwidth(dev) > 16 ?
    at32_getreg32(dev, AT32_GTIM_CNT_OFFSET) :
    (uint32_t)at32_getreg16(dev, AT32_GTIM_CNT_OFFSET);
}

/****************************************************************************
 * Name: at32_tim_setcounter
 ****************************************************************************/

static void at32_tim_setcounter(struct at32_tim_dev_s *dev,
                                 uint32_t count)
{
  DEBUGASSERT(dev != NULL);

  if (at32_tim_getwidth(dev) > 16)
    {
      at32_putreg32(dev, AT32_GTIM_CNT_OFFSET, count);
    }
  else
    {
      at32_putreg16(dev, AT32_GTIM_CNT_OFFSET, (uint16_t)count);
    }
}

/****************************************************************************
 * Name: at32_tim_getwidth
 ****************************************************************************/

static int at32_tim_getwidth(struct at32_tim_dev_s *dev)
{
  /* Only TIM2 and TIM5 timers may be 32-bits in width
   *
   * Reference Table 2 of en.DM00042534.pdf
   */

  switch (((struct at32_tim_priv_s *)dev)->base)
    {
      /* TIM2 is 32-bits  */

      case AT32_TMR2_BASE:
        return 32;

      /* TIM5 is 32-bits  */

      case AT32_TMR5_BASE:
        return 32;

      /* All others are 16-bit times */

      default:
        return 16;
    }
}

/****************************************************************************
 * Name: at32_tim_setchannel
 ****************************************************************************/

static int at32_tim_setchannel(struct at32_tim_dev_s *dev,
                                uint8_t channel, at32_tim_channel_t mode)
{
  uint16_t ccmr_orig   = 0;
  uint16_t ccmr_val    = 0;
  uint16_t ccmr_mask   = 0xff;
  uint16_t ccer_val    = at32_getreg16(dev, AT32_GTIM_CCER_OFFSET);
  uint8_t  ccmr_offset = AT32_GTIM_CCMR1_OFFSET;

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

#if AT32_NBTIM > 0
  if (((struct at32_tim_priv_s *)dev)->base == AT32_TMR6_BASE
#endif
#if AT32_NBTIM > 1
      || ((struct at32_tim_priv_s *)dev)->base == AT32_TMR7_BASE
#endif
#if AT32_NBTIM > 0
  )
    {
      return -EINVAL;
    }
#endif

  /* Decode configuration */

  switch (mode & AT32_TIM_CH_MODE_MASK)
    {
      case AT32_TIM_CH_DISABLED:
        break;

      case AT32_TIM_CH_OUTPWM:
        ccmr_val = (GTIM_CCMR_MODE_PWM1 << GTIM_CCMR1_OC1M_SHIFT) +
                   GTIM_CCMR1_OC1PE;
        ccer_val |= GTIM_CCER_CC1E << GTIM_CCER_CCXBASE(channel);
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity */

  if (mode & AT32_TIM_CH_POLARITY_NEG)
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
      ccmr_offset = AT32_GTIM_CCMR2_OFFSET;
    }

  ccmr_orig  = at32_getreg16(dev, ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  at32_putreg16(dev, ccmr_offset, ccmr_orig);
  at32_putreg16(dev, AT32_GTIM_CCER_OFFSET, ccer_val);

  /* set GPIO */

  switch (((struct at32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_AT32_TIM1
      case AT32_TMR1_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM1_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM1_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM1_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM1_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM1_CH4OUT, mode); break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case AT32_TMR2_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM2_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM2_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM2_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM2_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM2_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM2_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM2_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM2_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case AT32_TMR3_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM3_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM3_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM3_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM3_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM3_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM3_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM3_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM3_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case AT32_TMR4_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM4_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM4_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM4_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM4_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM4_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM4_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM4_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM4_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case AT32_TMR5_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM5_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM5_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM5_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM5_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM5_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM5_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM5_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM5_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case AT32_TMR8_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM8_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM8_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM8_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM8_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM8_CH4OUT, mode); break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case AT32_TMR9_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM9_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM9_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM9_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM9_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM9_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM9_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM9_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM9_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case AT32_TMR10_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM10_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM10_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM10_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM10_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM10_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM10_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM10_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM10_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case AT32_TMR11_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM11_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM11_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM11_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM11_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM11_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM11_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM11_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM11_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case AT32_TMR12_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM12_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM12_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM12_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM12_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM12_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM12_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM12_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM12_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case AT32_TMR13_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM13_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM13_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM13_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM13_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM13_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM13_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM13_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM13_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case AT32_TMR14_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM14_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM14_CH1OUT, mode);
              break;
#endif
#if defined(GPIO_TIM14_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM14_CH2OUT, mode);
              break;
#endif
#if defined(GPIO_TIM14_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM14_CH3OUT, mode);
              break;
#endif
#if defined(GPIO_TIM14_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM14_CH4OUT, mode);
              break;
#endif
            default:
              return -EINVAL;
          }
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case AT32_TMR20_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM20_CH1OUT)
            case 0:
              at32_tim_gpioconfig(GPIO_TIM20_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM20_CH2OUT)
            case 1:
              at32_tim_gpioconfig(GPIO_TIM20_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM20_CH3OUT)
            case 2:
              at32_tim_gpioconfig(GPIO_TIM20_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM20_CH4OUT)
            case 3:
              at32_tim_gpioconfig(GPIO_TIM20_CH4OUT, mode); break;
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
 * Name: at32_tim_setcompare
 ****************************************************************************/

static int at32_tim_setcompare(struct at32_tim_dev_s *dev,
                                uint8_t channel, uint32_t compare)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        at32_putreg32(dev, AT32_GTIM_CCR1_OFFSET, compare);
        break;

      case 2:
        at32_putreg32(dev, AT32_GTIM_CCR2_OFFSET, compare);
        break;

      case 3:
        at32_putreg32(dev, AT32_GTIM_CCR3_OFFSET, compare);
        break;

      case 4:
        at32_putreg32(dev, AT32_GTIM_CCR4_OFFSET, compare);
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: at32_tim_getcapture
 ****************************************************************************/

static int at32_tim_getcapture(struct at32_tim_dev_s *dev,
                                uint8_t channel)
{
  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case 1:
        return at32_getreg32(dev, AT32_GTIM_CCR1_OFFSET);
      case 2:
        return at32_getreg32(dev, AT32_GTIM_CCR2_OFFSET);
      case 3:
        return at32_getreg32(dev, AT32_GTIM_CCR3_OFFSET);
      case 4:
        return at32_getreg32(dev, AT32_GTIM_CCR4_OFFSET);
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: at32_tim_setisr
 ****************************************************************************/

static int at32_tim_setisr(struct at32_tim_dev_s *dev, xcpt_t handler,
                            void * arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct at32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_AT32_TIM1
      case AT32_TMR1_BASE:
        vectorno = AT32_IRQ_TIM1UP;
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case AT32_TMR2_BASE:
        vectorno = AT32_IRQ_TIM2;
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case AT32_TMR3_BASE:
        vectorno = AT32_IRQ_TIM3;
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case AT32_TMR4_BASE:
        vectorno = AT32_IRQ_TIM4;
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case AT32_TMR5_BASE:
        vectorno = AT32_IRQ_TIM5;
        break;
#endif
#ifdef CONFIG_AT32_TIM6
      case AT32_TMR6_BASE:
        vectorno = AT32_IRQ_TIM6;
        break;
#endif
#ifdef CONFIG_AT32_TIM7
      case AT32_TMR7_BASE:
        vectorno = AT32_IRQ_TIM7;
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case AT32_TMR8_BASE:
        vectorno = AT32_IRQ_TIM8UP;
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case AT32_TMR9_BASE:
        vectorno = AT32_IRQ_TIM9;
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case AT32_TMR10_BASE:
        vectorno = AT32_IRQ_TIM10;
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case AT32_TMR11_BASE:
        vectorno = AT32_IRQ_TIM11;
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case AT32_TMR12_BASE:
        vectorno = AT32_IRQ_TIM12;
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case AT32_TMR13_BASE:
        vectorno = AT32_IRQ_TIM13;
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case AT32_TMR14_BASE:
        vectorno = AT32_IRQ_TIM14;
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case AT32_TMR20_BASE:
        vectorno = AT32_IRQ_TIM20UP;
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
 * Name: at32_tim_enableint
 ****************************************************************************/

static void at32_tim_enableint(struct at32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  at32_modifyreg16(dev, AT32_GTIM_DIER_OFFSET, 0, source);
}

/****************************************************************************
 * Name: at32_tim_disableint
 ****************************************************************************/

static void at32_tim_disableint(struct at32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  at32_modifyreg16(dev, AT32_GTIM_DIER_OFFSET, source, 0);
}

/****************************************************************************
 * Name: at32_tim_ackint
 ****************************************************************************/

static void at32_tim_ackint(struct at32_tim_dev_s *dev, int source)
{
  at32_putreg16(dev, AT32_GTIM_SR_OFFSET, ~source);
}

/****************************************************************************
 * Name: at32_tim_checkint
 ****************************************************************************/

static int at32_tim_checkint(struct at32_tim_dev_s *dev, int source)
{
  uint16_t regval = at32_getreg16(dev, AT32_GTIM_SR_OFFSET);
  return (regval & source) ? 1 : 0;
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_tim_init
 ****************************************************************************/

struct at32_tim_dev_s *at32_tim_init(int timer)
{
  struct at32_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_AT32_TIM1
      case 1:
        dev = (struct at32_tim_dev_s *)&at32_tim1_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR1EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case 2:
        dev = (struct at32_tim_dev_s *)&at32_tim2_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR2EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case 3:
        dev = (struct at32_tim_dev_s *)&at32_tim3_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR3EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case 4:
        dev = (struct at32_tim_dev_s *)&at32_tim4_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR4EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case 5:
        dev = (struct at32_tim_dev_s *)&at32_tim5_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR5EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM6
      case 6:
        dev = (struct at32_tim_dev_s *)&at32_tim6_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR6EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM7
      case 7:
        dev = (struct at32_tim_dev_s *)&at32_tim7_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR7EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case 8:
        dev = (struct at32_tim_dev_s *)&at32_tim8_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR8EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case 9:
        dev = (struct at32_tim_dev_s *)&at32_tim9_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR9EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case 10:
        dev = (struct at32_tim_dev_s *)&at32_tim10_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR10EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case 11:
        dev = (struct at32_tim_dev_s *)&at32_tim11_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR11EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case 12:
        dev = (struct at32_tim_dev_s *)&at32_tim12_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR12EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case 13:
        dev = (struct at32_tim_dev_s *)&at32_tim13_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR13EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case 14:
        dev = (struct at32_tim_dev_s *)&at32_tim14_priv;
        modifyreg32(AT32_CRM_APB1EN, 0, CRM_APB1EN_TMR14EN);
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case 20:
        dev = (struct at32_tim_dev_s *)&at32_tim20_priv;
        modifyreg32(AT32_CRM_APB2EN, 0, CRM_APB2EN_TMR20EN);
        break;
#endif
      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct at32_tim_priv_s *)dev)->mode != AT32_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  at32_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: at32_tim_deinit
 *
 * TODO: Detach interrupts, and close down all TIM Channels
 *
 ****************************************************************************/

int at32_tim_deinit(struct at32_tim_dev_s * dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct at32_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_AT32_TIM1
      case AT32_TMR1_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR1EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM2
      case AT32_TMR2_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR2EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM3
      case AT32_TMR3_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR3EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM4
      case AT32_TMR4_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR4EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM5
      case AT32_TMR5_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR5EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM6
      case AT32_TMR6_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR6EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM7
      case AT32_TMR7_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR7EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM8
      case AT32_TMR8_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR8EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM9
      case AT32_TMR9_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR9EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM10
      case AT32_TMR10_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR10EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM11
      case AT32_TMR11_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR11EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM12
      case AT32_TMR12_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR12EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM13
      case AT32_TMR13_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR13EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM14
      case AT32_TMR14_BASE:
        modifyreg32(AT32_CRM_APB1EN, CRM_APB1EN_TMR14EN, 0);
        break;
#endif
#ifdef CONFIG_AT32_TIM20
      case AT32_TMR20_BASE:
        modifyreg32(AT32_CRM_APB2EN, CRM_APB2EN_TMR20EN, 0);
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct at32_tim_priv_s *)dev)->mode = AT32_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_AT32_TIM1 || ... || TIM20) */
