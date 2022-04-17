/****************************************************************************
 * arch/arm/src/stm32f7/stm32_pwm.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"
#include "chip.h"
#include "stm32_pwm.h"
#include "stm32_gpio.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_STM32F7_TIM1_PWM)  || defined(CONFIG_STM32F7_TIM2_PWM)  || \
    defined(CONFIG_STM32F7_TIM3_PWM)  || defined(CONFIG_STM32F7_TIM4_PWM)  || \
    defined(CONFIG_STM32F7_TIM5_PWM)  || defined(CONFIG_STM32F7_TIM8_PWM)  || \
    defined(CONFIG_STM32F7_TIM9_PWM)  || defined(CONFIG_STM32F7_TIM10_PWM) || \
    defined(CONFIG_STM32F7_TIM11_PWM) || defined(CONFIG_STM32F7_TIM12_PWM) || \
    defined(CONFIG_STM32F7_TIM13_PWM) || defined(CONFIG_STM32F7_TIM14_PWM) || \
    defined(CONFIG_STM32F7_TIM15_PWM) || defined(CONFIG_STM32F7_TIM16_PWM) || \
    defined(CONFIG_STM32F7_TIM17_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types */

#define TIMTYPE_BASIC      0  /* Basic timers: TIM6-7 */
#define TIMTYPE_GENERAL16  1  /* General 16-bit timers: TIM3-4 */
#define TIMTYPE_COUNTUP16  2  /* General 16-bit count-up timers: TIM9-14 */
#define TIMTYPE_GENERAL32  3  /* General 32-bit timers: TIM2-5 */
#define TIMTYPE_ADVANCED   4  /* Advanced timers:  TIM1-8 */

#define TIMTYPE_TIM1       TIMTYPE_ADVANCED
#define TIMTYPE_TIM2       TIMTYPE_GENERAL32
#define TIMTYPE_TIM3       TIMTYPE_GENERAL16
#define TIMTYPE_TIM4       TIMTYPE_GENERAL16
#define TIMTYPE_TIM5       TIMTYPE_GENERAL32
#define TIMTYPE_TIM6       TIMTYPE_BASIC
#define TIMTYPE_TIM7       TIMTYPE_BASIC
#define TIMTYPE_TIM8       TIMTYPE_ADVANCED
#define TIMTYPE_TIM9       TIMTYPE_COUNTUP16
#define TIMTYPE_TIM10      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM11      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM12      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM13      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM14      TIMTYPE_COUNTUP16

/* Timer clock source, RCC EN offset, enable bit,
 * RCC RST offset, reset bit to use
 */

#define TIMCLK_TIM1      STM32_APB2_TIM1_CLKIN
#define TIMRCCEN_TIM1    STM32_RCC_APB2ENR
#define TIMEN_TIM1       RCC_APB2ENR_TIM1EN
#define TIMRCCRST_TIM1   STM32_RCC_APB2RSTR
#define TIMRST_TIM1      RCC_APB2RSTR_TIM1RST
#define TIMCLK_TIM2      STM32_APB1_TIM2_CLKIN
#define TIMRCCEN_TIM2    STM32_RCC_APB1ENR
#define TIMEN_TIM2       RCC_APB1ENR_TIM2EN
#define TIMRCCRST_TIM2   STM32_RCC_APB1RSTR
#define TIMRST_TIM2      RCC_APB1RSTR_TIM2RST
#define TIMCLK_TIM3      STM32_APB1_TIM3_CLKIN
#define TIMRCCEN_TIM3    STM32_RCC_APB1ENR
#define TIMEN_TIM3       RCC_APB1ENR_TIM3EN
#define TIMRCCRST_TIM3   STM32_RCC_APB1RSTR
#define TIMRST_TIM3      RCC_APB1RSTR_TIM3RST
#define TIMCLK_TIM4      STM32_APB1_TIM4_CLKIN
#define TIMRCCEN_TIM4    STM32_RCC_APB1ENR
#define TIMEN_TIM4       RCC_APB1ENR_TIM4EN
#define TIMRCCRST_TIM4   STM32_RCC_APB1RSTR
#define TIMRST_TIM4      RCC_APB1RSTR_TIM4RST
#define TIMCLK_TIM5      STM32_APB1_TIM5_CLKIN
#define TIMRCCEN_TIM5    STM32_RCC_APB1ENR
#define TIMEN_TIM5       RCC_APB1ENR_TIM5EN
#define TIMRCCRST_TIM5   STM32_RCC_APB1RSTR
#define TIMRST_TIM5      RCC_APB1RSTR_TIM5RST
#define TIMCLK_TIM8      STM32_APB2_TIM8_CLKIN
#define TIMRCCEN_TIM8    STM32_RCC_APB2ENR
#define TIMEN_TIM8       RCC_APB2ENR_TIM8EN
#define TIMRCCRST_TIM8   STM32_RCC_APB2RSTR
#define TIMRST_TIM8      RCC_APB2RSTR_TIM8RST
#define TIMCLK_TIM9      STM32_APB2_TIM9_CLKIN
#define TIMRCCEN_TIM9    STM32_RCC_APB2ENR
#define TIMEN_TIM9       RCC_APB2ENR_TIM9EN
#define TIMRCCRST_TIM9   STM32_RCC_APB2RSTR
#define TIMRST_TIM9      RCC_APB2RSTR_TIM9RST
#define TIMCLK_TIM10     STM32_APB2_TIM10_CLKIN
#define TIMRCCEN_TIM10   STM32_RCC_APB2ENR
#define TIMEN_TIM10      RCC_APB2ENR_TIM10EN
#define TIMRCCRST_TIM10  STM32_RCC_APB2RSTR
#define TIMRST_TIM10     RCC_APB2RSTR_TIM10RST
#define TIMCLK_TIM11     STM32_APB2_TIM11_CLKIN
#define TIMRCCEN_TIM11   STM32_RCC_APB2ENR
#define TIMEN_TIM11      RCC_APB2ENR_TIM11EN
#define TIMRCCRST_TIM11  STM32_RCC_APB2RSTR
#define TIMRST_TIM11     RCC_APB2RSTR_TIM11RST
#define TIMCLK_TIM12     STM32_APB1_TIM12_CLKIN
#define TIMRCCEN_TIM12   STM32_RCC_APB1ENR
#define TIMEN_TIM12      RCC_APB1ENR_TIM12EN
#define TIMRCCRST_TIM12  STM32_RCC_APB1RSTR
#define TIMRST_TIM12     RCC_APB1RSTR_TIM12RST
#define TIMCLK_TIM13     STM32_APB1_TIM13_CLKIN
#define TIMRCCEN_TIM13   STM32_RCC_APB1ENR
#define TIMEN_TIM13      RCC_APB1ENR_TIM13EN
#define TIMRCCRST_TIM13  STM32_RCC_APB1RSTR
#define TIMRST_TIM13     RCC_APB1RSTR_TIM13RST
#define TIMCLK_TIM14     STM32_APB1_TIM14_CLKIN
#define TIMRCCEN_TIM14   STM32_RCC_APB1ENR
#define TIMEN_TIM14      RCC_APB1ENR_TIM14EN
#define TIMRCCRST_TIM14  STM32_RCC_APB1RSTR
#define TIMRST_TIM14     RCC_APB1RSTR_TIM14RST

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) stm32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum stm32_timmode_e
{
  STM32_TIMMODE_COUNTUP   = 0,
  STM32_TIMMODE_COUNTDOWN = 1,
  STM32_TIMMODE_CENTER1   = 2,
  STM32_TIMMODE_CENTER2   = 3,
  STM32_TIMMODE_CENTER3   = 4,
};

enum stm32_chanmode_e
{
  STM32_CHANMODE_PWM1        = 0,
  STM32_CHANMODE_PWM2        = 1,
  STM32_CHANMODE_COMBINED1   = 2,
  STM32_CHANMODE_COMBINED2   = 3,
  STM32_CHANMODE_ASYMMETRIC1 = 4,
  STM32_CHANMODE_ASYMMETRIC2 = 5,
};

struct stm32_pwmchan_s
{
  uint8_t channel;                     /* Timer output channel: {1,..4} */
  uint32_t pincfg;                     /* Output pin configuration */
  enum stm32_chanmode_e mode;
};

/* This structure represents the state of one PWM timer */

struct stm32_pwmtimer_s
{
  const struct pwm_ops_s *ops; /* PWM operations */
  uint8_t timid;               /* Timer ID {1,...,17} */
  struct stm32_pwmchan_s channels[PWM_NCHANNELS];
  uint8_t timtype;             /* See the TIMTYPE_* definitions */
  enum stm32_timmode_e mode;
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t irq;                 /* Timer update IRQ */
  uint8_t prev;                /* The previous value of the RCR (pre-loaded) */
  uint8_t curr;                /* The current value of the RCR (pre-loaded) */
  uint32_t count;              /* Remaining pulse count */
#else
  uint32_t frequency;          /* Current frequency setting */
#endif
  uint32_t base;               /* The base address of the timer */
  uint32_t pclk;               /* The frequency of the peripheral clock
                                * that drives the timer module. */
#ifdef CONFIG_PWM_PULSECOUNT
  void *handle;                /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint16_t pwm_getreg(struct stm32_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                       uint16_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct stm32_pwmtimer_s *priv, const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_timer(struct stm32_pwmtimer_s *priv,
                     const struct pwm_info_s *info);

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM8_PWM))
static int pwm_interrupt(struct stm32_pwmtimer_s *priv);
#if defined(CONFIG_STM32F7_TIM1_PWM)
static int pwm_tim1interrupt(int irq, void *context, void *arg);
#endif
#if defined(CONFIG_STM32F7_TIM8_PWM)
static int pwm_tim8interrupt(int irq, void *context, void *arg);
#endif
static uint8_t pwm_pulsecount(uint32_t count);
#endif

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle);
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
#endif

static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_STM32F7_TIM1_PWM
static struct stm32_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .timid       = 1,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM1_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM1_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM1_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM1_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM1_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM1_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM1_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM1_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM1_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM1_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM1,
  .mode        = CONFIG_STM32F7_TIM1_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM1UP,
#endif
  .base        = STM32_TIM1_BASE,
  .pclk        = TIMCLK_TIM1,
};
#endif

#ifdef CONFIG_STM32F7_TIM2_PWM
static struct stm32_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .timid       = 2,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM2_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM2_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM2_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM2_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM2_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM2_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM2_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM2_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM2_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM2_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM2,
  .mode        = CONFIG_STM32F7_TIM2_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM2,
#endif
  .base        = STM32_TIM2_BASE,
  .pclk        = TIMCLK_TIM2,
};
#endif

#ifdef CONFIG_STM32F7_TIM3_PWM
static struct stm32_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
  .timid       = 3,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM3_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM3_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM3_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM3_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM3_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM3_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM3_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM3_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM3_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM3_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM3,
  .mode        = CONFIG_STM32F7_TIM3_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM3,
#endif
  .base        = STM32_TIM3_BASE,
  .pclk        = TIMCLK_TIM3,
};
#endif

#ifdef CONFIG_STM32F7_TIM4_PWM
static struct stm32_pwmtimer_s g_pwm4dev =
{
  .ops         = &g_pwmops,
  .timid       = 4,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM4_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM4_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM4_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM4_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM4_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM4_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM4_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM4_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM4_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM4_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM4,
  .mode        = CONFIG_STM32F7_TIM4_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM4,
#endif
  .base        = STM32_TIM4_BASE,
  .pclk        = TIMCLK_TIM4,
};
#endif

#ifdef CONFIG_STM32F7_TIM5_PWM
static struct stm32_pwmtimer_s g_pwm5dev =
{
  .ops         = &g_pwmops,
  .timid       = 5,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM5_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM5_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM5_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM5_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM5_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM5_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM5_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM5_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM5_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM5_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM5,
  .mode        = CONFIG_STM32F7_TIM5_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM5,
#endif
  .base        = STM32_TIM5_BASE,
  .pclk        = TIMCLK_TIM5,
};
#endif

#ifdef CONFIG_STM32F7_TIM8_PWM
static struct stm32_pwmtimer_s g_pwm8dev =
{
  .ops         = &g_pwmops,
  .timid       = 8,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM8_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM8_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM8_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM8_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM8_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM8_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM8_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM8_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM8_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM8_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM8,
  .mode        = CONFIG_STM32F7_TIM8_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM8UP,
#endif
  .base        = STM32_TIM8_BASE,
  .pclk        = TIMCLK_TIM8,
};
#endif

#ifdef CONFIG_STM32F7_TIM9_PWM
static struct stm32_pwmtimer_s g_pwm9dev =
{
  .ops         = &g_pwmops,
  .timid       = 9,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM9_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM9_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM9_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM9_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM9_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM9_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM9_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM9_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM9_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM9_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM9_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM9_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM9,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM9,
#endif
  .base        = STM32_TIM9_BASE,
  .pclk        = TIMCLK_TIM9,
};
#endif

#ifdef CONFIG_STM32F7_TIM10_PWM
static struct stm32_pwmtimer_s g_pwm10dev =
{
  .ops         = &g_pwmops,
  .timid       = 10,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM10_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM10_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM10_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM10_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM10_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM10_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM10_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM10_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM10_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM10_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM10_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM10_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM10,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM10,
#endif
  .base        = STM32_TIM10_BASE,
  .pclk        = TIMCLK_TIM10,
};
#endif

#ifdef CONFIG_STM32F7_TIM11_PWM
static struct stm32_pwmtimer_s g_pwm11dev =
{
  .ops         = &g_pwmops,
  .timid       = 11,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM11_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM11_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM11_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM11_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM11_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM11_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM11_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM11_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM11_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM11_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM11_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM11_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM11,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM11,
#endif
  .base        = STM32_TIM11_BASE,
  .pclk        = TIMCLK_TIM11,
};
#endif

#ifdef CONFIG_STM32F7_TIM12_PWM
static struct stm32_pwmtimer_s g_pwm12dev =
{
  .ops         = &g_pwmops,
  .timid       = 12,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM12_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM12_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM12_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM12_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM12_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM12_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM12_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM12_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM12_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM12_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM12_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM12_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM12,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM12,
#endif
  .base        = STM32_TIM12_BASE,
  .pclk        = TIMCLK_TIM12,
};
#endif

#ifdef CONFIG_STM32F7_TIM13_PWM
static struct stm32_pwmtimer_s g_pwm13dev =
{
  .ops         = &g_pwmops,
  .timid       = 13,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM13_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM13_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM13_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM13_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM13_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM13_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM13_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM13_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM13_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM13_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM13_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM13_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM13,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM13,
#endif
  .base        = STM32_TIM13_BASE,
  .pclk        = TIMCLK_TIM13,
};
#endif

#ifdef CONFIG_STM32F7_TIM14_PWM
static struct stm32_pwmtimer_s g_pwm14dev =
{
  .ops         = &g_pwmops,
  .timid       = 14,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM14_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM14_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM14_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM14_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM14_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM14_CH2MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM14_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM14_CH3CFG,
      .mode    = CONFIG_STM32F7_TIM14_CH3MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM14_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM14_CH4CFG,
      .mode    = CONFIG_STM32F7_TIM14_CH4MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM14,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM14,
#endif
  .base        = STM32_TIM14_BASE,
  .pclk        = TIMCLK_TIM14,
};
#endif

#ifdef CONFIG_STM32F7_TIM15_PWM
static struct stm32_pwmtimer_s g_pwm15dev =
{
  .ops         = &g_pwmops,
  .timid       = 15,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM15_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM15_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM15_CH1MODE,
    },
#endif
#ifdef CONFIG_STM32F7_TIM15_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM15_CH2CFG,
      .mode    = CONFIG_STM32F7_TIM15_CH2MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM15,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM15,
#endif
  .base        = STM32_TIM15_BASE,
  .pclk        = TIMCLK_TIM15,
};
#endif

#ifdef CONFIG_STM32F7_TIM16_PWM
static struct stm32_pwmtimer_s g_pwm16dev =
{
  .ops         = &g_pwmops,
  .timid       = 16,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM16_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM16_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM16_CH1MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM16,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM16,
#endif
  .base        = STM32_TIM16_BASE,
  .pclk        = TIMCLK_TIM16,
};
#endif

#ifdef CONFIG_STM32F7_TIM17_PWM
static struct stm32_pwmtimer_s g_pwm17dev =
{
  .ops         = &g_pwmops,
  .timid       = 17,
  .channels    =
  {
#ifdef CONFIG_STM32F7_TIM17_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM17_CH1CFG,
      .mode    = CONFIG_STM32F7_TIM17_CH1MODE,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM17,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM17,
#endif
  .base        = STM32_TIM17_BASE,
  .pclk        = TIMCLK_TIM17,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint16_t pwm_getreg(struct stm32_pwmtimer_s *priv, int offset)
{
  return getreg16(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                       uint16_t value)
{
  if (priv->timtype == TIMTYPE_GENERAL32 &&
      (offset == STM32_GTIM_CNT_OFFSET ||
       offset == STM32_GTIM_ARR_OFFSET ||
       offset == STM32_GTIM_CCR1_OFFSET ||
       offset == STM32_GTIM_CCR2_OFFSET ||
       offset == STM32_GTIM_CCR3_OFFSET ||
       offset == STM32_GTIM_CCR4_OFFSET))
    {
      /* a 32 bit access is required for a 32 bit register:
       * if only a 16 bit write would be performed, then the
       * upper 16 bits of the 32 bit register will be a copy of
       * the lower 16 bits.
       */

      putreg32(value, priv->base + offset);
    }
  else
    {
      putreg16(value, priv->base + offset);
    }
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct stm32_pwmtimer_s *priv, const char *msg)
{
  pwminfo("%s:\n", msg);
  if (priv->timid == 16 || priv->timid == 17)
    {
      pwminfo("  CR1: %04x CR2:  %04x DIER:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
    }
  else
    {
      pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_SMCR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
    }

  if (priv->timid >= 15 || priv->timid <= 17)
    {
      pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x\n",
              pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET));
    }
  else
    {
      pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
              pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
    }

  /* REVISIT: CNT and ARR may be 32-bits wide */

  pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          pwm_getreg(priv, STM32_GTIM_CCER_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CNT_OFFSET),
          pwm_getreg(priv, STM32_GTIM_PSC_OFFSET),
          pwm_getreg(priv, STM32_GTIM_ARR_OFFSET));

  if (priv->timid >= 15 || priv->timid <= 17)
    {
      pwminfo("  RCR: %04x BDTR: %04x\n",
          pwm_getreg(priv, STM32_ATIM_RCR_OFFSET),
          pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET));
    }

  /* REVISIT: CCR1-CCR4 may be 32-bits wide */

  if (priv->timid == 16 || priv->timid == 17)
    {
      pwminfo(" CCR1: %04x\n",
              pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET));
    }
  else
    {
      pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR3_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR4_OFFSET));
    }

  pwminfo("  DCR: %04x DMAR: %04x\n",
      pwm_getreg(priv, STM32_GTIM_DCR_OFFSET),
      pwm_getreg(priv, STM32_GTIM_DMAR_OFFSET));
}
#endif

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_timer(struct stm32_pwmtimer_s *priv,
                     const struct pwm_info_s *info)
{
#ifdef CONFIG_PWM_MULTICHAN
  int      i;
#endif

  /* Calculated values */

  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint32_t ccr;

  /* Register contents */

  uint16_t cr1;
  uint16_t ccer;
  uint16_t cr2;
  uint16_t ccmr1;
  uint16_t ccmr2;

  /* New timer register bit settings */

  uint16_t ccenable;
  uint16_t ocmode1;
  uint16_t ocmode2;

  DEBUGASSERT(priv != NULL && info != NULL);

#if defined(CONFIG_PWM_MULTICHAN)
  pwminfo("TIM%u frequency: %lu\n",
          priv->timid, info->frequency);
#elif defined(CONFIG_PWM_PULSECOUNT)
  pwminfo("TIM%u channel: %u frequency: %u duty: %08x count: %u\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty, info->count);
#else
  pwminfo("TIM%u channel: %u frequency: %u duty: %08x\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* Disable all interrupts and DMA requests, clear all pending status */

#ifdef CONFIG_PWM_PULSECOUNT
  pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
#endif

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

  prescaler = (priv->pclk / info->frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = priv->pclk / prescaler;

  reload = timclk / info->frequency;
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

  pwminfo("TIM%u PCLK: %lu frequency: %lu "
          "TIMCLK: %lu prescaler: %lu reload: %lu\n",
          priv->timid, priv->pclk, info->frequency,
          timclk, prescaler, reload);

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = pwm_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Disable the timer until we get it configured */

  cr1 &= ~GTIM_CR1_CEN;

  /* Set the counter mode for the advanced timers (1,8) and most general
   * purpose timers (all 2-5, but not 9-17), i.e., all but TIMTYPE_COUNTUP16
   * and TIMTYPE_BASIC
   */

#if defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM2_PWM) || \
    defined(CONFIG_STM32F7_TIM3_PWM) || defined(CONFIG_STM32F7_TIM4_PWM) || \
    defined(CONFIG_STM32F7_TIM5_PWM) || defined(CONFIG_STM32F7_TIM8_PWM)

  if (priv->timtype != TIMTYPE_BASIC && priv->timtype != TIMTYPE_COUNTUP16)
    {
      /* Select the Counter Mode:
       *
       * GTIM_CR1_EDGE: The counter counts up or down depending on the
       *   direction bit (DIR).
       * GTIM_CR1_CENTER1, GTIM_CR1_CENTER2, GTIM_CR1_CENTER3: The counter
       *   counts up then down.
       * GTIM_CR1_DIR: 0: count up, 1: count down
       */

      cr1 &= ~(GTIM_CR1_DIR | GTIM_CR1_CMS_MASK);

      switch (priv->mode)
        {
          case STM32_TIMMODE_COUNTUP:
            cr1 |= GTIM_CR1_EDGE;
            break;

          case STM32_TIMMODE_COUNTDOWN:
            cr1 |= GTIM_CR1_EDGE | GTIM_CR1_DIR;
            break;

          case STM32_TIMMODE_CENTER1:
            cr1 |= GTIM_CR1_CENTER1;
            break;

          case STM32_TIMMODE_CENTER2:
            cr1 |= GTIM_CR1_CENTER2;
            break;

          case STM32_TIMMODE_CENTER3:
            cr1 |= GTIM_CR1_CENTER3;
            break;

          default:
            pwmerr("ERROR: No such timer mode: %u\n",
                   (unsigned int)priv->mode);
            return -EINVAL;
        }
    }
#endif

  /* Set the clock division to zero for all (but the basic timers, but there
   * should be no basic timers in this context
   */

  cr1 &= ~GTIM_CR1_CKD_MASK;
  pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Set the reload and prescaler values */

  pwm_putreg(priv, STM32_GTIM_ARR_OFFSET, (uint16_t)reload);
  pwm_putreg(priv, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  /* Set the advanced timer's repetition counter */

#if defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repetition counter to the count-1 (pwm_start() has already
       * assured us that the count value is within range).
       */

#ifdef CONFIG_PWM_PULSECOUNT
      if (info->count > 0)
        {
          /* Save the remaining count and the number of counts that will have
           * elapsed on the first interrupt.
           */

          /* If the first interrupt occurs at the end end of the first
           * repetition count, then the count will be the same as the RCR
           * value.
           */

          priv->prev  = pwm_pulsecount(info->count);
          pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, (uint16_t)priv->prev - 1);

          /* Generate an update event to reload the prescaler.  This should
           * preload the RCR into active repetition counter.
           */

          pwm_putreg(priv, STM32_GTIM_EGR_OFFSET, ATIM_EGR_UG);

          /* Now set the value of the RCR that will be loaded on the next
           * update event.
           */

          priv->count = info->count;
          priv->curr  = pwm_pulsecount(info->count - priv->prev);
          pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
        }

      /* Otherwise, just clear the repetition counter */

      else
#endif
        {
          /* Set the repetition counter to zero */

          pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);

          /* Generate an update event to reload the prescaler */

          pwm_putreg(priv, STM32_ATIM_EGR_OFFSET, ATIM_EGR_UG);
        }
    }
  else
#endif
    {
      /* Generate an update event to reload the prescaler (all timers) */

      pwm_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);
    }

  /* Handle channel specific setup */

  ccenable = 0;
  ocmode1  = 0;
  ocmode2  = 0;

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
      ub16_t                duty;
      uint16_t              chanmode;
      uint8_t               channel;
#ifdef CONFIG_PWM_MULTICHAN
      int                   j;
#endif
      enum stm32_chanmode_e mode;

#ifdef CONFIG_PWM_MULTICHAN
      /* Break the loop if all following channels are not configured */

      if (info->channels[i].channel == -1)
        {
          break;
        }

      duty = info->channels[i].duty;
      channel = info->channels[i].channel;

      /* A value of zero means to skip this channel */

      if (channel == 0)
        {
          continue;
        }

      /* Find the channel */

      for (j = 0; j < PWM_NCHANNELS; j++)
        {
          if (priv->channels[j].channel == channel)
            {
              mode = priv->channels[j].mode;
              break;
            }
        }

      if (j >= PWM_NCHANNELS)
        {
          pwmerr("ERROR: No such channel: %u\n", channel);
          return -EINVAL;
        }
#else
      duty = info->duty;
      channel = priv->channels[0].channel;
      mode = priv->channels[0].mode;
#endif

      /* Duty cycle:
       *
       * duty cycle = ccr / reload (fractional value)
       */

      ccr = b16toi(duty * reload + b16HALF);

      pwminfo("ccr: %lu\n", ccr);

      switch (mode)
        {
          case STM32_CHANMODE_PWM1:
            chanmode = GTIM_CCMR_MODE_PWM1;
            break;

          case STM32_CHANMODE_PWM2:
            chanmode = GTIM_CCMR_MODE_PWM2;
            break;

          default:
            pwmerr("ERROR: No such mode: %u\n", (unsigned int)mode);
            return -EINVAL;
        }

      switch (channel)
        {
          case 1:  /* PWM Mode configuration: Channel 1 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= GTIM_CCER_CC1E;

              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC1S_SHIFT) |
                          (chanmode << GTIM_CCMR1_OC1M_SHIFT) |
                          GTIM_CCMR1_OC1PE;

              /* Set the duty cycle by writing to the CCR register for this
               * channel
               */

              pwm_putreg(priv, STM32_GTIM_CCR1_OFFSET, (uint16_t)ccr);
            }
            break;

          case 2:  /* PWM Mode configuration: Channel 2 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= GTIM_CCER_CC2E;

              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC2S_SHIFT) |
                          (chanmode << GTIM_CCMR1_OC2M_SHIFT) |
                          GTIM_CCMR1_OC2PE;

              /* Set the duty cycle by writing to the CCR register for this
               * channel
               */

              pwm_putreg(priv, STM32_GTIM_CCR2_OFFSET, (uint16_t)ccr);
            }
            break;

          case 3:  /* PWM Mode configuration: Channel 3 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= GTIM_CCER_CC3E;

              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR2_CC3S_SHIFT) |
                          (chanmode << GTIM_CCMR2_OC3M_SHIFT) |
                          GTIM_CCMR2_OC3PE;

              /* Set the duty cycle by writing to the CCR register for this
               * channel
               */

              pwm_putreg(priv, STM32_GTIM_CCR3_OFFSET, (uint16_t)ccr);
            }
            break;

          case 4:  /* PWM Mode configuration: Channel 4 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= GTIM_CCER_CC4E;

              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR2_CC4S_SHIFT) |
                          (chanmode << GTIM_CCMR2_OC4M_SHIFT) |
                          GTIM_CCMR2_OC4PE;

              /* Set the duty cycle by writing to the CCR register for this
               * channel
               */

              pwm_putreg(priv, STM32_GTIM_CCR4_OFFSET, (uint16_t)ccr);
            }
            break;

          default:
            pwmerr("ERROR: No such channel: %u\n", channel);
            return -EINVAL;
        }
    }

  /* Disable the Channel by resetting the CCxE Bit in the CCER register */

  ccer = pwm_getreg(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~ccenable;
  pwm_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Fetch the CR2, CCMR1, and CCMR2 register (already have cr1 and ccer) */

  cr2   = pwm_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccmr1 = pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET);
  ccmr2 = pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET);

  /* Reset the Output Compare Mode Bits and set the select output compare
   * mode
   */

  ccmr1 &= ~(GTIM_CCMR1_CC1S_MASK | GTIM_CCMR1_OC1M_MASK | GTIM_CCMR1_OC1PE |
             GTIM_CCMR1_CC2S_MASK | GTIM_CCMR1_OC2M_MASK | GTIM_CCMR1_OC2PE);
  ccmr2 &= ~(GTIM_CCMR2_CC3S_MASK | GTIM_CCMR2_OC3M_MASK | GTIM_CCMR2_OC3PE |
             GTIM_CCMR2_CC4S_MASK | GTIM_CCMR2_OC4M_MASK | GTIM_CCMR2_OC4PE);
  ccmr1 |= ocmode1;
  ccmr2 |= ocmode2;

  /* Reset the output polarity level of all channels (selects high
   * polarity)
   */

  ccer &= ~(GTIM_CCER_CC1P | GTIM_CCER_CC2P | GTIM_CCER_CC3P |
            GTIM_CCER_CC4P);

  /* Enable the output state of the selected channels */

  ccer &= ~(GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E |
            GTIM_CCER_CC4E);
  ccer |= ccenable;

  /* Some special setup for advanced timers */

#if defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM8_PWM) || \
    defined(CONFIG_STM32F7_TIM15_PWM) || defined(CONFIG_STM32F7_TIM16_PWM) || \
    defined(CONFIG_STM32F7_TIM17_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16)
    {
      uint16_t bdtr;

      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP | ATIM_CCER_CC2NE |
                ATIM_CCER_CC2NP | ATIM_CCER_CC3NE | ATIM_CCER_CC3NP |
                ATIM_CCER_CC4NP);

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N | ATIM_CR2_OIS2 |
               ATIM_CR2_OIS2N | ATIM_CR2_OIS3 | ATIM_CR2_OIS3N |
               ATIM_CR2_OIS4);

      /* Set the main output enable (MOE) bit and clear the OSSI and OSSR
       * bits in the BDTR register.
       */

      bdtr  = pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);
      bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);
      bdtr |= ATIM_BDTR_MOE;
      pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, bdtr);
    }
  else
#endif
    {
      /* CCxNP must be cleared in any case */

      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP |
                GTIM_CCER_CC4NP);
    }

  /* Save the modified register values */

  pwm_putreg(priv, STM32_GTIM_CR2_OFFSET, cr2);
  pwm_putreg(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);
  pwm_putreg(priv, STM32_GTIM_CCMR2_OFFSET, ccmr2);
  pwm_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Set the ARR Preload Bit */

  cr1 = pwm_getreg(priv, STM32_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_ARPE;
  pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Setup update interrupt.  If info->count is > 0, then we can be
   * assured that pwm_start() has already verified: (1) that this is an
   * advanced timer, and that (2) the repetition count is within range.
   */

#ifdef CONFIG_PWM_PULSECOUNT
  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
      pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      cr1 |= GTIM_CR1_CEN;
      pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }
  else
#endif
    {
      /* Just enable the timer, leaving all interrupts disabled */

      cr1 |= GTIM_CR1_CEN;
      pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);
    }

  pwm_dumpregs(priv, "After starting");
  return OK;
}

#ifndef CONFIG_PWM_PULSECOUNT
/****************************************************************************
 * Name: pwm_update_duty
 *
 * Description:
 *   Try to change only channel duty.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static  int pwm_update_duty(struct stm32_pwmtimer_s *priv,
                            uint8_t channel, ub16_t duty)
{
  /* Register offset */

  int ccr_offset;

  /* Calculated values */

  uint32_t reload;
  uint32_t ccr;

  DEBUGASSERT(priv != NULL);

  pwminfo("TIM%u channel: %u duty: %08lx\n",
          priv->timid, channel, duty);

#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Get the reload values */

  reload = pwm_getreg(priv, STM32_GTIM_ARR_OFFSET);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  pwminfo("ccr: %lu\n", ccr);

  switch (channel)
    {
      case 1:  /* Register offset for Channel 1 */
        ccr_offset = STM32_GTIM_CCR1_OFFSET;
        break;

      case 2:  /* Register offset for Channel 2 */
        ccr_offset = STM32_GTIM_CCR2_OFFSET;
        break;

      case 3:  /* Register offset for Channel 3 */
        ccr_offset = STM32_GTIM_CCR3_OFFSET;
        break;

      case 4:  /* Register offset for Channel 4 */
        ccr_offset = STM32_GTIM_CCR4_OFFSET;
        break;

      default:
        pwmerr("ERROR: No such channel: %u\n", channel);
        return -EINVAL;
    }

  /* Set the duty cycle by writing to the CCR register for this channel */

  pwm_putreg(priv, ccr_offset, (uint16_t)ccr);

  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM8_PWM))
static int pwm_interrupt(struct stm32_pwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pwm_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrupts, stop and reset the timer */

      pwm_stop((struct pwm_lowerhalf_s *)priv);

      /* Then perform the callback into the upper half driver */

      pwm_expired(priv->handle);

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
      priv->curr = pwm_pulsecount(priv->count - priv->prev);
      pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output
   */

  pwminfo("Update interrupt SR: %04x prev: %u curr: %u count: %u\n",
          regval, priv->prev, priv->curr, priv->count);

  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_tim1/8interrupt
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

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32F7_TIM1_PWM)
static int pwm_tim1interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt(&g_pwm1dev);
}
#endif

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32F7_TIM8_PWM)
static int pwm_tim8interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt(&g_pwm8dev);
}
#endif

/****************************************************************************
 * Name: pwm_pulsecount
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

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM8_PWM))
static uint8_t pwm_pulsecount(uint32_t count)
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
#endif

/****************************************************************************
 * Name: pwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void pwm_set_apb_clock(struct stm32_pwmtimer_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  pwminfo("timer %d clock enable: %d\n", priv->timid, on ? 1 : 0);

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32F7_TIM1_PWM
      case 1:
        regaddr  = TIMRCCEN_TIM1;
        en_bit   = TIMEN_TIM1;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2_PWM
      case 2:
        regaddr  = TIMRCCEN_TIM2;
        en_bit   = TIMEN_TIM2;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3_PWM
      case 3:
        regaddr  = TIMRCCEN_TIM3;
        en_bit   = TIMEN_TIM3;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4_PWM
      case 4:
        regaddr  = TIMRCCEN_TIM4;
        en_bit   = TIMEN_TIM4;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5_PWM
      case 5:
        regaddr  = TIMRCCEN_TIM5;
        en_bit   = TIMEN_TIM5;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8_PWM
      case 8:
        regaddr  = TIMRCCEN_TIM8;
        en_bit   = TIMEN_TIM8;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9_PWM
      case 9:
        regaddr  = TIMRCCEN_TIM9;
        en_bit   = TIMEN_TIM9;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10_PWM
      case 10:
        regaddr  = TIMRCCEN_TIM10;
        en_bit   = TIMEN_TIM10;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11_PWM
      case 11:
        regaddr  = TIMRCCEN_TIM11;
        en_bit   = TIMEN_TIM11;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12_PWM
      case 12:
        regaddr  = TIMRCCEN_TIM12;
        en_bit   = TIMEN_TIM12;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13_PWM
      case 13:
        regaddr  = TIMRCCEN_TIM13;
        en_bit   = TIMEN_TIM13;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14_PWM
      case 14:
        regaddr  = TIMRCCEN_TIM14;
        en_bit   = TIMEN_TIM14;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM15_PWM
      case 15:
        regaddr  = TIMRCCEN_TIM15;
        en_bit   = TIMEN_TIM15;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM16_PWM
      case 16:
        regaddr  = TIMRCCEN_TIM16;
        en_bit   = TIMEN_TIM16;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM17_PWM
      case 17:
        regaddr  = TIMRCCEN_TIM17;
        en_bit   = TIMEN_TIM17;
        break;
#endif
      default:
        return;
    }

  /* Enable/disable APB 1/2 clock for timer */

  pwminfo("RCC_APBxENR base: %08lx bits: %04lx\n",
            regaddr, en_bit);

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
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);

  /* Enable APB1/2 clocking for timer. */

  pwm_set_apb_clock(priv, true);

  pwm_dumpregs(priv, "Initially");

  /* Configure the PWM output pins, but do not start the timer yet */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg == 0)
        {
          continue;
        }

      pwminfo("pincfg: %08lx\n", pincfg);

      stm32_configgpio(pincfg);
      pwm_dumpgpio(pincfg, "PWM setup");
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  pwm_set_apb_clock(priv, false);

  /* Then put the GPIO pins back to the default state */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg == 0)
        {
          continue;
        }

      pwminfo("pincfg: %08lx\n", pincfg);

      pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
      pincfg |= GPIO_INPUT | GPIO_FLOAT;

      stm32_configgpio(pincfg);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info,
                     void *handle)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* Check if a pulsecount has been selected */

  if (info->count > 0)
    {
      /* Only the advanced timers (TIM1,8 can support the pulse counting) */

      if (priv->timtype != TIMTYPE_ADVANCED)
        {
          pwmerr("ERROR: TIM%u cannot support pulse count: %u\n",
                 priv->timid, info->count);
          return -EPERM;
        }
    }

  /* Save the handle */

  priv->handle = handle;

  /* Start the time */

  return pwm_timer(priv, info);
}
#else
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  int ret = OK;
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

#ifndef CONFIG_PWM_PULSECOUNT
  /* if frequency has not changed we just update duty */

  if (info->frequency == priv->frequency)
    {
#ifdef CONFIG_PWM_MULTICHAN
      int i;

      for (i = 0; ret == OK && i < CONFIG_PWM_NCHANNELS; i++)
        {
          /* Break the loop if all following channels are not configured */

          if (info->channels[i].channel == -1)
            {
              break;
            }

          /* Set output if channel configured */

          if (info->channels[i].channel != 0)
            {
              ret = pwm_update_duty(priv, info->channels[i].channel,
                                    info->channels[i].duty);
            }
        }
#else
      ret = pwm_update_duty(priv, priv->channels[0].channel,
                            info->duty);
#endif
    }
  else
#endif
    {
      ret = pwm_timer(priv, info);

#ifndef CONFIG_PWM_PULSECOUNT
      /* Save current frequency */

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
#endif
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
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

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  pwminfo("TIM%u\n", priv->timid);

  /* Determine which timer to reset */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32F7_TIM1_PWM
      case 1:
        regaddr  = TIMRCCRST_TIM1;
        resetbit = TIMRST_TIM1;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2_PWM
      case 2:
        regaddr  = TIMRCCRST_TIM2;
        resetbit = TIMRST_TIM2;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3_PWM
      case 3:
        regaddr  = TIMRCCRST_TIM3;
        resetbit = TIMRST_TIM3;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4_PWM
      case 4:
        regaddr  = TIMRCCRST_TIM4;
        resetbit = TIMRST_TIM4;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5_PWM
      case 5:
        regaddr  = TIMRCCRST_TIM5;
        resetbit = TIMRST_TIM5;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM8_PWM
      case 8:
        regaddr  = TIMRCCRST_TIM8;
        resetbit = TIMRST_TIM8;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9_PWM
      case 9:
        regaddr  = TIMRCCRST_TIM9;
        resetbit = TIMRST_TIM9;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10_PWM
      case 10:
        regaddr  = TIMRCCRST_TIM10;
        resetbit = TIMRST_TIM10;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11_PWM
      case 11:
        regaddr  = TIMRCCRST_TIM11;
        resetbit = TIMRST_TIM11;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12_PWM
      case 12:
        regaddr  = TIMRCCRST_TIM12;
        resetbit = TIMRST_TIM12;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13_PWM
      case 13:
        regaddr  = TIMRCCRST_TIM13;
        resetbit = TIMRST_TIM13;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14_PWM
      case 14:
        regaddr  = TIMRCCRST_TIM14;
        resetbit = TIMRST_TIM14;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM15_PWM
      case 15:
        regaddr  = TIMRCCRST_TIM15;
        resetbit = TIMRST_TIM15;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM16_PWM
      case 16:
        regaddr  = TIMRCCRST_TIM16;
        resetbit = TIMRST_TIM16;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM17_PWM
      case 17:
        regaddr  = TIMRCCRST_TIM17;
        resetbit = TIMRST_TIM17;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

#ifndef CONFIG_PWM_PULSECOUNT
  /* Stopped so frequency is zero */

  priv->frequency = 0;
#endif

  /* Disable further interrupts and stop the timer */

  pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where pwm_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  pwminfo("regaddr: %08lx resetbit: %08lx\n", regaddr, resetbit);
  pwm_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%u\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,17}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *stm32_pwminitialize(int timer)
{
  struct stm32_pwmtimer_s *lower;

  pwminfo("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32F7_TIM1_PWM
      case 1:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM1 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_tim1interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32F7_TIM2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM5_PWM
      case 5:
        lower = &g_pwm5dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM8_PWM
      case 8:
        lower = &g_pwm8dev;

        /* Attach but disable the TIM8 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_tim8interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32F7_TIM9_PWM
      case 9:
        lower = &g_pwm9dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM10_PWM
      case 10:
        lower = &g_pwm10dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM11_PWM
      case 11:
        lower = &g_pwm11dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM12_PWM
      case 12:
        lower = &g_pwm12dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM13_PWM
      case 13:
        lower = &g_pwm13dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM14_PWM
      case 14:
        lower = &g_pwm14dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM15_PWM
      case 15:
        lower = &g_pwm15dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM16_PWM
      case 16:
        lower = &g_pwm16dev;
        break;
#endif

#ifdef CONFIG_STM32F7_TIM17_PWM
      case 17:
        lower = &g_pwm17dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_STM32F7_TIMn_PWM, n = 1,...,17 */
