/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwm.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32l4_pwm.h"
#include "stm32l4.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_STM32L4_TIM1_PWM)  || defined(CONFIG_STM32L4_TIM2_PWM)   || \
    defined(CONFIG_STM32L4_TIM3_PWM)  || defined(CONFIG_STM32L4_TIM4_PWM)   || \
    defined(CONFIG_STM32L4_TIM5_PWM)  || defined(CONFIG_STM32L4_TIM8_PWM)   || \
    defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM)  || \
    defined(CONFIG_STM32L4_TIM17_PWM) || defined(CONFIG_STM32L4_LPTIM1_PWM) || \
    defined(CONFIG_STM32L4_LPTIM2_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types */

#define TIMTYPE_BASIC        0  /* Basic timers: TIM6,7 */
#define TIMTYPE_GENERAL16    1  /* General 16-bit timers: TIM3,4 */
#define TIMTYPE_COUNTUP16    2  /* General 16-bit count-up timers: TIM15-17 */
#define TIMTYPE_COUNTUP16_N  3  /* General 16-bit count-up timers with
                                 * complementary outptus
                                 */
#define TIMTYPE_GENERAL32    4  /* General 32-bit timers: TIM2,5 */
#define TIMTYPE_ADVANCED     5  /* Advanced timers:  TIM1,8 */
#define TIMTYPE_LOWPOWER     6  /* Low Power timers: LPTIM1,2 */

#define TIMTYPE_TIM1       TIMTYPE_ADVANCED
#define TIMTYPE_TIM2       TIMTYPE_GENERAL32
#define TIMTYPE_TIM3       TIMTYPE_GENERAL16
#define TIMTYPE_TIM4       TIMTYPE_GENERAL16
#define TIMTYPE_TIM5       TIMTYPE_GENERAL32
#define TIMTYPE_TIM6       TIMTYPE_BASIC
#define TIMTYPE_TIM7       TIMTYPE_BASIC
#define TIMTYPE_TIM8       TIMTYPE_ADVANCED
#define TIMTYPE_TIM15      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM16      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM17      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_LPTIM1     TIMTYPE_LOWPOWER
#define TIMTYPE_LPTIM2     TIMTYPE_LOWPOWER

/* Advanced Timer support
 * NOTE: TIM15-17 are not ADVTIM but they support most of the
 *       ADVTIM functionality.  The main difference is the number of
 *       supported capture/compare.
 */

#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM) || \
    defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM) || \
    defined(CONFIG_STM32L4_TIM17_PWM)
#  define HAVE_ADVTIM
#else
#  undef HAVE_ADVTIM
#endif

/* Low power Timer support */

#if defined(CONFIG_STM32L4_LPTIM1_PWM) || defined(CONFIG_STM32L4_LPTIM2_PWM)
#  define HAVE_LPTIM
#else
#  undef HAVE_LPTIM
#endif

/* Pulsecount support */

#ifdef CONFIG_PWM_PULSECOUNT
#  ifndef HAVE_ADVTIM
#    error "PWM_PULSECOUNT requires HAVE_ADVTIM"
#  endif
#  if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)
#    define HAVE_PWM_INTERRUPT
#  endif
#endif

/* Synchronisation support */

#ifdef CONFIG_STM32L4_PWM_TRGO
#  define HAVE_TRGO
#endif

/* Break support */

#if defined(CONFIG_STM32L4_TIM1_BREAK1) || defined(CONFIG_STM32L4_TIM1_BREAK2) || \
    defined(CONFIG_STM32L4_TIM8_BREAK1) || defined(CONFIG_STM32L4_TIM8_BREAK2) || \
    defined(CONFIG_STM32L4_TIM15_BREAK1) || defined(CONFIG_STM32L4_TIM16_BREAK1) || \
    defined(CONFIG_STM32L4_TIM17_BREAK1)
#  defined HAVE_BREAK
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) stm32l4_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PWM output configuration */

struct stm32l4_pwm_out_s
{
  uint8_t  in_use:1;                    /* Output in use */
  uint8_t  pol:1;                       /* Polarity. Default: positive */
  uint8_t  idle:1;                      /* Idle state. Default: inactive */
  uint8_t  _res:5;                      /* Reserved */
  uint32_t pincfg;                      /* Output pin configuration */
};

/* PWM channel configuration */

struct stm32l4_pwmchan_s
{
  uint8_t                    channel:4;   /* Timer output channel: {1,..4} */
  uint8_t                    mode:4;      /* PWM channel mode (see stm32l4_pwm_chanmode_e) */
  struct stm32l4_pwm_out_s   out1;        /* PWM output configuration */
#ifdef HAVE_BREAK
  struct stm32l4_pwm_break_s brk;         /* PWM break configuration */
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  struct stm32l4_pwm_out_s   out2;        /* PWM complementary output configuration */
#endif
};

/* This structure represents the state of one PWM timer */

struct stm32l4_pwmtimer_s
{
  FAR const struct pwm_ops_s *ops;      /* PWM operations */
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  FAR const struct stm32l4_pwm_ops_s *llops; /* Low-level PWM ops */
#endif
  FAR struct stm32l4_pwmchan_s *channels; /* Channels configuration */

  uint8_t  timid:5;                     /* Timer ID {1,...,17} */
  uint8_t  chan_num:3;                  /* Number of configured channels */
  uint8_t  timtype:3;                   /* See the TIMTYPE_* definitions */
  uint8_t  mode:3;                      /* Timer mode (see stm32l4_pwm_tim_mode_e) */
  uint8_t  lock:2;                      /* TODO: Lock configuration */
  uint8_t  t_dts:3;                     /* Clock division for t_DTS */
  uint8_t  _res:5;                      /* Reserved */
#ifdef HAVE_PWM_COMPLEMENTARY
  uint8_t  deadtime;                    /* Dead-time value */
#endif
#ifdef HAVE_TRGO
  uint8_t  trgo;                        /* TRGO configuration:
                                         * 4 LSB = TRGO, 4 MSB = TRGO2
                                         */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t irq;                         /* Timer update IRQ */
  uint8_t prev;                        /* The previous value of the RCR (pre-loaded) */
  uint8_t curr;                        /* The current value of the RCR (pre-loaded) */
  uint32_t count;                      /* Remaining pulse count */
#else
  uint32_t frequency;                  /* Current frequency setting */
#endif
  uint32_t base;                       /* The base address of the timer */
  uint32_t pclk;                       /* Frequency of the peripheral clock
                                        * that drives the timer module. */
#ifdef CONFIG_PWM_PULSECOUNT
  FAR void *handle;                    /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint16_t pwm_getreg(struct stm32l4_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct stm32l4_pwmtimer_s *priv, int offset,
                       uint16_t value);
static void pwm_modifyreg(struct stm32l4_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(FAR struct pwm_lowerhalf_s *dev,
                         FAR const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_pulsecount_timer(FAR struct pwm_lowerhalf_s *dev,
                                FAR const struct pwm_info_s *info);
#else
static int pwm_timer(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);
#  ifdef HAVE_LPTIM
static int pwm_lptimer(FAR struct pwm_lowerhalf_s *dev,
                       FAR const struct pwm_info_s *info);
#  endif
#endif

static int pwm_configure(FAR struct pwm_lowerhalf_s *dev);

static int pwm_soft_break(FAR struct pwm_lowerhalf_s *dev, bool state); /* REVISIT: valid for all timers? */
static int pwm_ccr_update(FAR struct pwm_lowerhalf_s *dev, uint8_t index,
                          uint32_t ccr);
static int pwm_mode_configure(FAR struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode);
#ifdef CONFIG_STM32L4_PWM_LL_OPS
static uint32_t pwm_ccr_get(FAR struct pwm_lowerhalf_s *dev, uint8_t index);
#endif
static int pwm_arr_update(FAR struct pwm_lowerhalf_s *dev, uint32_t arr);
static uint32_t pwm_arr_get(FAR struct pwm_lowerhalf_s *dev);
static int pwm_outputs_enable(FAR struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state);
static int pwm_soft_update(FAR struct pwm_lowerhalf_s *dev);
static int pwm_frequency_update(FAR struct pwm_lowerhalf_s *dev,
                                uint32_t frequency);
static int pwm_timer_enable(FAR struct pwm_lowerhalf_s *dev, bool state);
#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_STM32L4_PWM_LL_OPS)
static int pwm_deadtime_update(FAR struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
#ifdef CONFIG_STM32L4_PWM_LL_OPS
static uint32_t pwm_ccr_get(FAR struct pwm_lowerhalf_s *dev, uint8_t index);
#endif

#ifdef HAVE_PWM_INTERRUPT
static int pwm_interrupt(FAR struct pwm_lowerhalf_s *dev);
#  ifdef CONFIG_STM32L4_TIM1_PWM
static int pwm_tim1interrupt(int irq, void *context, FAR void *arg);
#  endif
#  ifdef CONFIG_STM32L4_TIM8_PWM
static int pwm_tim8interrupt(int irq, void *context, FAR void *arg);
#  endif
static uint8_t pwm_pulsecount(uint32_t count);
#endif

/* PWM driver methods */

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info,
                     FAR void *handle);
#else
static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info);
#endif

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_STM32L4_PWM_LL_OPS
static const struct stm32l4_pwm_ops_s g_llpwmops =
{
  .configure       = pwm_configure,
  .soft_break      = pwm_soft_break,
  .ccr_update      = pwm_ccr_update,
  .mode_update     = pwm_mode_configure,
  .ccr_get         = pwm_ccr_get,
  .arr_update      = pwm_arr_update,
  .arr_get         = pwm_arr_get,
  .outputs_enable  = pwm_outputs_enable,
  .soft_update     = pwm_soft_update,
  .freq_update     = pwm_frequency_update,
  .tim_enable      = pwm_timer_enable,
#  ifdef CONFIG_DEBUG_PWM_INFO
  .dump_regs       = pwm_dumpregs,
#  endif
#  ifdef HAVE_PWM_COMPLEMENTARY
  .dt_update       = pwm_deadtime_update,
#  endif
};
#endif

#ifdef CONFIG_STM32L4_TIM1_PWM

static struct stm32l4_pwmchan_s g_pwm1channels[] =
{
  /* TIM1 has 4 channels, 4 complementary */

#ifdef CONFIG_STM32L4_TIM1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM1_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32L4_TIM1_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32L4_TIM1_BRK1POL,
#endif
#ifdef CONFIG_STM32L4_TIM1_BREAK2
      .en2 = 1,
      .pol2 = CONFIG_STM32L4_TIM1_BRK2POL,
      .flt2 = CONFIG_STM32L4_TIM1_BRK2FLT,
#endif
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH1POL,
      .idle    = CONFIG_STM32L4_TIM1_CH1IDLE,
      .pincfg  = PWM_TIM1_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH1NPOL,
      .idle    = CONFIG_STM32L4_TIM1_CH1NIDLE,
      .pincfg  = PWM_TIM1_CH1NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM1_CH2MODE,
#ifdef CONFIG_STM32L4_TIM1_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH2POL,
      .idle    = CONFIG_STM32L4_TIM1_CH2IDLE,
      .pincfg  = PWM_TIM1_CH2CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH2NPOL,
      .idle    = CONFIG_STM32L4_TIM1_CH2NIDLE,
      .pincfg  = PWM_TIM1_CH2NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM1_CH3MODE,
#ifdef CONFIG_STM32L4_TIM1_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH3POL,
      .idle    = CONFIG_STM32L4_TIM1_CH3IDLE,
      .pincfg  = PWM_TIM1_CH3CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH3NPOL,
      .idle    = CONFIG_STM32L4_TIM1_CH3NIDLE,
      .pincfg  = PWM_TIM1_CH3NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM1_CH4MODE,
#ifdef CONFIG_STM32L4_TIM1_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4L4_TIM1_CH4POL,
      .idle    = CONFIG_STM32L4L4_TIM1_CH4IDLE,
      .pincfg  = PWM_TIM1_CH4CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL5
  {
    .channel = 5,
    .mode    = CONFIG_STM32L4_TIM1_CH5MODE,
#ifdef CONFIG_STM32L4_TIM1_CH5OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH5POL,
      .idle    = CONFIG_STM32L4_TIM1_CH5IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL6
  {
    .channel = 6,
    .mode    = CONFIG_STM32L4_TIM1_CH6MODE,
#ifdef CONFIG_STM32L4_TIM1_CH6OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM1_CH6POL,
      .idle    = CONFIG_STM32L4_TIM1_CH6IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  }
#endif
};

static struct stm32l4_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 1,
  .chan_num    = PWM_TIM1_NCHANNELS,
  .channels    = g_pwm1channels,
  .timtype     = TIMTYPE_TIM1,
  .mode        = CONFIG_STM32L4_TIM1_MODE,
  .lock        = CONFIG_STM32L4_TIM1_LOCK,
  .t_dts       = CONFIG_STM32L4_TIM1_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32L4_TIM1_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM1_TRGO)
  .trgo        = STM32L4_TIM1_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM1UP,
#endif
  .base        = STM32L4_TIM1_BASE,
  .pclk        = STM32L4_APB2_TIM1_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM1_PWM */

#ifdef CONFIG_STM32L4_TIM2_PWM

static struct stm32l4_pwmchan_s g_pwm2channels[] =
{
  /* TIM2 has 4 channels */

#ifdef CONFIG_STM32L4_TIM2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM2_CH1MODE,
#ifdef CONFIG_STM32L4_TIM2_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM2_CH1POL,
      .idle    = CONFIG_STM32L4_TIM2_CH1IDLE,
      .pincfg  = PWM_TIM2_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM2_CH2MODE,
#ifdef CONFIG_STM32L4_TIM2_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM2_CH2POL,
      .idle    = CONFIG_STM32L4_TIM2_CH2IDLE,
      .pincfg  = PWM_TIM2_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM2_CH3MODE,
#ifdef CONFIG_STM32L4_TIM2_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM2_CH3POL,
      .idle    = CONFIG_STM32L4_TIM2_CH3IDLE,
      .pincfg  = PWM_TIM2_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM2_CH4MODE,
#ifdef CONFIG_STM32L4_TIM2_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM2_CH4POL,
      .idle    = CONFIG_STM32L4_TIM2_CH4IDLE,
      .pincfg  = PWM_TIM2_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32l4_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 2,
  .chan_num    = PWM_TIM2_NCHANNELS,
  .channels    = g_pwm2channels,
  .timtype     = TIMTYPE_TIM2,
  .mode        = CONFIG_STM32L4_TIM2_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM2_TRGO)
  .trgo        = STM32L4_TIM2_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM2,
#endif
  .base        = STM32L4_TIM2_BASE,
  .pclk        = STM32L4_APB1_TIM2_CLKIN,
};

#endif /* CONFIG_STM32L4_TIM2_PWM */

#ifdef CONFIG_STM32L4_TIM3_PWM

static struct stm32l4_pwmchan_s g_pwm3channels[] =
{
  /* TIM3 has 4 channels */

#ifdef CONFIG_STM32L4_TIM3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM3_CH1MODE,
#ifdef CONFIG_STM32L4_TIM3_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM3_CH1POL,
      .idle    = CONFIG_STM32L4_TIM3_CH1IDLE,
      .pincfg  = PWM_TIM3_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM3_CH2MODE,
#ifdef CONFIG_STM32L4_TIM3_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM3_CH2POL,
      .idle    = CONFIG_STM32L4_TIM3_CH2IDLE,
      .pincfg  = PWM_TIM3_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM3_CH3MODE,
#ifdef CONFIG_STM32L4_TIM3_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM3_CH3POL,
      .idle    = CONFIG_STM32L4_TIM3_CH3IDLE,
      .pincfg  = PWM_TIM3_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM3_CH4MODE,
#ifdef CONFIG_STM32L4_TIM3_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM3_CH4POL,
      .idle    = CONFIG_STM32L4_TIM3_CH4IDLE,
      .pincfg  = PWM_TIM3_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32l4_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 3,
  .chan_num    = PWM_TIM3_NCHANNELS,
  .channels    = g_pwm3channels,
  .timtype     = TIMTYPE_TIM3,
  .mode        = CONFIG_STM32L4_TIM3_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM3_TRGO)
  .trgo        = STM32L4_TIM3_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM3,
#endif
  .base        = STM32L4_TIM3_BASE,
  .pclk        = STM32L4_APB1_TIM3_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM3_PWM */

#ifdef CONFIG_STM32L4_TIM4_PWM

static struct stm32l4_pwmchan_s g_pwm4channels[] =
{
  /* TIM4 has 4 channels */

#ifdef CONFIG_STM32L4_TIM4_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM4_CH1MODE,
#ifdef CONFIG_STM32L4_TIM4_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM4_CH1POL,
      .idle    = CONFIG_STM32L4_TIM4_CH1IDLE,
      .pincfg  = PWM_TIM4_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM4_CH2MODE,
#ifdef CONFIG_STM32L4_TIM4_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM4_CH2POL,
      .idle    = CONFIG_STM32L4_TIM4_CH2IDLE,
      .pincfg  = PWM_TIM4_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM4_CH3MODE,
#ifdef CONFIG_STM32L4_TIM4_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM4_CH3POL,
      .idle    = CONFIG_STM32L4_TIM4_CH3IDLE,
      .pincfg  = PWM_TIM4_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM4_CH4MODE,
#ifdef CONFIG_STM32L4_TIM4_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM4_CH4POL,
      .idle    = CONFIG_STM32L4_TIM4_CH4IDLE,
      .pincfg  = PWM_TIM4_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32l4_pwmtimer_s g_pwm4dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 4,
  .chan_num    = PWM_TIM4_NCHANNELS,
  .channels    = g_pwm4channels,
  .timtype     = TIMTYPE_TIM4,
  .mode        = CONFIG_STM32L4_TIM4_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM4_TRGO)
  .trgo        = STM32L4_TIM4_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM4,
#endif
  .base        = STM32L4_TIM4_BASE,
  .pclk        = STM32L4_APB1_TIM4_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM4_PWM */

#ifdef CONFIG_STM32L4_TIM5_PWM

static struct stm32l4_pwmchan_s g_pwm5channels[] =
{
  /* TIM5 has 4 channels */

#ifdef CONFIG_STM32L4_TIM5_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM5_CH1MODE,
#ifdef CONFIG_STM32L4_TIM5_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM5_CH1POL,
      .idle    = CONFIG_STM32L4_TIM5_CH1IDLE,
      .pincfg  = PWM_TIM5_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM5_CH2MODE,
#ifdef CONFIG_STM32L4_TIM5_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM5_CH2POL,
      .idle    = CONFIG_STM32L4_TIM5_CH2IDLE,
      .pincfg  = PWM_TIM5_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM5_CH3MODE,
#ifdef CONFIG_STM32L4_TIM5_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM5_CH3POL,
      .idle    = CONFIG_STM32L4_TIM5_CH3IDLE,
      .pincfg  = PWM_TIM5_CH3CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM5_CH4MODE,
#ifdef CONFIG_STM32L4_TIM5_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM5_CH4POL,
      .idle    = CONFIG_STM32L4_TIM5_CH4IDLE,
      .pincfg  = PWM_TIM5_CH4CFG,
    }
#endif
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwm5dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 5,
  .chan_num    = PWM_TIM5_NCHANNELS,
  .channels    = g_pwm5channels,
  .timtype     = TIMTYPE_TIM5,
  .mode        = CONFIG_STM32L4_TIM5_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM5_TRGO)
  .trgo        = STM32L4_TIM5_TRGO
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM5,
#endif
  .base        = STM32L4_TIM5_BASE,
  .pclk        = STM32L4_APB1_TIM5_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM5_PWM */

#ifdef CONFIG_STM32L4_TIM8_PWM

static struct stm32l4_pwmchan_s g_pwm8channels[] =
{
  /* TIM8 has 4 channels, 4 complementary */

#ifdef CONFIG_STM32L4_TIM8_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM8_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32L4_TIM8_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32L4_TIM8_BRK1POL,
#endif
#ifdef CONFIG_STM32L4_TIM8_BREAK2
      .en2 = 1,
      .pol2 = CONFIG_STM32L4_TIM8_BRK2POL,
      .flt2 = CONFIG_STM32L4_TIM8_BRK2FLT,
#endif
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH1POL,
      .idle    = CONFIG_STM32L4_TIM8_CH1IDLE,
      .pincfg  = PWM_TIM8_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH1NPOL,
      .idle    = CONFIG_STM32L4_TIM8_CH1NIDLE,
      .pincfg  = PWM_TIM8_CH1NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM8_CH2MODE,
#ifdef CONFIG_STM32L4_TIM8_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH2POL,
      .idle    = CONFIG_STM32L4_TIM8_CH2IDLE,
      .pincfg  = PWM_TIM8_CH2CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH2NPOL,
      .idle    = CONFIG_STM32L4_TIM8_CH2NIDLE,
      .pincfg  = PWM_TIM8_CH2NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32L4_TIM8_CH3MODE,
#ifdef CONFIG_STM32L4_TIM8_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH3POL,
      .idle    = CONFIG_STM32L4_TIM8_CH3IDLE,
      .pincfg  = PWM_TIM8_CH3CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH3NPOL,
      .idle    = CONFIG_STM32L4_TIM8_CH3NIDLE,
      .pincfg  = PWM_TIM8_CH3NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32L4_TIM8_CH4MODE,
#ifdef CONFIG_STM32L4_TIM8_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH4POL,
      .idle    = CONFIG_STM32L4_TIM8_CH4IDLE,
      .pincfg  = PWM_TIM8_CH4CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL5
  {
    .channel = 5,
    .mode    = CONFIG_STM32L4_TIM8_CH5MODE,
#ifdef CONFIG_STM32L4_TIM8_CH5OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH5POL,
      .idle    = CONFIG_STM32L4_TIM8_CH5IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL6
  {
    .channel = 6,
    .mode    = CONFIG_STM32L4_TIM8_CH6MODE,
#ifdef CONFIG_STM32L4_TIM8_CH6OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM8_CH6POL,
      .idle    = CONFIG_STM32L4_TIM8_CH6IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  }
#endif
};

static struct stm32l4_pwmtimer_s g_pwm8dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 8,
  .chan_num    = PWM_TIM8_NCHANNELS,
  .channels    = g_pwm8channels,
  .timtype     = TIMTYPE_TIM8,
  .mode        = CONFIG_STM32L4_TIM8_MODE,
  .lock        = CONFIG_STM32L4_TIM8_LOCK,
  .t_dts       = CONFIG_STM32L4_TIM8_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32L4_TIM8_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM8_TRGO)
  .trgo        = STM32L4_TIM8_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM8UP,
#endif
  .base        = STM32L4_TIM8_BASE,
  .pclk        = STM32L4_APB2_TIM8_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM8_PWM */

#ifdef CONFIG_STM32L4_TIM15_PWM

static struct stm32l4_pwmchan_s g_pwm15channels[] =
{
  /* TIM15 has 2 channels, 1 complementary */

#ifdef CONFIG_STM32L4_TIM15_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM15_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32L4_TIM15_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32L4_TIM15_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32L4_TIM15_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM15_CH1POL,
      .idle    = CONFIG_STM32L4_TIM15_CH1IDLE,
      .pincfg  = PWM_TIM15_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM15_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM15_CH1NPOL,
      .idle    = CONFIG_STM32L4_TIM15_CH1NIDLE,
      .pincfg  = PWM_TIM15_CH2CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32L4_TIM15_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32L4_TIM15_CH2MODE,
#ifdef CONFIG_STM32L4_TIM15_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM15_CH2POL,
      .idle    = CONFIG_STM32L4_TIM15_CH2IDLE,
      .pincfg  = PWM_TIM15_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwm15dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 15,
  .chan_num    = PWM_TIM15_NCHANNELS,
  .channels    = g_pwm15channels,
  .timtype     = TIMTYPE_TIM15,
  .mode        = STM32L4_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32L4_TIM15_LOCK,
  .t_dts       = CONFIG_STM32L4_TIM15_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32L4_TIM15_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32L4_TIM15_TRGO)
  .trgo        = STM32L4_TIM15_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM15,
#endif
  .base        = STM32L4_TIM15_BASE,
  .pclk        = STM32L4_APB2_TIM15_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM15_PWM */

#ifdef CONFIG_STM32L4_TIM16_PWM

static struct stm32l4_pwmchan_s g_pwm16channels[] =
{
  /* TIM16 has 1 channel, 1 complementary */

#ifdef CONFIG_STM32L4_TIM16_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM16_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32L4_TIM16_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32L4_TIM16_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32L4_TIM16_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM16_CH1POL,
      .idle    = CONFIG_STM32L4_TIM16_CH1IDLE,
      .pincfg  = PWM_TIM16_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM16_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM16_CH1NPOL,
      .idle    = CONFIG_STM32L4_TIM16_CH1NIDLE,
      .pincfg  = PWM_TIM16_CH2CFG,
    }
#endif
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwm16dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 16,
  .chan_num    = PWM_TIM16_NCHANNELS,
  .channels    = g_pwm16channels,
  .timtype     = TIMTYPE_TIM16,
  .mode        = STM32L4_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32L4_TIM16_LOCK,
  .t_dts       = CONFIG_STM32L4_TIM16_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32L4_TIM16_DEADTIME,
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM16 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM16,
#endif
  .base        = STM32L4_TIM16_BASE,
  .pclk        = STM32L4_APB2_TIM16_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM16_PWM */

#ifdef CONFIG_STM32L4_TIM17_PWM

static struct stm32l4_pwmchan_s g_pwm17channels[] =
{
  /* TIM17 has 1 channel, 1 complementary */

#ifdef CONFIG_STM32L4_TIM17_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32L4_TIM17_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32L4_TIM17_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32L4_TIM17_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32L4_TIM17_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM17_CH1POL,
      .idle    = CONFIG_STM32L4_TIM17_CH1IDLE,
      .pincfg  = PWM_TIM17_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM17_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32L4_TIM17_CH1NPOL,
      .idle    = CONFIG_STM32L4_TIM17_CH1NIDLE,
      .pincfg  = PWM_TIM17_CH2CFG,
    }
#endif
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwm17dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 17,
  .chan_num    = PWM_TIM17_NCHANNELS,
  .channels    = g_pwm17channels,
  .timtype     = TIMTYPE_TIM17,
  .mode        = STM32L4_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32L4_TIM17_LOCK,
  .t_dts       = CONFIG_STM32L4_TIM17_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32L4_TIM17_DEADTIME,
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM17 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM17,
#endif
  .base        = STM32L4_TIM17_BASE,
  .pclk        = STM32L4_APB2_TIM17_CLKIN,
};
#endif /* CONFIG_STM32L4_TIM17_PWM */

#ifdef CONFIG_STM32L4_LPTIM1_PWM

static struct stm32l4_pwmchan_s g_pwmlp1channels[] =
{
  /* LPTIM1 has 1 channel */

#ifdef CONFIG_STM32L4_LPTIM1_CHANNEL1
  {
    .channel = 1,
    .mode    = 0,
#ifdef CONFIG_STM32L4_LPTIM1_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = 0,             /* REVISIT: Configure using CONFIG_STM32L4_LPTIM1_CH1POL, */
      .idle    = 0,             /* No idle */
      .pincfg  = PWM_LPTIM1_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwmlp1dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 1,
  .chan_num    = PWM_LPTIM1_NCHANNELS,
  .channels    = g_pwmlp1channels,
  .timtype     = TIMTYPE_LPTIM1,
  .mode        = STM32L4_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for LPTIM1 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_LPTIM1,
#endif
  .base        = STM32L4_LPTIM1_BASE,
#if defined(CONFIG_STM32L4_LPTIM1_CLK_APB1)
  .pclk        = STM32L4_PCLK1_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_LSE)
  .pclk        = STM32L4_LSE_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_LSI)
  .pclk        = STM32L4_LSI_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_HSI)
  .pclk        = STM32L4_HSI_FREQUENCY,
#endif
};
#endif /* CONFIG_STM32L4_LPTIM1_PWM */

#ifdef CONFIG_STM32L4_LPTIM2_PWM

static struct stm32l4_pwmchan_s g_pwmlp2channels[] =
{
  /* LPTIM2 has 1 channel */

#ifdef CONFIG_STM32L4_LPTIM2_CHANNEL1
  {
    .channel = 1,
    .mode    = 0,
#ifdef CONFIG_STM32L4_LPTIM2_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = 0,             /* REVISIT: Configure using CONFIG_STM32L4_LPTIM2_CH1POL, */
      .idle    = 0,             /* No idle */
      .pincfg  = PWM_LPTIM2_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
};

static struct stm32l4_pwmtimer_s g_pwmlp2dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32L4_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 2,
  .chan_num    = PWM_LPTIM2_NCHANNELS,
  .channels    = g_pwmlp2channels,
  .timtype     = TIMTYPE_LPTIM2,
  .mode        = STM32L4_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for LPTIM2 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_LPTIM2,
#endif
  .base        = STM32L4_LPTIM2_BASE,
#if defined(CONFIG_STM32L4_LPTIM2_CLK_APB1)
  .pclk        = STM32L4_PCLK1_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_LSE)
  .pclk        = STM32L4_LSE_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_LSI)
  .pclk        = STM32L4_LSI_FREQUENCY,
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_HSI)
  .pclk        = STM32L4_HSI_FREQUENCY,
#endif
};
#endif /* CONFIG_STM32L4_LPTIM2_PWM */

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

static uint16_t pwm_getreg(struct stm32l4_pwmtimer_s *priv, int offset)
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

static void pwm_putreg(struct stm32l4_pwmtimer_s *priv, int offset,
                              uint16_t value)
{
  if (priv->timtype == TIMTYPE_GENERAL32 &&
      (offset == STM32L4_GTIM_CNT_OFFSET ||
       offset == STM32L4_GTIM_ARR_OFFSET ||
       offset == STM32L4_GTIM_CCR1_OFFSET ||
       offset == STM32L4_GTIM_CCR2_OFFSET ||
       offset == STM32L4_GTIM_CCR3_OFFSET ||
       offset == STM32L4_GTIM_CCR4_OFFSET))
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
 * Name: pwm_modifyreg
 *
 * Description:
 *   Modify PWM register (32-bit or 16-bit)
 *
 * Input Parameters:
 *   priv    - A reference to the PWM block status
 *   offset  - The offset to the register to read
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_modifyreg(struct stm32l4_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits)
{
  if (priv->timtype == TIMTYPE_GENERAL32 &&
      (offset == STM32L4_GTIM_CNT_OFFSET ||
       offset == STM32L4_GTIM_ARR_OFFSET ||
       offset == STM32L4_GTIM_CCR1_OFFSET ||
       offset == STM32L4_GTIM_CCR2_OFFSET ||
       offset == STM32L4_GTIM_CCR3_OFFSET ||
       offset == STM32L4_GTIM_CCR4_OFFSET))
    {
      /* a 32 bit access is required for a 32 bit register:
       * if only a 16 bit write would be performed, then the
       * upper 16 bits of the 32 bit register will be a copy of
       * the lower 16 bits.
       */

      modifyreg32(priv->base + offset, clearbits, setbits);
    }
  else
    {
      modifyreg16(priv->base + offset, (uint16_t)clearbits,
                  (uint16_t)setbits);
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
static void pwm_dumpregs(FAR struct pwm_lowerhalf_s *dev,
                         FAR const char *msg)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      pwminfo("%s:\n", msg);
      pwminfo("  CFGR: %04x CR:  %04x CMP:  %04x ARR:  %04x\n",
              pwm_getreg(priv, STM32L4_LPTIM_CFGR_OFFSET),
              pwm_getreg(priv, STM32L4_LPTIM_CR_OFFSET),
              pwm_getreg(priv, STM32L4_LPTIM_CMP_OFFSET),
              pwm_getreg(priv, STM32L4_LPTIM_ARR_OFFSET));
      pwminfo("  ISR:  %04x CNT: %04x\n",
              pwm_getreg(priv, STM32L4_LPTIM_ISR_OFFSET),
              pwm_getreg(priv, STM32L4_LPTIM_CNT_OFFSET));
    }
  else
    {
      pwminfo("%s:\n", msg);
      pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
              pwm_getreg(priv, STM32L4_GTIM_CR1_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CR2_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_SMCR_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_DIER_OFFSET));
      pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
              pwm_getreg(priv, STM32L4_GTIM_SR_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_EGR_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CCMR1_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CCMR2_OFFSET));
      pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
              pwm_getreg(priv, STM32L4_GTIM_CCER_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CNT_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_PSC_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_ARR_OFFSET));
      pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
              pwm_getreg(priv, STM32L4_GTIM_CCR1_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CCR2_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CCR3_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)
      if (priv->timtype == TIMTYPE_ADVANCED)
        {
          pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
              pwm_getreg(priv, STM32L4_ATIM_RCR_OFFSET),
              pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET),
              pwm_getreg(priv, STM32L4_ATIM_DCR_OFFSET),
              pwm_getreg(priv, STM32L4_ATIM_DMAR_OFFSET));
        }
      else
#endif
        {
          pwminfo("  DCR: %04x DMAR: %04x\n",
              pwm_getreg(priv, STM32L4_GTIM_DCR_OFFSET),
              pwm_getreg(priv, STM32L4_GTIM_DMAR_OFFSET));
        }
    }
}
#endif

/****************************************************************************
 * Name: pwm_ccr_update
 ****************************************************************************/

static int pwm_ccr_update(FAR struct pwm_lowerhalf_s *dev, uint8_t index,
                          uint32_t ccr)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t offset = 0;

#ifdef HAVE_LPTIM
  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      /* REVISIT: What about index? Is it necessary for LPTIM? */

      offset = STM32L4_LPTIM_CMP_OFFSET;
      pwm_putreg(priv, offset, ccr);

      return OK;
    }
#endif

  /* Only ADV timers have CC5 and CC6 */

  if (priv->timtype != TIMTYPE_ADVANCED && (index == 5 || index == 6))
    {
      pwmerr("ERROR: No such CCR: %u\n", index);
      return -EINVAL;
    }

  /* REVISIT: start index from 0? */

  switch (index)
    {
      case STM32L4_PWM_CHAN1:
        {
          offset = STM32L4_GTIM_CCR1_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN2:
        {
          offset = STM32L4_GTIM_CCR2_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN3:
        {
          offset = STM32L4_GTIM_CCR3_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN4:
        {
          offset = STM32L4_GTIM_CCR4_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN5:
        {
          offset = STM32L4_ATIM_CCR5_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN6:
        {
          offset = STM32L4_ATIM_CCR6_OFFSET;
          break;
        }

      default:
        {
          pwmerr("ERROR: No such CCR: %u\n", index);
          return -EINVAL;
        }
    }

  /* Update CCR register */

  pwm_putreg(priv, offset, ccr);

  return OK;
}

/****************************************************************************
 * Name: pwm_ccr_get
 ****************************************************************************/

#ifdef CONFIG_STM32L4_PWM_LL_OPS
static uint32_t pwm_ccr_get(FAR struct pwm_lowerhalf_s *dev, uint8_t index)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t offset = 0;

  switch (index)
    {
      case STM32L4_PWM_CHAN1:
        {
          offset = STM32L4_GTIM_CCR1_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN2:
        {
          offset = STM32L4_GTIM_CCR2_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN3:
        {
          offset = STM32L4_GTIM_CCR3_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN4:
        {
          offset = STM32L4_GTIM_CCR4_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN5:
        {
          offset = STM32L4_ATIM_CCR5_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN6:
        {
          offset = STM32L4_ATIM_CCR6_OFFSET;
          break;
        }

      default:
        {
          pwmerr("ERROR: No such CCR: %u\n", index);
          return -EINVAL;
        }
    }

  /* Return CCR register */

  return pwm_getreg(priv, offset);
}
#endif /* CONFIG_STM32L4_PWM_LL_OPS */

/****************************************************************************
 * Name: pwm_arr_update
 ****************************************************************************/

static int pwm_arr_update(FAR struct pwm_lowerhalf_s *dev, uint32_t arr)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  /* Update ARR register */

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      pwm_putreg(priv, STM32L4_LPTIM_ARR_OFFSET, arr);
    }
  else
    {
      pwm_putreg(priv, STM32L4_GTIM_ARR_OFFSET, arr);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_arr_get
 ****************************************************************************/

static uint32_t pwm_arr_get(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      return pwm_getreg(priv, STM32L4_LPTIM_ARR_OFFSET);
    }
  else
    {
      return pwm_getreg(priv, STM32L4_GTIM_ARR_OFFSET);
    }
}

/****************************************************************************
 * Name: pwm_duty_update
 *
 * Description:
 *   Try to change only channel duty
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_duty_update(FAR struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t reload = 0;
  uint32_t ccr    = 0;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL);

  pwminfo("TIM%u channel: %u duty: %08" PRIx32 "\n",
          priv->timid, channel, duty);

#ifndef CONFIG_STM32L4_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Get the reload values */

  reload = pwm_arr_get(dev);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  pwminfo("ccr: %" PRIu32 "\n", ccr);

  /* Write corresponding CCR register */

  pwm_ccr_update(dev, channel, ccr);

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_enable
 ****************************************************************************/

static int pwm_timer_enable(FAR struct pwm_lowerhalf_s *dev, bool state)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

#ifdef HAVE_LPTIM
  if (priv->timtype != TIMTYPE_LOWPOWER)
    {
#endif
      if (state == true)
        {
          /* Enable timer counter */

          pwm_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
        }
      else
        {
          /* Disable timer counter */

          pwm_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
        }
#ifdef HAVE_LPTIM
    }
  else
    {
      if (state == true)
        {
          /* Enable timer counter */

          pwm_modifyreg(priv, STM32L4_LPTIM_CR_OFFSET, 0, LPTIM_CR_ENABLE);
        }
      else
        {
          /* Disable timer counter */

          pwm_modifyreg(priv, STM32L4_LPTIM_CR_OFFSET, LPTIM_CR_ENABLE, 0);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: pwm_frequency_update
 *
 * Description:
 *   Update a PWM timer frequency
 *
 ****************************************************************************/

static int pwm_frequency_update(FAR struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
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

  /* If timer mode is center aligned the frequency of PWM is the half of
   * intended so multiply by x2
   */

  if ((priv->mode == STM32L4_TIMMODE_CENTER1) ||
      (priv->mode == STM32L4_TIMMODE_CENTER2) ||
      (priv->mode == STM32L4_TIMMODE_CENTER3))
    {
      frequency = frequency * 2;
    }

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

  pwminfo("TIM%u PCLK: %" PRIu32 " frequency: %" PRIu32 " TIMCLK: %" PRIu32
          " prescaler: %" PRIu32 " reload: %" PRIu32 "\n",
          priv->timid, priv->pclk, frequency, timclk, prescaler, reload);

  /* Set the reload and prescaler values */

  pwm_arr_update(dev, reload);
  pwm_putreg(priv, STM32L4_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  return OK;
}

/****************************************************************************
 * Name: pwm_lp_frequency_update
 *
 * Description:
 *   Update a PWM timer frequency
 *
 ****************************************************************************/

#ifdef HAVE_LPTIM
static int pwm_lp_frequency_update(FAR struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  /* Calculated values */

  uint8_t prescaler;
  uint32_t timclk;
  uint32_t reload;

  /* Register contents */

  uint32_t cfgr;

  /* LPTIM only has 8 possible prescaler values, from /1 to /128
   * We will attempt to find the lowest prescaler that results
   * in a maximum reload value which can be represented in 16 bit.
   * For certain desired frequencies it is possible that the clock
   * is too high and other one needs to be selected.
   */

  for (prescaler = 0; prescaler < 8; prescaler++)
    {
      timclk = priv->pclk / (1 << prescaler);
      reload = timclk / frequency;

      if (reload <= 65535)
        {
          /* The reload counter is feasible, go with it */

          break;
        }
    }

  if (reload < 2)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }

  /* TODO: if the desired frequency is not possible this should give an error
   * and not simply return the feasible frequency without complaining.
   */

  pwminfo("LPTIM%u PCLK: %u frequency: %u TIMCLK: %u "
          "prescaler: %u reload: %u\n",
          priv->timid, priv->pclk, frequency, timclk, prescaler, reload);

  /* Set the reload register value */

  pwm_arr_update(dev, reload);

  /* Set the prescaler value */

  cfgr = pwm_getreg(priv, STM32L4_LPTIM_CFGR_OFFSET);

  cfgr &= ~LPTIM_CFGR_PRESC_MASK;
  cfgr |= (prescaler << LPTIM_CFGR_PRESC_SHIFT);

  pwm_putreg(priv, STM32L4_LPTIM_CFGR_OFFSET, cfgr);

  return OK;
}
#endif /* HAVE_LPTIM */

/****************************************************************************
 * Name: pwm_timer_configure
 *
 * Description:
 *   Initial configuration for PWM timer
 *
 ****************************************************************************/

static int pwm_timer_configure(FAR struct stm32l4_pwmtimer_s *priv)
{
  uint16_t cr1 = 0;
  int      ret = OK;

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = pwm_getreg(priv, STM32L4_GTIM_CR1_OFFSET);

  /* Set the counter mode for the advanced timers (1,8) and most general
   * purpose timers (all 2-5, but not 9-17), i.e., all but TIMTYPE_COUNTUP16
   * and TIMTYPE_BASIC
   */

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
          case STM32L4_TIMMODE_COUNTUP:
            {
              cr1 |= GTIM_CR1_EDGE;
              break;
            }

          case STM32L4_TIMMODE_COUNTDOWN:
            {
              cr1 |= GTIM_CR1_EDGE | GTIM_CR1_DIR;
              break;
            }

          case STM32L4_TIMMODE_CENTER1:
            {
              cr1 |= GTIM_CR1_CENTER1;
              break;
            }

          case STM32L4_TIMMODE_CENTER2:
            {
              cr1 |= GTIM_CR1_CENTER2;
              break;
            }

          case STM32L4_TIMMODE_CENTER3:
            {
              cr1 |= GTIM_CR1_CENTER3;
              break;
            }

          default:
            {
              pwmerr("ERROR: No such timer mode: %u\n",
                     (unsigned int)priv->mode);
              ret = -EINVAL;
              goto errout;
            }
        }
    }

  /* Enable ARR Preload
   * TODO: this should be configurable
   */

  cr1 |= GTIM_CR1_ARPE;

  /* Write CR1 */

  pwm_putreg(priv, STM32L4_GTIM_CR1_OFFSET, cr1);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_mode_configure
 *
 * Description:
 *   Configure a PWM mode for given channel
 *
 ****************************************************************************/

static int pwm_mode_configure(FAR struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t chanmode = 0;
  uint32_t ocmode   = 0;
  uint32_t ccmr     = 0;
  uint32_t offset   = 0;
  int      ret      = OK;
  bool     ocmbit   = false;

  /* Only advanced timers have channels 5-6 */

  if (channel > 4 && priv->timtype != TIMTYPE_ADVANCED)
    {
      pwmerr("ERROR: No such channel: %u\n", channel);
      ret = -EINVAL;
      goto errout;
    }

  /* Get channel mode
   * TODO: configurable preload for CCxR
   */

  switch (mode)
    {
      case STM32L4_CHANMODE_FRZN:
        {
          chanmode = GTIM_CCMR_MODE_FRZN;
          break;
        }

      case STM32L4_CHANMODE_CHACT:
        {
          chanmode = GTIM_CCMR_MODE_CHACT;
          break;
        }

      case STM32L4_CHANMODE_CHINACT:
        {
          chanmode = GTIM_CCMR_MODE_CHINACT;
          break;
        }

      case STM32L4_CHANMODE_OCREFTOG:
        {
          chanmode = GTIM_CCMR_MODE_OCREFTOG;
          break;
        }

      case STM32L4_CHANMODE_OCREFLO:
        {
          chanmode = GTIM_CCMR_MODE_OCREFLO;
          break;
        }

      case STM32L4_CHANMODE_OCREFHI:
        {
          chanmode = GTIM_CCMR_MODE_OCREFHI;
          break;
        }

      case STM32L4_CHANMODE_PWM1:
        {
          chanmode = GTIM_CCMR_MODE_PWM1;
          break;
        }

      case STM32L4_CHANMODE_PWM2:
        {
          chanmode = GTIM_CCMR_MODE_PWM2;
          break;
        }

      case STM32L4_CHANMODE_COMBINED1:
        {
          chanmode = ATIM_CCMR_MODE_COMBINED1;
          ocmbit   = true;
          break;
        }

      case STM32L4_CHANMODE_COMBINED2:
        {
          chanmode = ATIM_CCMR_MODE_COMBINED2;
          ocmbit   = true;
          break;
        }

      case STM32L4_CHANMODE_ASYMMETRIC1:
        {
          chanmode = ATIM_CCMR_MODE_ASYMMETRIC1;
          ocmbit   = true;
          break;
        }

      case STM32L4_CHANMODE_ASYMMETRIC2:
        {
          chanmode = ATIM_CCMR_MODE_ASYMMETRIC2;
          ocmbit   = true;
          break;
        }

      default:
        {
          pwmerr("ERROR: No such mode: %u\n", (unsigned int)mode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* PWM mode configuration */

  switch (channel)
    {
      /* Get CCMR offset */

      case STM32L4_PWM_CHAN1:
      case STM32L4_PWM_CHAN2:
        {
          offset = STM32L4_GTIM_CCMR1_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN3:
      case STM32L4_PWM_CHAN4:
        {
          offset = STM32L4_GTIM_CCMR2_OFFSET;
          break;
        }

      case STM32L4_PWM_CHAN5:
      case STM32L4_PWM_CHAN6:
        {
          offset = STM32L4_ATIM_CCMR3_OFFSET;
          break;
        }

      default:
        {
          pwmerr("ERROR: No such channel: %u\n", channel);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Get current registers */

  ccmr = pwm_getreg(priv, offset);

  /* PWM mode configuration.
   * NOTE: The CCMRx registers are identical if the channels are outputs.
   */

  switch (channel)
    {
      /* Configure channel 1/3/5 */

      case STM32L4_PWM_CHAN1:
      case STM32L4_PWM_CHAN3:
      case STM32L4_PWM_CHAN5:
        {
          /* Reset current channel 1/3/5 mode configuration */

          ccmr &= ~(GTIM_CCMR1_CC1S_MASK | GTIM_CCMR1_OC1M_MASK |
                     GTIM_CCMR1_OC1PE);

          /* Configure CC1/3/5 as output */

          ocmode |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC1S_SHIFT);

          /* Configure Compare 1/3/5 mode */

          ocmode |= (chanmode << GTIM_CCMR1_OC1M_SHIFT);

          /* Enable CCR1/3/5 preload */

          ocmode |= GTIM_CCMR1_OC1PE;

          /* Reset current OC bit */

          ccmr &= ~(GTIM_CCMR1_OC1M);

          /* Set an additional OC1/3/5M bit */

          if (ocmbit)
            {
              ocmode |= GTIM_CCMR1_OC1M;
            }
          break;
        }

      /* Configure channel 2/4/6 */

      case STM32L4_PWM_CHAN2:
      case STM32L4_PWM_CHAN4:
      case STM32L4_PWM_CHAN6:
        {
          /* Reset current channel 2/4/6 mode configuration */

          ccmr &= ~(GTIM_CCMR1_CC2S_MASK | GTIM_CCMR1_OC2M_MASK |
                     GTIM_CCMR1_OC2PE);

          /* Configure CC2/4/6 as output */

          ocmode |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC2S_SHIFT);

          /* Configure Compare 2/4/6 mode */

          ocmode |= (chanmode << GTIM_CCMR1_OC2M_SHIFT);

          /* Enable CCR2/4/6 preload */

          ocmode |= GTIM_CCMR1_OC2PE;

          /* Reset current OC bit */

          ccmr &= ~(GTIM_CCMR1_OC2M);

          /* Set an additioneal OC2/4/6M bit */

          if (ocmbit)
            {
              ocmode |= GTIM_CCMR1_OC2M;
            }
          break;
        }
    }

  /* Set the selected output compare mode */

  ccmr |= ocmode;

  /* Write CCMRx registers */

  pwm_putreg(priv, offset, ccmr);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_output_configure
 *
 * Description:
 *   Configure PWM output for given channel
 *
 ****************************************************************************/

static int pwm_output_configure(FAR struct stm32l4_pwmtimer_s *priv,
                                uint8_t channel)
{
  uint32_t cr2  = 0;
  uint32_t ccer = 0;

  /* Get current registers state */

  cr2  = pwm_getreg(priv, STM32L4_GTIM_CR2_OFFSET);
  ccer = pwm_getreg(priv, STM32L4_GTIM_CCER_OFFSET);

  /* | OISx/OISxN  | IDLE | for ADVANCED and COUNTUP16 | CR2 register
   * | CCxP/CCxNP  | POL  | all PWM timers             | CCER register
   */

  /* Configure output polarity (all PWM timers) */

  if (priv->channels[channel - 1].out1.pol == STM32L4_POL_NEG)
    {
      ccer |= (GTIM_CCER_CC1P << ((channel - 1) * 4));
    }
  else
    {
      ccer &= ~(GTIM_CCER_CC1P << ((channel - 1) * 4));
    }

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* Configure output IDLE State */

      if (priv->channels[channel - 1].out1.idle == STM32L4_IDLE_ACTIVE)
        {
          cr2 |= (ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }
      else
        {
          cr2 &= ~(ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      /* Configure complementary output IDLE state */

      if (priv->channels[channel - 1].out2.idle == STM32L4_IDLE_ACTIVE)
        {
          cr2 |= (ATIM_CR2_OIS1N << ((channel - 1) * 2));
        }
      else
        {
          cr2 &= ~(ATIM_CR2_OIS1N << ((channel - 1)* 2));
        }

      /* Configure complementary output polarity */

      if (priv->channels[channel - 1].out2.pol == STM32L4_POL_NEG)
        {
          ccer |= (ATIM_CCER_CC1NP << ((channel - 1) * 4));
        }
      else
        {
          ccer &= ~(ATIM_CCER_CC1NP << ((channel - 1) * 4));
        }
#endif /* HAVE_PWM_COMPLEMENTARY */

      /* TODO: OIS5 and OIS6 */

      cr2 &= ~(ATIM_CR2_OIS5 | ATIM_CR2_OIS6);

      /* TODO: CC5P and CC6P */

      ccer &= ~(ATIM_CCER_CC5P | ATIM_CCER_CC6P);
    }
#ifdef HAVE_GTIM_CCXNP
  else
#endif /* HAVE_GTIM_CCXNP */
#endif /* HAVE_ADVTIM */
#ifdef HAVE_GTIM_CCXNP
    {
      /* CCxNP must be cleared if not ADVANCED timer.
       *
       * REVISIT: not all families have CCxNP bits for GTIM,
       *          which causes an ugly condition above
       */

      ccer &= ~(GTIM_CCER_CC1NP << ((channel - 1) * 4));
    }
#endif /* HAVE_GTIM_CCXNP */

  /* Write registers */

  pwm_modifyreg(priv, STM32L4_GTIM_CR2_OFFSET, 0, cr2);
  pwm_modifyreg(priv, STM32L4_GTIM_CCER_OFFSET, 0, ccer);

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_enable
 *
 * Description:
 *   Enable/disable given timer PWM outputs.
 *
 *   NOTE: This is bulk operation - we can enable/disable many outputs
 *   at one time
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   outputs - outputs to set (look at enum stm32l4_chan_e in stm32l4_pwm.h)
 *   state   - Enable/disable operation
 *
 ****************************************************************************/

static int pwm_outputs_enable(FAR struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t ccer   = 0;
  uint32_t regval = 0;

  /* Get curren register state */

  ccer = pwm_getreg(priv, STM32L4_GTIM_CCER_OFFSET);

  /* Get outputs configuration */

  regval |= ((outputs & STM32L4_PWM_OUT1)  ? GTIM_CCER_CC1E  : 0);
  regval |= ((outputs & STM32L4_PWM_OUT1N) ? ATIM_CCER_CC1NE : 0);
  regval |= ((outputs & STM32L4_PWM_OUT2)  ? GTIM_CCER_CC2E  : 0);
  regval |= ((outputs & STM32L4_PWM_OUT2N) ? ATIM_CCER_CC2NE : 0);
  regval |= ((outputs & STM32L4_PWM_OUT3)  ? GTIM_CCER_CC3E  : 0);
  regval |= ((outputs & STM32L4_PWM_OUT3N) ? ATIM_CCER_CC3NE : 0);
  regval |= ((outputs & STM32L4_PWM_OUT4)  ? GTIM_CCER_CC4E  : 0);

  /* NOTE: CC4N does not exist, but some docs show configuration bits for it
   */

  regval |= ((outputs & STM32L4_PWM_OUT5)  ? ATIM_CCER_CC5E  : 0);
  regval |= ((outputs & STM32L4_PWM_OUT6)  ? ATIM_CCER_CC6E  : 0);

  if (state == true)
    {
      /* Enable outpus - set bits */

      ccer |= regval;
    }
  else
    {
      /* Disable outputs - reset bits */

      ccer &= ~regval;
    }

  /* Write register */

  pwm_putreg(priv, STM32L4_GTIM_CCER_OFFSET, ccer);

  return OK;
}

#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_STM32L4_PWM_LL_OPS)

/****************************************************************************
 * Name: pwm_deadtime_update
 ****************************************************************************/

static int pwm_deadtime_update(FAR struct pwm_lowerhalf_s *dev, uint8_t dt)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t bdtr = 0;
  int      ret  = OK;

  /* Check if locked */

  if (priv->lock > 0)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get current register state */

  bdtr = pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET);

  /* TODO: check if BDTR not locked */

  /* Update deadtime */

  bdtr &= ~(ATIM_BDTR_DTG_MASK);
  bdtr |= (dt << ATIM_BDTR_DTG_SHIFT);

  /* Write BDTR register */

  pwm_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, bdtr);

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: pwm_soft_update
 *
 * Description:
 *   Generate an software update event
 *
 ****************************************************************************/

static int pwm_soft_update(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  pwm_putreg(priv, STM32L4_GTIM_EGR_OFFSET, GTIM_EGR_UG);

  return OK;
}

/****************************************************************************
 * Name: pwm_soft_break
 *
 * Description:
 *   Generate an software break event
 *
 *   Outputs are enabled if state is false.
 *   Outputs are disabled if state is true.
 *
 *   NOTE: only timers with complementary outputs have BDTR register and
 *         support software break.
 *
 ****************************************************************************/

static int pwm_soft_break(FAR struct pwm_lowerhalf_s *dev, bool state)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  if (state == true)
    {
      /* Reset MOE bit */

      pwm_modifyreg(priv, STM32L4_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE, 0);
    }
  else
    {
      /* Set MOE bit */

      pwm_modifyreg(priv, STM32L4_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_from_channels
 *
 * Description:
 *   Get enabled outputs configuration from the PWM timer state
 *
 ****************************************************************************/

static uint16_t
  pwm_outputs_from_channels(FAR struct stm32l4_pwmtimer_s *priv)
{
  uint16_t outputs = 0;
  uint8_t  channel = 0;
  uint8_t  i       = 0;

  for (i = 0; i < priv->chan_num; i += 1)
    {
      /* Get channel */

      channel = priv->channels[i].channel;

      /* Set outputs if channel configured */

      if (channel != 0)
        {
          /* Enable output if confiugred */

          if (priv->channels[i].out1.in_use == 1)
            {
              outputs |= (STM32L4_PWM_OUT1 << ((channel - 1) * 2));
            }

#ifdef HAVE_PWM_COMPLEMENTARY
          /* Enable complementary output if configured */

          if (priv->channels[i].out2.in_use == 1)
            {
              outputs |= (STM32L4_PWM_OUT1N << ((channel - 1) * 2));
            }
#endif
        }
    }

  return outputs;
}

#ifdef HAVE_ADVTIM

/****************************************************************************
 * Name: pwm_break_dt_configure
 *
 * Description:
 *   Configure break and deadtime
 *
 * NOTE: we have to configure all BDTR registers at once due to possible
 *       lock configuration
 *
 ****************************************************************************/

static int pwm_break_dt_configure(FAR struct stm32l4_pwmtimer_s *priv)
{
  uint32_t bdtr = 0;

  /* Set the clock division to zero for all (but the basic timers, but there
   * should be no basic timers in this context
   */

  pwm_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, GTIM_CR1_CKD_MASK,
                priv->t_dts << GTIM_CR1_CKD_SHIFT);

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Initialize deadtime */

  bdtr |= (priv->deadtime << ATIM_BDTR_DTG_SHIFT);
#endif

#ifdef HAVE_BREAK
  /* Configure Break 1 */

  if (priv->brk.en1 == 1)
    {
      /* Enable Break 1 */

      bdtr |= ATIM_BDTR_BKE;

      /* Set Break 1 polarity */

      bdtr |= (priv->brk.pol1 == STM32L4_POL_NEG ? ATIM_BDTR_BKP : 0);
    }

  /* Configure Break 1 */

  if (priv->brk.en2 == 1)
    {
      /* Enable Break 2 */

      bdtr |= ATIM_BDTR_BK2E;

      /* Set Break 2 polarity */

      bdtr |= (priv->brk.pol2 == STM32L4_POL_NEG ? ATIM_BDTR_BK2P : 0);

      /* Configure BRK2 filter */

      bdtr |= (priv->brk.flt2 << ATIM_BDTR_BK2F_SHIFT);
    }

#endif /* HAVE_BREAK */

  /* Clear the OSSI and OSSR bits in the BDTR register.
   *
   * REVISIT: this should be configurable
   */

  bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);

  /* Configure lock */

  bdtr |= priv->lock << ATIM_BDTR_LOCK_SHIFT;

  /* Write BDTR register at once */

  pwm_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, bdtr);

  return OK;
}

#endif /* HAVE_ADVTIM */

/****************************************************************************
 * Name: pwm_configure
 *
 * Description:
 *   Configure PWM timer in normal mode (no PULSECOUNT)
 *
 ****************************************************************************/

static int pwm_configure(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint16_t outputs = 0;
  uint8_t j        = 0;
  int     ret      = OK;

  /* NOTE: leave timer counter disabled and all outputs disabled! */

  /* Disable the timer until we get it configured */

  pwm_timer_enable(dev, false);

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Disable outputs */

  ret = pwm_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      pwmerr("ERROR on pwm_outputs_enable()\n");
      goto errout;
    }

  /* Initial timer configuration */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      pwmerr("ERROR on pwm_timer_configure()\n");
      goto errout;
    }

  /* Some special setup for advanced timers */

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* Configure break and deadtime register */

      ret = pwm_break_dt_configure(priv);
      if (ret < 0)
        {
          pwmerr("ERROR on pwm_break_dt_configure()\n");
          goto errout;
        }

#ifdef HAVE_TRGO
      /* Configure TRGO/TRGO2 */

      ret = pwm_sync_configure(priv, priv->trgo);
      if (ret < 0)
        {
          pwmerr("ERROR on pwm_sync_configure()\n");
          goto errout;
        }
#endif
    }
#endif

  /* Configure timer channels */

  for (j = 0; j < priv->chan_num; j++)
    {
      /* Skip channel if not in use */

      if (priv->channels[j].channel != 0)
        {
          /* Update PWM mode */

          ret = pwm_mode_configure(dev, priv->channels[j].channel,
                                   priv->channels[j].mode);
          if (ret < 0)
            {
              pwmerr("ERROR on pwm_mode_configure()\n");
              goto errout;
            }

          /* PWM outputs configuration */

          ret = pwm_output_configure(priv, priv->channels[j].channel);
          if (ret < 0)
            {
              pwmerr("ERROR on pwm_output_configure()\n");
              goto errout;
            }
        }
    }

  /* Disable software break at the end of the outputs configuration (enablei
   * outputs).
   *
   * NOTE: Only timers with complementary outputs have BDTR register and
   *       support software break.
   */

  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      pwminfo("pwm_soft_break(dev, false)\n");
      ret = pwm_soft_break(dev, false);
      if (ret < 0)
        {
          pwmerr("ERROR on pwm_soft_break()\n");
          goto errout;
        }
    }

errout:
  return ret;
}

#ifdef CONFIG_PWM_PULSECOUNT

/****************************************************************************
 * Name: pwm_pulsecount_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * TODO: PWM_PULSECOUNT should be configurable for each timer instance
 * TODO: PULSECOUNT doesn't work with MULTICHAN at this moment
 *
 ****************************************************************************/

static int pwm_pulsecount_timer(FAR struct pwm_lowerhalf_s *dev,
                                FAR const struct pwm_info_s *info)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  ub16_t    duty    = 0;
  uint8_t   channel = 0;
  uint16_t  outputs = 0;
  int       ret     = OK;

  /* If we got here it means that timer instance support pulsecount mode! */

  DEBUGASSERT(priv != NULL && info != NULL);

  pwminfo("TIM%u channel: %u frequency: %u duty: %08x count: %u\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty, info->count);

  DEBUGASSERT(info->frequency > 0);

  /* Channel specific setup */

  duty    = info->duty;
  channel = priv->channels[0].channel;

  /* Disable all interrupts and DMA requests, clear all pending status */

  pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Update duty cycle */

  ret = pwm_duty_update(dev, channel, duty);
  if (ret < 0)
    {
      goto errout;
    }

  /* If a non-zero repetition count has been selected, then set the
   * repetition counter to the count-1 (pwm_pulsecount_start() has already
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

      priv->prev  = pwm_pulsecount(info->count);
      pwm_putreg(priv, STM32L4_GTIM_RCR_OFFSET, (uint16_t)priv->prev - 1);

      /* Generate an update event to reload the prescaler.  This should
       * preload the RCR into active repetition counter.
       */

      pwm_soft_update(dev);

      /* Now set the value of the RCR that will be loaded on the next
       * update event.
       */

      priv->count = info->count;
      priv->curr  = pwm_pulsecount(info->count - priv->prev);
      pwm_putreg(priv, STM32L4_GTIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
    }

  /* Otherwise, just clear the repetition counter */

  else
    {
      /* Set the repetition counter to zero */

      pwm_putreg(priv, STM32L4_GTIM_RCR_OFFSET, 0);

      /* Generate an update event to reload the prescaler */

      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable output */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Setup update interrupt.  If info->count is > 0, then we can be
   * assured that pwm_pulsecount_start() has already verified: (1) that this
   * is an advanced timer, and that (2) the repetition count is within range.
   */

  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);
      pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      pwm_timer_enable(dev, true);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}

#else  /* !CONFIG_PWM_PULSECOUNT */

/****************************************************************************
 * Name: pwm_duty_channels_update
 *
 * Description:
 *   Update duty cycle for given channels
 *
 ****************************************************************************/

static int pwm_duty_channels_update(FAR struct pwm_lowerhalf_s *dev,
                                    FAR const struct pwm_info_s *info)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint8_t   channel = 0;
  ub16_t    duty    = 0;
  int       ret     = OK;
#ifdef CONFIG_STM32L4_PWM_MULTICHAN
  int       i       = 0;
  int       j       = 0;
#endif

#ifdef CONFIG_STM32L4_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
#ifdef CONFIG_STM32L4_PWM_MULTICHAN
      /* Break the loop if all following channels are not configured */

      if (info->channels[i].channel == -1)
        {
          break;
        }

      duty    = info->channels[i].duty;
      channel = info->channels[i].channel;

      /* A value of zero means to skip this channel */

      if (channel != 0)
        {
          /* Find the channel */

          for (j = 0; j < priv->chan_num; j++)
            {
              if (priv->channels[j].channel == channel)
                {
                  break;
                }
            }

          /* Check range */

          if (j >= priv->chan_num)
            {
              pwmerr("ERROR: No such channel: %u\n", channel);
              ret = -EINVAL;
              goto errout;
            }
#else
          duty = info->duty;
          channel = priv->channels[0].channel;
#endif

          /* Update duty cycle */

          ret = pwm_duty_update(dev, channel, duty);
          if (ret < 0)
            {
              goto errout;
            }
#ifdef CONFIG_STM32L4_PWM_MULTICHAN
        }
#endif
    }

errout:
  return OK;
}

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_timer(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint16_t outputs = 0;
  int      ret     = OK;

  DEBUGASSERT(priv != NULL && info != NULL);

#if defined(CONFIG_STM32L4_PWM_MULTICHAN)
  pwminfo("TIM%u frequency: %" PRIu32 "\n",
          priv->timid, info->frequency);
#else
  pwminfo("TIM%u channel: %u frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
          priv->timid, priv->channels[0].channel,
          info->frequency, info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_STM32L4_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* TODO: what if we have pwm running and we want disable some channels ? */

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Channel specific configuration */

  ret = pwm_duty_channels_update(dev, info);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set the advanced timer's repetition counter */

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repetition counter to the count-1 (pwm_start() has already
       * assured us that the count value is within range).
       */

      /* Set the repetition counter to zero */

      pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, 0);

      /* Generate an update event to reload the prescaler */

      pwm_soft_update(dev);
    }
  else
#endif
    {
      /* Generate an update event to reload the prescaler (all timers) */

      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable outputs */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Just enable the timer, leaving all interrupts disabled */

  pwm_timer_enable(dev, true);

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_lptimer
 *
 * Description:
 *   (Re-)initialize the low-power timer resources and start the
 *   pulsed output
 *
 * Input Parameters:
 *   priv - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef HAVE_LPTIM
static int pwm_lptimer(FAR struct pwm_lowerhalf_s *dev,
                       FAR const struct pwm_info_s *info)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint16_t cr;
  int      ret     = OK;

  DEBUGASSERT(priv != NULL && info != NULL);

#if defined(CONFIG_STM32L4_PWM_MULTICHAN)
  pwminfo("LPTIM%u frequency: %u\n",
          priv->timid, info->frequency);
#else
  pwminfo("LPTIM%u channel: %u frequency: %u duty: %08x\n",
          priv->timid, priv->channels[0].channel,
          info->frequency, info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_STM32L4_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* Enable again, ARR and CMP need to be written while enabled */

  pwm_timer_enable(dev, true);

  /* Set timer frequency */

  ret = pwm_lp_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_STM32L4_PWM_MULTICHAN
  ub16_t duty = info->channels[0].duty;
#else
  ub16_t duty = info->duty;
#endif

  /* Update duty cycle */

  ret = pwm_duty_update(dev, priv->channels[0].channel, duty);
  if (ret < 0)
    {
      goto errout;
    }

  /* Start counter */

  cr = pwm_getreg(priv, STM32L4_LPTIM_CR_OFFSET);
  cr |= LPTIM_CR_CNTSTRT;
  pwm_putreg(priv, STM32L4_LPTIM_CR_OFFSET, cr);

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}
#endif /* HAVE_LPTIM */

#endif /* CONFIG_PWM_PULSECOUNT */

#ifdef HAVE_PWM_INTERRUPT

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_interrupt(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pwm_getreg(priv, STM32L4_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pwm_putreg(priv, STM32L4_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      pwm_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrupts, stop and reset the timer */

      pwm_stop(dev);

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
      pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output
   */

  pwminfo("Update interrupt SR: %04x prev: %u curr: %u count: %u\n",
          regval, priv->prev, priv->curr, priv->count);

  return OK;
}

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

#ifdef CONFIG_STM32L4_TIM1_PWM
static int pwm_tim1interrupt(int irq, void *context, FAR void *arg)
{
  return pwm_interrupt((FAR struct pwm_lowerhalf_s *)&g_pwm1dev);
}
#endif /* CONFIG_STM32L4_TIM1_PWM */

#ifdef CONFIG_STM32L4_TIM8_PWM
static int pwm_tim8interrupt(int irq, void *context, FAR void *arg)
{
  return pwm_interrupt((FAR struct pwm_lowerhalf_s *)&g_pwm8dev);
}
#endif /* CONFIG_STM32L4_TIM8_PWM */

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

static uint8_t pwm_pulsecount(uint32_t count)
{
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
#endif /* HAVE_PWM_INTERRUPT */

/****************************************************************************
 * Name: pwm_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static int pwm_setapbclock(FAR struct stm32l4_pwmtimer_s *priv,
                                  bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;
  int      ret     = OK;

  /* Determine which timer to configure */

  if (priv->timtype != TIMTYPE_LOWPOWER)
    {
      switch (priv->timid)
        {
#ifdef CONFIG_STM32L4_TIM1_PWM
          case 1:
            {
              regaddr  = STM32L4_RCC_APB2ENR;
              en_bit   = RCC_APB2ENR_TIM1EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM2_PWM
          case 2:
            {
              regaddr  = STM32L4_RCC_APB1ENR1;
              en_bit   = RCC_APB1ENR1_TIM2EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM3_PWM
          case 3:
            {
              regaddr  = STM32L4_RCC_APB1ENR1;
              en_bit   = RCC_APB1ENR1_TIM3EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM4_PWM
          case 4:
            {
              regaddr  = STM32L4_RCC_APB1ENR1;
              en_bit   = RCC_APB1ENR1_TIM4EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM5_PWM
          case 5:
            {
              regaddr  = STM32L4_RCC_APB1ENR1;
              en_bit   = RCC_APB1ENR1_TIM5EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM8_PWM
          case 8:
            {
              regaddr  = STM32L4_RCC_APB2ENR;
              en_bit   = RCC_APB2ENR_TIM8EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM15_PWM
          case 15:
            {
              regaddr  = STM32L4_RCC_APB2ENR;
              en_bit   = RCC_APB2ENR_TIM15EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM16_PWM
          case 16:
            {
              regaddr  = STM32L4_RCC_APB2ENR;
              en_bit   = RCC_APB2ENR_TIM16EN;
              break;
            }
#endif

#ifdef CONFIG_STM32L4_TIM17_PWM
          case 17:
            {
              regaddr  = STM32L4_RCC_APB2ENR;
              en_bit   = RCC_APB2ENR_TIM17EN;
              break;
          }
#endif

          default:
          {
            pwmerr("ERROR: No such timer configured %d\n", priv->timid);
            ret = -EINVAL;
            goto errout;
          }
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
#ifdef HAVE_LPTIM
  else
    {
      uint32_t clock_bits;

      switch (priv->timid)
        {
#ifdef CONFIG_STM32L4_LPTIM1_PWM
          case 1:
            {
#if defined(CONFIG_STM32L4_LPTIM1_CLK_APB1)
              /* Enable APB clock for LPTIM1 */

              if (on)
                {
                  modifyreg32(STM32L4_RCC_APB1ENR1,
                              0, RCC_APB1ENR1_LPTIM1EN);
                }
              else
                {
                  modifyreg32(STM32L4_RCC_APB1ENR1,
                              RCC_APB1ENR1_LPTIM1EN, 0);
                }

              clock_bits = RCC_CCIPR_LPTIM1SEL_PCLK;
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_LSI)
              clock_bits = RCC_CCIPR_LPTIM1SEL_LSI;
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_LSE)
              clock_bits = RCC_CCIPR_LPTIM1SEL_LSE;
#elif defined(CONFIG_STM32L4_LPTIM1_CLK_HSI)
              clock_bits = RCC_CCIPR_LPTIM1SEL_HSI;
#endif
              /* Choose which clock will be used for LPTIM1 */

              modifyreg32(STM32L4_RCC_CCIPR, RCC_CCIPR_LPTIM1SEL_MASK,
                          clock_bits);
              break;
            }
#endif

#ifdef CONFIG_STM32L4_LPTIM2_PWM
          case 2:
            {
#if defined(CONFIG_STM32L4_LPTIM2_CLK_APB1)
              /* Enable APB clock for LPTIM2 */

              if (on)
                {
                  modifyreg32(STM32L4_RCC_APB1ENR2,
                              0, RCC_APB1ENR2_LPTIM2EN);
                }
              else
                {
                  modifyreg32(STM32L4_RCC_APB1ENR2,
                              RCC_APB1ENR2_LPTIM2EN, 0);
                }

              clock_bits = RCC_CCIPR_LPTIM2SEL_PCLK;
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_LSI)
              clock_bits = RCC_CCIPR_LPTIM2SEL_LSI;
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_LSE)
              clock_bits = RCC_CCIPR_LPTIM2SEL_LSE;
#elif defined(CONFIG_STM32L4_LPTIM2_CLK_HSI)
              clock_bits = RCC_CCIPR_LPTIM2SEL_HSI;
#endif
              /* Choose which clock will be used for LPTIM2 */

              modifyreg32(STM32L4_RCC_CCIPR, RCC_CCIPR_LPTIM2SEL_MASK,
                          clock_bits);
              break;
            }

#endif
          default:
            {
              pwmerr("ERROR: No such timer configured %d\n", priv->timid);
              ret = -EINVAL;
              goto errout;
            }
        }
    }

#endif /* HAVE_LPTIM */
errout:
  return ret;
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

static int pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t pincfg;
  int ret;
  int i;

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      pwminfo("LPTIM%u\n", priv->timid);
    }
  else
    {
      pwminfo("TIM%u\n", priv->timid);
    }

  /* Enable APB1/2 clocking for timer. */

  pwm_setapbclock(priv, true);

  pwm_dumpregs(dev, "Initially");

  /* Configure the PWM output pins, but do not start the timer yet */

  for (i = 0; i < priv->chan_num; i++)
    {
      if (priv->channels[i].out1.in_use == 1)
        {
          pincfg = priv->channels[i].out1.pincfg;
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          stm32l4_configgpio(pincfg);
          pwm_dumpgpio(pincfg, "PWM setup");
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      if (priv->channels[i].out2.in_use == 1)
        {
          pincfg = priv->channels[i].out2.pincfg;
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          stm32l4_configgpio(pincfg);
          pwm_dumpgpio(pincfg, "PWM setup");
        }
#endif
    }

  /* Configure PWM timer with the selected configuration.
   *
   * NOTE: We configure PWM here during setup, but leave timer with disabled
   *       counter, disabled outputs, not configured frequency and duty cycle
   */

  ret = pwm_configure(dev);
  if (ret < 0)
    {
      pwmerr("failed to configure PWM %d\n", priv->timid);
      return ERROR;
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

static int pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t pincfg = 0;
  int      i      = 0;
  int      ret    = OK;

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      pwminfo("LPTIM%u\n", priv->timid);
    }
  else
    {
      pwminfo("TIM%u\n", priv->timid);
    }

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  ret = pwm_setapbclock(priv, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Then put the GPIO pins back to the default state */

  for (i = 0; i < priv->chan_num; i++)
    {
      pincfg = priv->channels[i].out1.pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32l4_configgpio(pincfg);
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      pincfg = priv->channels[i].out2.pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32l4_configgpio(pincfg);
        }
#endif
    }

errout:
  return ret;
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
static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info,
                     FAR void *handle)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

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

  return pwm_pulsecount_timer(dev, info);
}
#else /* !CONFIG_PWM_PULSECOUNT */

static int pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  int ret = OK;
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  /* if frequency has not changed we just update duty */

  if (info->frequency == priv->frequency)
    {
#ifdef CONFIG_STM32L4_PWM_MULTICHAN
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
              ret = pwm_duty_update(dev, info->channels[i].channel,
                                    info->channels[i].duty);
            }
        }
#else
      ret = pwm_duty_update(dev, priv->channels[0].channel, info->duty);
#endif /* CONFIG_STM32L4_PWM_MULTICHAN */
    }
  else
    {
      if (priv->timtype != TIMTYPE_LOWPOWER)
        {
          ret = pwm_timer(dev, info);
        }
#ifdef HAVE_LPTIM
      else
        {
          ret = pwm_lptimer(dev, info);
        }
#endif

      /* Save current frequency */

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
    }

  return ret;
}
#endif /* CONFIG_PWM_PULSECOUNT */

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

static int pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t resetbit = 0;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  if (priv->timtype == TIMTYPE_LOWPOWER)
    {
      pwminfo("LPTIM%u\n", priv->timid);
    }
  else
    {
      pwminfo("TIM%u\n", priv->timid);
    }

  /* Determine which timer to reset */

  if (priv->timtype != TIMTYPE_LOWPOWER)
    {
      switch (priv->timid)
        {
#ifdef CONFIG_STM32L4_TIM1_PWM
          case 1:
            regaddr  = STM32L4_RCC_APB2RSTR;
            resetbit = RCC_APB2RSTR_TIM1RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM2_PWM
          case 2:
            regaddr  = STM32L4_RCC_APB1RSTR1;
            resetbit = RCC_APB1RSTR1_TIM2RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM3_PWM
          case 3:
            regaddr  = STM32L4_RCC_APB1RSTR1;
            resetbit = RCC_APB1RSTR1_TIM3RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM4_PWM
          case 4:
            regaddr  = STM32L4_RCC_APB1RSTR1;
            resetbit = RCC_APB1RSTR1_TIM4RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM5_PWM
          case 5:
            regaddr  = STM32L4_RCC_APB1RSTR1;
            resetbit = RCC_APB1RSTR1_TIM5RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM8_PWM
          case 8:
            regaddr  = STM32L4_RCC_APB2RSTR;
            resetbit = RCC_APB2RSTR_TIM8RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM16_PWM
          case 16:
            regaddr  = STM32L4_RCC_APB2RSTR;
            resetbit = RCC_APB2RSTR_TIM16RST;
            break;
#endif
#ifdef CONFIG_STM32L4_TIM17_PWM
          case 17:
            regaddr  = STM32L4_RCC_APB2RSTR;
            resetbit = RCC_APB2RSTR_TIM17RST;
            break;
#endif
          default:
            return -EINVAL;
        }
    }
  else
    {
      switch (priv->timid)
        {
#ifdef CONFIG_STM32L4_LPTIM1_PWM
          case 1:
            regaddr  = STM32L4_RCC_APB1RSTR1;
            resetbit = RCC_APB1RSTR1_LPTIM1RST;
            break;
#endif
#ifdef CONFIG_STM32L4_LPTIM2_PWM
          case 2:
            regaddr  = STM32L4_RCC_APB1RSTR2;
            resetbit = RCC_APB1RSTR2_LPTIM2RST;
            break;
#endif
          default:
            return -EINVAL;
        }
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

  pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where pwm_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  pwminfo("regaddr: %08" PRIx32 " resetbit: %08" PRIx32 "\n",
          regaddr, resetbit);
  pwm_dumpregs(dev, "After stop");
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

static int pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%u\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_pwminitialize
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

FAR struct pwm_lowerhalf_s *stm32l4_pwminitialize(int timer)
{
  FAR struct stm32l4_pwmtimer_s *lower;

  pwminfo("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32L4_TIM1_PWM
      case 1:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM1 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_tim1interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32L4_TIM2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM5_PWM
      case 5:
        lower = &g_pwm5dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM8_PWM
      case 8:
        lower = &g_pwm8dev;

        /* Attach but disable the TIM8 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_tim8interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32L4_TIM15_PWM
      case 15:
        lower = &g_pwm15dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM16_PWM
      case 16:
        lower = &g_pwm16dev;
        break;
#endif

#ifdef CONFIG_STM32L4_TIM17_PWM
      case 17:
        lower = &g_pwm17dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  return (FAR struct pwm_lowerhalf_s *)lower;
}

/****************************************************************************
 * Name: stm32l4_lp_pwminitialize
 *
 * Description:
 *   Initialize one low-power timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,2}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef HAVE_LPTIM
FAR struct pwm_lowerhalf_s *stm32l4_lp_pwminitialize(int timer)
{
  FAR struct stm32l4_pwmtimer_s *lower;

  pwminfo("LPTIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32L4_LPTIM1_PWM
      case 1:
        lower = &g_pwmlp1dev;
        break;
#endif

#ifdef CONFIG_STM32L4_LPTIM2_PWM
      case 2:
        lower = &g_pwmlp2dev;
        break;
#endif

      default:
        pwmerr("ERROR: No such timer configured\n");
        return NULL;
    }

  return (FAR struct pwm_lowerhalf_s *)lower;
}
#endif /* HAVE_LPTIM */

#endif /* CONFIG_STM32L4_TIMn_PWM, n = 1,...,17 */
