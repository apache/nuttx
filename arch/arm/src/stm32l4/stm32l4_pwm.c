/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pwm.c
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/drivers/pwm.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32l4_pwm.h"
#include "stm32l4.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_STM32L4_TIM1_PWM)  || defined(CONFIG_STM32L4_TIM2_PWM)  || \
    defined(CONFIG_STM32L4_TIM3_PWM)  || defined(CONFIG_STM32L4_TIM4_PWM)  || \
    defined(CONFIG_STM32L4_TIM5_PWM)  || defined(CONFIG_STM32L4_TIM8_PWM)  || \
    defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM) || \
    defined(CONFIG_STM32L4_TIM17_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* PWM/Timer Definitions ****************************************************/
/* The following definitions are used to identify the various time types */

#define TIMTYPE_BASIC      0  /* Basic timers: TIM6,7 */
#define TIMTYPE_GENERAL16  1  /* General 16-bit timers: TIM3,4 */
#define TIMTYPE_COUNTUP16  2  /* General 16-bit count-up timers: TIM15-17 */
#define TIMTYPE_GENERAL32  3  /* General 32-bit timers: TIM2,5 */
#define TIMTYPE_ADVANCED   4  /* Advanced timers:  TIM1,8 */

#define TIMTYPE_TIM1       TIMTYPE_ADVANCED
#define TIMTYPE_TIM2       TIMTYPE_GENERAL32
#define TIMTYPE_TIM3       TIMTYPE_GENERAL16
#define TIMTYPE_TIM4       TIMTYPE_GENERAL16
#define TIMTYPE_TIM5       TIMTYPE_GENERAL32
#define TIMTYPE_TIM6       TIMTYPE_BASIC
#define TIMTYPE_TIM7       TIMTYPE_BASIC
#define TIMTYPE_TIM8       TIMTYPE_ADVANCED
#define TIMTYPE_TIM15      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM16      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM17      TIMTYPE_COUNTUP16

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) stm32l4_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum stm32l4_timmode_e
{
  STM32L4_TIMMODE_COUNTUP   = 0,
  STM32L4_TIMMODE_COUNTDOWN = 1,
  STM32L4_TIMMODE_CENTER1   = 2,
  STM32L4_TIMMODE_CENTER2   = 3,
  STM32L4_TIMMODE_CENTER3   = 4,
};

enum stm32l4_chanmode_e
{
  STM32L4_CHANMODE_PWM1        = 0,
  STM32L4_CHANMODE_PWM2        = 1,
  STM32L4_CHANMODE_COMBINED1   = 2,
  STM32L4_CHANMODE_COMBINED2   = 3,
  STM32L4_CHANMODE_ASYMMETRIC1 = 4,
  STM32L4_CHANMODE_ASYMMETRIC2 = 5,
};

struct stm32l4_pwmchan_s
{
  uint8_t channel;                     /* Timer output channel: {1,..4} */
  enum stm32l4_chanmode_e mode;
  uint32_t pincfg;                     /* Output pin configuration */
  uint32_t npincfg;                    /* Complementary output pin configuration
                                        * (only TIM1,8 CH1-3 and TIM15,16,17 CH1)
                                        */
};

/* This structure represents the state of one PWM timer */

struct stm32l4_pwmtimer_s
{
  FAR const struct pwm_ops_s *ops;     /* PWM operations */
  struct stm32l4_pwmchan_s channels[PWM_NCHANNELS];
  uint8_t timid;                       /* Timer ID {1,...,17} */
  uint8_t timtype;                     /* See the TIMTYPE_* definitions */
  enum stm32l4_timmode_e mode;
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t irq;                         /* Timer update IRQ */
  uint8_t prev;                        /* The previous value of the RCR (pre-loaded) */
  uint8_t curr;                        /* The current value of the RCR (pre-loaded) */
  uint32_t count;                      /* Remaining pluse count */
#else
  uint32_t frequency;                  /* Current frequency setting */
#endif
  uint32_t base;                       /* The base address of the timer */
  uint32_t pclk;                       /* The frequency of the peripheral clock
                                        * that drives the timer module. */
#ifdef CONFIG_PWM_PULSECOUNT
  FAR void *handle;                    /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/
/* Register access */

static uint16_t stm32l4pwm_getreg(struct stm32l4_pwmtimer_s *priv, int offset);
static void stm32l4pwm_putreg(struct stm32l4_pwmtimer_s *priv, int offset, uint16_t value);

#ifdef CONFIG_DEBUG_PWM_INFO
static void stm32l4pwm_dumpregs(struct stm32l4_pwmtimer_s *priv, FAR const char *msg);
#else
#  define stm32l4pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int stm32l4pwm_timer(FAR struct stm32l4_pwmtimer_s *priv,
                            FAR const struct pwm_info_s *info);

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM))
static int stm32l4pwm_interrupt(struct stm32l4_pwmtimer_s *priv);
#if defined(CONFIG_STM32L4_TIM1_PWM)
static int stm32l4pwm_tim1interrupt(int irq, void *context, FAR void *arg);
#endif
#if defined(CONFIG_STM32L4_TIM8_PWM)
static int stm32l4pwm_tim8interrupt(int irq, void *context, FAR void *arg);
#endif
static uint8_t stm32l4pwm_pulsecount(uint32_t count);
#endif

/* PWM driver methods */

static int stm32l4pwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int stm32l4pwm_shutdown(FAR struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int stm32l4pwm_start(FAR struct pwm_lowerhalf_s *dev,
                            FAR const struct pwm_info_s *info,
                            FAR void *handle);
#else
static int stm32l4pwm_start(FAR struct pwm_lowerhalf_s *dev,
                            FAR const struct pwm_info_s *info);
#endif

static int stm32l4pwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int stm32l4pwm_ioctl(FAR struct pwm_lowerhalf_s *dev,
                            int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* This is the list of lower half PWM driver methods used by the upper half driver */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = stm32l4pwm_setup,
  .shutdown    = stm32l4pwm_shutdown,
  .start       = stm32l4pwm_start,
  .stop        = stm32l4pwm_stop,
  .ioctl       = stm32l4pwm_ioctl,
};

#ifdef CONFIG_STM32L4_TIM1_PWM
static struct stm32l4_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .timid       = 1,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM1_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM1_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM1_CH1MODE,
      .npincfg = PWM_TIM1_CH1NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM1_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM1_CH2MODE,
      .npincfg = PWM_TIM1_CH2NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM1_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM1_CH3MODE,
      .npincfg = PWM_TIM1_CH3NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM1_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM1_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM1_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM1,
  .mode        = CONFIG_STM32L4_TIM1_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM1UP,
#endif
  .base        = STM32L4_TIM1_BASE,
  .pclk        = STM32L4_APB2_TIM1_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM2_PWM
static struct stm32l4_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .timid       = 2,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM2_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM2_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM2_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM2_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM2_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM2_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM2_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM2_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM2_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM2_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM2,
  .mode        = CONFIG_STM32L4_TIM2_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM2,
#endif
  .base        = STM32L4_TIM2_BASE,
  .pclk        = STM32L4_APB1_TIM2_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM3_PWM
static struct stm32l4_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
  .timid       = 3,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM3_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM3_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM3_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM3_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM3_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM3_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM3_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM3_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM3_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM3_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM3,
  .mode        = CONFIG_STM32L4_TIM3_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM3,
#endif
  .base        = STM32L4_TIM3_BASE,
  .pclk        = STM32L4_APB1_TIM3_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM4_PWM
static struct stm32l4_pwmtimer_s g_pwm4dev =
{
  .ops         = &g_pwmops,
  .timid       = 4,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM4_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM4_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM4_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM4_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM4_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM4_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM4_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM4_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM4_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM4_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM4,
  .mode        = CONFIG_STM32L4_TIM4_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM4,
#endif
  .base        = STM32L4_TIM4_BASE,
  .pclk        = STM32L4_APB1_TIM4_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM5_PWM
static struct stm32l4_pwmtimer_s g_pwm5dev =
{
  .ops         = &g_pwmops,
  .timid       = 5,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM5_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM5_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM5_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM5_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM5_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM5_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM5_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32L4_TIM5_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM5_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM5_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM5,
  .mode        = CONFIG_STM32L4_TIM5_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM5,
#endif
  .base        = STM32L4_TIM5_BASE,
  .pclk        = STM32L4_APB1_TIM5_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM8_PWM
static struct stm32l4_pwmtimer_s g_pwm8dev =
{
  .ops         = &g_pwmops,
  .timid       = 8,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM8_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM8_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM8_CH1MODE,
      .npincfg = PWM_TIM8_CH1NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM8_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM8_CH2MODE,
      .npincfg = PWM_TIM8_CH2NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM8_CH3CFG,
      .mode    = CONFIG_STM32L4_TIM8_CH3MODE,
      .npincfg = PWM_TIM8_CH3NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM8_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM8_CH4CFG,
      .mode    = CONFIG_STM32L4_TIM8_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM8,
  .mode        = CONFIG_STM32L4_TIM8_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM8UP,
#endif
  .base        = STM32L4_TIM8_BASE,
  .pclk        = STM32L4_APB2_TIM8_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM15_PWM
static struct stm32l4_pwmtimer_s g_pwm15dev =
{
  .ops         = &g_pwmops,
  .timid       = 15,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM15_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM15_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM15_CH1MODE,
      .npincfg = PWM_TIM15_CH1NCFG,
    },
#endif
#ifdef CONFIG_STM32L4_TIM15_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM15_CH2CFG,
      .mode    = CONFIG_STM32L4_TIM15_CH2MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM15,
  .mode        = STM32L4_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM15,
#endif
  .base        = STM32L4_TIM15_BASE,
  .pclk        = STM32L4_APB2_TIM15_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM16_PWM
static struct stm32l4_pwmtimer_s g_pwm16dev =
{
  .ops         = &g_pwmops,
  .timid       = 16,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM16_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM16_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM16_CH1MODE,
      .npincfg = PWM_TIM16_CH1NCFG,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM16,
  .mode        = STM32L4_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM16,
#endif
  .base        = STM32L4_TIM16_BASE,
  .pclk        = STM32L4_APB2_TIM16_CLKIN,
};
#endif

#ifdef CONFIG_STM32L4_TIM17_PWM
static struct stm32l4_pwmtimer_s g_pwm17dev =
{
  .ops         = &g_pwmops,
  .timid       = 17,
  .channels    =
  {
#ifdef CONFIG_STM32L4_TIM17_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM17_CH1CFG,
      .mode    = CONFIG_STM32L4_TIM17_CH1MODE,
      .npincfg = PWM_TIM17_CH1NCFG,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM17,
  .mode        = STM32L4_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32L4_IRQ_TIM17,
#endif
  .base        = STM32L4_TIM17_BASE,
  .pclk        = STM32L4_APB2_TIM17_CLKIN,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4pwm_getreg
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

static uint16_t stm32l4pwm_getreg(struct stm32l4_pwmtimer_s *priv, int offset)
{
  return getreg16(priv->base + offset);
}

/****************************************************************************
 * Name: stm32l4pwm_putreg
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

static void stm32l4pwm_putreg(struct stm32l4_pwmtimer_s *priv, int offset,
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
 * Name: stm32l4pwm_dumpregs
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
static void stm32l4pwm_dumpregs(struct stm32l4_pwmtimer_s *priv,
                                FAR const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CR1_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CR2_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_SMCR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_DIER_OFFSET));
  pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_GTIM_SR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_EGR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCMR1_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCMR2_OFFSET));
  pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCER_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CNT_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_PSC_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_ARR_OFFSET));
  pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCR1_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCR2_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCR3_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_ATIM_RCR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_ATIM_DCR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_ATIM_DMAR_OFFSET));
    }
  else
#endif
    {
      pwminfo("  DCR: %04x DMAR: %04x\n",
          stm32l4pwm_getreg(priv, STM32L4_GTIM_DCR_OFFSET),
          stm32l4pwm_getreg(priv, STM32L4_GTIM_DMAR_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: stm32l4pwm_timer
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

static int stm32l4pwm_timer(FAR struct stm32l4_pwmtimer_s *priv,
                            FAR const struct pwm_info_s *info)
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
  uint32_t ccmr1;
  uint32_t ccmr2;

  /* New timer register bit settings */

  uint16_t ccenable;
  uint16_t ccnenable;
  uint32_t ocmode1;
  uint32_t ocmode2;

  DEBUGASSERT(priv != NULL && info != NULL);

#if defined(CONFIG_PWM_MULTICHAN)
  pwminfo("TIM%u frequency: %u\n",
          priv->timid, info->frequency);
#elif defined(CONFIG_PWM_PULSECOUNT)
  pwminfo("TIM%u channel: %u frequency: %u duty: %08x count: %u\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty, info->count);
#else
  pwminfo("TIM%u channel: %u frequency: %u duty: %08x\n",
          priv->timid, priv->channels[0].channel, info->frequency, info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* Disable all interrupts and DMA requests, clear all pending status */

#ifdef CONFIG_PWM_PULSECOUNT
  stm32l4pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, 0);
  stm32l4pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);
#endif

  /* Calculate optimal values for the timer prescaler and for the timer reload
   * register.  If 'frequency' is the desired frequency, then
   *
   *   reload = timclk / frequency
   *   timclk = pclk / presc
   *
   * Or,
   *
   *   reload = pclk / presc / frequency
   *
   * There are many solutions to this this, but the best solution will be the
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

  pwminfo("TIM%u PCLK: %u frequency: %u TIMCLK: %u prescaler: %u reload: %u\n",
          priv->timid, priv->pclk, info->frequency, timclk, prescaler, reload);

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = stm32l4pwm_getreg(priv, STM32L4_GTIM_CR1_OFFSET);

  /* Disable the timer until we get it configured */

  cr1 &= ~GTIM_CR1_CEN;

  /* Set the counter mode for the advanced timers (1,8) and most general
   * purpose timers (all 2-5, but not 9-17), i.e., all but TIMTYPE_COUNTUP16
   * and TIMTYPE_BASIC
   */

#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM2_PWM) || \
    defined(CONFIG_STM32L4_TIM3_PWM) || defined(CONFIG_STM32L4_TIM4_PWM) || \
    defined(CONFIG_STM32L4_TIM5_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)

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
            cr1 |= GTIM_CR1_EDGE;
            break;

          case STM32L4_TIMMODE_COUNTDOWN:
            cr1 |= GTIM_CR1_EDGE | GTIM_CR1_DIR;
            break;

          case STM32L4_TIMMODE_CENTER1:
            cr1 |= GTIM_CR1_CENTER1;
            break;

          case STM32L4_TIMMODE_CENTER2:
            cr1 |= GTIM_CR1_CENTER2;
            break;

          case STM32L4_TIMMODE_CENTER3:
            cr1 |= GTIM_CR1_CENTER3;
            break;

          default:
            pwmerr("ERROR: No such timer mode: %u\n", (unsigned int)priv->mode);
            return -EINVAL;
        }
    }
#endif

  /* Set the clock division to zero for all (but the basic timers, but there
   * should be no basic timers in this context
   */

  cr1 &= ~GTIM_CR1_CKD_MASK;
  stm32l4pwm_putreg(priv, STM32L4_GTIM_CR1_OFFSET, cr1);

  /* Set the reload and prescaler values */

  stm32l4pwm_putreg(priv, STM32L4_GTIM_ARR_OFFSET, (uint16_t)reload);
  stm32l4pwm_putreg(priv, STM32L4_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  /* Set the advanced timer's repetition counter */

#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repitition counter to the count-1 (stm32l4pwm_start() has already
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

          priv->prev  = stm32l4pwm_pulsecount(info->count);
          stm32l4pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, (uint16_t)priv->prev - 1);

          /* Generate an update event to reload the prescaler.  This should
           * preload the RCR into active repetition counter.
           */

          stm32l4pwm_putreg(priv, STM32L4_GTIM_EGR_OFFSET, ATIM_EGR_UG);

          /* Now set the value of the RCR that will be loaded on the next
           * update event.
           */

          priv->count = info->count;
          priv->curr  = stm32l4pwm_pulsecount(info->count - priv->prev);
          stm32l4pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
        }

      /* Otherwise, just clear the repetition counter */

      else
#endif
        {
          /* Set the repetition counter to zero */

          stm32l4pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, 0);

          /* Generate an update event to reload the prescaler */

          stm32l4pwm_putreg(priv, STM32L4_GTIM_EGR_OFFSET, ATIM_EGR_UG);
        }
    }
  else
#endif
    {
      /* Generate an update event to reload the prescaler (all timers) */

      stm32l4pwm_putreg(priv, STM32L4_GTIM_EGR_OFFSET, ATIM_EGR_UG);
    }

  /* Handle channel specific setup */

  ccenable  = 0;
  ccnenable = 0;
  ocmode1   = 0;
  ocmode2   = 0;

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
      ub16_t                duty;
      uint32_t              chanmode;
      uint32_t              compout; /* Complementary output config */
      bool                  ocmbit = false;
      uint8_t               channel;
#ifdef CONFIG_PWM_MULTICHAN
      int                   j;
#endif
      enum stm32l4_chanmode_e mode;

#ifdef CONFIG_PWM_MULTICHAN
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
              compout = priv->channels[j].npincfg;
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
      compout = priv->channels[0].npincfg;
#endif

      /* Duty cycle:
       *
       * duty cycle = ccr / reload (fractional value)
       */

      ccr = b16toi(duty * reload + b16HALF);

      pwminfo("ccr: %u\n", ccr);

      switch (mode)
        {
          case STM32L4_CHANMODE_PWM1:
            chanmode = ATIM_CCMR_MODE_PWM1;
            break;

          case STM32L4_CHANMODE_PWM2:
            chanmode = ATIM_CCMR_MODE_PWM2;
            break;

          case STM32L4_CHANMODE_COMBINED1:
            chanmode = ATIM_CCMR_MODE_COMBINED1;
            ocmbit = true;
            break;

          case STM32L4_CHANMODE_COMBINED2:
            chanmode = ATIM_CCMR_MODE_COMBINED2;
            ocmbit = true;
            break;

          case STM32L4_CHANMODE_ASYMMETRIC1:
            chanmode = ATIM_CCMR_MODE_ASYMMETRIC1;
            ocmbit = true;
            break;

          case STM32L4_CHANMODE_ASYMMETRIC2:
            chanmode = ATIM_CCMR_MODE_ASYMMETRIC2;
            ocmbit = true;
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

              ccenable |= ATIM_CCER_CC1E;

              /* Conditionally enable the complementary output */

              if (compout)
                {
                  ccnenable |= ATIM_CCER_CC1NE;
                }

              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT) |
                          (chanmode << ATIM_CCMR1_OC1M_SHIFT) |
                          ATIM_CCMR1_OC1PE;

              if (ocmbit)
                {
                  ocmode1 |= ATIM_CCMR1_OC1M;
                }

              /* Set the duty cycle by writing to the CCR register for this channel */

              stm32l4pwm_putreg(priv, STM32L4_GTIM_CCR1_OFFSET, (uint16_t)ccr);
            }
            break;

          case 2:  /* PWM Mode configuration: Channel 2 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= ATIM_CCER_CC2E;

              /* Conditionally enable the complementary output */

              if (compout)
                {
                  ccnenable |= ATIM_CCER_CC2NE;
                }

              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC2S_SHIFT) |
                          (chanmode << ATIM_CCMR1_OC2M_SHIFT) |
                          ATIM_CCMR1_OC2PE;

              if (ocmbit)
                {
                  ocmode1 |= ATIM_CCMR1_OC2M;
                }

              /* Set the duty cycle by writing to the CCR register for this channel */

              stm32l4pwm_putreg(priv, STM32L4_GTIM_CCR2_OFFSET, (uint16_t)ccr);
            }
            break;

          case 3:  /* PWM Mode configuration: Channel 3 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= ATIM_CCER_CC3E;

              /* Conditionally enable the complementary output */

              if (compout)
                {
                  ccnenable |= ATIM_CCER_CC3NE;
                }

              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC3S_SHIFT) |
                          (chanmode << ATIM_CCMR2_OC3M_SHIFT) |
                          ATIM_CCMR2_OC3PE;

              if (ocmbit)
                {
                  ocmode2 |= ATIM_CCMR2_OC3M;
                }

              /* Set the duty cycle by writing to the CCR register for this channel */

              stm32l4pwm_putreg(priv, STM32L4_GTIM_CCR3_OFFSET, (uint16_t)ccr);
            }
            break;

          case 4:  /* PWM Mode configuration: Channel 4 */
            {
              /* Select the CCER enable bit for this channel */

              ccenable |= ATIM_CCER_CC4E;

              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC4S_SHIFT) |
                          (chanmode << ATIM_CCMR2_OC4M_SHIFT) |
                          ATIM_CCMR2_OC4PE;

              if (ocmbit)
                {
                  ocmode2 |= ATIM_CCMR2_OC4M;
                }

              /* Set the duty cycle by writing to the CCR register for this channel */

              stm32l4pwm_putreg(priv, STM32L4_GTIM_CCR4_OFFSET, (uint16_t)ccr);
            }
            break;

          default:
            pwmerr("ERROR: No such channel: %u\n", channel);
            return -EINVAL;
        }
    }

  /* Disable the Channel by resetting the CCxE Bit in the CCER register */

  ccer = stm32l4pwm_getreg(priv, STM32L4_GTIM_CCER_OFFSET);
  ccer &= ~ccenable;
  stm32l4pwm_putreg(priv, STM32L4_GTIM_CCER_OFFSET, ccer);

  /* Fetch the CR2, CCMR1, and CCMR2 register (already have cr1 and ccer) */

  cr2   = stm32l4pwm_getreg(priv, STM32L4_GTIM_CR2_OFFSET);
  ccmr1 = stm32l4pwm_getreg(priv, STM32L4_GTIM_CCMR1_OFFSET);
  ccmr2 = stm32l4pwm_getreg(priv, STM32L4_GTIM_CCMR2_OFFSET);

  /* Reset the Output Compare Mode Bits and set the select output compare mode */

  ccmr1 &= ~(ATIM_CCMR1_CC1S_MASK | ATIM_CCMR1_OC1M_MASK | ATIM_CCMR1_OC1PE |
             ATIM_CCMR1_CC2S_MASK | ATIM_CCMR1_OC2M_MASK | ATIM_CCMR1_OC2PE
             | ATIM_CCMR1_OC1M | ATIM_CCMR1_OC2M
             );
  ccmr2 &= ~(ATIM_CCMR2_CC3S_MASK | ATIM_CCMR2_OC3M_MASK | ATIM_CCMR2_OC3PE |
             ATIM_CCMR2_CC4S_MASK | ATIM_CCMR2_OC4M_MASK | ATIM_CCMR2_OC4PE
             | ATIM_CCMR2_OC3M | ATIM_CCMR2_OC4M
             );
  ccmr1 |= ocmode1;
  ccmr2 |= ocmode2;

  /* Reset the output polarity level of all channels (selects high polarity)*/

  ccer &= ~(ATIM_CCER_CC1P | ATIM_CCER_CC2P | ATIM_CCER_CC3P | ATIM_CCER_CC4P);

  /* Enable the output state of the selected channels */

  ccer &= ~(ATIM_CCER_CC1E | ATIM_CCER_CC2E | ATIM_CCER_CC3E | ATIM_CCER_CC4E);
  ccer |= ccenable;

  /* Some special setup for advanced timers */

#if defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      uint16_t bdtr;

      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP | ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP);

      ccer |= ccnenable;

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N | ATIM_CR2_OIS2 | ATIM_CR2_OIS2N |
               ATIM_CR2_OIS3 | ATIM_CR2_OIS3N | ATIM_CR2_OIS4);

      /* Set the main output enable (MOE) bit and clear the OSSI and OSSR
       * bits in the BDTR register.
       */

      bdtr  = stm32l4pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET);
      bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);
      bdtr |= ATIM_BDTR_MOE;
      stm32l4pwm_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, bdtr);
    }
  else
#if defined(CONFIG_STM32L4_TIM15_PWM) || defined(CONFIG_STM32L4_TIM16_PWM) || \
    defined(CONFIG_STM32L4_TIM17_PWM)
  if (priv->timtype == TIMTYPE_COUNTUP16)
    {

      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP);
      ccer |= ccnenable;
    }
  else
#endif
#endif
    {
      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP); /* Not sure why? */
    }

  /* Save the modified register values */

  stm32l4pwm_putreg(priv, STM32L4_GTIM_CR2_OFFSET, cr2);
  putreg32(ccmr1, priv->base + STM32L4_GTIM_CCMR1_OFFSET);
  putreg32(ccmr2, priv->base + STM32L4_GTIM_CCMR2_OFFSET);
  stm32l4pwm_putreg(priv, STM32L4_GTIM_CCER_OFFSET, ccer);

  /* Set the ARR Preload Bit */

  cr1 = stm32l4pwm_getreg(priv, STM32L4_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_ARPE;
  stm32l4pwm_putreg(priv, STM32L4_GTIM_CR1_OFFSET, cr1);

  /* Setup update interrupt.  If info->count is > 0, then we can be
   * assured that stm32l4pwm_start() has already verified: (1) that this is an
   * advanced timer, and that (2) the repetition count is within range.
   */

#ifdef CONFIG_PWM_PULSECOUNT
  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      stm32l4pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);
      stm32l4pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, ATIM_DIER_UIE);

      /* Enable the timer */

      cr1 |= GTIM_CR1_CEN;
      stm32l4pwm_putreg(priv, STM32L4_GTIM_CR1_OFFSET, cr1);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }
  else
#endif
    {
      /* Just enable the timer, leaving all interrupts disabled */

      cr1 |= GTIM_CR1_CEN;
      stm32l4pwm_putreg(priv, STM32L4_GTIM_CR1_OFFSET, cr1);
    }

  stm32l4pwm_dumpregs(priv, "After starting");
  return OK;
}

#ifndef CONFIG_PWM_PULSECOUNT
/****************************************************************************
 * Name: stm32l4pwm_update_duty
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

static  int stm32l4pwm_update_duty(FAR struct stm32l4_pwmtimer_s *priv,
                                   uint8_t channel, ub16_t duty)
{
  /* Register offset */

  int ccr_offset;

  /* Calculated values */

  uint32_t reload;
  uint32_t ccr;

  DEBUGASSERT(priv != NULL);

  pwminfo("TIM%u channel: %u duty: %08x\n",
          priv->timid, channel, duty);

#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Get the reload values */

  reload = stm32l4pwm_getreg(priv, STM32L4_GTIM_ARR_OFFSET);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  pwminfo("ccr: %u\n", ccr);

  switch (channel)
    {
      case 1:  /* Register offset for Channel 1 */
        ccr_offset = STM32L4_GTIM_CCR1_OFFSET;
        break;

      case 2:  /* Register offset for Channel 2 */
        ccr_offset = STM32L4_GTIM_CCR2_OFFSET;
        break;

      case 3:  /* Register offset for Channel 3 */
        ccr_offset = STM32L4_GTIM_CCR3_OFFSET;
        break;

      case 4:  /* Register offset for Channel 4 */
        ccr_offset = STM32L4_GTIM_CCR4_OFFSET;
        break;

      default:
        pwmerr("ERROR: No such channel: %u\n", channel);
        return -EINVAL;
    }

  /* Set the duty cycle by writing to the CCR register for this channel */

  stm32l4pwm_putreg(priv, ccr_offset, (uint16_t)ccr);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32l4pwm_interrupt
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

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM))
static int stm32l4pwm_interrupt(struct stm32l4_pwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = stm32l4pwm_getreg(priv, STM32L4_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  stm32l4pwm_putreg(priv, STM32L4_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = stm32l4pwm_getreg(priv, STM32L4_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      stm32l4pwm_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrtups, stop and reset the timer */

      (void)stm32l4pwm_stop((FAR struct pwm_lowerhalf_s *)priv);

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
      priv->curr = stm32l4pwm_pulsecount(priv->count - priv->prev);
      stm32l4pwm_putreg(priv, STM32L4_ATIM_RCR_OFFSET, (uint16_t)priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug output */

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

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32L4_TIM1_PWM)
static int stm32l4pwm_tim1interrupt(int irq, void *context, FAR void *arg)
{
  return stm32l4pwm_interrupt(&g_pwm1dev);
}
#endif

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32L4_TIM8_PWM)
static int stm32l4pwm_tim8interrupt(int irq, void *context, FAR void *arg)
{
  return stm32l4pwm_interrupt(&g_pwm8dev);
}
#endif

/****************************************************************************
 * Name: stm32l4pwm_pulsecount
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

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32L4_TIM1_PWM) || defined(CONFIG_STM32L4_TIM8_PWM))
static uint8_t stm32l4pwm_pulsecount(uint32_t count)
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
 * Name: stm32l4pwm_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void stm32l4pwm_setapbclock(FAR struct stm32l4_pwmtimer_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32L4_TIM1_PWM
      case 1:
        regaddr  = STM32L4_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM1EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM2_PWM
      case 2:
        regaddr  = STM32L4_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM2EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM3_PWM
      case 3:
        regaddr  = STM32L4_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM3EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM4_PWM
      case 4:
        regaddr  = STM32L4_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM4EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM5_PWM
      case 5:
        regaddr  = STM32L4_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM5EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM8_PWM
      case 8:
        regaddr  = STM32L4_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM8EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM15_PWM
      case 15:
        regaddr  = STM32L4_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM15EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM16_PWM
      case 16:
        regaddr  = STM32L4_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM16EN;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM17_PWM
      case 17:
        regaddr  = STM32L4_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM17EN;
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
 * Name: stm32l4pwm_setup
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

static int stm32l4pwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);
  stm32l4pwm_dumpregs(priv, "Initially");

  /* Enable APB1/2 clocking for timer. */

  stm32l4pwm_setapbclock(priv, true);

  /* Configure the PWM output pins, but do not start the timer yet */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08x\n", pincfg);

          stm32l4_configgpio(pincfg);
        }


      /* Enable complementary channel if available */

      pincfg = priv->channels[i].npincfg;
      if (pincfg != 0)
        {
          pwminfo("npincfg: %08x\n", pincfg);

          stm32l4_configgpio(pincfg);
        }

      pwm_dumpgpio(pincfg, "PWM setup");
    }

  return OK;
}

/****************************************************************************
 * Name: stm32l4pwm_shutdown
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

static int stm32l4pwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  stm32l4pwm_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  stm32l4pwm_setapbclock(priv, false);

  /* Then put the GPIO pins back to the default state */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08x\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32l4_configgpio(pincfg);
        }

      pincfg = priv->channels[i].npincfg;
      if (pincfg != 0)
        {
          pwminfo("npincfg: %08x\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32l4_configgpio(pincfg);
        }

    }

  return OK;
}

/****************************************************************************
 * Name: stm32l4pwm_start
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
static int stm32l4pwm_start(FAR struct pwm_lowerhalf_s *dev,
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

  return stm32l4pwm_timer(priv, info);
}
#else
static int stm32l4pwm_start(FAR struct pwm_lowerhalf_s *dev,
                     FAR const struct pwm_info_s *info)
{
  int ret = OK;
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;

#ifndef CONFIG_PWM_PULSECOUNT
  /* if frequency has not changed we just update duty */

  if (info->frequency == priv->frequency)
    {
#ifdef CONFIG_PWM_MULTICHAN
      int i;

      for (i = 0; ret == OK && i < CONFIG_PWM_NCHANNELS; i++)
        {
          ret = stm32l4pwm_update_duty(priv,info->channels[i].channel,
                                info->channels[i].duty);
        }
#else
      ret = stm32l4pwm_update_duty(priv,priv->channels[0].channel,info->duty);
#endif
    }
  else
#endif
    {
      ret = stm32l4pwm_timer(priv, info);

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
 * Name: stm32l4pwm_stop
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

static int stm32l4pwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct stm32l4_pwmtimer_s *priv = (FAR struct stm32l4_pwmtimer_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  pwminfo("TIM%u\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Stopped so frequency is zero */

  priv->frequency = 0;

  /* Disable further interrupts and stop the timer */

  stm32l4pwm_putreg(priv, STM32L4_GTIM_DIER_OFFSET, 0);
  stm32l4pwm_putreg(priv, STM32L4_GTIM_SR_OFFSET, 0);

  /* Determine which timer to reset */

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
        regaddr  = STM32L4_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM2RST;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM3_PWM
      case 3:
        regaddr  = STM32L4_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM3RST;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM4_PWM
      case 4:
        regaddr  = STM32L4_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM4RST;
        break;
#endif
#ifdef CONFIG_STM32L4_TIM5_PWM
      case 5:
        regaddr  = STM32L4_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM5RST;
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

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where stm32l4pwm_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  pwminfo("regaddr: %08x resetbit: %08x\n", regaddr, resetbit);
  stm32l4pwm_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: stm32l4pwm_ioctl
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

static int stm32l4pwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
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
        irq_attach(lower->irq, stm32l4pwm_tim1interrupt, NULL);
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
        irq_attach(lower->irq, stm32l4pwm_tim8interrupt, NULL);
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

#endif /* CONFIG_STM32L4_TIMn_PWM, n = 1,...,17 */
