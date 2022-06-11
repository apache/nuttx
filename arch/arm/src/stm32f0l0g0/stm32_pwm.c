/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_pwm.c
 *
 *   Copyright (C) 2019 Fundação CERTI. All rights reserved.
 *   Author: Daniel Pereira Volpato <dpo@certi.org.br>
 *           Guillherme da Silva amaral <gvr@certi.org.br>
 *
 *   Based on: arch/arm/src/stm32h7/stm32_pwm.c
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *           Mateusz Szafoni <raiden00@railab.me>
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

#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_pwm.h"
#include "stm32_rcc.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM)  || defined(CONFIG_STM32F0L0G0_TIM2_PWM)   || \
    defined(CONFIG_STM32F0L0G0_TIM3_PWM)  || defined(CONFIG_STM32F0L0G0_TIM14_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM15_PWM) || defined(CONFIG_STM32F0L0G0_TIM16_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM17_PWM)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types */

#define TIMTYPE_BASIC        0  /* Basic timers: TIM6,7 */
#define TIMTYPE_GENERAL16    1  /* General 16-bit timers: TIM3 */
#define TIMTYPE_COUNTUP16    2  /* General 16-bit count-up timers: TIM14 */
#define TIMTYPE_GENERAL32    3  /* General 32-bit timers: TIM2 */
#define TIMTYPE_ADVANCED     4  /* Advanced timers:  TIM1 */
#define TIMTYPE_COUNTUP16_N  5  /* General 16-bit count-up timers with
                                 * one complementary output: TIM15-17
                                 */

#define TIMTYPE_TIM1         TIMTYPE_ADVANCED
#define TIMTYPE_TIM2         TIMTYPE_GENERAL32
#define TIMTYPE_TIM3         TIMTYPE_GENERAL16
#define TIMTYPE_TIM6         TIMTYPE_BASIC
#define TIMTYPE_TIM7         TIMTYPE_BASIC
#define TIMTYPE_TIM14        TIMTYPE_COUNTUP16
#define TIMTYPE_TIM15        TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM16        TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM17        TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */

/* Advanced timer */

#if defined (CONFIG_STM32F0L0G0_TIM1_PWM)
#  define HAVE_IP_TIMERS_V2 1
#endif

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM8_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM15_PWM) || \
    defined(CONFIG_STM32F0L0G0_TIM16_PWM) || \
    defined(CONFIG_STM32F0L0G0_TIM17_PWM)
#   define HAVE_ADVTIM
#else
#   undef HAVE_ADVTIM
#endif

/* CCMR2 */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM)  || \
    defined(CONFIG_STM32F0L0G0_TIM3_PWM)
#   define HAVE_CCMR2
#else
#   undef HAVE_CCMR2
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m)
#  warning "pwm_dumpgpio not implemented"
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
  enum stm32_chanmode_e mode;
  uint32_t pincfg;                     /* Output pin configuration */
  uint32_t npincfg;                    /* Complementary output pin configuration
                                        * (only TIM1,8 CH1-3 and TIM15,16,17 CH1)
                                        */
};

/* This structure represents the state of one PWM timer */

struct stm32_pwmtimer_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  struct stm32_pwmchan_s channels[PWM_NCHANNELS];
  uint8_t timid;                       /* Timer ID {1,...,17} */
  uint8_t timtype;                     /* See the TIMTYPE_* definitions */
  enum stm32_timmode_e mode;
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t irq;                         /* Timer update IRQ */
  uint8_t prev;                        /* The previous value of the RCR (pre-loaded) */
  uint8_t curr;                        /* The current value of the RCR (pre-loaded) */
  uint32_t count;                      /* Remaining pulse count */
#else
  uint32_t frequency;                  /* Current frequency setting */
#endif
  uint32_t base;                       /* The base address of the timer */
  uint32_t pclk;                       /* The frequency of the peripheral clock
                                        * that drives the timer module. */
#ifdef CONFIG_PWM_PULSECOUNT
  void *handle;                    /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t stm32pwm_getreg(struct stm32_pwmtimer_s *priv, int offset);
static void stm32pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                            uint32_t value);
static void stm32pwm_modifyreg(struct stm32_pwmtimer_s *priv,
                               uint32_t offset, uint32_t clearbits,
                               uint32_t setbits);

#ifdef CONFIG_DEBUG_PWM_INFO
static void stm32pwm_dumpregs(struct stm32_pwmtimer_s *priv,
                              const char *msg);
#else
#  define stm32pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int stm32pwm_timer(struct stm32_pwmtimer_s *priv,
                          const struct pwm_info_s *info);
static int stm32pwm_output_configure(struct stm32_pwmtimer_s *priv,
                                     uint8_t channel);
static  int stm32pwm_update_duty(struct stm32_pwmtimer_s *priv,
                                 uint8_t channel, ub16_t duty);
static void stm32pwm_setapbclock(struct stm32_pwmtimer_s *priv, bool on);

#if defined(CONFIG_PWM_PULSECOUNT) && \
    (defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM))
static int stm32pwm_interrupt(struct stm32_pwmtimer_s *priv);
#  if defined(CONFIG_STM32F0L0G0_TIM1_PWM)
static int stm32pwm_tim1interrupt(int irq, void *context, void *arg);
#  endif
#  if defined(CONFIG_STM32F0L0G0_TIM8_PWM)
static int stm32pwm_tim8interrupt(int irq, void *context, void *arg);
#  endif
static uint8_t stm32pwm_pulsecount(uint32_t count);
#endif /* CONFIG_PWM_PULSECOUNT && CONFIG_STM32F0L0G0_TIM{1,8}_PWM */

/* PWM driver methods */

static int stm32pwm_setup(struct pwm_lowerhalf_s *dev);
static int stm32pwm_shutdown(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int stm32pwm_start(struct pwm_lowerhalf_s *dev,
                          const struct pwm_info_s *info,
                          void *handle);
#else
static int stm32pwm_start(struct pwm_lowerhalf_s *dev,
                          const struct pwm_info_s *info);
#endif

static int stm32pwm_stop(struct pwm_lowerhalf_s *dev);
static int stm32pwm_ioctl(struct pwm_lowerhalf_s *dev,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = stm32pwm_setup,
  .shutdown    = stm32pwm_shutdown,
  .start       = stm32pwm_start,
  .stop        = stm32pwm_stop,
  .ioctl       = stm32pwm_ioctl,
};

#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
static struct stm32_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
  .timid       = 1,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM1_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM1_CH1MODE,
      .npincfg = PWM_TIM1_CH1NCFG,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM1_CH2CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM1_CH2MODE,
      .npincfg = PWM_TIM1_CH2NCFG,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM1_CH3CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM1_CH3MODE,
      .npincfg = PWM_TIM1_CH3NCFG,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM1_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM1_CH4CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM1_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM1,
  .mode        = CONFIG_STM32F0L0G0_TIM1_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM1UP,
#endif
  .base        = STM32_TIM1_BASE,
  .pclk        = STM32_APB2_TIM1_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
static struct stm32_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
  .timid       = 2,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM2_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM2_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM2_CH2CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM2_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM2_CH3CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM2_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM2_CH4CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM2_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM2,
  .mode        = CONFIG_STM32F0L0G0_TIM2_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM2,
#endif
  .base        = STM32_TIM2_BASE,
  .pclk        = STM32_APB1_TIM2_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
static struct stm32_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
  .timid       = 3,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM3_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM3_CH1MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM3_CH2CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM3_CH2MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL3
    {
      .channel = 3,
      .pincfg  = PWM_TIM3_CH3CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM3_CH3MODE,
      .npincfg = 0,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_CHANNEL4
    {
      .channel = 4,
      .pincfg  = PWM_TIM3_CH4CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM3_CH4MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM3,
  .mode        = CONFIG_STM32F0L0G0_TIM3_MODE,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM3,
#endif
  .base        = STM32_TIM3_BASE,
  .pclk        = STM32_APB1_TIM3_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
static struct stm32_pwmtimer_s g_pwm14dev =
{
  .ops         = &g_pwmops,
  .timid       = 14,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM14_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM14_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM14_CH1MODE,
      .npincfg = PWM_TIM14_CH1NCFG,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM14,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM14,
#endif
  .base        = STM32_TIM14_BASE,
  .pclk        = STM32_APB2_TIM14_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
static struct stm32_pwmtimer_s g_pwm15dev =
{
  .ops         = &g_pwmops,
  .timid       = 15,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM15_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM15_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM15_CH1MODE,
      .npincfg = PWM_TIM15_CH1NCFG,
    },
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15_CHANNEL2
    {
      .channel = 2,
      .pincfg  = PWM_TIM15_CH2CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM15_CH2MODE,
      .npincfg = 0,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM15,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM15,
#endif
  .base        = STM32_TIM15_BASE,
  .pclk        = STM32_APB2_TIM15_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
static struct stm32_pwmtimer_s g_pwm16dev =
{
  .ops         = &g_pwmops,
  .timid       = 16,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM16_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM16_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM16_CH1MODE,
      .npincfg = PWM_TIM16_CH1NCFG,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM16,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM16,
#endif
  .base        = STM32_TIM16_BASE,
  .pclk        = STM32_APB2_TIM16_CLKIN,
};
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
static struct stm32_pwmtimer_s g_pwm17dev =
{
  .ops         = &g_pwmops,
  .timid       = 17,
  .channels    =
  {
#ifdef CONFIG_STM32F0L0G0_TIM17_CHANNEL1
    {
      .channel = 1,
      .pincfg  = PWM_TIM17_CH1CFG,
      .mode    = CONFIG_STM32F0L0G0_TIM17_CH1MODE,
      .npincfg = PWM_TIM17_CH1NCFG,
    },
#endif
  },
  .timtype     = TIMTYPE_TIM17,
  .mode        = STM32_TIMMODE_COUNTUP,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM17,
#endif
  .base        = STM32_TIM17_BASE,
  .pclk        = STM32_APB2_TIM17_CLKIN,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32pwm_reg_is_32bit
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

static bool stm32pwm_reg_is_32bit(uint8_t timtype, uint32_t offset)
{
  if (offset == STM32_GTIM_CCMR1_OFFSET ||
      offset == STM32_GTIM_SMCR_OFFSET  ||
      offset == STM32_GTIM_BDTR_OFFSET)
    {
      return true;
    }

  if (timtype == TIMTYPE_GENERAL16)
    {
      if (offset == STM32_GTIM_CCMR2_OFFSET ||
          offset == STM32_GTIM_AF1_OFFSET   ||
          offset == STM32_GTIM_TISEL_OFFSET)
        {
          return true;
        }
    }
  else if (timtype == TIMTYPE_GENERAL32)
    {
      if (offset == STM32_GTIM_CNT_OFFSET  ||
          offset == STM32_GTIM_ARR_OFFSET  ||
          offset == STM32_GTIM_CCR1_OFFSET ||
          offset == STM32_GTIM_CCR2_OFFSET ||
          offset == STM32_GTIM_CCR3_OFFSET ||
          offset == STM32_GTIM_CCR4_OFFSET)
        {
          return true;
        }
    }
  else if (timtype == TIMTYPE_ADVANCED)
    {
      if (offset == STM32_ATIM_CR2_OFFSET   ||
          offset == STM32_ATIM_CCMR2_OFFSET ||
          offset == STM32_ATIM_CCER_OFFSET  ||
          offset == STM32_ATIM_DMAR_OFFSET  ||
          offset == STM32_ATIM_CCMR3_OFFSET ||
          offset == STM32_ATIM_CCR5_OFFSET  ||
          offset == STM32_ATIM_AF1_OFFSET   ||
          offset == STM32_ATIM_TISEL_OFFSET)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: stm32pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t stm32pwm_getreg(struct stm32_pwmtimer_s *priv, int offset)
{
  uint32_t retval;

  if (stm32pwm_reg_is_32bit(priv->timtype, offset) == true)
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
 * Name: stm32pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                            uint32_t value)
{
  if (stm32pwm_reg_is_32bit(priv->timtype, offset) == true)
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
 * Name: stm32pwm_modifyreg
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

static void stm32pwm_modifyreg(struct stm32_pwmtimer_s *priv,
                               uint32_t offset, uint32_t clearbits,
                               uint32_t setbits)
{
  if (stm32pwm_reg_is_32bit(priv->timtype, offset) == true)
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
 * Name: stm32pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   msg  - A message to be printed on the screen
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void stm32pwm_dumpregs(struct stm32_pwmtimer_s *priv,
                              const char *msg)
{
  pwminfo("%s:\n", msg);
  pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_SMCR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
  pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_CCER_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CNT_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_PSC_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_ARR_OFFSET));
  pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CCR2_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CCR3_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
          stm32pwm_getreg(priv, STM32_ATIM_RCR_OFFSET),
          stm32pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET),
          stm32pwm_getreg(priv, STM32_ATIM_DCR_OFFSET),
          stm32pwm_getreg(priv, STM32_ATIM_DMAR_OFFSET));

          pwminfo("  AF1: %04x TISEL: %04x\n",
          stm32pwm_getreg(priv, STM32_ATIM_AF1_OFFSET),
          stm32pwm_getreg(priv, STM32_ATIM_TISEL_OFFSET));
    }
  else
#endif
    {
      pwminfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_RCR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_BDTR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_DCR_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_DMAR_OFFSET));
      pwminfo("  AF1: %04x TISEL: %04x\n",
          stm32pwm_getreg(priv, STM32_GTIM_AF1_OFFSET),
          stm32pwm_getreg(priv, STM32_GTIM_TISEL_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: stm32pwm_output_configure
 *
 * Description:
 *   Configure PWM output for given channel
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   channel - Timer output channel
 *
 * Returned Value:
 *   Zero on success;
 ****************************************************************************/

static int stm32pwm_output_configure(struct stm32_pwmtimer_s *priv,
                                     uint8_t channel)
{
  uint32_t cr2;
  uint32_t ccer;

  /* Get current registers state */

  cr2  = stm32pwm_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccer = stm32pwm_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* Reset the output polarity level of all channels (selects high
   * polarity)
   */

  ccer &= ~(GTIM_CCER_CC1P << ((channel - 1) * 4));

  /* Enable the output state of the selected channels */

  ccer |= (GTIM_CCER_CC1E << ((channel - 1) * 4));

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      cr2 &= ~(ATIM_CR2_OIS1 << ((channel - 1) * 2));
    }
#ifdef HAVE_PWM_COMPLEMENTARY

  /* Verify if the current complementary channel is defined */

  if (priv->channels[channel - 1].npincfg != 0)
    {
      /* Configure complementary output IDLE state */

      cr2 &= ~(ATIM_CR2_OIS1N << ((channel - 1) * 2));

      /* Enable the complementary output state of the selected channels */

      ccer |= (ATIM_CCER_CC1NE << ((channel - 1) * 4));

      /* Configure complementary output polarity */

      ccer &= ~(ATIM_CCER_CC1NP << ((channel - 1) * 4));
    }
#endif /* HAVE_PWM_COMPLEMENTARY */
#endif /* HAVE_ADVTIM */

  stm32pwm_modifyreg(priv, STM32_GTIM_CR2_OFFSET, 0, cr2);
  stm32pwm_modifyreg(priv, STM32_GTIM_CCER_OFFSET, 0, ccer);

  return OK;
}

/****************************************************************************
 * Name: stm32pwm_timer
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

static int stm32pwm_timer(struct stm32_pwmtimer_s *priv,
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

  uint32_t cr1;
  uint32_t ccmr1;
#if defined(HAVE_CCMR2)
  uint32_t ccmr2;
  uint32_t ocmode2;
#endif

  /* New timer register bit settings */

  uint32_t ocmode1;

  DEBUGASSERT(priv != NULL && info != NULL);

  ccmr1 = stm32pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET);

#if defined(HAVE_CCMR2)
  ccmr2 = stm32pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET);
#endif

#if defined(CONFIG_PWM_MULTICHAN)
  pwminfo("TIM%u frequency: %" PRIu32 "\n",
          priv->timid, info->frequency);
#elif defined(CONFIG_PWM_PULSECOUNT)
  pwminfo("TIM%u channel: %u frequency: %" PRIu32 " duty: %08" PRIx32
          " count: %u\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty, info->count);
#else
  pwminfo("TIM%u channel: %u frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* Disable all interrupts and DMA requests, clear all pending status */

#ifdef CONFIG_PWM_PULSECOUNT
  stm32pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stm32pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
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

  /* In center-aligned mode, the timer performs upcounting from zero to ARR
   * value and then performs downcounting from ARR to zero and repeat. In
   * other words, in one cycle the timer counts 2*ARR. For that reason, the
   * reload (ARR) value is divided by 2.
   */

  if (priv->mode == STM32_TIMMODE_CENTER1 ||
      priv->mode == STM32_TIMMODE_CENTER2 ||
      priv->mode == STM32_TIMMODE_CENTER3)
    {
      reload /= 2;
    }

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

  pwminfo("TIM%u PCLK: %" PRIu32 " frequency: %" PRIu32 " "
          "TIMCLK: %" PRIu32 " prescaler: %" PRIu32
          " reload: %" PRIu32 "\n",
          priv->timid, priv->pclk, info->frequency, timclk,
          prescaler, reload);

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = stm32pwm_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Disable the timer until we get it configured */

  cr1 &= ~GTIM_CR1_CEN;

  /* Set the counter mode for the advanced timers (1,8) and most general
   * purpose timers (all 2-5, but not 9-17), i.e., all but TIMTYPE_COUNTUP16
   * and TIMTYPE_BASIC
   */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM2_PWM) || \
    defined(CONFIG_STM32F0L0G0_TIM3_PWM) || defined(CONFIG_STM32F0L0G0_TIM4_PWM) || \
    defined(CONFIG_STM32F0L0G0_TIM5_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM)

  if (priv->timtype != TIMTYPE_BASIC && priv->timtype != TIMTYPE_COUNTUP16 &&
      priv->timtype != TIMTYPE_COUNTUP16_N)
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
  stm32pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Set the reload and prescaler values */

  stm32pwm_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);
  stm32pwm_putreg(priv, STM32_GTIM_PSC_OFFSET, (prescaler - 1));

  /* Set the advanced timer's repetition counter */

#if defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM)
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repetition counter to the count-1 (stm32pwm_start() has already
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

          priv->prev  = stm32pwm_pulsecount(info->count);
          stm32pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->prev - 1);

          /* Generate an update event to reload the prescaler.  This should
           * preload the RCR into active repetition counter.
           */

          stm32pwm_putreg(priv, STM32_ATIM_EGR_OFFSET, ATIM_EGR_UG);

          /* Now set the value of the RCR that will be loaded on the next
           * update event.
           */

          priv->count = info->count;
          priv->curr  = stm32pwm_pulsecount(info->count - priv->prev);
          stm32pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->curr - 1);
        }

      /* Otherwise, just clear the repetition counter */

      else
#endif
        {
          /* Set the repetition counter to zero */

          stm32pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);

          /* Generate an update event to reload the prescaler */

          stm32pwm_putreg(priv, STM32_ATIM_EGR_OFFSET, ATIM_EGR_UG);
        }
    }
  else
#endif
    {
      /* Generate an update event to reload the prescaler (all timers) */

      stm32pwm_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);
    }

  /* Handle channel specific setup */

  ocmode1   = 0;
#if defined(HAVE_CCMR2)
  ocmode2   = 0;
#endif

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
      ub16_t                duty;
      uint32_t              chanmode;
      bool                  ocmbit = false;
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

      pwminfo("ccr: %" PRIu32 "\n", ccr);

      switch (mode)
        {
          case STM32_CHANMODE_PWM1:
            chanmode = GTIM_CCMR_MODE_PWM1;
            break;

          case STM32_CHANMODE_PWM2:
            chanmode = GTIM_CCMR_MODE_PWM2;
            break;

          case STM32_CHANMODE_COMBINED1:
            chanmode = GTIM_CCMR_MODE_COMBINED1;
            ocmbit = true;
            break;

          case STM32_CHANMODE_COMBINED2:
            chanmode = GTIM_CCMR_MODE_COMBINED2;
            ocmbit = true;
            break;

          case STM32_CHANMODE_ASYMMETRIC1:
            chanmode = GTIM_CCMR_MODE_ASYMMETRIC1;
            ocmbit = true;
            break;

          case STM32_CHANMODE_ASYMMETRIC2:
            chanmode = GTIM_CCMR_MODE_ASYMMETRIC2;
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
              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC1S_SHIFT) |
                          (chanmode << GTIM_CCMR1_OC1M_SHIFT) |
                          GTIM_CCMR1_OC1PE;

              if (ocmbit)
                {
                  ocmode1 |= GTIM_CCMR1_OC1M;
                }

              /* Set the duty cycle by writing to the CCR register for this
               * channel.
               */

              stm32pwm_putreg(priv, STM32_GTIM_CCR1_OFFSET, ccr);

              /* Reset the Output Compare Mode Bits and set the select
               * output compare mode.
               */

              ccmr1 &= ~(GTIM_CCMR1_CC1S_MASK | GTIM_CCMR1_OC1M_MASK |
                         GTIM_CCMR1_OC1PE | GTIM_CCMR1_OC1M);
              stm32pwm_output_configure(priv, channel);
            }
            break;

          case 2:  /* PWM Mode configuration: Channel 2 */
            {
              /* Set the CCMR1 mode values (leave CCMR2 zero) */

              ocmode1  |= (GTIM_CCMR_CCS_CCOUT << GTIM_CCMR1_CC2S_SHIFT) |
                          (chanmode << GTIM_CCMR1_OC2M_SHIFT) |
                          GTIM_CCMR1_OC2PE;

              if (ocmbit)
                {
                  ocmode1 |= GTIM_CCMR1_OC2M;
                }

              /* Set the duty cycle by writing to the CCR register for this
               * channel.
               */

              stm32pwm_putreg(priv, STM32_GTIM_CCR2_OFFSET, ccr);

              /* Reset the Output Compare Mode Bits and set the select
               * output compare mode.
               */

              ccmr1 &= ~(GTIM_CCMR1_CC2S_MASK | GTIM_CCMR1_OC2M_MASK |
                         GTIM_CCMR1_OC2PE | GTIM_CCMR1_OC2M);
              stm32pwm_output_configure(priv, channel);
            }
            break;

#if defined(HAVE_CCMR2)
          case 3:  /* PWM Mode configuration: Channel 3 */
            {
              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC3S_SHIFT) |
                          (chanmode << ATIM_CCMR2_OC3M_SHIFT) |
                          ATIM_CCMR2_OC3PE;

              if (ocmbit)
                {
                  ocmode2 |= ATIM_CCMR2_OC3M;
                }

              /* Set the duty cycle by writing to the CCR register for this
               * channel.
               */

              stm32pwm_putreg(priv, STM32_ATIM_CCR3_OFFSET, ccr);

              /* Reset the Output Compare Mode Bits and set the select
               * output compare mode.
               */

              ccmr2 &= ~(ATIM_CCMR2_CC3S_MASK | ATIM_CCMR2_OC3M_MASK |
                         ATIM_CCMR2_OC3PE | ATIM_CCMR2_OC3M);
              stm32pwm_output_configure(priv, channel);
            }
            break;

          case 4:  /* PWM Mode configuration: Channel 4 */
            {
              /* Set the CCMR2 mode values (leave CCMR1 zero) */

              ocmode2  |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC4S_SHIFT) |
                          (chanmode << ATIM_CCMR2_OC4M_SHIFT) |
                          ATIM_CCMR2_OC4PE;

              if (ocmbit)
                {
                  ocmode2 |= ATIM_CCMR2_OC4M;
                }

              /* Set the duty cycle by writing to the CCR register for this
               * channel.
               */

              stm32pwm_putreg(priv, STM32_ATIM_CCR4_OFFSET, ccr);

              /* Reset the Output Compare Mode Bits and set the select
               * output compare mode.
               */

              ccmr2 &= ~(ATIM_CCMR2_CC4S_MASK | ATIM_CCMR2_OC4M_MASK |
                         ATIM_CCMR2_OC4PE | ATIM_CCMR2_OC4M);
              stm32pwm_output_configure(priv, channel);
            }
            break;
#endif /* HAVE_CCMR2 */
          default:
            pwmerr("ERROR: No such channel: %u\n", channel);
            return -EINVAL;
        }
    }

  ccmr1 |= ocmode1;
#if defined(HAVE_CCMR2)
  ccmr2 |= ocmode2;
#endif

  /* Special configuration for HAVE_ADVTIM */

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      uint32_t bdtr;

      /* Get current register state */

      bdtr  = stm32pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);

      /* Update deadtime */

      bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);
      bdtr |= ATIM_BDTR_MOE;

      stm32pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, bdtr);
    }
#endif

  /* Save the modified register values */

  putreg32(ccmr1, priv->base + STM32_GTIM_CCMR1_OFFSET);
#if defined(HAVE_CCMR2)
  putreg32(ccmr2, priv->base + STM32_ATIM_CCMR2_OFFSET);
#endif

  /* Set the ARR Preload Bit */

  cr1 = stm32pwm_getreg(priv, STM32_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_ARPE;
  stm32pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Setup update interrupt.  If info->count is > 0, then we can be
   * assured that stm32pwm_start() has already verified: (1) that this is an
   * advanced timer, and that (2) the repetition count is within range.
   */

#ifdef CONFIG_PWM_PULSECOUNT
  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      stm32pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
      stm32pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      cr1 |= GTIM_CR1_CEN;
      stm32pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }
  else
#endif
    {
      /* Just enable the timer, leaving all interrupts disabled */

      cr1 |= GTIM_CR1_CEN;
      stm32pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);
    }

  stm32pwm_dumpregs(priv, "After starting");
  return OK;
}

#ifndef CONFIG_PWM_PULSECOUNT
/****************************************************************************
 * Name: stm32pwm_update_duty
 *
 * Description:
 *   Try to change only channel duty.
 *
 * Input Parameters:
 *   priv    - A reference to the lower half PWM driver state structure
 *   channel - Channel to be updated
 *   duty    - New duty.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static  int stm32pwm_update_duty(struct stm32_pwmtimer_s *priv,
                                 uint8_t channel, ub16_t duty)
{
  /* Register offset */

  int ccr_offset;

  /* Calculated values */

  uint32_t reload;
  uint32_t ccr;

  DEBUGASSERT(priv != NULL);

  pwminfo("TIM%u channel: %u duty: %08" PRIx32 "\n",
          priv->timid, channel, duty);

#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Get the reload values */

  reload = stm32pwm_getreg(priv, STM32_GTIM_ARR_OFFSET);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  pwminfo("ccr: %" PRIu32 "\n", ccr);

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

  stm32pwm_putreg(priv, ccr_offset, ccr);

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32pwm_interrupt
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
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   channel - Timer output channel
 *   mode    - PWM mode. See stm32_chanmode_e
 *
 * Returned Values:
 *   Zero on success; a negated errno value on failure
 ****************************************************************************/

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM))
static int stm32pwm_interrupt(struct stm32_pwmtimer_s *priv)
{
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = stm32pwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  stm32pwm_putreg(priv, STM32_ATIM_SR_OFFSET, regval & ~ATIM_SR_UIF);

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the mast output to stop the output as
       * quickly as possible.
       */

      regval  = stm32pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);
      regval &= ~ATIM_BDTR_MOE;
      stm32pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, regval);

      /* Disable first interrupts, stop and reset the timer */

      stm32pwm_stop((struct pwm_lowerhalf_s *)priv);

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
      priv->curr = stm32pwm_pulsecount(priv->count - priv->prev);
      stm32pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, priv->curr - 1);
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

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32F0L0G0_TIM1_PWM)
static int stm32pwm_tim1interrupt(int irq, void *context, void *arg)
{
  return stm32pwm_interrupt(&g_pwm1dev);
}
#endif

#if defined(CONFIG_PWM_PULSECOUNT) && defined(CONFIG_STM32F0L0G0_TIM8_PWM)
static int stm32pwm_tim8interrupt(int irq, void *context, void *arg)
{
  return stm32pwm_interrupt(&g_pwm8dev);
}
#endif

/****************************************************************************
 * Name: stm32pwm_pulsecount
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

#if defined(CONFIG_PWM_PULSECOUNT) && (defined(CONFIG_STM32F0L0G0_TIM1_PWM) || defined(CONFIG_STM32F0L0G0_TIM8_PWM))
static uint8_t stm32pwm_pulsecount(uint32_t count)
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
 * Name: stm32pwm_setapbclock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32pwm_setapbclock(struct stm32_pwmtimer_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
      case 1:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM1EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
      case 2:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM2EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
      case 3:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM3EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
      case 14:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM14EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
      case 15:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM15EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
      case 16:
        regaddr  = STM32_RCC_APB2ENR;
        en_bit   = RCC_APB2ENR_TIM16EN;
        break;
#endif
#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
      case 17:
        regaddr  = STM32_RCC_APB2ENR;
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
 * Name: stm32pwm_setup
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

static int stm32pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);
  stm32pwm_dumpregs(priv, "Initially");

  /* Enable APB1/2 clocking for timer. */

  stm32pwm_setapbclock(priv, true);

  /* Configure the PWM output pins, but do not start the timer yet */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          stm32_configgpio(pincfg);
        }

      /* Enable complementary channel if available */

      pincfg = priv->channels[i].npincfg;
      if (pincfg != 0)
        {
          pwminfo("npincfg: %08" PRIx32 "\n", pincfg);

          stm32_configgpio(pincfg);
        }

      pwm_dumpgpio(pincfg, "PWM setup");
    }

  return OK;
}

/****************************************************************************
 * Name: stm32pwm_shutdown
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

static int stm32pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg;
  int i;

  pwminfo("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  stm32pwm_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  stm32pwm_setapbclock(priv, false);

  /* Then put the GPIO pins back to the default state */

  for (i = 0; i < PWM_NCHANNELS; i++)
    {
      pincfg = priv->channels[i].pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32_configgpio(pincfg);
        }

      pincfg = priv->channels[i].npincfg;
      if (pincfg != 0)
        {
          pwminfo("npincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= GPIO_INPUT | GPIO_FLOAT;

          stm32_configgpio(pincfg);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32pwm_start
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
static int stm32pwm_start(struct pwm_lowerhalf_s *dev,
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

  return stm32pwm_timer(priv, info);
}
#else
static int stm32pwm_start(struct pwm_lowerhalf_s *dev,
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
              ret = stm32pwm_update_duty(priv, info->channels[i].channel,
                                         info->channels[i].duty);
            }
        }
#else
      ret = stm32pwm_update_duty(priv, priv->channels[0].channel,
                                 info->duty);
#endif
    }
  else
#endif
    {
      ret = stm32pwm_timer(priv, info);

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
 * Name: stm32pwm_stop
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

static int stm32pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
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

  stm32pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stm32pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Determine which timer to reset */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
      case 1:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM1RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
      case 2:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM2RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
      case 3:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM3RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM4_PWM
      case 4:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM4RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM5_PWM
      case 5:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM5RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM8_PWM
      case 8:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM8RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
      case 14:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM14RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
      case 15:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM15RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
      case 16:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM16RST;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
      case 17:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM17RST;
        break;
#endif

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where stm32pwm_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  pwminfo("regaddr: %08" PRIx32 " resetbit: %08" PRIx32 "\n",
          regaddr, resetbit);
  stm32pwm_dumpregs(priv, "After stop");
  return OK;
}

/****************************************************************************
 * Name: stm32pwm_ioctl
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

static int stm32pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
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
#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
      case 1:
        lower = &g_pwm1dev;

        /* Attach but disable the TIM1 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, stm32pwm_tim1interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
      case 2:
        lower = &g_pwm2dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
      case 3:
        lower = &g_pwm3dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM4_PWM
      case 4:
        lower = &g_pwm4dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM5_PWM
      case 5:
        lower = &g_pwm5dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM8_PWM
      case 8:
        lower = &g_pwm8dev;

        /* Attach but disable the TIM8 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, stm32pwm_tim8interrupt, NULL);
        up_disable_irq(lower->irq);
#endif
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
      case 14:
        lower = &g_pwm14dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
      case 15:
        lower = &g_pwm15dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
      case 16:
        lower = &g_pwm16dev;
        break;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
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

#endif /* CONFIG_STM32F0L0G0_TIMx_PWM */
