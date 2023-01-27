/****************************************************************************
 * arch/arm/src/stm32h7/stm32_qencoder.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/sensors/qencoder.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include "stm32_qencoder.h"
#include "stm32_rcc.h"

#ifdef CONFIG_SENSORS_QENCODER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timers *******************************************************************/

#undef HAVE_32BIT_TIMERS
#undef HAVE_16BIT_TIMERS

/* On the H7 series, TIM2 and TIM5 are 32-bit.  All of the rest are 16-bit */

/* If TIM2 or TIM5 are enabled, then we have 32-bit timers */

#if defined(CONFIG_STM32H7_TIM2_QE) || defined(CONFIG_STM32H7_TIM5_QE)
#  define HAVE_32BIT_TIMERS   1
#endif

/* If TIM1,3,4, or 8 are enabled, then we have 16-bit timers */

#if defined(CONFIG_STM32H7_TIM1_QE) || defined(CONFIG_STM32H7_TIM3_QE) || \
    defined(CONFIG_STM32H7_TIM4_QE) || defined(CONFIG_STM32H7_TIM8_QE)
#  define HAVE_16BIT_TIMERS   1
#endif

/* The width in bits of each timer */

#define TIM1_BITWIDTH         16
#define TIM2_BITWIDTH         32
#define TIM3_BITWIDTH         16
#define TIM4_BITWIDTH         16
#define TIM5_BITWIDTH         32
#define TIM8_BITWIDTH         16

/* Do we need to support mixed 16- and 32-bit timers */

#undef HAVE_MIXEDWIDTH_TIMERS
#if defined(HAVE_16BIT_TIMERS) && defined(HAVE_32BIT_TIMERS)
#  define HAVE_MIXEDWIDTH_TIMERS 1
#endif

/* Input filter *************************************************************/

#ifdef CONFIG_STM32H7_QENCODER_FILTER
#  if defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_1)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_NOFILT
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_CKINT)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_2)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FCKINT2
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_4)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FCKINT4
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FCKINT8
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS_2)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_6)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd26
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd28
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS_4)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_6)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd46
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd48
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS_8)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_6)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd86
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd88
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS_16)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_5)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd165
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_6)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd166
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd168
#    endif
#  elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_FDTS_32)
#    if defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_5)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd325
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_6)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd326
#    elif defined(CONFIG_STM32H7_QENCODER_SAMPLE_EVENT_8)
#      define STM32_QENCODER_ICF GTIM_CCMR_ICF_FDTSd328
#    endif
#  endif

#  ifndef STM32_QENCODER_ICF
#    warning "Invalid encoder filter combination, filter disabled"
#  endif
#endif

#ifndef STM32_QENCODER_ICF
#  define STM32_QENCODER_ICF GTIM_CCMR_ICF_NOFILT
#endif

#define STM32_GPIO_INPUT_FLOAT (GPIO_INPUT | GPIO_FLOAT)

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the quadrature
 * encoder
 */

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_DEBUG_SENSORS
#endif

#ifdef CONFIG_DEBUG_SENSORS
#  ifdef CONFIG_DEBUG_INFO
#    define qe_dumpgpio(p,m)    stm32_dumpgpio(p,m)
#  else
#    define qe_dumpgpio(p,m)
#  endif
#else
#  define qe_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Constant configuration structure that is retained in FLASH */

struct stm32_qeconfig_s
{
  uint8_t   timid;   /* Timer ID {1,2,3,4,5,8} */
  uint8_t   irq;     /* Timer update IRQ */
#ifdef HAVE_MIXEDWIDTH_TIMERS
  uint8_t   width;   /* Timer width (16- or 32-bits) */
#endif
#ifdef CONFIG_STM32_STM32F10XX
  uint16_t  ti1cfg;  /* TI1 input pin configuration (16-bit encoding) */
  uint16_t  ti2cfg;  /* TI2 input pin configuration (16-bit encoding) */
#else
  uint32_t  ti1cfg;  /* TI1 input pin configuration (20-bit encoding) */
  uint32_t  ti2cfg;  /* TI2 input pin configuration (20-bit encoding) */
#endif
  uintptr_t regaddr; /* RCC clock enable register address */
  uint32_t  enable;  /* RCC clock enable bit */
  uint32_t  base;    /* Register base address */
  uint32_t  psc;     /* Timer input clock prescaler */
};

/* Overall, RAM-based state structure */

struct stm32_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the lower-
   * half callback structure:
   */

  const struct qe_ops_s *ops;  /* Lower half callback structure */

  /* STM32 driver-specific fields: */

  const struct stm32_qeconfig_s *config; /* static onfiguration */

  bool             inuse;    /* True: The lower-half driver is in-use */

#ifdef HAVE_16BIT_TIMERS
  volatile int32_t position; /* The current position offset */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static uint16_t stm32_getreg16(struct stm32_lowerhalf_s *priv,
                               int offset);
static void stm32_putreg16(struct stm32_lowerhalf_s *priv, int offset,
                           uint16_t value);
static uint32_t stm32_getreg32(struct stm32_lowerhalf_s *priv,
                               int offset);
static void stm32_putreg32(struct stm32_lowerhalf_s *priv, int offset,
                           uint32_t value);

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void stm32_dumpregs(struct stm32_lowerhalf_s *priv,
                           const char *msg);
#else
#  define stm32_dumpregs(priv,msg)
#endif

static struct stm32_lowerhalf_s *stm32_tim2lower(int tim);

/* Interrupt handling */

#ifdef HAVE_16BIT_TIMERS
static int stm32_interrupt(int irq, void *context, void *arg);
#endif

/* Lower-half Quadrature Encoder Driver Methods */

static int stm32_setup(struct qe_lowerhalf_s *lower);
static int stm32_shutdown(struct qe_lowerhalf_s *lower);
static int stm32_position(struct qe_lowerhalf_s *lower,
                          int32_t *pos);
static int stm32_reset(struct qe_lowerhalf_s *lower);
static int stm32_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The lower half callback structure */

static const struct qe_ops_s g_qecallbacks =
{
  .setup     = stm32_setup,
  .shutdown  = stm32_shutdown,
  .position  = stm32_position,
  .setposmax = NULL,            /* not supported yet */
  .reset     = stm32_reset,
  .setindex  = NULL,            /* not supported yet */
  .ioctl     = stm32_ioctl,
};

/* Per-timer state structures */

#ifdef CONFIG_STM32H7_TIM1_QE
static const struct stm32_qeconfig_s g_tim1config =
{
  .timid    = 1,
  .irq      = STM32_IRQ_TIM1UP,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM1_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB2ENR,
  .enable   = RCC_APB2ENR_TIM1EN,
  .base     = STM32_TIM1_BASE,
  .psc      = CONFIG_STM32H7_TIM1_QEPSC,
  .ti1cfg   = GPIO_TIM1_CH1IN,
  .ti2cfg   = GPIO_TIM1_CH2IN,
};

static struct stm32_lowerhalf_s g_tim1lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim1config,
  .inuse    = false,
};

#endif

#ifdef CONFIG_STM32H7_TIM2_QE
static const struct stm32_qeconfig_s g_tim2config =
{
  .timid    = 2,
  .irq      = STM32_IRQ_TIM2,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM2_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB1LENR,
  .enable   = RCC_APB1LENR_TIM2EN,
  .base     = STM32_TIM2_BASE,
  .psc      = CONFIG_STM32H7_TIM2_QEPSC,
  .ti1cfg   = GPIO_TIM2_CH1IN,
  .ti2cfg   = GPIO_TIM2_CH2IN,
};

static struct stm32_lowerhalf_s g_tim2lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim2config,
  .inuse    = false,
};

#endif

#ifdef CONFIG_STM32H7_TIM3_QE
static const struct stm32_qeconfig_s g_tim3config =
{
  .timid    = 3,
  .irq      = STM32_IRQ_TIM3,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM3_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB1LENR,
  .enable   = RCC_APB1LENR_TIM3EN,
  .base     = STM32_TIM3_BASE,
  .psc      = CONFIG_STM32H7_TIM3_QEPSC,
  .ti1cfg   = GPIO_TIM3_CH1IN,
  .ti2cfg   = GPIO_TIM3_CH2IN,
};

static struct stm32_lowerhalf_s g_tim3lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim3config,
  .inuse    = false,
};

#endif

#ifdef CONFIG_STM32H7_TIM4_QE
static const struct stm32_qeconfig_s g_tim4config =
{
  .timid    = 4,
  .irq      = STM32_IRQ_TIM4,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM4_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB1LENR,
  .enable   = RCC_APB1LENR_TIM4EN,
  .base     = STM32_TIM4_BASE,
  .psc      = CONFIG_STM32H7_TIM4_QEPSC,
  .ti1cfg   = GPIO_TIM4_CH1IN,
  .ti2cfg   = GPIO_TIM4_CH2IN,
};

static struct stm32_lowerhalf_s g_tim4lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim4config,
  .inuse    = false,
};

#endif

#ifdef CONFIG_STM32H7_TIM5_QE
static const struct stm32_qeconfig_s g_tim5config =
{
  .timid    = 5,
  .irq      = STM32_IRQ_TIM5,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM5_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB1LENR,
  .enable   = RCC_APB1LENR_TIM5EN,
  .base     = STM32_TIM5_BASE,
  .psc      = CONFIG_STM32H7_TIM5_QEPSC,
  .ti1cfg   = GPIO_TIM5_CH1IN,
  .ti2cfg   = GPIO_TIM5_CH2IN,
};

static struct stm32_lowerhalf_s g_tim5lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim5config,
  .inuse    = false,
};

#endif

#ifdef CONFIG_STM32H7_TIM8_QE
static const struct stm32_qeconfig_s g_tim8config =
{
  .timid    = 8,
  .irq      = STM32_IRQ_TIM8UP,
#ifdef HAVE_MIXEDWIDTH_TIMERS
  .width    = TIM8_BITWIDTH,
#endif
  .regaddr  = STM32_RCC_APB2ENR,
  .enable   = RCC_APB2ENR_TIM8EN,
  .base     = STM32_TIM8_BASE,
  .psc      = CONFIG_STM32H7_TIM8_QEPSC,
  .ti1cfg   = GPIO_TIM8_CH1IN,
  .ti2cfg   = GPIO_TIM8_CH2IN,
};

static struct stm32_lowerhalf_s g_tim8lower =
{
  .ops      = &g_qecallbacks,
  .config   = &g_tim8config,
  .inuse    = false,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_getreg16
 *
 * Description:
 *   Read the value of a 16-bit timer register.
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint16_t stm32_getreg16(struct stm32_lowerhalf_s *priv, int offset)
{
  return getreg16(priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_putreg16
 *
 * Description:
 *   Write a value to a 16-bit timer register.
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_putreg16(struct stm32_lowerhalf_s *priv, int offset,
                           uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_getreg32
 *
 * Description:
 *   Read the value of a 32-bit timer register.  This applies for the
 *   STM32 H7 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers
 *   TIM2-5 (but works OK with the 16-bit TIM1,8 as well).
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t stm32_getreg32(struct stm32_lowerhalf_s *priv,
                               int offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_putreg32
 *
 * Description:
 *   Write a value to a 32-bit timer register.  This applies for the
 *   STM32 H7 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers
 *   TIM2-5 (but works OK with the 16-bit TIM1,8 as well).
 *
 * Input Parameters:
 *   priv - A reference to the lower half status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stm32_putreg32(struct stm32_lowerhalf_s *priv, int offset,
                           uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: stm32_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the QENCODER block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_SENSORS) && defined(CONFIG_DEBUG_INFO)
static void stm32_dumpregs(struct stm32_lowerhalf_s *priv,
                           const char *msg)
{
  sninfo("%s:\n", msg);
  sninfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CR2_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_SMCR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_DIER_OFFSET));
  sninfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
         stm32_getreg16(priv, STM32_GTIM_SR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_EGR_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCMR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCMR2_OFFSET));
  sninfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CNT_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_PSC_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_ARR_OFFSET));
  sninfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
         stm32_getreg16(priv, STM32_GTIM_CCR1_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR2_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR3_OFFSET),
         stm32_getreg16(priv, STM32_GTIM_CCR4_OFFSET));
#if defined(CONFIG_STM32H7_TIM1_QE) || defined(CONFIG_STM32H7_TIM8_QE)
  if (priv->config->timid == 1 || priv->config->timid == 8)
    {
      sninfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
             stm32_getreg16(priv, STM32_ATIM_RCR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_BDTR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_DCR_OFFSET),
             stm32_getreg16(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
#endif
    {
      sninfo("  DCR: %04x DMAR: %04x\n",
             stm32_getreg16(priv, STM32_GTIM_DCR_OFFSET),
             stm32_getreg16(priv, STM32_GTIM_DMAR_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: stm32_tim2lower
 *
 * Description:
 *   Map a timer number to a device structure
 *
 ****************************************************************************/

static struct stm32_lowerhalf_s *stm32_tim2lower(int tim)
{
  switch (tim)
    {
#ifdef CONFIG_STM32H7_TIM1_QE
    case 1:
      return &g_tim1lower;
#endif
#ifdef CONFIG_STM32H7_TIM2_QE
    case 2:
      return &g_tim2lower;
#endif
#ifdef CONFIG_STM32H7_TIM3_QE
    case 3:
      return &g_tim3lower;
#endif
#ifdef CONFIG_STM32H7_TIM4_QE
    case 4:
      return &g_tim4lower;
#endif
#ifdef CONFIG_STM32H7_TIM5_QE
    case 5:
      return &g_tim5lower;
#endif
#ifdef CONFIG_STM32H7_TIM8_QE
    case 8:
      return &g_tim8lower;
#endif
    default:
      return NULL;
    }
}

/****************************************************************************
 * Name: stm32_interrupt
 *
 * Description:
 *   Common timer interrupt handling.  NOTE: Only 16-bit timers require
 *   timer interrupts.
 *
 ****************************************************************************/

#ifdef HAVE_16BIT_TIMERS
static int stm32_interrupt(int irq, void *context, void *arg)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)arg;
  uint16_t regval;

  DEBUGASSERT(priv != NULL);

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = stm32_getreg16(priv, STM32_GTIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  stm32_putreg16(priv, STM32_GTIM_SR_OFFSET, regval & ~GTIM_SR_UIF);

  /* Check the direction bit in the CR1 register and add or subtract the
   * maximum value + 1, as appropriate.
   */

  regval = stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET);
  if ((regval & ATIM_CR1_DIR) != 0)
    {
      priv->position -= (int32_t)0x00010000;
    }
  else
    {
      priv->position += (int32_t)0x00010000;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: stm32_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   The initial position value should be zero. *
 *
 ****************************************************************************/

static int stm32_setup(struct qe_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  uint16_t dier;
  uint32_t smcr;
  uint32_t ccmr1;
  uint16_t ccer;
  uint16_t cr1;
#ifdef HAVE_16BIT_TIMERS
  uint16_t regval;
  int ret;
#endif

  /* Enable clocking to the timer */

  modifyreg32(priv->config->regaddr, 0, priv->config->enable);

  /* Timer base configuration */

  cr1 = stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET);

  /* Clear the direction bit (0=count up) and select the Counter Mode
   * (0=Edge aligned) (Timers 2-5 and 1-8 only)
   */

  cr1 &= ~(GTIM_CR1_DIR | GTIM_CR1_CMS_MASK);
  stm32_putreg16(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Set the Autoreload value */

#if defined(HAVE_MIXEDWIDTH_TIMERS)
  if (priv->config->width == 32)
    {
      stm32_putreg32(priv, STM32_GTIM_ARR_OFFSET, 0xffffffff);
    }
  else
    {
      stm32_putreg16(priv, STM32_GTIM_ARR_OFFSET, 0xffff);
    }
#elif defined(HAVE_32BIT_TIMERS)
  stm32_putreg32(priv, STM32_GTIM_ARR_OFFSET, 0xffffffff);
#else
  stm32_putreg16(priv, STM32_GTIM_ARR_OFFSET, 0xffff);
#endif

  /* Set the timer prescaler value.
   *
   * If we are doing precise shaft positioning, each qe pulse is important.
   * So the STM32 has direct config control on the pulse count prescaler.
   * This input clock just limits the incoming pulse rate, which should be
   * lower than the peripheral clock due to resynchronization, but it is the
   * responsibility of the system designer to decide the correct prescaler
   * value, because it has a direct influence on the encoder resolution.
   */

  stm32_putreg16(priv, STM32_GTIM_PSC_OFFSET, (uint16_t)priv->config->psc);

#if defined(CONFIG_STM32H7_TIM1_QE) || defined(CONFIG_STM32H7_TIM8_QE)
  if (priv->config->timid == 1 || priv->config->timid == 8)
    {
      /* Clear the Repetition Counter value */

      stm32_putreg16(priv, STM32_ATIM_RCR_OFFSET, 0);
    }
#endif

  /* Generate an update event to reload the Prescaler
   * and the repetition counter (only for TIM1 and TIM8) value immediately
   */

  stm32_putreg16(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);

  /* GPIO pin configuration */

  stm32_configgpio(priv->config->ti1cfg);
  stm32_configgpio(priv->config->ti2cfg);

  /* Set the encoder Mode 3 */

  smcr  = stm32_getreg32(priv, STM32_GTIM_SMCR_OFFSET);
  smcr &= ~GTIM_SMCR_SMS_MASK;
  smcr |= GTIM_SMCR_ENCMD3;
  stm32_putreg32(priv, STM32_GTIM_SMCR_OFFSET, smcr);

  /* TI1 Channel Configuration */

  /* Disable the Channel 1: Reset the CC1E Bit */

  ccer  = stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~GTIM_CCER_CC1E;
  stm32_putreg16(priv, STM32_GTIM_CCER_OFFSET, ccer);

  ccmr1 = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
  ccer  = stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET);

  /* Select the Input IC1=TI1 and set the filter fSAMPLING=fDTS/4, N=6 */

  ccmr1 &= ~(GTIM_CCMR1_CC1S_MASK | GTIM_CCMR1_IC1F_MASK);
  ccmr1 |= GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC1S_SHIFT;
  ccmr1 |= STM32_QENCODER_ICF << GTIM_CCMR1_IC1F_SHIFT;

  /* Select the Polarity=rising and set the CC1E Bit */

  ccer &= ~(GTIM_CCER_CC1P | GTIM_CCER_CC1NP);
  ccer |= GTIM_CCER_CC1E;

  /* Write to TIM CCMR1 and CCER registers */

  stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);
  stm32_putreg16(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Set the Input Capture Prescaler value: Capture performed each time an
   * edge is detected on the capture input.
   */

  ccmr1  = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
  ccmr1 &= ~GTIM_CCMR1_IC1PSC_MASK;
  ccmr1 |= (GTIM_CCMR_ICPSC_NOPSC << GTIM_CCMR1_IC1PSC_SHIFT);
  stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);

  /* TI2 Channel Configuration */

  /* Disable the Channel 2: Reset the CC2E Bit */

  ccer  = stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~GTIM_CCER_CC2E;
  stm32_putreg16(priv, STM32_GTIM_CCER_OFFSET, ccer);

  ccmr1 = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
  ccer  = stm32_getreg16(priv, STM32_GTIM_CCER_OFFSET);

  /* Select the Input IC2=TI2 and set the filter fSAMPLING=fDTS/4, N=6 */

  ccmr1 &= ~(GTIM_CCMR1_CC2S_MASK | GTIM_CCMR1_IC2F_MASK);
  ccmr1 |= GTIM_CCMR_CCS_CCIN1 << GTIM_CCMR1_CC2S_SHIFT;
  ccmr1 |= STM32_QENCODER_ICF << GTIM_CCMR1_IC2F_SHIFT;

  /* Select the Polarity=rising and set the CC2E Bit */

  ccer &= ~(GTIM_CCER_CC2P | GTIM_CCER_CC2NP);
  ccer |= GTIM_CCER_CC2E;

  /* Write to TIM CCMR1 and CCER registers */

  stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);
  stm32_putreg16(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Set the Input Capture Prescaler value: Capture performed each time an
   * edge is detected on the capture input.
   */

  ccmr1  = stm32_getreg32(priv, STM32_GTIM_CCMR1_OFFSET);
  ccmr1 &= ~GTIM_CCMR1_IC2PSC_MASK;
  ccmr1 |= (GTIM_CCMR_ICPSC_NOPSC << GTIM_CCMR1_IC2PSC_SHIFT);
  stm32_putreg32(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);

  /* Disable the update interrupt */

  dier  = stm32_getreg16(priv, STM32_GTIM_DIER_OFFSET);
  dier &= ~GTIM_DIER_UIE;
  stm32_putreg16(priv, STM32_GTIM_DIER_OFFSET, dier);

  /* There is no need for interrupts with 32-bit timers */

#ifdef HAVE_16BIT_TIMERS
#ifdef HAVE_MIXEDWIDTH_TIMERS
  if (priv->config->width != 32)
#endif
    {
      /* Attach the interrupt handler */

      ret = irq_attach(priv->config->irq, stm32_interrupt, priv);
      if (ret < 0)
        {
          stm32_shutdown(lower);
          return ret;
        }

      /* Enable the update/global interrupt at the NVIC */

      up_enable_irq(priv->config->irq);
    }
#endif

  /* Reset the Update Disable Bit */

  cr1 = stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET);
  cr1 &= ~GTIM_CR1_UDIS;
  stm32_putreg16(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Reset the URS Bit */

  cr1 &= ~GTIM_CR1_URS;
  stm32_putreg16(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* There is no need for interrupts with 32-bit timers */

#ifdef HAVE_16BIT_TIMERS
#ifdef HAVE_MIXEDWIDTH_TIMERS
  if (priv->config->width != 32)
#endif
    {
      /* Clear any pending update interrupts */

      regval = stm32_getreg16(priv, STM32_GTIM_SR_OFFSET);
      stm32_putreg16(priv, STM32_GTIM_SR_OFFSET, regval & ~GTIM_SR_UIF);

      /* Then enable the update interrupt */

      dier = stm32_getreg16(priv, STM32_GTIM_DIER_OFFSET);
      dier |= GTIM_DIER_UIE;
      stm32_putreg16(priv, STM32_GTIM_DIER_OFFSET, dier);
    }
#endif

  /* Enable the TIM Counter */

  cr1 = stm32_getreg16(priv, STM32_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_CEN;
  stm32_putreg16(priv, STM32_GTIM_CR1_OFFSET, cr1);

  stm32_dumpregs(priv, "After setup");

  return OK;
}

/****************************************************************************
 * Name: stm32_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half
 *   driver should stop data collection, free any resources, disable timer
 *   hardware, and put the system into the lowest possible power usage
 *   state
 *
 ****************************************************************************/

static int stm32_shutdown(struct qe_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t regaddr;
  uint32_t regval;
  uint32_t resetbit;
  uint32_t pincfg;

  /* Disable the update/global interrupt at the NVIC */

  flags = enter_critical_section();
  up_disable_irq(priv->config->irq);

  /* Detach the interrupt handler */

  irq_detach(priv->config->irq);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  /* Disable further interrupts and stop the timer */

  stm32_putreg16(priv, STM32_GTIM_DIER_OFFSET, 0);
  stm32_putreg16(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Determine which timer to reset */

  switch (priv->config->timid)
    {
#ifdef CONFIG_STM32H7_TIM1_QE
      case 1:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM1RST;
        break;
#endif
#ifdef CONFIG_STM32H7_TIM2_QE
      case 2:
        regaddr  = STM32_RCC_APB1LRSTR;
        resetbit = RCC_APB1LRSTR_TIM2RST;
        break;
#endif
#ifdef CONFIG_STM32H7_TIM3_QE
      case 3:
        regaddr  = STM32_RCC_APB1LRSTR;
        resetbit = RCC_APB1LRSTR_TIM3RST;
        break;
#endif
#ifdef CONFIG_STM32H7_TIM4_QE
      case 4:
        regaddr  = STM32_RCC_APB1LRSTR;
        resetbit = RCC_APB1LRSTR_TIM4RST;
        break;
#endif
#ifdef CONFIG_STM32H7_TIM5_QE
      case 5:
        regaddr  = STM32_RCC_APB1LRSTR;
        resetbit = RCC_APB1LRSTR_TIM5RST;
        break;
#endif
#ifdef CONFIG_STM32H7_TIM8_QE
      case 8:
        regaddr  = STM32_RCC_APB2RSTR;
        resetbit = RCC_APB2RSTR_TIM8RST;
        break;
#endif
      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  /* Reset the timer - stopping the output and putting the timer back
   * into a state where stm32_start() can be called.
   */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  leave_critical_section(flags);

  sninfo("regaddr: %08" PRIx32 " resetbit: %08" PRIx32 "\n",
          regaddr, resetbit);
  stm32_dumpregs(priv, "After stop");

  /* Disable clocking to the timer */

  modifyreg32(priv->config->regaddr, priv->config->enable, 0);

  /* Put the TI1 GPIO pin back to its default state */

  pincfg  = priv->config->ti1cfg & (GPIO_PORT_MASK | GPIO_PIN_MASK);
  pincfg |= STM32_GPIO_INPUT_FLOAT;

  stm32_configgpio(pincfg);

  /* Put the TI2 GPIO pin back to its default state */

  pincfg  = priv->config->ti2cfg & (GPIO_PORT_MASK | GPIO_PIN_MASK);
  pincfg |= STM32_GPIO_INPUT_FLOAT;

  stm32_configgpio(pincfg);
  return OK;
}

/****************************************************************************
 * Name: stm32_position
 *
 * Description:
 *   Return the current position measurement.
 *
 ****************************************************************************/

static int stm32_position(struct qe_lowerhalf_s *lower, int32_t *pos)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
#ifdef HAVE_16BIT_TIMERS
  int32_t position;
  int32_t verify;
  uint32_t count;

  DEBUGASSERT(lower && priv->inuse);

  /* Loop until we are certain that no interrupt occurred between samples */

  do
    {
      /* Don't let another task preempt us until we get the measurement.
       * The timer interrupt may still be processed
       */

      sched_lock();
      position = priv->position;
      count    = stm32_getreg32(priv, STM32_GTIM_CNT_OFFSET);
      verify   = priv->position;
      sched_unlock();
    }
  while (position != verify);

  /* Return the position measurement */

  *pos = position + (int32_t)count;
#else
  /* Return the counter value */

  *pos = (int32_t)stm32_getreg32(priv, STM32_GTIM_CNT_OFFSET);
#endif
  return OK;
}

/****************************************************************************
 * Name: stm32_reset
 *
 * Description:
 *   Reset the position measurement to zero.
 *
 ****************************************************************************/

static int stm32_reset(struct qe_lowerhalf_s *lower)
{
  struct stm32_lowerhalf_s *priv = (struct stm32_lowerhalf_s *)lower;
#ifdef HAVE_16BIT_TIMERS
  irqstate_t flags;

  sninfo("Resetting position to zero\n");
  DEBUGASSERT(lower && priv->inuse);

  /* Reset the timer and the counter.  Interrupts are disabled to make this
   * atomic (if possible)
   */

  flags = enter_critical_section();
  stm32_putreg32(priv, STM32_GTIM_CNT_OFFSET, 0);
  priv->position = 0;
  leave_critical_section(flags);
#else
  sninfo("Resetting position to zero\n");
  DEBUGASSERT(lower && priv->inuse);

  /* Reset the counter to zero */

  stm32_putreg32(priv, STM32_GTIM_CNT_OFFSET, 0);
#endif
  return OK;
}

/****************************************************************************
 * Name: stm32_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 ****************************************************************************/

static int stm32_ioctl(struct qe_lowerhalf_s *lower, int cmd,
                       unsigned long arg)
{
  /* No ioctl commands supported */

  /* TODO add an IOCTL to control the encoder pulse count prescaler */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_qeinitialize
 *
 * Description:
 *   Initialize a quadrature encoder interface.  This function must be
 *   called from board-specific logic.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/qe0"
 *   tim     - The timer number to used.  'tim' must be an element of
 *             {1,2,3,4,5,8}
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int stm32_qeinitialize(const char *devpath, int tim)
{
  struct stm32_lowerhalf_s *priv;
  int ret;

  /* Find the pre-allocated timer state structure corresponding to this
   * timer
   */

  priv = stm32_tim2lower(tim);
  if (!priv)
    {
      snerr("ERROR: TIM%d support not configured\n", tim);
      return -ENXIO;
    }

  /* Make sure that it is available */

  if (priv->inuse)
    {
      snerr("ERROR: TIM%d is in-use\n", tim);
      return -EBUSY;
    }

  /* Register the upper-half driver */

  ret = qe_register(devpath, (struct qe_lowerhalf_s *)priv);
  if (ret < 0)
    {
      snerr("ERROR: qe_register failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the timer is in the shutdown state */

  stm32_shutdown((struct qe_lowerhalf_s *)priv);

  /* The driver is now in-use */

  priv->inuse = true;
  return OK;
}

#endif /* CONFIG_SENSORS_QENCODER */
