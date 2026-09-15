/****************************************************************************
 * arch/arm/src/n32h7/n32_tim.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "n32_rcc.h"
#include "n32_gpio.h"
#include "n32_tim.h"
#include "hardware/n32h7_rcc.h"
#include "hardware/n32h76x_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer devices may be used for different purposes.  Such special purposes
 * include:
 *
 * - To generate modulated outputs for such things as motor control.  If
 *   CONFIG_N32H7_TIMn_PWM is defined then the timer is intended to be used
 *   for pulsed output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_N32H7_TIMn_ADC is
 *   defined then timer "n" is intended to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_N32H7_TIMn_DAC is defined
 *   then timer "n" is intended to be used for that purpose.
 *
 * - To use a Quadrature Encoder.  If CONFIG_N32H7_TIMn_QE is defined then
 *   timer "n" is intended to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

/* ATIM1 */
#if defined(CONFIG_N32H7_ATIM1_PWM) || defined(CONFIG_N32H7_ATIM1_ADC) || \
    defined(CONFIG_N32H7_ATIM1_DAC) || defined(CONFIG_N32H7_ATIM1_QE)
#  undef CONFIG_N32H7_ATIM1
#endif

/* ATIM2 */
#if defined(CONFIG_N32H7_ATIM2_PWM) || defined(CONFIG_N32H7_ATIM2_ADC) || \
    defined(CONFIG_N32H7_ATIM2_DAC) || defined(CONFIG_N32H7_ATIM2_QE)
#  undef CONFIG_N32H7_ATIM2
#endif

/* ATIM3 */
#if defined(CONFIG_N32H7_ATIM3_PWM) || defined(CONFIG_N32H7_ATIM3_ADC) || \
    defined(CONFIG_N32H7_ATIM3_DAC) || defined(CONFIG_N32H7_ATIM3_QE)
#  undef CONFIG_N32H7_ATIM3
#endif

/* ATIM4 */
#if defined(CONFIG_N32H7_ATIM4_PWM) || defined(CONFIG_N32H7_ATIM4_ADC) || \
    defined(CONFIG_N32H7_ATIM4_DAC) || defined(CONFIG_N32H7_ATIM4_QE)
#  undef CONFIG_N32H7_ATIM4
#endif

/* GTIMA1 */
#if defined(CONFIG_N32H7_GTIMA1_PWM) || defined(CONFIG_N32H7_GTIMA1_ADC) || \
    defined(CONFIG_N32H7_GTIMA1_DAC) || defined(CONFIG_N32H7_GTIMA1_QE)
#  undef CONFIG_N32H7_GTIMA1
#endif

/* GTIMA2 */
#if defined(CONFIG_N32H7_GTIMA2_PWM) || defined(CONFIG_N32H7_GTIMA2_ADC) || \
    defined(CONFIG_N32H7_GTIMA2_DAC) || defined(CONFIG_N32H7_GTIMA2_QE)
#  undef CONFIG_N32H7_GTIMA2
#endif

/* GTIMA3 */
#if defined(CONFIG_N32H7_GTIMA3_PWM) || defined(CONFIG_N32H7_GTIMA3_ADC) || \
    defined(CONFIG_N32H7_GTIMA3_DAC) || defined(CONFIG_N32H7_GTIMA3_QE)
#  undef CONFIG_N32H7_GTIMA3
#endif

/* GTIMA4 */
#if defined(CONFIG_N32H7_GTIMA4_PWM) || defined(CONFIG_N32H7_GTIMA4_ADC) || \
    defined(CONFIG_N32H7_GTIMA4_DAC) || defined(CONFIG_N32H7_GTIMA4_QE)
#  undef CONFIG_N32H7_GTIMA4
#endif

/* GTIMA5 */
#if defined(CONFIG_N32H7_GTIMA5_PWM) || defined(CONFIG_N32H7_GTIMA5_ADC) || \
    defined(CONFIG_N32H7_GTIMA5_DAC) || defined(CONFIG_N32H7_GTIMA5_QE)
#  undef CONFIG_N32H7_GTIMA5
#endif

/* GTIMA6 */
#if defined(CONFIG_N32H7_GTIMA6_PWM) || defined(CONFIG_N32H7_GTIMA6_ADC) || \
    defined(CONFIG_N32H7_GTIMA6_DAC) || defined(CONFIG_N32H7_GTIMA6_QE)
#  undef CONFIG_N32H7_GTIMA6
#endif

/* GTIMA7 */
#if defined(CONFIG_N32H7_GTIMA7_PWM) || defined(CONFIG_N32H7_GTIMA7_ADC) || \
    defined(CONFIG_N32H7_GTIMA7_DAC) || defined(CONFIG_N32H7_GTIMA7_QE)
#  undef CONFIG_N32H7_GTIMA7
#endif

/* GTIMB1 */
#if defined(CONFIG_N32H7_GTIMB1_PWM) || defined(CONFIG_N32H7_GTIMB1_ADC) || \
    defined(CONFIG_N32H7_GTIMB1_DAC) || defined(CONFIG_N32H7_GTIMB1_QE)
#  undef CONFIG_N32H7_GTIMB1
#endif

/* GTIMB2 */
#if defined(CONFIG_N32H7_GTIMB2_PWM) || defined(CONFIG_N32H7_GTIMB2_ADC) || \
    defined(CONFIG_N32H7_GTIMB2_DAC) || defined(CONFIG_N32H7_GTIMB2_QE)
#  undef CONFIG_N32H7_GTIMB2
#endif

/* GTIMB3 */
#if defined(CONFIG_N32H7_GTIMB3_PWM) || defined(CONFIG_N32H7_GTIMB3_ADC) || \
    defined(CONFIG_N32H7_GTIMB3_DAC) || defined(CONFIG_N32H7_GTIMB3_QE)
#  undef CONFIG_N32H7_GTIMB3
#endif

/* BTIM1 */
#if defined(CONFIG_N32H7_BTIM1_PWM) || defined(CONFIG_N32H7_BTIM1_ADC) || \
    defined(CONFIG_N32H7_BTIM1_DAC) || defined(CONFIG_N32H7_BTIM1_QE)
#  undef CONFIG_N32H7_BTIM1
#endif

/* BTIM2 */
#if defined(CONFIG_N32H7_BTIM2_PWM) || defined(CONFIG_N32H7_BTIM2_ADC) || \
    defined(CONFIG_N32H7_BTIM2_DAC) || defined(CONFIG_N32H7_BTIM2_QE)
#  undef CONFIG_N32H7_BTIM2
#endif

/* BTIM3 */
#if defined(CONFIG_N32H7_BTIM3_PWM) || defined(CONFIG_N32H7_BTIM3_ADC) || \
    defined(CONFIG_N32H7_BTIM3_DAC) || defined(CONFIG_N32H7_BTIM3_QE)
#  undef CONFIG_N32H7_BTIM3
#endif

/* BTIM4 */
#if defined(CONFIG_N32H7_BTIM4_PWM) || defined(CONFIG_N32H7_BTIM4_ADC) || \
    defined(CONFIG_N32H7_BTIM4_DAC) || defined(CONFIG_N32H7_BTIM4_QE)
#  undef CONFIG_N32H7_BTIM4
#endif

/* GPIO output pin configuration support */

#if defined(CONFIG_N32H7_ATIM1)
#  if defined(GPIO_ATIM1_CH1OUT) || defined(GPIO_ATIM1_CH2OUT) || \
      defined(GPIO_ATIM1_CH3OUT) || defined(GPIO_ATIM1_CH4OUT)
#    define HAVE_ATIM1_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_ATIM2)
#  if defined(GPIO_ATIM2_CH1OUT) || defined(GPIO_ATIM2_CH2OUT) || \
      defined(GPIO_ATIM2_CH3OUT) || defined(GPIO_ATIM2_CH4OUT)
#    define HAVE_ATIM2_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_ATIM3)
#  if defined(GPIO_ATIM3_CH1OUT) || defined(GPIO_ATIM3_CH2OUT) || \
      defined(GPIO_ATIM3_CH3OUT) || defined(GPIO_ATIM3_CH4OUT)
#    define HAVE_ATIM3_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_ATIM4)
#  if defined(GPIO_ATIM4_CH1OUT) || defined(GPIO_ATIM4_CH2OUT) || \
      defined(GPIO_ATIM4_CH3OUT) || defined(GPIO_ATIM4_CH4OUT)
#    define HAVE_ATIM4_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA1)
#  if defined(GPIO_GTIMA1_CH1OUT) || defined(GPIO_GTIMA1_CH2OUT) || \
      defined(GPIO_GTIMA1_CH3OUT) || defined(GPIO_GTIMA1_CH4OUT)
#    define HAVE_GTIMA1_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA2)
#  if defined(GPIO_GTIMA2_CH1OUT) || defined(GPIO_GTIMA2_CH2OUT) || \
      defined(GPIO_GTIMA2_CH3OUT) || defined(GPIO_GTIMA2_CH4OUT)
#    define HAVE_GTIMA2_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA3)
#  if defined(GPIO_GTIMA3_CH1OUT) || defined(GPIO_GTIMA3_CH2OUT) || \
      defined(GPIO_GTIMA3_CH3OUT) || defined(GPIO_GTIMA3_CH4OUT)
#    define HAVE_GTIMA3_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA4)
#  if defined(GPIO_GTIMA4_CH1OUT) || defined(GPIO_GTIMA4_CH2OUT) || \
      defined(GPIO_GTIMA4_CH3OUT) || defined(GPIO_GTIMA4_CH4OUT)
#    define HAVE_GTIMA4_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA5)
#  if defined(GPIO_GTIMA5_CH1OUT) || defined(GPIO_GTIMA5_CH2OUT) || \
      defined(GPIO_GTIMA5_CH3OUT) || defined(GPIO_GTIMA5_CH4OUT)
#    define HAVE_GTIMA5_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA6)
#  if defined(GPIO_GTIMA6_CH1OUT) || defined(GPIO_GTIMA6_CH2OUT) || \
      defined(GPIO_GTIMA6_CH3OUT) || defined(GPIO_GTIMA6_CH4OUT)
#    define HAVE_GTIMA6_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMA7)
#  if defined(GPIO_GTIMA7_CH1OUT) || defined(GPIO_GTIMA7_CH2OUT) || \
      defined(GPIO_GTIMA7_CH3OUT) || defined(GPIO_GTIMA7_CH4OUT)
#    define HAVE_GTIMA7_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMB1)
#  if defined(GPIO_GTIMB1_CH1POUT) || defined(GPIO_GTIMB1_CH2OUT) || \
      defined(GPIO_GTIMB1_CH3OUT) || defined(GPIO_GTIMB1_CH4OUT)
#    define HAVE_GTIMB1_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMB2)
#  if defined(GPIO_GTIMB2_CH1POUT) || defined(GPIO_GTIMB2_CH2OUT) || \
      defined(GPIO_GTIMB2_CH3OUT) || defined(GPIO_GTIMB2_CH4OUT)
#    define HAVE_GTIMB2_GPIOCONFIG 1
#  endif
#endif

#if defined(CONFIG_N32H7_GTIMB3)
#  if defined(GPIO_GTIMB3_CH1POUT) || defined(GPIO_GTIMB3_CH2OUT) || \
      defined(GPIO_GTIMB3_CH3OUT) || defined(GPIO_GTIMB3_CH4OUT)
#    define HAVE_GTIMB3_GPIOCONFIG 1
#  endif
#endif

/* This module compiles only if there are enabled timers not intended for
 * some other purpose.
 */

#if defined(CONFIG_N32H7_ATIM1)  || defined(CONFIG_N32H7_ATIM2)  || \
    defined(CONFIG_N32H7_ATIM3)  || defined(CONFIG_N32H7_ATIM4)  || \
    defined(CONFIG_N32H7_GTIMA1) || defined(CONFIG_N32H7_GTIMA2) || \
    defined(CONFIG_N32H7_GTIMA3) || defined(CONFIG_N32H7_GTIMA4) || \
    defined(CONFIG_N32H7_GTIMA5) || defined(CONFIG_N32H7_GTIMA6) || \
    defined(CONFIG_N32H7_GTIMA7) || defined(CONFIG_N32H7_GTIMB1) || \
    defined(CONFIG_N32H7_GTIMB2) || defined(CONFIG_N32H7_GTIMB3) || \
    defined(CONFIG_N32H7_BTIM1)  || defined(CONFIG_N32H7_BTIM2)  || \
    defined(CONFIG_N32H7_BTIM3)  || defined(CONFIG_N32H7_BTIM4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Timer type enumeration */

enum n32_tim_type_e
{
  TIM_TYPE_ATIM,    /* Advanced timer (ATIM1-4) */
  TIM_TYPE_GTIMA,   /* General-purpose timer A (GTIMA1-7) */
  TIM_TYPE_GTIMB,   /* General-purpose timer B (GTIMB1-3) */
  TIM_TYPE_BTIM     /* Basic timer (BTIM1-4) */
};

/* Timer device private structure */

struct n32_tim_priv_s
{
  const struct n32_tim_ops_s *ops;
  enum n32_tim_type_e  type;
  uint32_t             base;   /* Timer base address */
  uint32_t             freqin; /* Input clock frequency (for setclock) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Timer helpers */

static void n32_tim_reload_counter(struct n32_tim_dev_s *dev);
static void n32_tim_enable(struct n32_tim_dev_s *dev);
static void n32_tim_disable(struct n32_tim_dev_s *dev);
static void n32_tim_reset(struct n32_tim_dev_s *dev);

/* Timer methods */

static int      n32_tim_setmode(struct n32_tim_dev_s *dev,
                                n32_tim_mode_t mode);
static int      n32_tim_setclock(struct n32_tim_dev_s *dev,
                                 uint32_t freq);
static void     n32_tim_setperiod(struct n32_tim_dev_s *dev,
                                  uint32_t period);
static uint32_t n32_tim_getperiod(struct n32_tim_dev_s *dev);
static uint32_t n32_tim_getcounter(struct n32_tim_dev_s *dev);
static void     n32_tim_setcounter(struct n32_tim_dev_s *dev,
                                   uint32_t count);
static int      n32_tim_getwidth(struct n32_tim_dev_s *dev);
static int      n32_tim_setchannel(struct n32_tim_dev_s *dev,
                                   uint8_t channel,
                                   n32_tim_channel_t mode);
static int      n32_tim_setcompare(struct n32_tim_dev_s *dev,
                                   uint8_t channel, uint32_t compare);
static int      n32_tim_getcapture(struct n32_tim_dev_s *dev,
                                   uint8_t channel);
static int      n32_tim_setisr(struct n32_tim_dev_s *dev,
                               xcpt_t handler, void *arg, int source);
static void     n32_tim_enableint(struct n32_tim_dev_s *dev, int source);
static void     n32_tim_disableint(struct n32_tim_dev_s *dev, int source);
static void     n32_tim_ackint(struct n32_tim_dev_s *dev, int source);
static int      n32_tim_checkint(struct n32_tim_dev_s *dev, int source);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct n32_tim_ops_s n32_tim_ops =
{
  .enable         = &n32_tim_enable,
  .disable        = &n32_tim_disable,
  .setmode        = &n32_tim_setmode,
  .setclock       = &n32_tim_setclock,
  .setperiod      = &n32_tim_setperiod,
  .getperiod      = &n32_tim_getperiod,
  .getcounter     = &n32_tim_getcounter,
  .setcounter     = &n32_tim_setcounter,
  .getwidth       = &n32_tim_getwidth,
  .setchannel     = &n32_tim_setchannel,
  .setcompare     = &n32_tim_setcompare,
  .getcapture     = &n32_tim_getcapture,
  .setisr         = &n32_tim_setisr,
  .enableint      = &n32_tim_enableint,
  .disableint     = &n32_tim_disableint,
  .ackint         = &n32_tim_ackint,
  .checkint       = &n32_tim_checkint,
};

/* Timer private structures - ATIM1-4 */

#ifdef CONFIG_N32H7_ATIM1
static struct n32_tim_priv_s n32_tim1_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_ATIM,
  .base       = N32_ATIMER1_BASE,
  .freqin     = N32_AHB_FREQUENCY,  /* 300 MHz */
};
#endif

#ifdef CONFIG_N32H7_ATIM2
static struct n32_tim_priv_s n32_tim2_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_ATIM,
  .base       = N32_ATIMER2_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_ATIM3
static struct n32_tim_priv_s n32_tim3_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_ATIM,
  .base       = N32_ATIMER3_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_ATIM4
static struct n32_tim_priv_s n32_tim4_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_ATIM,
  .base       = N32_ATIMER4_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

/* Timer private structures - GTIMA1-7 */

#ifdef CONFIG_N32H7_GTIMA1
static struct n32_tim_priv_s n32_gtima1_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA1_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA2
static struct n32_tim_priv_s n32_gtima2_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA2_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA3
static struct n32_tim_priv_s n32_gtima3_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA3_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA4
static struct n32_tim_priv_s n32_gtima4_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA4_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA5
static struct n32_tim_priv_s n32_gtima5_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA5_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA6
static struct n32_tim_priv_s n32_gtima6_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA6_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMA7
static struct n32_tim_priv_s n32_gtima7_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMA,
  .base       = N32_GTIMERA7_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

/* Timer private structures - GTIMB1-3 */

#ifdef CONFIG_N32H7_GTIMB1
static struct n32_tim_priv_s n32_gtimb1_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMB,
  .base       = N32_GTIMERB1_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMB2
static struct n32_tim_priv_s n32_gtimb2_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMB,
  .base       = N32_GTIMERB2_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_GTIMB3
static struct n32_tim_priv_s n32_gtimb3_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_GTIMB,
  .base       = N32_GTIMERB3_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

/* Timer private structures - BTIM1-4 */

#ifdef CONFIG_N32H7_BTIM1
static struct n32_tim_priv_s n32_btim1_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_BTIM,
  .base       = N32_BTIMER1_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_BTIM2
static struct n32_tim_priv_s n32_btim2_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_BTIM,
  .base       = N32_BTIMER2_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_BTIM3
static struct n32_tim_priv_s n32_btim3_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_BTIM,
  .base       = N32_BTIMER3_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

#ifdef CONFIG_N32H7_BTIM4
static struct n32_tim_priv_s n32_btim4_priv =
{
  .ops        = &n32_tim_ops,
  .type       = TIM_TYPE_BTIM,
  .base       = N32_BTIMER4_BASE,
  .freqin     = N32_AHB_FREQUENCY,
};
#endif

/* Unified register offsets (same for all timer types) */

#define N32_TIM_CTRL1_OFFSET      0x0000  /* Control register 1 */
#define N32_TIM_CTRL2_OFFSET      0x0004  /* Control register 2 */
#define N32_TIM_STS_OFFSET        0x0008  /* Status register */
#define N32_TIM_ETGEN_OFFSET      0x000c  /* Event generation register */
#define N32_TIM_SMCTRL_OFFSET     0x0010  /* Slave mode control (not for BTIM) */
#define N32_TIM_DINTEN_OFFSET     0x0014  /* DMA/Interrupt enable register */
#define N32_TIM_CCMOD1_OFFSET     0x0018  /* Capture/compare mode 1 (not for BTIM) */
#define N32_TIM_CCMOD2_OFFSET     0x001c  /* Capture/compare mode 2 (not for BTIM) */
#define N32_TIM_CCMOD3_OFFSET     0x0020  /* Capture/compare mode 3 (ATIM/GTIMB only) */
#define N32_TIM_CCEN_OFFSET       0x0024  /* Capture/compare enable (not for BTIM) */
#define N32_TIM_CCDAT1_OFFSET     0x0028  /* Capture/compare reg 1 (not for BTIM) */
#define N32_TIM_CCDAT2_OFFSET     0x002c  /* Capture/compare reg 2 (not for BTIM) */
#define N32_TIM_CCDAT3_OFFSET     0x0030  /* Capture/compare reg 3 (not for BTIM) */
#define N32_TIM_CCDAT4_OFFSET     0x0034  /* Capture/compare reg 4 (not for BTIM) */
#define N32_TIM_PSC_OFFSET        0x0040  /* Prescaler */
#define N32_TIM_AR_OFFSET         0x0044  /* Auto-reload register */
#define N32_TIM_CNT_OFFSET        0x0048  /* Counter */
#define N32_TIM_BKDT_OFFSET       0x0050  /* Break/dead-time (ATIM/GTIMB only) */

/* Helper macros for register access */

#define n32_getreg16(dev, off)    getreg16(((struct n32_tim_priv_s *)dev)->base + (off))
#define n32_putreg16(dev, off, v) putreg16((v), ((struct n32_tim_priv_s *)dev)->base + (off))
#define n32_modreg16(dev, off, c, s) modifyreg16(((struct n32_tim_priv_s *)dev)->base + (off), (c), (s))

#define n32_getreg32(dev, off)    getreg32(((struct n32_tim_priv_s *)dev)->base + (off))
#define n32_putreg32(dev, off, v) putreg32((v), ((struct n32_tim_priv_s *)dev)->base + (off))
#define n32_modreg32(dev, off, c, s) modifyreg32(((struct n32_tim_priv_s *)dev)->base + (off), (c), (s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_tim_is_btim
 *
 * Description:
 *   Check if the timer is a basic timer (BTIM).
 *
 ****************************************************************************/

static inline bool n32_tim_is_btim(struct n32_tim_dev_s *dev)
{
  return ((struct n32_tim_priv_s *)dev)->type == TIM_TYPE_BTIM;
}

/****************************************************************************
 * Name: n32_tim_is_atim
 *
 * Description:
 *   Check if the timer is an advanced timer (ATIM).
 *
 ****************************************************************************/

static inline bool n32_tim_is_atim(struct n32_tim_dev_s *dev)
{
  return ((struct n32_tim_priv_s *)dev)->type == TIM_TYPE_ATIM;
}

/****************************************************************************
 * Name: n32_tim_is_gtimb
 *
 * Description:
 *   Check if the timer is a GTIMB timer.
 *
 ****************************************************************************/

static inline bool n32_tim_is_gtimb(struct n32_tim_dev_s *dev)
{
  return ((struct n32_tim_priv_s *)dev)->type == TIM_TYPE_GTIMB;
}

/****************************************************************************
 * Name: n32_tim_get_width
 *
 * Description:
 *   Determine the counter width (16 or 32 bits) for the timer.
 *
 ****************************************************************************/

static inline int n32_tim_get_width(struct n32_tim_dev_s *dev)
{
  /* Only BTIM timers are 32-bit */

  if (n32_tim_is_btim(dev))
    {
      return 32;
    }

  return 16;
}

/****************************************************************************
 * Name: n32_tim_reload_counter
 *
 * Description:
 *   Generate an update event to reload the counter.
 *
 ****************************************************************************/

static void n32_tim_reload_counter(struct n32_tim_dev_s *dev)
{
  n32_putreg16(dev, N32_TIM_ETGEN_OFFSET, N32_ATIM_ETGEN_UDGN);
}

/****************************************************************************
 * Name: n32_tim_enable
 *
 * Description:
 *   Enable the timer counter.
 *
 ****************************************************************************/

static void n32_tim_enable(struct n32_tim_dev_s *dev)
{
  uint32_t regval;

  regval = n32_getreg32(dev, N32_TIM_CTRL1_OFFSET);
  regval |= N32_ATIM_CTRL1_CNTEN;

  /* For ATIM, also set MOE (Main Output Enable) */

  if (n32_tim_is_atim(dev))
    {
      n32_modreg32(dev, N32_TIM_BKDT_OFFSET, 0, N32_ATIM_BKDT_MOEN);
    }

  n32_tim_reload_counter(dev);
  n32_putreg32(dev, N32_TIM_CTRL1_OFFSET, regval);
}

/****************************************************************************
 * Name: n32_tim_disable
 *
 * Description:
 *   Disable the timer counter.
 *
 ****************************************************************************/

static void n32_tim_disable(struct n32_tim_dev_s *dev)
{
  uint32_t regval;

  regval = n32_getreg32(dev, N32_TIM_CTRL1_OFFSET);
  regval &= ~N32_ATIM_CTRL1_CNTEN;
  n32_putreg32(dev, N32_TIM_CTRL1_OFFSET, regval);

  /* For ATIM, clear MOE as well */

  if (n32_tim_is_atim(dev))
    {
      n32_modreg32(dev, N32_TIM_BKDT_OFFSET, N32_ATIM_BKDT_MOEN, 0);
    }
}

/****************************************************************************
 * Name: n32_tim_reset
 *
 * Description:
 *   Reset the timer to its default state (all relevant registers cleared).
 *
 ****************************************************************************/

static void n32_tim_reset(struct n32_tim_dev_s *dev)
{
  bool is_btim = n32_tim_is_btim(dev);
  bool is_atim = n32_tim_is_atim(dev);

  /* Disable counter */

  n32_tim_disable(dev);

  /* Reset CTRL1: clear all bits except ARPE */

  n32_putreg32(dev, N32_TIM_CTRL1_OFFSET, N32_ATIM_CTRL1_ARPEN);

  /* Reset CTRL2 */

  n32_putreg32(dev, N32_TIM_CTRL2_OFFSET, 0);

  /* Clear status register */

  n32_putreg32(dev, N32_TIM_STS_OFFSET, 0);

  /* Clear event generation register */

  n32_putreg32(dev, N32_TIM_ETGEN_OFFSET, 0);

  /* Reset DIER (all interrupts/DMA disabled) */

  n32_putreg32(dev, N32_TIM_DINTEN_OFFSET, 0);

  if (!is_btim)
    {
      /* Reset CCMOD1 and CCMOD2 */

      n32_putreg32(dev, N32_TIM_CCMOD1_OFFSET, 0);
      n32_putreg32(dev, N32_TIM_CCMOD2_OFFSET, 0);

      /* Reset CCEN */

      n32_putreg32(dev, N32_TIM_CCEN_OFFSET, 0);

      /* Reset CCR1-4 */

      n32_putreg32(dev, N32_TIM_CCDAT1_OFFSET, 0);
      n32_putreg32(dev, N32_TIM_CCDAT2_OFFSET, 0);
      n32_putreg32(dev, N32_TIM_CCDAT3_OFFSET, 0);
      n32_putreg32(dev, N32_TIM_CCDAT4_OFFSET, 0);

      /* Reset SMCTRL */

      n32_putreg32(dev, N32_TIM_SMCTRL_OFFSET, 0);

      /* CCMOD3 exists for ATIM and GTIMB */

      if (is_atim || n32_tim_is_gtimb(dev))
        {
          n32_putreg32(dev, N32_TIM_CCMOD3_OFFSET, 0);
        }

      /* BKDT exists for ATIM and GTIMB */

      if (is_atim || n32_tim_is_gtimb(dev))
        {
          n32_putreg32(dev, N32_TIM_BKDT_OFFSET, 0);
        }
    }

  /* Reset prescaler */

  n32_putreg32(dev, N32_TIM_PSC_OFFSET, 0);

  /* Reset ARR to max value (16-bit or 32-bit) */

  if (is_btim)
    {
      n32_putreg32(dev, N32_TIM_AR_OFFSET, 0xffffffff);
    }
  else
    {
      n32_putreg32(dev, N32_TIM_AR_OFFSET, 0xffff);
    }

  /* Reset counter */

  n32_putreg32(dev, N32_TIM_CNT_OFFSET, 0);

  /* For BTIM, PSC and ARR are 32-bit registers */

  if (is_btim)
    {
      /* BTIM PSC is 16-bit, ARR is 32-bit, CNT is 32-bit */
    }
}

/****************************************************************************
 * Name: n32_tim_setmode
 *
 * Description:
 *   Set the timer counting mode.
 *
 ****************************************************************************/

static int n32_tim_setmode(struct n32_tim_dev_s *dev, n32_tim_mode_t mode)
{
  uint32_t val = N32_ATIM_CTRL1_CNTEN | N32_ATIM_CTRL1_ARPEN;

  DEBUGASSERT(dev != NULL);

  /* BTIM does not support mode setting */

  if (n32_tim_is_btim(dev) && (mode & ~N32_TIM_MODE_UP))
    {
      return -EINVAL;
    }

  switch (mode & N32_TIM_MODE_MASK)
    {
      case N32_TIM_MODE_DISABLED:
        val &= ~N32_ATIM_CTRL1_CNTEN;
        break;

      case N32_TIM_MODE_DOWN:
        val |= N32_ATIM_CTRL1_DIR;

        /* Fall through */

      case N32_TIM_MODE_UP:
        break;

      case N32_TIM_MODE_UPDOWN:
        val |= N32_ATIM_CTRL1_CENTER1;
        break;

      case N32_TIM_MODE_PULSE:
        val |= N32_ATIM_CTRL1_ONEPM;
        break;

      default:
        return -EINVAL;
    }

  n32_tim_reload_counter(dev);
  n32_tim_ackint(dev, N32_ATIM_STS_UDITF);
  n32_putreg32(dev, N32_TIM_CTRL1_OFFSET, val);

  /* For ATIM, ensure MOE is set when counter is enabled */

  if (n32_tim_is_atim(dev))
    {
      n32_modreg32(dev, N32_TIM_BKDT_OFFSET, 0, N32_ATIM_BKDT_MOEN);
    }

  return OK;
}

/****************************************************************************
 * Name: n32_tim_setclock
 *
 * Description:
 *   Set the timer clock frequency by configuring the prescaler.
 *
 *   Timer core clock is AHB frequency (300 MHz) for all timers.
 *
 ****************************************************************************/

static int n32_tim_setclock(struct n32_tim_dev_s *dev, uint32_t freq)
{
  struct n32_tim_priv_s *priv = (struct n32_tim_priv_s *)dev;
  uint32_t freqin;
  uint32_t prescaler;
  uint32_t psc_mask;

  DEBUGASSERT(dev != NULL);

  /* If frequency is 0, disable the timer */

  if (freq == 0)
    {
      n32_tim_disable(dev);
      return 0;
    }

  /* All timers use AHB clock as their core clock */

  freqin = priv->freqin;

  /* Calculate prescaler value */

  prescaler = freqin / freq;

  /* Decrement by 1 (pre-scaler value is PSC+1) */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow (16-bit prescaler for all timer types) */

  psc_mask = 0xffff;
  if (prescaler > psc_mask)
    {
      prescaler = psc_mask;
    }

  n32_putreg32(dev, N32_TIM_PSC_OFFSET, prescaler);

  return prescaler;
}

/****************************************************************************
 * Name: n32_tim_setperiod
 *
 * Description:
 *   Set the auto-reload value (period).
 *
 ****************************************************************************/

static void n32_tim_setperiod(struct n32_tim_dev_s *dev, uint32_t period)
{
  bool is_btim = n32_tim_is_btim(dev);
  uint32_t max_period;

  DEBUGASSERT(dev != NULL);

  max_period = is_btim ? 0xffffffff : 0xffff;

  if (period > max_period)
    {
      period = max_period;
    }

  n32_putreg32(dev, N32_TIM_AR_OFFSET, period);
}

/****************************************************************************
 * Name: n32_tim_getperiod
 *
 * Description:
 *   Get the current period value.
 *
 ****************************************************************************/

static uint32_t n32_tim_getperiod(struct n32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  return n32_getreg32(dev, N32_TIM_AR_OFFSET);
}

/****************************************************************************
 * Name: n32_tim_getcounter
 *
 * Description:
 *   Read the current counter value.
 *
 ****************************************************************************/

static uint32_t n32_tim_getcounter(struct n32_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  if (n32_tim_is_btim(dev))
    {
      return n32_getreg32(dev, N32_TIM_CNT_OFFSET);
    }
  else
    {
      return n32_getreg16(dev, N32_TIM_CNT_OFFSET);
    }
}

/****************************************************************************
 * Name: n32_tim_setcounter
 *
 * Description:
 *   Set the counter value.
 *
 ****************************************************************************/

static void n32_tim_setcounter(struct n32_tim_dev_s *dev, uint32_t count)
{
  DEBUGASSERT(dev != NULL);

  if (n32_tim_is_btim(dev))
    {
      n32_putreg32(dev, N32_TIM_CNT_OFFSET, count);
    }
  else
    {
      n32_putreg16(dev, N32_TIM_CNT_OFFSET, (uint16_t)count);
    }
}

/****************************************************************************
 * Name: n32_tim_getwidth
 *
 * Description:
 *   Get the counter width in bits.
 *
 ****************************************************************************/

static int n32_tim_getwidth(struct n32_tim_dev_s *dev)
{
  return n32_tim_get_width(dev);
}

/****************************************************************************
 * Name: n32_tim_gpioconfig
 *
 * Description:
 *   Configure or unconfigure a GPIO pin for timer output.
 *
 ****************************************************************************/

#if defined(HAVE_ATIM1_GPIOCONFIG) || defined(HAVE_ATIM2_GPIOCONFIG) || \
    defined(HAVE_ATIM3_GPIOCONFIG) || defined(HAVE_ATIM4_GPIOCONFIG) || \
    defined(HAVE_GTIMA1_GPIOCONFIG) || defined(HAVE_GTIMA2_GPIOCONFIG) || \
    defined(HAVE_GTIMA3_GPIOCONFIG) || defined(HAVE_GTIMA4_GPIOCONFIG) || \
    defined(HAVE_GTIMA5_GPIOCONFIG) || defined(HAVE_GTIMA6_GPIOCONFIG) || \
    defined(HAVE_GTIMA7_GPIOCONFIG) || defined(HAVE_GTIMB1_GPIOCONFIG) || \
    defined(HAVE_GTIMB2_GPIOCONFIG) || defined(HAVE_GTIMB3_GPIOCONFIG)

static void n32_tim_gpioconfig(uint32_t cfg, n32_tim_channel_t mode)
{
  if (mode & N32_TIM_CH_MODE_MASK)
    {
      n32_configgpio(cfg);
    }
  else
    {
      n32_unconfiggpio(cfg);
    }
}
#endif

/****************************************************************************
 * Name: n32_tim_setchannel
 *
 * Description:
 *   Configure a timer channel for output or input.
 *
 ****************************************************************************/

static int n32_tim_setchannel(struct n32_tim_dev_s *dev,
                              uint8_t channel, n32_tim_channel_t mode)
{
  struct n32_tim_priv_s *priv = (struct n32_tim_priv_s *)dev;
  uint32_t ccmr_orig;
  uint32_t ccmr_val;
  uint32_t ccmr_mask;
  uint32_t ccmr_offset;
  uint32_t ccer_val;
  uint32_t ccer_shift;
  uint32_t ccmr_mode_shift;
  uint32_t oc_pe_bit;
  uint32_t ccen_bit;

  DEBUGASSERT(dev != NULL);

  /* BTIM does not support channels */

  if (n32_tim_is_btim(dev))
    {
      return -EINVAL;
    }

  /* Channel numbering: 1..4 (or more for ATIM/GTIMB) */

  if (channel < 1 || channel > 4)
    {
      /* For now, only support channels 1-4 */

      return -EINVAL;
    }

  ccer_val = n32_getreg32(dev, N32_TIM_CCEN_OFFSET);
  ccer_shift = (uint32_t)(channel - 1) * 4;
  ccen_bit = N32_ATIM_CCEN_CC1EN << ccer_shift;

  /* Determine CCMOD register and shift */

  if (channel <= 2)
    {
      ccmr_offset = N32_TIM_CCMOD1_OFFSET;
      ccmr_mode_shift = (channel == 1) ?  0 : (N32_ATIM_CCMOD1_OC2MD_SHIFT -
                        N32_ATIM_CCMOD1_OC1MD_SHIFT);
      oc_pe_bit = (channel == 1) ? N32_ATIM_CCMOD1_OC1PEN :
                                   N32_ATIM_CCMOD1_OC2PEN;
    }
  else
    {
      ccmr_offset = N32_TIM_CCMOD2_OFFSET;
      ccmr_mode_shift = (channel == 3) ?  0 : (N32_ATIM_CCMOD2_OC4MD_SHIFT -
                        N32_ATIM_CCMOD2_OC3MD_SHIFT);
      oc_pe_bit = (channel == 3) ? N32_ATIM_CCMOD2_OC3PEN :
                                   N32_ATIM_CCMOD2_OC4PEN;
    }

  /* Clear channel enable bit */

  ccer_val &= ~ccen_bit;
  ccer_val &= ~(N32_ATIM_CCEN_CC1P << ccer_shift);

  /* Default: channel disabled */

  ccmr_val = 0;
  ccmr_mask = 0xff << ccmr_mode_shift;

  /* Decode channel mode */

  switch (mode & N32_TIM_CH_MODE_MASK)
    {
      case N32_TIM_CH_DISABLED:
        break;

      case N32_TIM_CH_OUTTOGGLE:
        ccmr_val = (N32_ATIM_CCMOD1_OCMODE_TOGGLE << ccmr_mode_shift) |
                   oc_pe_bit;
        ccer_val |= ccen_bit;
        break;

      case N32_TIM_CH_OUTPWM:
        ccmr_val = (N32_ATIM_CCMOD1_OCMODE_PWM1 << ccmr_mode_shift) |
                   oc_pe_bit;
        ccer_val |= ccen_bit;
        break;

      default:
        return -EINVAL;
    }

  /* Set polarity (negative if requested) */

  if (mode & N32_TIM_CH_POLARITY_NEG)
    {
      ccer_val |= (N32_ATIM_CCEN_CC1P << ccer_shift);
    }

  /* Write CCMOD */

  ccmr_orig = n32_getreg32(dev, ccmr_offset);
  ccmr_orig &= ~ccmr_mask;
  ccmr_orig |= ccmr_val;
  n32_putreg32(dev, ccmr_offset, ccmr_orig);

  /* Write CCEN */

  n32_putreg32(dev, N32_TIM_CCEN_OFFSET, ccer_val);

  /* Configure GPIO */

  switch (priv->base)
    {
#ifdef CONFIG_N32H7_ATIM1
      case N32_ATIMER1_BASE:
        switch (channel)
          {
#  if defined(GPIO_ATIM1_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_ATIM1_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM1_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_ATIM1_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM1_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_ATIM1_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM1_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_ATIM1_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM2
      case N32_ATIMER2_BASE:
        switch (channel)
          {
#  if defined(GPIO_ATIM2_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_ATIM2_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM2_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_ATIM2_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM2_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_ATIM2_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM2_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_ATIM2_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM3
      case N32_ATIMER3_BASE:
        switch (channel)
          {
#  if defined(GPIO_ATIM3_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_ATIM3_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM3_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_ATIM3_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM3_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_ATIM3_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM3_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_ATIM3_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM4
      case N32_ATIMER4_BASE:
        switch (channel)
          {
#  if defined(GPIO_ATIM4_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_ATIM4_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM4_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_ATIM4_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM4_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_ATIM4_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_ATIM4_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_ATIM4_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA1
      case N32_GTIMERA1_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA1_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA1_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA1_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA1_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA1_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA1_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA1_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA1_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA2
      case N32_GTIMERA2_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA2_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA2_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA2_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA2_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA2_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA2_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA2_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA2_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA3
      case N32_GTIMERA3_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA3_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA3_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA3_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA3_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA3_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA3_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA3_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA3_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA4
      case N32_GTIMERA4_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA4_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA4_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA4_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA4_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA4_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA4_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA4_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA4_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA5
      case N32_GTIMERA5_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA5_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA5_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA5_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA5_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA5_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA5_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA5_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA5_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA6
      case N32_GTIMERA6_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA6_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA6_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA6_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA6_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA6_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA6_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA6_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA6_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA7
      case N32_GTIMERA7_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMA7_CH1OUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMA7_CH1OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA7_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMA7_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA7_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMA7_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMA7_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMA7_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB1
      case N32_GTIMERB1_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMB1_CH1POUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMB1_CH1POUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB1_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMB1_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB1_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMB1_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB1_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMB1_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB2
      case N32_GTIMERB2_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMB2_CH1POUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMB2_CH1POUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB2_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMB2_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB2_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMB2_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB2_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMB2_CH4OUT, mode);
              break;
#  endif
            default:
              return -EINVAL;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB3
      case N32_GTIMERB3_BASE:
        switch (channel)
          {
#  if defined(GPIO_GTIMB3_CH1POUT)
            case 1:
              n32_tim_gpioconfig(GPIO_GTIMB3_CH1POUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB3_CH2OUT)
            case 2:
              n32_tim_gpioconfig(GPIO_GTIMB3_CH2OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB3_CH3OUT)
            case 3:
              n32_tim_gpioconfig(GPIO_GTIMB3_CH3OUT, mode);
              break;
#  endif
#  if defined(GPIO_GTIMB3_CH4OUT)
            case 4:
              n32_tim_gpioconfig(GPIO_GTIMB3_CH4OUT, mode);
              break;
#  endif
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
 * Name: n32_tim_setcompare
 *
 * Description:
 *   Set the compare value for a channel.
 *
 ****************************************************************************/

static int n32_tim_setcompare(struct n32_tim_dev_s *dev,
                              uint8_t channel, uint32_t compare)
{
  uint32_t max_val;

  DEBUGASSERT(dev != NULL);

  /* BTIM does not support channels */

  if (n32_tim_is_btim(dev))
    {
      return -EINVAL;
    }

  max_val = n32_tim_get_width(dev) > 16 ? 0xffffffff : 0xffff;
  if (compare > max_val)
    {
      compare = max_val;
    }

  switch (channel)
    {
      case 1:
        n32_putreg32(dev, N32_TIM_CCDAT1_OFFSET, compare);
        break;
      case 2:
        n32_putreg32(dev, N32_TIM_CCDAT2_OFFSET, compare);
        break;
      case 3:
        n32_putreg32(dev, N32_TIM_CCDAT3_OFFSET, compare);
        break;
      case 4:
        n32_putreg32(dev, N32_TIM_CCDAT4_OFFSET, compare);
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: n32_tim_getcapture
 *
 * Description:
 *   Get the captured value for a channel (input capture mode).
 *
 ****************************************************************************/

static int n32_tim_getcapture(struct n32_tim_dev_s *dev, uint8_t channel)
{
  DEBUGASSERT(dev != NULL);

  /* BTIM does not support capture */

  if (n32_tim_is_btim(dev))
    {
      return -EINVAL;
    }

  switch (channel)
    {
      case 1:
        return n32_getreg32(dev, N32_TIM_CCDAT1_OFFSET);
      case 2:
        return n32_getreg32(dev, N32_TIM_CCDAT2_OFFSET);
      case 3:
        return n32_getreg32(dev, N32_TIM_CCDAT3_OFFSET);
      case 4:
        return n32_getreg32(dev, N32_TIM_CCDAT4_OFFSET);
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: n32_tim_setisr
 *
 * Description:
 *   Attach an interrupt handler to the timer based on the source.
 *
 *   Supported sources (see N32_TIM_ISR_* definitions):
 *     N32_TIM_ISR_UPDATE  - Update event (overflow/underflow)
 *     N32_TIM_ISR_CC      - Capture/Compare event (CC1-CC4)
 *     N32_TIM_ISR_TRG_COM - Trigger or COM event
 *     N32_TIM_ISR_BRK     - Break event (advanced timers only)
 *
 * Input Parameters:
 *   dev    - Timer device handle
 *   handler - Interrupt handler (NULL to detach)
 *   arg    - Argument passed to handler
 *   source - Bitmask of desired interrupt sources
 *            (only one bit should be set)
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

static int n32_tim_setisr(struct n32_tim_dev_s *dev,
                          xcpt_t handler, void *arg, int source)
{
  struct n32_tim_priv_s *priv = (struct n32_tim_priv_s *)dev;
  int irqno = -1;
  bool valid_source = false;

  DEBUGASSERT(dev != NULL);

  /* Determine which IRQ number corresponds to the requested source
   * based on the timer type and hardware capabilities.
   */

  switch (priv->base)
    {
#ifdef CONFIG_N32H7_ATIM1
      case N32_ATIMER1_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_ATIM1_UP;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_CC)
          {
            irqno = N32_IRQ_ATIM1_CC;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_TRG_COM)
          {
            irqno = N32_IRQ_ATIM1_TRG_COM;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_BRK)
          {
            irqno = N32_IRQ_ATIM1_BRK;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM2
      case N32_ATIMER2_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_ATIM2_UP;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_CC)
          {
            irqno = N32_IRQ_ATIM2_CC;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_TRG_COM)
          {
            irqno = N32_IRQ_ATIM2_TRG_COM;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_BRK)
          {
            irqno = N32_IRQ_ATIM2_BRK;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM3
      case N32_ATIMER3_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_ATIM3_UP;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_CC)
          {
            irqno = N32_IRQ_ATIM3_CC;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_TRG_COM)
          {
            irqno = N32_IRQ_ATIM3_TRG_COM;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_BRK)
          {
            irqno = N32_IRQ_ATIM3_BRK;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_ATIM4
      case N32_ATIMER4_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_ATIM4_UP;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_CC)
          {
            irqno = N32_IRQ_ATIM4_CC;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_TRG_COM)
          {
            irqno = N32_IRQ_ATIM4_TRG_COM;
            valid_source = true;
          }
        else if (source & N32_TIM_ISR_BRK)
          {
            irqno = N32_IRQ_ATIM4_BRK;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA1
      case N32_GTIMERA1_BASE:

        /* GTIMA timers have only one combined interrupt vector */

        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA1;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA2
      case N32_GTIMERA2_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA2;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA3
      case N32_GTIMERA3_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GIIMA3;  /* Note: typo in original */
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA4
      case N32_GTIMERA4_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA4;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA5
      case N32_GTIMERA5_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA5;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA6
      case N32_GTIMERA6_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA6;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMA7
      case N32_GTIMERA7_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMA7;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB1
      case N32_GTIMERB1_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMB1;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB2
      case N32_GTIMERB2_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMB2;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_GTIMB3
      case N32_GTIMERB3_BASE:
        if (source & (N32_TIM_ISR_UPDATE | N32_TIM_ISR_CC |
                      N32_TIM_ISR_TRG_COM | N32_TIM_ISR_BRK))
          {
            irqno = N32_IRQ_GTIMB3;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_BTIM1
      case N32_BTIMER1_BASE:

        /* BTIM timers only support update interrupt */

        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_BTIM1;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_BTIM2
      case N32_BTIMER2_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_BTIM2;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_BTIM3
      case N32_BTIMER3_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_BTIM3;
            valid_source = true;
          }
        break;
#endif

#ifdef CONFIG_N32H7_BTIM4
      case N32_BTIMER4_BASE:
        if (source & N32_TIM_ISR_UPDATE)
          {
            irqno = N32_IRQ_BTIM4;
            valid_source = true;
          }
        break;
#endif

      default:
        break;
    }

  if (!valid_source || irqno < 0)
    {
      return -EINVAL;
    }

  /* If handler is NULL, detach and disable IRQ */

  if (handler == NULL)
    {
      up_disable_irq(irqno);
      irq_detach(irqno);
      return OK;
    }

  /* Attach handler and enable IRQ */

  irq_attach(irqno, handler, arg);
  up_enable_irq(irqno);

#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(irqno, NVIC_SYSH_HIGH_PRIORITY);
#endif

  return OK;
}

/****************************************************************************
 * Name: n32_tim_enableint
 *
 * Description:
 *   Enable the specified interrupt source.
 *
 ****************************************************************************/

static void n32_tim_enableint(struct n32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  n32_modreg32(dev, N32_TIM_DINTEN_OFFSET, 0, source);
}

/****************************************************************************
 * Name: n32_tim_disableint
 *
 * Description:
 *   Disable the specified interrupt source.
 *
 ****************************************************************************/

static void n32_tim_disableint(struct n32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  n32_modreg32(dev, N32_TIM_DINTEN_OFFSET, source, 0);
}

/****************************************************************************
 * Name: n32_tim_checkint
 *
 * Description:
 *   Check if the specified interrupt source is pending.
 *
 ****************************************************************************/

static int n32_tim_checkint(struct n32_tim_dev_s *dev, int source)
{
  uint32_t regval;

  DEBUGASSERT(dev != NULL);
  regval = n32_getreg32(dev, N32_TIM_STS_OFFSET);
  return (regval & source) ? 1 : 0;
}

/****************************************************************************
 * Name: n32_tim_ackint
 *
 * Description:
 *   Acknowledge the specified interrupt source.
 *
 ****************************************************************************/

static void n32_tim_ackint(struct n32_tim_dev_s *dev, int source)
{
  DEBUGASSERT(dev != NULL);
  n32_putreg32(dev, N32_TIM_STS_OFFSET, ~source);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_tim_init
 *
 * Description:
 *   Initialize a timer device.
 *
 * Input Parameters:
 *   timer - Timer number (1-4 for ATIM, 1-7 for GTIMA, etc.)
 *           Determined by CONFIG_N32H7_TIMx definitions.
 *
 * Returned Value:
 *   Pointer to timer device structure, or NULL on failure.
 *
 ****************************************************************************/

struct n32_tim_dev_s *n32_tim_init(int timer)
{
  struct n32_tim_dev_s *dev = NULL;

  /* Select the timer private structure and enable its clock */

  switch (timer)
    {
#ifdef CONFIG_N32H7_ATIM1
      case 1:
        dev = (struct n32_tim_dev_s *)&n32_tim1_priv;
        modifyreg32(N32_RCC_APB2EN1, 0, RCC_APB2EN1_M7ATIM1EN);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM2
      case 2:
        dev = (struct n32_tim_dev_s *)&n32_tim2_priv;
        modifyreg32(N32_RCC_APB2EN1, 0, RCC_APB2EN1_M7ATIM2EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA1
      case 3:
        dev = (struct n32_tim_dev_s *)&n32_gtima1_priv;
        modifyreg32(N32_RCC_APB2EN1, 0, RCC_APB2EN1_M7GTIMA1EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA2
      case 4:
        dev = (struct n32_tim_dev_s *)&n32_gtima2_priv;
        modifyreg32(N32_RCC_APB2EN1, 0, RCC_APB2EN1_M7GTIMA2EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA3
      case 5:
        dev = (struct n32_tim_dev_s *)&n32_gtima3_priv;
        modifyreg32(N32_RCC_APB2EN1, 0, RCC_APB2EN1_M7GTIMA3EN);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM3
      case 6:
        dev = (struct n32_tim_dev_s *)&n32_tim3_priv;
        modifyreg32(N32_RCC_APB5EN1, 0, RCC_APB5EN1_M7ATIM3EN);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM4
      case 7:
        dev = (struct n32_tim_dev_s *)&n32_tim4_priv;
        modifyreg32(N32_RCC_APB5EN1, 0, RCC_APB5EN1_M7ATIM4EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA4
      case 8:
        dev = (struct n32_tim_dev_s *)&n32_gtima4_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7GTIMA4EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA5
      case 9:
        dev = (struct n32_tim_dev_s *)&n32_gtima5_priv;
        modifyreg32(N32_RCC_APB1EN2, 0, RCC_APB1EN2_M7GTIMA5EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA6
      case 10:
        dev = (struct n32_tim_dev_s *)&n32_gtima6_priv;
        modifyreg32(N32_RCC_APB1EN2, 0, RCC_APB1EN2_M7GTIMA6EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA7
      case 11:
        dev = (struct n32_tim_dev_s *)&n32_gtima7_priv;
        modifyreg32(N32_RCC_APB1EN2, 0, RCC_APB1EN2_M7GTIMA7EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB1
      case 12:
        dev = (struct n32_tim_dev_s *)&n32_gtimb1_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7GTIMB1EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB2
      case 13:
        dev = (struct n32_tim_dev_s *)&n32_gtimb2_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7GTIMB2EN);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB3
      case 14:
        dev = (struct n32_tim_dev_s *)&n32_gtimb3_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7GTIMB3EN);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM1
      case 15:
        dev = (struct n32_tim_dev_s *)&n32_btim1_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7BTIM1EN);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM2
      case 16:
        dev = (struct n32_tim_dev_s *)&n32_btim2_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7BTIM2EN);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM3
      case 17:
        dev = (struct n32_tim_dev_s *)&n32_btim3_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7BTIM3EN);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM4
      case 18:
        dev = (struct n32_tim_dev_s *)&n32_btim4_priv;
        modifyreg32(N32_RCC_APB1EN1, 0, RCC_APB1EN1_M7BTIM4EN);
        break;
#endif
      default:
        return NULL;
    }

  /* Reset the timer to a known state */

  n32_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: n32_tim_deinit
 *
 * Description:
 *   De-initialize a timer device.
 *
 * Input Parameters:
 *   dev - Timer device structure.
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

int n32_tim_deinit(struct n32_tim_dev_s *dev)
{
  struct n32_tim_priv_s *priv = (struct n32_tim_priv_s *)dev;
  uint32_t base;

  DEBUGASSERT(dev != NULL);

  base = priv->base;

  /* Disable the timer */

  n32_tim_disable(dev);

  /* Reset to default state */

  n32_tim_reset(dev);

  /* Disable the clock */

  switch (base)
    {
#ifdef CONFIG_N32H7_ATIM1
      case N32_ATIMER1_BASE:
        modifyreg32(N32_RCC_APB2EN1, RCC_APB2EN1_M7ATIM1EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM2
      case N32_ATIMER2_BASE:
        modifyreg32(N32_RCC_APB2EN1, RCC_APB2EN1_M7ATIM2EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM3
      case N32_ATIMER3_BASE:
        modifyreg32(N32_RCC_APB5EN1, RCC_APB5EN1_M7ATIM3EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_ATIM4
      case N32_ATIMER4_BASE:
        modifyreg32(N32_RCC_APB5EN1, RCC_APB5EN1_M7ATIM4EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA1
      case N32_GTIMERA1_BASE:
        modifyreg32(N32_RCC_APB2EN1, RCC_APB2EN1_M7GTIMA1EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA2
      case N32_GTIMERA2_BASE:
        modifyreg32(N32_RCC_APB2EN1, RCC_APB2EN1_M7GTIMA2EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA3
      case N32_GTIMERA3_BASE:
        modifyreg32(N32_RCC_APB2EN1, RCC_APB2EN1_M7GTIMA3EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA4
      case N32_GTIMERA4_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7GTIMA4EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA5
      case N32_GTIMERA5_BASE:
        modifyreg32(N32_RCC_APB1EN2, RCC_APB1EN2_M7GTIMA5EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA6
      case N32_GTIMERA6_BASE:
        modifyreg32(N32_RCC_APB1EN2, RCC_APB1EN2_M7GTIMA6EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMA7
      case N32_GTIMERA7_BASE:
        modifyreg32(N32_RCC_APB1EN2, RCC_APB1EN2_M7GTIMA7EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB1
      case N32_GTIMERB1_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7GTIMB1EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB2
      case N32_GTIMERB2_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7GTIMB2EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_GTIMB3
      case N32_GTIMERB3_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7GTIMB3EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM1
      case N32_BTIMER1_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7BTIM1EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM2
      case N32_BTIMER2_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7BTIM2EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM3
      case N32_BTIMER3_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7BTIM3EN, 0);
        break;
#endif
#ifdef CONFIG_N32H7_BTIM4
      case N32_BTIMER4_BASE:
        modifyreg32(N32_RCC_APB1EN1, RCC_APB1EN1_M7BTIM4EN, 0);
        break;
#endif
      default:
        return -EINVAL;
    }

  return OK;
}

#endif /* defined(CONFIG_N32H7_ATIM1 || ... || CONFIG_N32H7_BTIM4) */
