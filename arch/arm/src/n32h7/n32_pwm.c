/****************************************************************************
 * arch/arm/src/n32h7/n32_pwm.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>
#include "arm_internal.h"
#include "n32_pwm.h"
#include "n32_gpio.h"
#include "n32_rcc.h"
#include "hardware/n32h7_tim.h"
#include "hardware/n32h76x_pinmap.h"
#include "hardware/n32h7_rcc.h"

#ifdef CONFIG_N32H7_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets (common for ATIM/GTIMA/GTIMB) */
#define N32_PWM_CTRL1_OFFSET       0x0000
#define N32_PWM_CTRL2_OFFSET       0x0004
#define N32_PWM_STS_OFFSET         0x0008
#define N32_PWM_ETGEN_OFFSET       0x000c
#define N32_PWM_SMCTRL_OFFSET      0x0010
#define N32_PWM_DINTEN_OFFSET      0x0014
#define N32_PWM_CCMOD1_OFFSET      0x0018
#define N32_PWM_CCMOD2_OFFSET      0x001c
#define N32_PWM_CCMOD3_OFFSET      0x0020
#define N32_PWM_CCEN_OFFSET        0x0024
#define N32_PWM_CCDAT1_OFFSET      0x0028
#define N32_PWM_CCDAT2_OFFSET      0x002c
#define N32_PWM_CCDAT3_OFFSET      0x0030
#define N32_PWM_CCDAT4_OFFSET      0x0034
#define N32_PWM_PSC_OFFSET         0x0040
#define N32_PWM_AR_OFFSET          0x0044
#define N32_PWM_CNT_OFFSET         0x0048
#define N32_PWM_REPCNT_OFFSET      0x004c
#define N32_PWM_BKDT_OFFSET        0x0050

/* Bit definitions (same as ATIM) */
#define PWM_CR1_CEN        N32_ATIM_CTRL1_CNTEN
#define PWM_CR1_DIR        N32_ATIM_CTRL1_DIR
#define PWM_CR1_CMS_MASK   N32_ATIM_CTRL1_CAMSEL_MASK
#define PWM_CR1_ARPE       N32_ATIM_CTRL1_ARPEN
#define PWM_CR1_OPM        N32_ATIM_CTRL1_ONEPM
#define PWM_CR1_CKD_MASK   N32_ATIM_CTRL1_CLKD_MASK

#define PWM_CCER_CC1E      N32_ATIM_CCEN_CC1EN
#define PWM_CCER_CC1P      N32_ATIM_CCEN_CC1P
#define PWM_CCER_CC1NE     N32_ATIM_CCEN_CC1NEN
#define PWM_CCER_CC1NP     N32_ATIM_CCEN_CC1NP
#define PWM_CCER_CC2E      N32_ATIM_CCEN_CC2EN
#define PWM_CCER_CC2P      N32_ATIM_CCEN_CC2P
#define PWM_CCER_CC2NE     N32_ATIM_CCEN_CC2NEN
#define PWM_CCER_CC2NP     N32_ATIM_CCEN_CC2NP
#define PWM_CCER_CC3E      N32_ATIM_CCEN_CC3EN
#define PWM_CCER_CC3P      N32_ATIM_CCEN_CC3P
#define PWM_CCER_CC3NE     N32_ATIM_CCEN_CC3NEN
#define PWM_CCER_CC3NP     N32_ATIM_CCEN_CC3NP
#define PWM_CCER_CC4E      N32_ATIM_CCEN_CC4EN
#define PWM_CCER_CC4P      N32_ATIM_CCEN_CC4P

#define PWM_CCMR_CCS_OUT   N32_ATIM_CCMOD1_CC1OUT
#define PWM_CCMR_OC1PE     N32_ATIM_CCMOD1_OC1PEN
#define PWM_CCMR_OC2PE     N32_ATIM_CCMOD1_OC2PEN
#define PWM_CCMR_OC3PE     N32_ATIM_CCMOD2_OC3PEN
#define PWM_CCMR_OC4PE     N32_ATIM_CCMOD2_OC4PEN

#define PWM_BDTR_MOE       N32_ATIM_BKDT_MOEN
#define PWM_BDTR_BKE       N32_ATIM_BKDT_BKEN
#define PWM_BDTR_BKP       N32_ATIM_BKDT_BKP
#define PWM_BDTR_DTG_MASK  N32_ATIM_BKDT_DTGN_MASK
#define PWM_BDTR_LOCK_MASK N32_ATIM_BKDT_LCKCFG_MASK

#define PWM_DIER_UIE       N32_ATIM_DINTEN_UIEN
#define PWM_DIER_CC1IE     N32_ATIM_DINTEN_CC1IEN
#define PWM_DIER_CC2IE     N32_ATIM_DINTEN_CC2IEN
#define PWM_DIER_CC3IE     N32_ATIM_DINTEN_CC3IEN
#define PWM_DIER_CC4IE     N32_ATIM_DINTEN_CC4IEN

#define PWM_SR_UIF         N32_ATIM_STS_UDITF
#define PWM_SR_CC1IF       N32_ATIM_STS_CC1ITF
#define PWM_SR_CC2IF       N32_ATIM_STS_CC2ITF
#define PWM_SR_CC3IF       N32_ATIM_STS_CC3ITF
#define PWM_SR_CC4IF       N32_ATIM_STS_CC4ITF

#define PWM_EGR_UG         N32_ATIM_ETGEN_UDGN

/* Helpers for timer type */

static inline bool pwm_has_comp(struct n32_pwmtimer_s *priv)
{
  return (priv->timtype == TIMTYPE_ADVANCED ||
          priv->timtype == TIMTYPE_GTIMB);
}

static inline bool pwm_has_break(struct n32_pwmtimer_s *priv)
{
  return (priv->timtype == TIMTYPE_ADVANCED ||
          priv->timtype == TIMTYPE_GTIMB);
}

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Forward reference for ops */

struct pwm_lowerhalf_s;

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg);
#else
#  define pwm_dumpregs(dev,msg)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_pwmops =
{
    .setup    = pwm_setup,
    .shutdown = pwm_shutdown,
    .start    = pwm_start,
    .stop     = pwm_stop,
    .ioctl    = pwm_ioctl,
};

/* --------------------------------------------------------------------------
 * Timer instance data (generated)
 * --------------------------------------------------------------------------
 */

#ifdef CONFIG_N32H7_ATIM1_PWM
static struct n32_pwmchan_s g_pwmatim1channels[] =
{
#ifdef CONFIG_N32H7_ATIM1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_ATIM1_CH1MODE,
#ifdef CONFIG_N32H7_ATIM1_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_ATIM1_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH1POL,
      .idle    = CONFIG_N32H7_ATIM1_CH1IDLE,
      .pincfg  = GPIO_ATIM1_CH1OUT,
    },
#ifdef CONFIG_N32H7_ATIM1_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH1NPOL,
      .idle    = CONFIG_N32H7_ATIM1_CH1NIDLE,
      .pincfg  = GPIO_ATIM1_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_ATIM1_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH2POL,
      .idle    = CONFIG_N32H7_ATIM1_CH2IDLE,
      .pincfg  = GPIO_ATIM1_CH2OUT,
    },
#ifdef CONFIG_N32H7_ATIM1_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH2NPOL,
      .idle    = CONFIG_N32H7_ATIM1_CH2NIDLE,
      .pincfg  = GPIO_ATIM1_CH2NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_ATIM1_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH3POL,
      .idle    = CONFIG_N32H7_ATIM1_CH3IDLE,
      .pincfg  = GPIO_ATIM1_CH3OUT,
    },
#ifdef CONFIG_N32H7_ATIM1_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH3NPOL,
      .idle    = CONFIG_N32H7_ATIM1_CH3NIDLE,
      .pincfg  = GPIO_ATIM1_CH3NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM1_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_ATIM1_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH4POL,
      .idle    = CONFIG_N32H7_ATIM1_CH4IDLE,
      .pincfg  = GPIO_ATIM1_CH4OUT,
    },
#ifdef CONFIG_N32H7_ATIM1_CH4NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM1_CH4NPOL,
      .idle    = CONFIG_N32H7_ATIM1_CH4NIDLE,
      .pincfg  = GPIO_ATIM1_CH4NOUT,
    },
#endif
  },
#endif
};

static struct n32_pwmtimer_s g_pwmatim1dev =
{
  .ops         = &g_pwmops,
  .timid       = 1,
  .channels    = g_pwmatim1channels,
  .timtype     = TIMTYPE_ADVANCED,
  .mode        = CONFIG_N32H7_ATIM1_MODE,
  .lock        = CONFIG_N32H7_ATIM1_LOCK,
  .deadtime    = CONFIG_N32H7_ATIM1_DEADTIME,
  .t_dts       = CONFIG_N32H7_ATIM1_TDTS,
  .base        = N32_ATIMER1_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_ATIM1,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_ATIM1_PWM */

#ifdef CONFIG_N32H7_ATIM2_PWM
static struct n32_pwmchan_s g_pwmatim2channels[] =
{
#ifdef CONFIG_N32H7_ATIM2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_ATIM2_CH1MODE,
#ifdef CONFIG_N32H7_ATIM2_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_ATIM2_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH1POL,
      .idle    = CONFIG_N32H7_ATIM2_CH1IDLE,
      .pincfg  = GPIO_ATIM2_CH1OUT,
    },
#ifdef CONFIG_N32H7_ATIM2_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH1NPOL,
      .idle    = CONFIG_N32H7_ATIM2_CH1NIDLE,
      .pincfg  = GPIO_ATIM2_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_ATIM2_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH2POL,
      .idle    = CONFIG_N32H7_ATIM2_CH2IDLE,
      .pincfg  = GPIO_ATIM2_CH2OUT,
    },
#ifdef CONFIG_N32H7_ATIM2_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH2NPOL,
      .idle    = CONFIG_N32H7_ATIM2_CH2NIDLE,
      .pincfg  = GPIO_ATIM2_CH2NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_ATIM2_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH3POL,
      .idle    = CONFIG_N32H7_ATIM2_CH3IDLE,
      .pincfg  = GPIO_ATIM2_CH3OUT,
    },
#ifdef CONFIG_N32H7_ATIM2_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH3NPOL,
      .idle    = CONFIG_N32H7_ATIM2_CH3NIDLE,
      .pincfg  = GPIO_ATIM2_CH3NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM2_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_ATIM2_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH4POL,
      .idle    = CONFIG_N32H7_ATIM2_CH4IDLE,
      .pincfg  = GPIO_ATIM2_CH4OUT,
    },
#ifdef CONFIG_N32H7_ATIM2_CH4NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM2_CH4NPOL,
      .idle    = CONFIG_N32H7_ATIM2_CH4NIDLE,
      .pincfg  = GPIO_ATIM2_CH4NOUT,
    },
#endif
  },
#endif
};

static struct n32_pwmtimer_s g_pwmatim2dev =
{
  .ops         = &g_pwmops,
  .timid       = 2,
  .channels    = g_pwmatim2channels,
  .timtype     = TIMTYPE_ADVANCED,
  .mode        = CONFIG_N32H7_ATIM2_MODE,
  .lock        = CONFIG_N32H7_ATIM2_LOCK,
  .deadtime    = CONFIG_N32H7_ATIM2_DEADTIME,
  .t_dts       = CONFIG_N32H7_ATIM2_TDTS,
  .base        = N32_ATIMER2_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_ATIM2,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_ATIM2_PWM */

#ifdef CONFIG_N32H7_ATIM3_PWM
static struct n32_pwmchan_s g_pwmatim3channels[] =
{
#ifdef CONFIG_N32H7_ATIM3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_ATIM3_CH1MODE,
#ifdef CONFIG_N32H7_ATIM3_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_ATIM3_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH1POL,
      .idle    = CONFIG_N32H7_ATIM3_CH1IDLE,
      .pincfg  = GPIO_ATIM3_CH1OUT,
    },
#ifdef CONFIG_N32H7_ATIM3_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH1NPOL,
      .idle    = CONFIG_N32H7_ATIM3_CH1NIDLE,
      .pincfg  = GPIO_ATIM3_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_ATIM3_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH2POL,
      .idle    = CONFIG_N32H7_ATIM3_CH2IDLE,
      .pincfg  = GPIO_ATIM3_CH2OUT,
    },
#ifdef CONFIG_N32H7_ATIM3_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH2NPOL,
      .idle    = CONFIG_N32H7_ATIM3_CH2NIDLE,
      .pincfg  = GPIO_ATIM3_CH2NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_ATIM3_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH3POL,
      .idle    = CONFIG_N32H7_ATIM3_CH3IDLE,
      .pincfg  = GPIO_ATIM3_CH3OUT,
    },
#ifdef CONFIG_N32H7_ATIM3_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH3NPOL,
      .idle    = CONFIG_N32H7_ATIM3_CH3NIDLE,
      .pincfg  = GPIO_ATIM3_CH3NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM3_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_ATIM3_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH4POL,
      .idle    = CONFIG_N32H7_ATIM3_CH4IDLE,
      .pincfg  = GPIO_ATIM3_CH4OUT,
    },
#ifdef CONFIG_N32H7_ATIM3_CH4NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM3_CH4NPOL,
      .idle    = CONFIG_N32H7_ATIM3_CH4NIDLE,
      .pincfg  = GPIO_ATIM3_CH4NOUT,
    },
#endif
  },
#endif
};

static struct n32_pwmtimer_s g_pwmatim3dev =
{
  .ops         = &g_pwmops,
  .timid       = 3,
  .channels    = g_pwmatim3channels,
  .timtype     = TIMTYPE_ADVANCED,
  .mode        = CONFIG_N32H7_ATIM3_MODE,
  .lock        = CONFIG_N32H7_ATIM3_LOCK,
  .deadtime    = CONFIG_N32H7_ATIM3_DEADTIME,
  .t_dts       = CONFIG_N32H7_ATIM3_TDTS,
  .base        = N32_ATIMER3_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_ATIM3,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_ATIM3_PWM */

#ifdef CONFIG_N32H7_ATIM4_PWM
static struct n32_pwmchan_s g_pwmatim4channels[] =
{
#ifdef CONFIG_N32H7_ATIM4_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_ATIM4_CH1MODE,
#ifdef CONFIG_N32H7_ATIM4_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_ATIM4_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH1POL,
      .idle    = CONFIG_N32H7_ATIM4_CH1IDLE,
      .pincfg  = GPIO_ATIM4_CH1OUT,
    },
#ifdef CONFIG_N32H7_ATIM4_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH1NPOL,
      .idle    = CONFIG_N32H7_ATIM4_CH1NIDLE,
      .pincfg  = GPIO_ATIM4_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM4_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_ATIM4_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH2POL,
      .idle    = CONFIG_N32H7_ATIM4_CH2IDLE,
      .pincfg  = GPIO_ATIM4_CH2OUT,
    },
#ifdef CONFIG_N32H7_ATIM4_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH2NPOL,
      .idle    = CONFIG_N32H7_ATIM4_CH2NIDLE,
      .pincfg  = GPIO_ATIM4_CH2NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM4_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_ATIM4_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH3POL,
      .idle    = CONFIG_N32H7_ATIM4_CH3IDLE,
      .pincfg  = GPIO_ATIM4_CH3OUT,
    },
#ifdef CONFIG_N32H7_ATIM4_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH3NPOL,
      .idle    = CONFIG_N32H7_ATIM4_CH3NIDLE,
      .pincfg  = GPIO_ATIM4_CH3NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_ATIM4_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_ATIM4_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH4POL,
      .idle    = CONFIG_N32H7_ATIM4_CH4IDLE,
      .pincfg  = GPIO_ATIM4_CH4OUT,
    },
#ifdef CONFIG_N32H7_ATIM4_CH4NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_ATIM4_CH4NPOL,
      .idle    = CONFIG_N32H7_ATIM4_CH4NIDLE,
      .pincfg  = GPIO_ATIM4_CH4NOUT,
    },
#endif
  },
#endif
};

static struct n32_pwmtimer_s g_pwmatim4dev =
{
  .ops         = &g_pwmops,
  .timid       = 4,
  .channels    = g_pwmatim4channels,
  .timtype     = TIMTYPE_ADVANCED,
  .mode        = CONFIG_N32H7_ATIM4_MODE,
  .lock        = CONFIG_N32H7_ATIM4_LOCK,
  .deadtime    = CONFIG_N32H7_ATIM4_DEADTIME,
  .t_dts       = CONFIG_N32H7_ATIM4_TDTS,
  .base        = N32_ATIMER4_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_ATIM4,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_ATIM4_PWM */

#ifdef CONFIG_N32H7_GTIMA1_PWM
static struct n32_pwmchan_s g_pwmgtima1channels[] =
{
#ifdef CONFIG_N32H7_GTIMA1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA1_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA1_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA1_CH1IDLE,
      .pincfg  = GPIO_GTIMA1_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA1_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA1_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA1_CH2IDLE,
      .pincfg  = GPIO_GTIMA1_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA1_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA1_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA1_CH3IDLE,
      .pincfg  = GPIO_GTIMA1_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA1_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA1_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA1_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA1_CH4IDLE,
      .pincfg  = GPIO_GTIMA1_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima1dev =
{
  .ops         = &g_pwmops,
  .timid       = 5,
  .channels    = g_pwmgtima1channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA1_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA1_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA1,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA1_PWM */

#ifdef CONFIG_N32H7_GTIMA2_PWM
static struct n32_pwmchan_s g_pwmgtima2channels[] =
{
#ifdef CONFIG_N32H7_GTIMA2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA2_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA2_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA2_CH1IDLE,
      .pincfg  = GPIO_GTIMA2_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA2_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA2_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA2_CH2IDLE,
      .pincfg  = GPIO_GTIMA2_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA2_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA2_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA2_CH3IDLE,
      .pincfg  = GPIO_GTIMA2_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA2_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA2_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA2_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA2_CH4IDLE,
      .pincfg  = GPIO_GTIMA2_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima2dev =
{
  .ops         = &g_pwmops,
  .timid       = 6,
  .channels    = g_pwmgtima2channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA2_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA2_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA2,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA2_PWM */

#ifdef CONFIG_N32H7_GTIMA3_PWM
static struct n32_pwmchan_s g_pwmgtima3channels[] =
{
#ifdef CONFIG_N32H7_GTIMA3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA3_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA3_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA3_CH1IDLE,
      .pincfg  = GPIO_GTIMA3_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA3_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA3_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA3_CH2IDLE,
      .pincfg  = GPIO_GTIMA3_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA3_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA3_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA3_CH3IDLE,
      .pincfg  = GPIO_GTIMA3_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA3_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA3_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA3_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA3_CH4IDLE,
      .pincfg  = GPIO_GTIMA3_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima3dev =
{
  .ops         = &g_pwmops,
  .timid       = 7,
  .channels    = g_pwmgtima3channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA3_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA3_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA3,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA3_PWM */

#ifdef CONFIG_N32H7_GTIMA4_PWM
static struct n32_pwmchan_s g_pwmgtima4channels[] =
{
#ifdef CONFIG_N32H7_GTIMA4_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA4_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA4_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA4_CH1IDLE,
      .pincfg  = GPIO_GTIMA4_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA4_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA4_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA4_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA4_CH2IDLE,
      .pincfg  = GPIO_GTIMA4_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA4_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA4_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA4_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA4_CH3IDLE,
      .pincfg  = GPIO_GTIMA4_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA4_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA4_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA4_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA4_CH4IDLE,
      .pincfg  = GPIO_GTIMA4_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima4dev =
{
  .ops         = &g_pwmops,
  .timid       = 8,
  .channels    = g_pwmgtima4channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA4_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA4_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA4,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA4_PWM */

#ifdef CONFIG_N32H7_GTIMA5_PWM
static struct n32_pwmchan_s g_pwmgtima5channels[] =
{
#ifdef CONFIG_N32H7_GTIMA5_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA5_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA5_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA5_CH1IDLE,
      .pincfg  = GPIO_GTIMA5_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA5_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA5_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA5_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA5_CH2IDLE,
      .pincfg  = GPIO_GTIMA5_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA5_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA5_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA5_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA5_CH3IDLE,
      .pincfg  = GPIO_GTIMA5_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA5_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA5_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA5_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA5_CH4IDLE,
      .pincfg  = GPIO_GTIMA5_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima5dev =
{
  .ops         = &g_pwmops,
  .timid       = 9,
  .channels    = g_pwmgtima5channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA5_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA5_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA5,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA5_PWM */

#ifdef CONFIG_N32H7_GTIMA6_PWM
static struct n32_pwmchan_s g_pwmgtima6channels[] =
{
#ifdef CONFIG_N32H7_GTIMA6_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA6_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA6_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA6_CH1IDLE,
      .pincfg  = GPIO_GTIMA6_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA6_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA6_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA6_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA6_CH2IDLE,
      .pincfg  = GPIO_GTIMA6_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA6_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA6_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA6_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA6_CH3IDLE,
      .pincfg  = GPIO_GTIMA6_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA6_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA6_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA6_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA6_CH4IDLE,
      .pincfg  = GPIO_GTIMA6_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima6dev =
{
  .ops         = &g_pwmops,
  .timid       = 10,
  .channels    = g_pwmgtima6channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA6_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA6_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA6,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA6_PWM */

#ifdef CONFIG_N32H7_GTIMA7_PWM
static struct n32_pwmchan_s g_pwmgtima7channels[] =
{
#ifdef CONFIG_N32H7_GTIMA7_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMA7_CH1MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA7_CH1POL,
      .idle    = CONFIG_N32H7_GTIMA7_CH1IDLE,
      .pincfg  = GPIO_GTIMA7_CH1OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA7_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMA7_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA7_CH2POL,
      .idle    = CONFIG_N32H7_GTIMA7_CH2IDLE,
      .pincfg  = GPIO_GTIMA7_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA7_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMA7_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA7_CH3POL,
      .idle    = CONFIG_N32H7_GTIMA7_CH3IDLE,
      .pincfg  = GPIO_GTIMA7_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMA7_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMA7_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMA7_CH4POL,
      .idle    = CONFIG_N32H7_GTIMA7_CH4IDLE,
      .pincfg  = GPIO_GTIMA7_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtima7dev =
{
  .ops         = &g_pwmops,
  .timid       = 11,
  .channels    = g_pwmgtima7channels,
  .timtype     = TIMTYPE_GENERAL16,
  .mode        = CONFIG_N32H7_GTIMA7_MODE,
  .lock        = 0,
  .deadtime    = 0,
  .t_dts       = 0,
  .base        = N32_GTIMERA7_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMA7,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMA7_PWM */

#ifdef CONFIG_N32H7_GTIMB1_PWM
static struct n32_pwmchan_s g_pwmgtimb1channels[] =
{
#ifdef CONFIG_N32H7_GTIMB1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMB1_CH1MODE,
#ifdef CONFIG_N32H7_GTIMB1_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_GTIMB1_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB1_CH1POL,
      .idle    = CONFIG_N32H7_GTIMB1_CH1IDLE,
      .pincfg  = GPIO_GTIMB1_CH1OUT,
    },
#ifdef CONFIG_N32H7_GTIMB1_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB1_CH1NPOL,
      .idle    = CONFIG_N32H7_GTIMB1_CH1NIDLE,
      .pincfg  = GPIO_GTIMB1_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_GTIMB1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMB1_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB1_CH2POL,
      .idle    = CONFIG_N32H7_GTIMB1_CH2IDLE,
      .pincfg  = GPIO_GTIMB1_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMB1_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB1_CH3POL,
      .idle    = CONFIG_N32H7_GTIMB1_CH3IDLE,
      .pincfg  = GPIO_GTIMB1_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB1_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMB1_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB1_CH4POL,
      .idle    = CONFIG_N32H7_GTIMB1_CH4IDLE,
      .pincfg  = GPIO_GTIMB1_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtimb1dev =
{
  .ops         = &g_pwmops,
  .timid       = 12,
  .channels    = g_pwmgtimb1channels,
  .timtype     = TIMTYPE_GTIMB,
  .mode        = CONFIG_N32H7_GTIMB1_MODE,
  .lock        = CONFIG_N32H7_GTIMB1_LOCK,
  .deadtime    = CONFIG_N32H7_GTIMB1_DEADTIME,
  .t_dts       = CONFIG_N32H7_GTIMB1_TDTS,
  .base        = N32_GTIMERB1_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMB1,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMB1_PWM */

#ifdef CONFIG_N32H7_GTIMB2_PWM
static struct n32_pwmchan_s g_pwmgtimb2channels[] =
{
#ifdef CONFIG_N32H7_GTIMB2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMB2_CH1MODE,
#ifdef CONFIG_N32H7_GTIMB2_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_GTIMB2_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB2_CH1POL,
      .idle    = CONFIG_N32H7_GTIMB2_CH1IDLE,
      .pincfg  = GPIO_GTIMB2_CH1OUT,
    },
#ifdef CONFIG_N32H7_GTIMB2_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB2_CH1NPOL,
      .idle    = CONFIG_N32H7_GTIMB2_CH1NIDLE,
      .pincfg  = GPIO_GTIMB2_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_GTIMB2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMB2_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB2_CH2POL,
      .idle    = CONFIG_N32H7_GTIMB2_CH2IDLE,
      .pincfg  = GPIO_GTIMB2_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMB2_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB2_CH3POL,
      .idle    = CONFIG_N32H7_GTIMB2_CH3IDLE,
      .pincfg  = GPIO_GTIMB2_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB2_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMB2_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB2_CH4POL,
      .idle    = CONFIG_N32H7_GTIMB2_CH4IDLE,
      .pincfg  = GPIO_GTIMB2_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtimb2dev =
{
  .ops         = &g_pwmops,
  .timid       = 13,
  .channels    = g_pwmgtimb2channels,
  .timtype     = TIMTYPE_GTIMB,
  .mode        = CONFIG_N32H7_GTIMB2_MODE,
  .lock        = CONFIG_N32H7_GTIMB2_LOCK,
  .deadtime    = CONFIG_N32H7_GTIMB2_DEADTIME,
  .t_dts       = CONFIG_N32H7_GTIMB2_TDTS,
  .base        = N32_GTIMERB2_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMB2,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMB2_PWM */

#ifdef CONFIG_N32H7_GTIMB3_PWM
static struct n32_pwmchan_s g_pwmgtimb3channels[] =
{
#ifdef CONFIG_N32H7_GTIMB3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_N32H7_GTIMB3_CH1MODE,
#ifdef CONFIG_N32H7_GTIMB3_BREAK1
    .brk =
    {
      .en1 = 1,
      .pol1 = CONFIG_N32H7_GTIMB3_BRK1POL,
    },
#endif
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB3_CH1POL,
      .idle    = CONFIG_N32H7_GTIMB3_CH1IDLE,
      .pincfg  = GPIO_GTIMB3_CH1OUT,
    },
#ifdef CONFIG_N32H7_GTIMB3_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB3_CH1NPOL,
      .idle    = CONFIG_N32H7_GTIMB3_CH1NIDLE,
      .pincfg  = GPIO_GTIMB3_CH1NOUT,
    },
#endif
  },
#endif
#ifdef CONFIG_N32H7_GTIMB3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_N32H7_GTIMB3_CH2MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB3_CH2POL,
      .idle    = CONFIG_N32H7_GTIMB3_CH2IDLE,
      .pincfg  = GPIO_GTIMB3_CH2OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_N32H7_GTIMB3_CH3MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB3_CH3POL,
      .idle    = CONFIG_N32H7_GTIMB3_CH3IDLE,
      .pincfg  = GPIO_GTIMB3_CH3OUT,
    },
  },
#endif
#ifdef CONFIG_N32H7_GTIMB3_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_N32H7_GTIMB3_CH4MODE,
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_N32H7_GTIMB3_CH4POL,
      .idle    = CONFIG_N32H7_GTIMB3_CH4IDLE,
      .pincfg  = GPIO_GTIMB3_CH4OUT,
    },
  },
#endif
};

static struct n32_pwmtimer_s g_pwmgtimb3dev =
{
  .ops         = &g_pwmops,
  .timid       = 14,
  .channels    = g_pwmgtimb3channels,
  .timtype     = TIMTYPE_GTIMB,
  .mode        = CONFIG_N32H7_GTIMB3_MODE,
  .lock        = CONFIG_N32H7_GTIMB3_LOCK,
  .deadtime    = CONFIG_N32H7_GTIMB3_DEADTIME,
  .t_dts       = CONFIG_N32H7_GTIMB3_TDTS,
  .base        = N32_GTIMERB3_BASE,
  .pclk        = N32_AHB_FREQUENCY,
  .frequency   = 0,
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = N32_IRQ_GTIMB3,
  .handle      = NULL,
#endif
};
#endif /* CONFIG_N32H7_GTIMB3_PWM */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_reg_is_32bit
 ****************************************************************************/

static bool pwm_reg_is_32bit(uint8_t timtype, uint32_t offset)
{
  bool ret = false;

  /* Only GENERAL32 (not used for PWM) would have 32-bit registers */

  if (timtype == TIMTYPE_GENERAL32)
    {
      if (offset == N32_PWM_CNT_OFFSET    ||
          offset == N32_PWM_AR_OFFSET     ||
          offset == N32_PWM_CCDAT1_OFFSET ||
          offset == N32_PWM_CCDAT2_OFFSET ||
          offset == N32_PWM_CCDAT3_OFFSET ||
          offset == N32_PWM_CCDAT4_OFFSET)
        {
          ret = true;
        }
    }

  /* Advanced timers have some 32-bit registers (CR2, CCMR, CCER, BDTR) */

  else if (timtype == TIMTYPE_ADVANCED)
    {
      if (offset == N32_PWM_CTRL2_OFFSET  ||
          offset == N32_PWM_CCMOD1_OFFSET ||
          offset == N32_PWM_CCMOD2_OFFSET ||
          offset == N32_PWM_CCEN_OFFSET   ||
          offset == N32_PWM_BKDT_OFFSET   ||
          offset == N32_PWM_CCMOD3_OFFSET)
        {
          ret = true;
        }
    }

  /* GTIMB also has some 32-bit regs (CTRL2, CCMOD1/2/3, CCEN, BKDT) */

  else if (timtype == TIMTYPE_GTIMB)
    {
      if (offset == N32_PWM_CTRL2_OFFSET  ||
          offset == N32_PWM_CCMOD1_OFFSET ||
          offset == N32_PWM_CCMOD2_OFFSET ||
          offset == N32_PWM_CCMOD3_OFFSET ||
          offset == N32_PWM_CCEN_OFFSET   ||
          offset == N32_PWM_BKDT_OFFSET)
        {
          ret = true;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pwm_getreg / pwm_putreg / pwm_modifyreg
 ****************************************************************************/

static uint32_t pwm_getreg(struct n32_pwmtimer_s *priv, uint32_t offset)
{
  if (pwm_reg_is_32bit(priv->timtype, offset))
    {
      return getreg32(priv->base + offset);
    }
  else
    {
      return getreg16(priv->base + offset);
    }
}

static void pwm_putreg(struct n32_pwmtimer_s *priv, uint32_t offset,
                       uint32_t value)
{
  if (pwm_reg_is_32bit(priv->timtype, offset))
    {
      putreg32(value, priv->base + offset);
    }
  else
    {
      putreg16((uint16_t)value, priv->base + offset);
    }
}

static void pwm_modifyreg(struct n32_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits)
{
  if (pwm_reg_is_32bit(priv->timtype, offset))
    {
      modifyreg32(priv->base + offset, clearbits, setbits);
    }
  else
    {
      modifyreg16(priv->base + offset, (uint16_t)clearbits,
                  (uint16_t)setbits);
    }
}

/****************************************************************************
 * Name: pwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 ****************************************************************************/

static int pwm_set_apb_clock(struct n32_pwmtimer_s *priv, bool on)
{
  uint32_t regaddr = 0;
  uint32_t en_bit  = 0;

  switch (priv->timid)
    {
  #ifdef CONFIG_N32H7_ATIM1_PWM
      case 1:
        regaddr = N32_RCC_APB2EN1;
        en_bit  = RCC_APB2EN1_M7ATIM1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_PWM
      case 2:
        regaddr = N32_RCC_APB2EN1;
        en_bit  = RCC_APB2EN1_M7ATIM2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_PWM
      case 3:
        regaddr = N32_RCC_APB2EN1;
        en_bit  = RCC_APB2EN1_M7GTIMA1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_PWM
      case 4:
        regaddr = N32_RCC_APB2EN1;
        en_bit  = RCC_APB2EN1_M7GTIMA2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_PWM
      case 5:
        regaddr = N32_RCC_APB2EN1;
        en_bit  = RCC_APB2EN1_M7GTIMA3EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_PWM
      case 6:
        regaddr = N32_RCC_APB5EN1;
        en_bit  = RCC_APB5EN1_M7ATIM3EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_PWM
      case 7:
        regaddr = N32_RCC_APB5EN1;
        en_bit  = RCC_APB5EN1_M7ATIM4EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_PWM
      case 8:
        regaddr = N32_RCC_APB1EN1;
        en_bit  = RCC_APB1EN1_M7GTIMA4EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_PWM
      case 9:
        regaddr = N32_RCC_APB1EN2;
        en_bit  = RCC_APB1EN2_M7GTIMA5EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_PWM
      case 10:
        regaddr = N32_RCC_APB1EN2;
        en_bit  = RCC_APB1EN2_M7GTIMA6EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_PWM
      case 11:
        regaddr = N32_RCC_APB1EN2;
        en_bit  = RCC_APB1EN2_M7GTIMA7EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_PWM
      case 12:
        regaddr = N32_RCC_APB1EN1;
        en_bit  = RCC_APB1EN1_M7GTIMB1EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_PWM
      case 13:
        regaddr = N32_RCC_APB1EN1;
        en_bit  = RCC_APB1EN1_M7GTIMB2EN;
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_PWM
      case 14:
        regaddr = N32_RCC_APB1EN1;
        en_bit  = RCC_APB1EN1_M7GTIMB3EN;
        break;
  #endif
      default:
        return -EINVAL;
    }

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_configure
 *
 * Description:
 *   Initial configuration for PWM timer (mode, ARPE, etc.)
 ****************************************************************************/

static int pwm_timer_configure(struct n32_pwmtimer_s *priv)
{
  uint32_t cr1 = pwm_getreg(priv, N32_PWM_CTRL1_OFFSET);

  /* Set counter mode (if not basic) */

  if (priv->timtype != TIMTYPE_BASIC)  /* BASIC not used for PWM */
    {
      cr1 &= ~(PWM_CR1_DIR | PWM_CR1_CMS_MASK);

      switch (priv->mode)
        {
          case 0: /* Up-count */
            cr1 |= N32_ATIM_CTRL1_EDGE;
            break;
          case 1: /* Down-count */
            cr1 |= N32_ATIM_CTRL1_EDGE | PWM_CR1_DIR;
            break;
          case 2: /* Center-aligned 1 */
            cr1 |= N32_ATIM_CTRL1_CENTER1;
            break;
          case 3: /* Center-aligned 2 */
            cr1 |= N32_ATIM_CTRL1_CENTER2;
            break;
          case 4: /* Center-aligned 3 */
            cr1 |= N32_ATIM_CTRL1_CENTER3;
            break;
          default:
            return -EINVAL;
        }
    }

  /* Enable ARR preload */

  cr1 |= PWM_CR1_ARPE;
  pwm_putreg(priv, N32_PWM_CTRL1_OFFSET, cr1);

  return OK;
}

/****************************************************************************
 * Name: pwm_break_dt_configure
 *
 * Description:
 *   Configure Break and Dead-time for advanced timers
 ****************************************************************************/

static int pwm_break_dt_configure(struct n32_pwmtimer_s *priv)
{
  uint32_t bdtr = 0;

  /* Set clock division (CKD) for dead-time */

  pwm_modifyreg(priv, N32_PWM_CTRL1_OFFSET, PWM_CR1_CKD_MASK,
                priv->t_dts << N32_ATIM_CTRL1_CLKD_SHIFT);

  /* Dead-time */

  bdtr |= (priv->deadtime << N32_ATIM_BKDT_DTGN_SHIFT);

  /* Break1 (if enabled) */

  if (priv->channels[0].brk.en1)
    {
      bdtr |= PWM_BDTR_BKE;
      if (priv->channels[0].brk.pol1 == N32_POL_NEG)
        {
          bdtr |= PWM_BDTR_BKP;
        }
    }

  /* Lock */

  bdtr |= (priv->lock << N32_ATIM_BKDT_LCKCFG_SHIFT);

  /* Clear OSSI/OSSR (idle mode settings) */

  bdtr &= ~(N32_ATIM_BKDT_OSSI | N32_ATIM_BKDT_OSSR);

  pwm_putreg(priv, N32_PWM_BKDT_OFFSET, bdtr);
  return OK;
}

/****************************************************************************
 * Name: pwm_frequency_update
 *
 * Description:
 *   Calculate and set prescaler and ARR for desired frequency
 ****************************************************************************/

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  uint32_t timclk = priv->pclk;
  uint32_t prescaler;
  uint32_t reload;

  prescaler = (timclk / frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }

  if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  reload = (timclk / prescaler) / frequency;
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

  pwm_putreg(priv, N32_PWM_PSC_OFFSET, prescaler - 1);
  pwm_putreg(priv, N32_PWM_AR_OFFSET, reload);
  priv->frequency = frequency;
  return OK;
}

/****************************************************************************
 * Name: pwm_mode_configure
 *
 * Description:
 *   Configure output compare mode for a channel
 ****************************************************************************/

static int pwm_mode_configure(struct pwm_lowerhalf_s *dev, uint8_t channel,
                              uint32_t mode)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  uint32_t ccmr_offset;
  uint32_t shift;
  uint32_t ccmr;
  uint32_t ccmr_val;
  uint32_t oc_pe_bit;

  if (channel <= 2)
    {
      ccmr_offset = N32_PWM_CCMOD1_OFFSET;
      shift = (channel == 1) ? 0 : (N32_ATIM_CCMOD1_OC2MD_SHIFT -
              N32_ATIM_CCMOD1_OC1MD_SHIFT);
      oc_pe_bit = (channel == 1) ? N32_ATIM_CCMOD1_OC1PEN :
                  N32_ATIM_CCMOD1_OC2PEN;
    }
  else
    {
      ccmr_offset = N32_PWM_CCMOD2_OFFSET;
      shift = (channel == 3) ? 0 : (N32_ATIM_CCMOD2_OC4MD_SHIFT -
              N32_ATIM_CCMOD2_OC3MD_SHIFT);
      oc_pe_bit = (channel == 3) ? N32_ATIM_CCMOD2_OC3PEN :
                  N32_ATIM_CCMOD2_OC4PEN;
    }

  ccmr = pwm_getreg(priv, ccmr_offset);
  ccmr &= ~(0xff << shift);  /* clear OCxM and OCxPE */
  switch (mode)
    {
      case N32_CHANMODE_FRZN:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_FRZN;
        break;
      case N32_CHANMODE_CHACT:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_ACTIVE;
        break;
      case N32_CHANMODE_CHINACT:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_INACTIVE;
        break;
      case N32_CHANMODE_OCREFTOG:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_TOGGLE;
        break;
      case N32_CHANMODE_OCREFLO:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_FORCELO;
        break;
      case N32_CHANMODE_OCREFHI:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_FORCEHI;
        break;
      case N32_CHANMODE_PWM1:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_PWM1;
        break;
      case N32_CHANMODE_PWM2:
        ccmr_val = N32_ATIM_CCMOD1_OCMODE_PWM2;
        break;
      default:
        return -EINVAL;
    }

  ccmr |= (ccmr_val << shift) | oc_pe_bit;
  pwm_putreg(priv, ccmr_offset, ccmr);
  return OK;
}

/****************************************************************************
 * Name: pwm_output_configure
 *
 * Description:
 *   Configure polarity, idle state, and complementary for a channel
 ****************************************************************************/

static int pwm_output_configure(struct n32_pwmtimer_s *priv,
                                struct n32_pwmchan_s *chan)
{
  uint32_t cr2 = pwm_getreg(priv, N32_PWM_CTRL2_OFFSET);
  uint32_t ccer = pwm_getreg(priv, N32_PWM_CCEN_OFFSET);
  uint8_t ch = chan->channel;
  uint32_t shift = (ch - 1) * 4;

  /* Main output polarity */

  if (chan->out1.pol == N32_POL_NEG)
    {
      ccer |= (PWM_CCER_CC1P << shift);
    }
  else
    {
      ccer &= ~(PWM_CCER_CC1P << shift);
    }

  if (pwm_has_comp(priv))
    {
      /* Main output idle state */

      if (chan->out1.idle == N32_IDLE_ACTIVE)
        {
          cr2 |= (N32_ATIM_CTRL2_O11 << ((ch - 1) * 2));
        }
      else
        {
          cr2 &= ~(N32_ATIM_CTRL2_O11 << ((ch - 1) * 2));
        }

      /* Complementary output */

      if (chan->out2.in_use)
        {
          /* Polarity */

          if (chan->out2.pol == N32_POL_NEG)
            {
              ccer |= (PWM_CCER_CC1NP << shift);
            }
          else
            {
              ccer &= ~(PWM_CCER_CC1NP << shift);
            }

          /* Idle state */

          if (chan->out2.idle == N32_IDLE_ACTIVE)
            {
              cr2 |= (N32_ATIM_CTRL2_O11N << ((ch - 1) * 2));
            }
          else
            {
              cr2 &= ~(N32_ATIM_CTRL2_O11N << ((ch - 1) * 2));
            }
        }
    }
  else
    {
      /* GTIMA: clear NP bits if any */

      ccer &= ~(PWM_CCER_CC1NP << shift);
    }

  pwm_putreg(priv, N32_PWM_CTRL2_OFFSET, cr2);
  pwm_putreg(priv, N32_PWM_CCEN_OFFSET, ccer);
  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_enable
 *
 * Description:
 *   Enable or disable outputs in bulk
 ****************************************************************************/

static int pwm_outputs_enable(struct pwm_lowerhalf_s *dev, uint16_t outputs,
                              bool state)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  uint32_t ccer = pwm_getreg(priv, N32_PWM_CCEN_OFFSET);
  uint32_t mask = 0;

  if (outputs & N32_PWM_OUT1)
    {
      mask |= PWM_CCER_CC1E;
    }

  if (outputs & N32_PWM_OUT2)
    {
      mask |= PWM_CCER_CC2E;
    }

  if (outputs & N32_PWM_OUT3)
    {
      mask |= PWM_CCER_CC3E;
    }

  if (outputs & N32_PWM_OUT4)
    {
      mask |= PWM_CCER_CC4E;
    }

  if (outputs & N32_PWM_OUT1N)
    {
      mask |= PWM_CCER_CC1NE;
    }

  if (outputs & N32_PWM_OUT2N)
    {
      mask |= PWM_CCER_CC2NE;
    }

  if (outputs & N32_PWM_OUT3N)
    {
      mask |= PWM_CCER_CC3NE;
    }

  if (state)
    {
      ccer |= mask;
    }
  else
    {
      ccer &= ~mask;
    }

  pwm_putreg(priv, N32_PWM_CCEN_OFFSET, ccer);
  return OK;
}

/****************************************************************************
 * Name: pwm_soft_break
 *
 * Description:
 *   Software break (enable/disable MOE)
 ****************************************************************************/

static int pwm_soft_break(struct pwm_lowerhalf_s *dev, bool state)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;

  if (pwm_has_break(priv))
    {
      if (state)
        {
          pwm_modifyreg(priv, N32_PWM_BKDT_OFFSET, PWM_BDTR_MOE, 0);
        }
      else
        {
          pwm_modifyreg(priv, N32_PWM_BKDT_OFFSET, 0, PWM_BDTR_MOE);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_soft_update
 *
 * Description:
 *   Generate software update event
 ****************************************************************************/

static int pwm_soft_update(struct pwm_lowerhalf_s *dev)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;

  pwm_putreg(priv, N32_PWM_ETGEN_OFFSET, PWM_EGR_UG);

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_from_channels
 *
 * Description:
 *   Build output mask from channel configuration
 ****************************************************************************/

static uint16_t pwm_outputs_from_channels(struct n32_pwmtimer_s *priv)
{
  uint16_t outputs = 0;

  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (priv->channels[i].channel == 0)
        {
          continue;
        }

      if (priv->channels[i].out1.in_use)
        {
          outputs |= (N32_PWM_OUT1 << ((priv->channels[i].channel - 1) * 2));
        }

      if (pwm_has_comp(priv) && priv->channels[i].out2.in_use)
        {
          outputs |= (N32_PWM_OUT1N << ((priv->channels[i].channel - 1) * 2)
                      );
        }
    }

  return outputs;
}

/****************************************************************************
 * Name: pwm_configure (non-pulsecount mode)
 *
 * Description:
 *   Initial configuration without pulsecount
 ****************************************************************************/

static int pwm_configure(struct pwm_lowerhalf_s *dev)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  int ret;
  int i;

  /* Disable timer */

  pwm_modifyreg(priv, N32_PWM_CTRL1_OFFSET, PWM_CR1_CEN, 0);

  /* Basic timer config */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Break/deadtime for advanced timers */

  if (pwm_has_break(priv))
    {
      ret = pwm_break_dt_configure(priv);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Configure each channel */

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      struct n32_pwmchan_s *chan = &priv->channels[i];

      if (chan->channel == 0)
        {
          continue;
        }

      ret = pwm_mode_configure(dev, chan->channel, chan->mode);
      if (ret < 0)
        {
          return ret;
        }

      ret = pwm_output_configure(priv, chan);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Enable main output for advanced timers */

  if (pwm_has_break(priv))
    {
      pwm_soft_break(dev, false);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  int ret;

  /* Set frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      return ret;
    }

  /* Set duty for each channel */

  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (info->channels[i].channel == -1)
        {
          break;
        }

      if (info->channels[i].channel == 0)
        {
          continue;
        }

      uint32_t reload = pwm_getreg(priv, N32_PWM_AR_OFFSET);
      uint32_t ccr = (uint32_t)(b16toi(info->channels[i].duty * reload +
                      b16HALF));
      int ch = info->channels[i].channel;

      pwm_putreg(priv, N32_PWM_CCDAT1_OFFSET + (ch - 1) * 4, ccr);
    }

  /* Enable outputs */

  uint16_t outputs = pwm_outputs_from_channels(priv);

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Generate update event and enable counter */

  pwm_soft_update(dev);
  pwm_modifyreg(priv, N32_PWM_CTRL1_OFFSET, 0, PWM_CR1_CEN);
  return OK;
}

/****************************************************************************
 * Name: pwm_stop
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;

  pwm_modifyreg(priv, N32_PWM_CTRL1_OFFSET, PWM_CR1_CEN, 0);
  pwm_outputs_enable(dev, 0xffff, false);
  pwm_putreg(priv, N32_PWM_DINTEN_OFFSET, 0);
  pwm_putreg(priv, N32_PWM_CNT_OFFSET, 0);

  priv->frequency = 0;
  return OK;
}

/****************************************************************************
 * Name: pwm_setup
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;
  int ret;

  /* Enable clock */

  ret = pwm_set_apb_clock(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure GPIOs */

  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (priv->channels[i].channel == 0)
        {
          continue;
        }

      if (priv->channels[i].out1.in_use && priv->channels[i].out1.pincfg)
        {
          n32_configgpio(priv->channels[i].out1.pincfg);
        }

      if (pwm_has_comp(priv) && priv->channels[i].out2.in_use &&
          priv->channels[i].out2.pincfg)
        {
          n32_configgpio(priv->channels[i].out2.pincfg);
        }
    }

  /* Configure timer */

  ret = pwm_configure(dev);
  if (ret < 0)
    {
      pwm_set_apb_clock(priv, false);
      return ret;
    }

  pwm_dumpregs(dev, "After setup");
  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;

  pwm_stop(dev);
  pwm_set_apb_clock(priv, false);

  /* Unconfigure GPIOs */

  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      if (priv->channels[i].channel == 0)
        {
          continue;
        }

      if (priv->channels[i].out1.pincfg)
        {
          n32_unconfiggpio(priv->channels[i].out1.pincfg);
        }

      if (priv->channels[i].out2.pincfg)
        {
          n32_unconfiggpio(priv->channels[i].out2.pincfg);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: pwm_dumpregs
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg)
{
  struct n32_pwmtimer_s *priv = (struct n32_pwmtimer_s *)dev;

  pwminfo("%s: CR1=%04x CR2=%04x CCER=%04x CNT=%04x ARR=%04x PSC=%04x\n",
          msg,
          pwm_getreg(priv, N32_PWM_CTRL1_OFFSET),
          pwm_getreg(priv, N32_PWM_CTRL2_OFFSET),
          pwm_getreg(priv, N32_PWM_CCEN_OFFSET),
          pwm_getreg(priv, N32_PWM_CNT_OFFSET),
          pwm_getreg(priv, N32_PWM_AR_OFFSET),
          pwm_getreg(priv, N32_PWM_PSC_OFFSET));
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct pwm_lowerhalf_s *n32_pwminitialize(int timer)
{
  struct n32_pwmtimer_s *lower = NULL;

  switch (timer)
    {
  #ifdef CONFIG_N32H7_ATIM1_PWM
      case 1:
        lower = &g_pwmatim1dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM2_PWM
      case 2:
        lower = &g_pwmatim2dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM3_PWM
      case 3:
        lower = &g_pwmatim3dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_ATIM4_PWM
      case 4:
        lower = &g_pwmatim4dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA1_PWM
      case 5:
        lower = &g_pwmgtima1dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA2_PWM
      case 6:
        lower = &g_pwmgtima2dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA3_PWM
      case 7:
        lower = &g_pwmgtima3dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA4_PWM
      case 8:
        lower = &g_pwmgtima4dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA5_PWM
      case 9:
        lower = &g_pwmgtima5dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA6_PWM
      case 10:
        lower = &g_pwmgtima6dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMA7_PWM
      case 11:
        lower = &g_pwmgtima7dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB1_PWM
      case 12:
        lower = &g_pwmgtimb1dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB2_PWM
      case 13:
        lower = &g_pwmgtimb2dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
  #ifdef CONFIG_N32H7_GTIMB3_PWM
      case 14:
        lower = &g_pwmgtimb3dev;
  #ifdef CONFIG_PWM_PULSECOUNT
        irq_attach(lower->irq, pwm_irq_handler, NULL);
        up_disable_irq(lower->irq);
  #endif
        break;
  #endif
      default:
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_N32H7_PWM */
