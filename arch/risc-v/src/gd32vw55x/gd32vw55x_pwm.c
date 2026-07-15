/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_pwm.c
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
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_gpio.h"
#include "gd32vw55x_pwm.h"
#include "hardware/gd32vw55x_rcu.h"
#include "hardware/gd32vw55x_timer.h"

#ifdef CONFIG_GD32VW55X_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer clock.
 *
 * The RCU_CFG1 TIMERSEL bit is left at its reset value (0).  With that
 * setting the timer clock is equal to CK_AHB whenever the APB prescaler is
 * 1 or 2.  This port uses APB1 = CK_AHB/2 and APB2 = CK_AHB, so all of the
 * timers, on both APB domains, are clocked at CK_AHB (160 MHz).
 */

#define GD32_APB1_TIMER_CLKIN  (GD32VW55X_PCLK1_FREQ * 2)
#define GD32_APB2_TIMER_CLKIN  (GD32VW55X_PCLK2_FREQ)

/* Maximum number of output channels of a single timer */

#define GD32_PWM_CHAN_MAX      4

/* Board GPIO pin configurations.  A pin that the board does not route is
 * left undefined and the corresponding output is not configured.
 */

#ifndef GPIO_TIMER0_CH0OUT
#  define GPIO_TIMER0_CH0OUT   0
#endif
#ifndef GPIO_TIMER0_CH1OUT
#  define GPIO_TIMER0_CH1OUT   0
#endif
#ifndef GPIO_TIMER0_CH2OUT
#  define GPIO_TIMER0_CH2OUT   0
#endif
#ifndef GPIO_TIMER0_CH3OUT
#  define GPIO_TIMER0_CH3OUT   0
#endif
#ifndef GPIO_TIMER0_CH0NOUT
#  define GPIO_TIMER0_CH0NOUT  0
#endif
#ifndef GPIO_TIMER0_CH1NOUT
#  define GPIO_TIMER0_CH1NOUT  0
#endif
#ifndef GPIO_TIMER0_CH2NOUT
#  define GPIO_TIMER0_CH2NOUT  0
#endif
#ifndef GPIO_TIMER1_CH0OUT
#  define GPIO_TIMER1_CH0OUT   0
#endif
#ifndef GPIO_TIMER1_CH1OUT
#  define GPIO_TIMER1_CH1OUT   0
#endif
#ifndef GPIO_TIMER1_CH2OUT
#  define GPIO_TIMER1_CH2OUT   0
#endif
#ifndef GPIO_TIMER1_CH3OUT
#  define GPIO_TIMER1_CH3OUT   0
#endif
#ifndef GPIO_TIMER2_CH0OUT
#  define GPIO_TIMER2_CH0OUT   0
#endif
#ifndef GPIO_TIMER2_CH1OUT
#  define GPIO_TIMER2_CH1OUT   0
#endif
#ifndef GPIO_TIMER2_CH2OUT
#  define GPIO_TIMER2_CH2OUT   0
#endif
#ifndef GPIO_TIMER2_CH3OUT
#  define GPIO_TIMER2_CH3OUT   0
#endif
#ifndef GPIO_TIMER15_CH0OUT
#  define GPIO_TIMER15_CH0OUT  0
#endif
#ifndef GPIO_TIMER15_CH0NOUT
#  define GPIO_TIMER15_CH0NOUT 0
#endif
#ifndef GPIO_TIMER16_CH0OUT
#  define GPIO_TIMER16_CH0OUT  0
#endif
#ifndef GPIO_TIMER16_CH0NOUT
#  define GPIO_TIMER16_CH0NOUT 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One output channel of a timer */

struct gd32_pwmchan_s
{
  uint8_t  channel;  /* Timer channel: 0..3 */
  uint32_t pincfg;   /* Output pin configuration (0 if not routed) */
  uint32_t npincfg;  /* Complementary output pin (0 if none) */
};

/* This structure represents the state of one PWM timer */

struct gd32_pwmtimer_s
{
  const struct pwm_ops_s *ops;     /* PWM operations (must be first) */
  uint8_t  timid;                  /* Timer ID {0,1,2,15,16} */
  uint8_t  nchannels;              /* Number of channels of this timer */
  bool     advanced;               /* Has break/dead-time (TIMER0/15/16) */
  bool     timer32;                /* 32-bit counter (TIMER1/2) */
  uint32_t base;                   /* Timer register base address */
  uint32_t pclk;                   /* Frequency of the timer clock */
  struct gd32_pwmchan_s channels[GD32_PWM_CHAN_MAX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct gd32_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct gd32_pwmtimer_s *priv, int offset,
                       uint32_t value);
static void pwm_modifyreg(struct gd32_pwmtimer_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits);

/* Helpers */

static void pwm_setclock(struct gd32_pwmtimer_s *priv, bool on);
static int  pwm_timer(struct gd32_pwmtimer_s *priv,
                      const struct pwm_info_s *info);
static void pwm_disable_outputs(struct gd32_pwmtimer_s *priv);

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
  .start       = pwm_start,
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_GD32VW55X_TIMER0
static struct gd32_pwmtimer_s g_pwm0 =
{
  .ops         = &g_pwmops,
  .timid       = 0,
  .nchannels   = 4,
  .advanced    = true,
  .timer32     = false,
  .base        = GD32VW55X_TIMER0_BASE,
  .pclk        = GD32_APB2_TIMER_CLKIN,
  .channels    =
  {
    {
      .channel = 0,
      .pincfg  = GPIO_TIMER0_CH0OUT,
      .npincfg = GPIO_TIMER0_CH0NOUT,
    },
    {
      .channel = 1,
      .pincfg  = GPIO_TIMER0_CH1OUT,
      .npincfg = GPIO_TIMER0_CH1NOUT,
    },
    {
      .channel = 2,
      .pincfg  = GPIO_TIMER0_CH2OUT,
      .npincfg = GPIO_TIMER0_CH2NOUT,
    },
    {
      .channel = 3,
      .pincfg  = GPIO_TIMER0_CH3OUT,
      .npincfg = 0,
    },
  },
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER1
static struct gd32_pwmtimer_s g_pwm1 =
{
  .ops         = &g_pwmops,
  .timid       = 1,
  .nchannels   = 4,
  .advanced    = false,
  .timer32     = true,
  .base        = GD32VW55X_TIMER1_BASE,
  .pclk        = GD32_APB1_TIMER_CLKIN,
  .channels    =
  {
    {
      .channel = 0,
      .pincfg  = GPIO_TIMER1_CH0OUT,
      .npincfg = 0,
    },
    {
      .channel = 1,
      .pincfg  = GPIO_TIMER1_CH1OUT,
      .npincfg = 0,
    },
    {
      .channel = 2,
      .pincfg  = GPIO_TIMER1_CH2OUT,
      .npincfg = 0,
    },
    {
      .channel = 3,
      .pincfg  = GPIO_TIMER1_CH3OUT,
      .npincfg = 0,
    },
  },
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER2
static struct gd32_pwmtimer_s g_pwm2 =
{
  .ops         = &g_pwmops,
  .timid       = 2,
  .nchannels   = 4,
  .advanced    = false,
  .timer32     = true,
  .base        = GD32VW55X_TIMER2_BASE,
  .pclk        = GD32_APB1_TIMER_CLKIN,
  .channels    =
  {
    {
      .channel = 0,
      .pincfg  = GPIO_TIMER2_CH0OUT,
      .npincfg = 0,
    },
    {
      .channel = 1,
      .pincfg  = GPIO_TIMER2_CH1OUT,
      .npincfg = 0,
    },
    {
      .channel = 2,
      .pincfg  = GPIO_TIMER2_CH2OUT,
      .npincfg = 0,
    },
    {
      .channel = 3,
      .pincfg  = GPIO_TIMER2_CH3OUT,
      .npincfg = 0,
    },
  },
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER15
static struct gd32_pwmtimer_s g_pwm15 =
{
  .ops         = &g_pwmops,
  .timid       = 15,
  .nchannels   = 1,
  .advanced    = true,
  .timer32     = false,
  .base        = GD32VW55X_TIMER15_BASE,
  .pclk        = GD32_APB2_TIMER_CLKIN,
  .channels    =
  {
    {
      .channel = 0,
      .pincfg  = GPIO_TIMER15_CH0OUT,
      .npincfg = GPIO_TIMER15_CH0NOUT,
    },
  },
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER16
static struct gd32_pwmtimer_s g_pwm16 =
{
  .ops         = &g_pwmops,
  .timid       = 16,
  .nchannels   = 1,
  .advanced    = true,
  .timer32     = false,
  .base        = GD32VW55X_TIMER16_BASE,
  .pclk        = GD32_APB2_TIMER_CLKIN,
  .channels    =
  {
    {
      .channel = 0,
      .pincfg  = GPIO_TIMER16_CH0OUT,
      .npincfg = GPIO_TIMER16_CH0NOUT,
    },
  },
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_getreg
 ****************************************************************************/

static uint32_t pwm_getreg(struct gd32_pwmtimer_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: pwm_putreg
 ****************************************************************************/

static void pwm_putreg(struct gd32_pwmtimer_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: pwm_modifyreg
 ****************************************************************************/

static void pwm_modifyreg(struct gd32_pwmtimer_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  modifyreg32(priv->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: pwm_setclock
 *
 * Description:
 *   Enable or disable the RCU clock of the timer.
 *
 ****************************************************************************/

static void pwm_setclock(struct gd32_pwmtimer_s *priv, bool on)
{
  uint32_t regaddr;
  uint32_t bit;

  switch (priv->timid)
    {
      case 0:
        regaddr = GD32VW55X_RCU_APB2EN;
        bit     = RCU_APB2EN_TIMER0EN;
        break;

      case 1:
        regaddr = GD32VW55X_RCU_APB1EN;
        bit     = RCU_APB1EN_TIMER1EN;
        break;

      case 2:
        regaddr = GD32VW55X_RCU_APB1EN;
        bit     = RCU_APB1EN_TIMER2EN;
        break;

      case 15:
        regaddr = GD32VW55X_RCU_APB2EN;
        bit     = RCU_APB2EN_TIMER15EN;
        break;

      case 16:
        regaddr = GD32VW55X_RCU_APB2EN;
        bit     = RCU_APB2EN_TIMER16EN;
        break;

      default:
        return;
    }

  if (on)
    {
      modifyreg32(regaddr, 0, bit);
    }
  else
    {
      modifyreg32(regaddr, bit, 0);
    }
}

/****************************************************************************
 * Name: pwm_disable_outputs
 *
 * Description:
 *   Disable every output of the timer and stop the counter.
 *
 ****************************************************************************/

static void pwm_disable_outputs(struct gd32_pwmtimer_s *priv)
{
  /* Disable the counter and all of the channel outputs */

  pwm_modifyreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, TIMER_CTL0_CEN, 0);
  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET, 0);

  /* Disable the primary output of the advanced timers */

  if (priv->advanced)
    {
      pwm_modifyreg(priv, GD32VW55X_TIMER_CCHP_OFFSET, TIMER_CCHP_POEN, 0);
    }
}

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)configure the timer resolution and the channel duty cycles.
 *
 ****************************************************************************/

static int pwm_timer(struct gd32_pwmtimer_s *priv,
                     const struct pwm_info_s *info)
{
  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint64_t period;
  uint64_t carmax;
  uint32_t chctl0;
  uint32_t chctl1;
  uint32_t chctl2;
  uint32_t ctl0;
  int      i;
  int      j;

  if (info->frequency == 0)
    {
      return -EINVAL;
    }

  carmax = priv->timer32 ? (uint64_t)TIMER_CAR_MAX32 :
                           (uint64_t)TIMER_CAR_MAX16;

  /* Select the smallest prescaler that brings the period within the range
   * of the counter:
   *
   *   reload = pclk / (prescaler * frequency) - 1 <= carmax
   */

  prescaler = (uint32_t)(((uint64_t)priv->pclk / info->frequency + carmax) /
                         (carmax + 1));
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > TIMER_PSC_MAX + 1)
    {
      prescaler = TIMER_PSC_MAX + 1;
    }

  timclk = priv->pclk / prescaler;

  /* Number of timer ticks of one period of the pulse train */

  period = (uint64_t)timclk / info->frequency;
  if (period < 1)
    {
      period = 1;
    }
  else if (period > carmax + 1)
    {
      period = carmax + 1;
    }

  reload = (uint32_t)(period - 1);

  pwminfo("TIMER%d frequency: %" PRIu32 " prescaler: %" PRIu32
          " reload: %" PRIu32 "\n",
          priv->timid, info->frequency, prescaler, reload);

  /* Stop the timer while it is being reconfigured */

  pwm_modifyreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, TIMER_CTL0_CEN, 0);

  /* Edge aligned, up counting, auto-reload preloaded */

  ctl0  = pwm_getreg(priv, GD32VW55X_TIMER_CTL0_OFFSET);
  ctl0 &= ~(TIMER_CTL0_DIR | TIMER_CTL0_CAM_MASK | TIMER_CTL0_CKDIV_MASK |
            TIMER_CTL0_SPM);
  ctl0 |= TIMER_CTL0_CAM_EDGE | TIMER_CTL0_CKDIV_DIV1 | TIMER_CTL0_ARSE;
  pwm_putreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, ctl0);

  pwm_putreg(priv, GD32VW55X_TIMER_PSC_OFFSET, prescaler - 1);
  pwm_putreg(priv, GD32VW55X_TIMER_CAR_OFFSET, reload);

  /* No repetition on the timers that have a repetition counter */

  if (priv->advanced)
    {
      pwm_putreg(priv, GD32VW55X_TIMER_CREP_OFFSET, 0);
    }

  chctl0 = pwm_getreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET);
  chctl1 = pwm_getreg(priv, GD32VW55X_TIMER_CHCTL1_OFFSET);
  chctl2 = pwm_getreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET);

  /* Configure the requested channels */

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      struct gd32_pwmchan_s *chan = NULL;
      uint32_t chctl;
      uint32_t ccr;
      ub16_t   duty;
      int8_t   channel;
      int      index;

      duty = info->channels[i].duty;

#ifdef CONFIG_PWM_MULTICHAN
      channel = info->channels[i].channel;

      /* A channel number of zero or less terminates the list */

      if (channel <= 0)
        {
          continue;
        }
#else
      /* Without multi-channel support only the first channel of the timer
       * is driven.
       */

      channel = priv->channels[0].channel + 1;
#endif

      /* The upper half numbers the channels from 1 */

      for (j = 0; j < priv->nchannels; j++)
        {
          if (priv->channels[j].channel == (uint8_t)(channel - 1))
            {
              chan = &priv->channels[j];
              break;
            }
        }

      if (chan == NULL)
        {
          pwmerr("ERROR: TIMER%u has no channel %d\n",
                 priv->timid, channel);
          return -EINVAL;
        }

      index = chan->channel;

      /* Duty cycle:
       *
       *   ccr / (reload + 1) = duty / 65536
       */

      ccr = (uint32_t)((((uint64_t)reload + 1) * (uint64_t)duty) >> 16);
      if ((uint64_t)ccr > (uint64_t)reload + 1)
        {
          ccr = reload;
        }

      /* PWM mode 0 with an output compare shadow register */

      chctl = TIMER_CHCTL_CHMS_OUTPUT | TIMER_CHCTL_CHCOMCTL_PWM0 |
              TIMER_CHCTL_CHCOMSEN;

      if (index < 2)
        {
          chctl0 &= ~TIMER_CHCTL_MASK(index);
          chctl0 |= chctl << TIMER_CHCTL_SHIFT(index);
        }
      else
        {
          chctl1 &= ~TIMER_CHCTL_MASK(index);
          chctl1 |= chctl << TIMER_CHCTL_SHIFT(index);
        }

      /* Output polarity and output enable */

      chctl2 &= ~TIMER_CHCTL2_MASK(index);
      chctl2 |= TIMER_CHCTL2_CHEN(index);

#ifdef CONFIG_PWM_MULTICHAN
      if (info->channels[i].cpol == PWM_CPOL_LOW)
        {
          chctl2 |= TIMER_CHCTL2_CHP(index);
        }
#endif

      /* Enable the complementary output when the board routes it */

      if (chan->npincfg != 0)
        {
          chctl2 |= TIMER_CHCTL2_CHNEN(index);
        }

      pwm_putreg(priv, GD32VW55X_TIMER_CHCV_OFFSET(index), ccr);
    }

  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET, chctl0);
  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL1_OFFSET, chctl1);
  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET, chctl2);

#ifdef CONFIG_PWM_DEADTIME
  /* The dead time is common to every complementary output of the timer.
   * Only the advanced timers have the CCHP register.  The value is given
   * in dead-time clock ticks (fDTS = the timer clock here).
   */

  if (priv->advanced)
    {
      uint32_t dtcfg = ub16toi(info->channels[0].dead_time_a);

      if (dtcfg > TIMER_CCHP_DTCFG_MASK)
        {
          dtcfg = TIMER_CCHP_DTCFG_MASK;
        }

      pwm_modifyreg(priv, GD32VW55X_TIMER_CCHP_OFFSET,
                    TIMER_CCHP_DTCFG_MASK, dtcfg);
    }
#endif

  /* Enable the primary output of the advanced timers */

  if (priv->advanced)
    {
      pwm_modifyreg(priv, GD32VW55X_TIMER_CCHP_OFFSET, 0, TIMER_CCHP_POEN);
    }

  /* Generate an update event to load the shadow registers, then start the
   * counter.
   */

  pwm_putreg(priv, GD32VW55X_TIMER_SWEVG_OFFSET, TIMER_SWEVG_UPG);
  pwm_modifyreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, 0, TIMER_CTL0_CEN);

  return OK;
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   configures and initializes the device so that it is ready for use.  It
 *   does not start the timer.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  int i;

  /* Enable the timer clock */

  pwm_setclock(priv, true);

  /* Configure the output pins that the board routes */

  for (i = 0; i < priv->nchannels; i++)
    {
      if (priv->channels[i].pincfg != 0)
        {
          gd32_gpio_config(priv->channels[i].pincfg);
        }

      if (priv->channels[i].npincfg != 0)
        {
          gd32_gpio_config(priv->channels[i].npincfg);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stops the pulsed output and puts the timer in its reset state.
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  /* Make sure that the outputs are inactive */

  pwm_stop(dev);

  /* Turn the timer clock off */

  pwm_setclock(priv, false);

  return OK;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output.
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;

  return pwm_timer(priv, info);
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct gd32_pwmtimer_s *priv = (struct gd32_pwmtimer_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();

  pwm_disable_outputs(priv);

  /* Reset the channel configuration and the time base */

  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET, 0);
  pwm_putreg(priv, GD32VW55X_TIMER_CHCTL1_OFFSET, 0);
  pwm_putreg(priv, GD32VW55X_TIMER_DMAINTEN_OFFSET, 0);
  pwm_putreg(priv, GD32VW55X_TIMER_INTF_OFFSET, 0);
  pwm_putreg(priv, GD32VW55X_TIMER_CNT_OFFSET, 0);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower half logic may support platform-specific ioctl commands.
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  /* There are no platform-specific ioctl commands */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use: {0,1,2,15,16}
 *
 * Returned Value:
 *   On success, a pointer to the lower half PWM driver is returned.  NULL
 *   is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *gd32_pwminitialize(int timer)
{
  struct gd32_pwmtimer_s *lower;

  pwminfo("TIMER%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_GD32VW55X_TIMER0
      case 0:
        lower = &g_pwm0;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER1
      case 1:
        lower = &g_pwm1;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER2
      case 2:
        lower = &g_pwm2;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER15
      case 15:
        lower = &g_pwm15;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER16
      case 16:
        lower = &g_pwm16;
        break;
#endif

      /* TIMER5 is a basic timer without any output channel */

      default:
        pwmerr("ERROR: No such timer configured: %d\n", timer);
        return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_GD32VW55X_PWM */
