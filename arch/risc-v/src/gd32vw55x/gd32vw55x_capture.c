/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_capture.c
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
#include <nuttx/timers/capture.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "chip.h"
#include "gd32vw55x_clockconfig.h"
#include "gd32vw55x_gpio.h"
#include "gd32vw55x_capture.h"
#include "hardware/gd32vw55x_rcu.h"
#include "hardware/gd32vw55x_timer.h"

#ifdef CONFIG_GD32VW55X_CAPTURE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer clock.  See gd32vw55x_pwm.c for the details: with TIMERSEL left at
 * its reset value every timer of this port is clocked at CK_AHB.
 */

#define GD32_APB1_TIMER_CLKIN  (GD32VW55X_PCLK1_FREQ * 2)
#define GD32_APB2_TIMER_CLKIN  (GD32VW55X_PCLK2_FREQ)

/* The counter is clocked at 1 MHz, which gives a 1 us resolution and, with
 * the 16-bit counter of TIMER0, a minimum measurable frequency of ~16 Hz.
 */

#define GD32_CAP_TICK_FREQ     1000000

/* Board pin configurations for the channel 0 input */

#ifndef GPIO_TIMER0_CH0IN
#  define GPIO_TIMER0_CH0IN    0
#endif
#ifndef GPIO_TIMER1_CH0IN
#  define GPIO_TIMER1_CH0IN    0
#endif
#ifndef GPIO_TIMER2_CH0IN
#  define GPIO_TIMER2_CH0IN    0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_cap_s
{
  const struct cap_ops_s *ops;  /* Capture operations (must be first) */
  uint8_t  timid;               /* Timer ID {0,1,2} */
  uint8_t  irq;                 /* Capture/compare interrupt number */
  bool     timer32;             /* 32-bit counter (TIMER1/2) */
  bool     running;             /* Capture is running */
  uint32_t base;                /* Timer register base address */
  uint32_t syscfg;              /* SYSCFG_TIMERxCFG register address */
  uint32_t pclk;                /* Frequency of the timer clock */
  uint32_t pincfg;              /* Channel 0 input pin configuration */
  uint32_t period;              /* Last captured period, in ticks */
  uint32_t pulse;               /* Last captured pulse width, in ticks */
  uint32_t edges;               /* Number of captured rising edges */
#ifdef CONFIG_CAPTURE_NOTIFY
  capture_notify_t cb;          /* Edge notification callback */
  void    *priv;                /* Private data of the callback */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t cap_getreg(struct gd32_cap_s *priv, int offset);
static void cap_putreg(struct gd32_cap_s *priv, int offset, uint32_t value);
static void cap_modifyreg(struct gd32_cap_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits);

/* Helpers */

static void cap_setclock(struct gd32_cap_s *priv, bool on);
static int  cap_interrupt(int irq, void *context, void *arg);

/* Capture driver methods */

static int cap_start(struct cap_lowerhalf_s *lower);
static int cap_stop(struct cap_lowerhalf_s *lower);
static int cap_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty);
static int cap_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq);
static int cap_getedges(struct cap_lowerhalf_s *lower, uint32_t *edges);
static int cap_ioctl(struct cap_lowerhalf_s *lower, int cmd,
                     unsigned long arg);
#ifdef CONFIG_CAPTURE_NOTIFY
static int cap_bind(struct cap_lowerhalf_s *lower, enum cap_type_e type,
                    capture_notify_t cb, void *priv);
static int cap_unbind(struct cap_lowerhalf_s *lower);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct cap_ops_s g_capops =
{
  .start     = cap_start,
  .stop      = cap_stop,
  .getduty   = cap_getduty,
  .getfreq   = cap_getfreq,
  .getedges  = cap_getedges,
  .ioctl     = cap_ioctl,
#ifdef CONFIG_CAPTURE_NOTIFY
  .bind      = cap_bind,
  .unbind    = cap_unbind,
#endif
};

#ifdef CONFIG_GD32VW55X_TIMER0
static struct gd32_cap_s g_cap0 =
{
  .ops       = &g_capops,
  .timid     = 0,
  .irq       = GD32VW55X_IRQ_TIMER0_CC,
  .timer32   = false,
  .base      = GD32VW55X_TIMER0_BASE,
  .syscfg    = GD32VW55X_SYSCFG_TIMER0CFG,
  .pclk      = GD32_APB2_TIMER_CLKIN,
  .pincfg    = GPIO_TIMER0_CH0IN,
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER1
static struct gd32_cap_s g_cap1 =
{
  .ops       = &g_capops,
  .timid     = 1,
  .irq       = GD32VW55X_IRQ_TIMER1,
  .timer32   = true,
  .base      = GD32VW55X_TIMER1_BASE,
  .syscfg    = GD32VW55X_SYSCFG_TIMER1CFG,
  .pclk      = GD32_APB1_TIMER_CLKIN,
  .pincfg    = GPIO_TIMER1_CH0IN,
};
#endif

#ifdef CONFIG_GD32VW55X_TIMER2
static struct gd32_cap_s g_cap2 =
{
  .ops       = &g_capops,
  .timid     = 2,
  .irq       = GD32VW55X_IRQ_TIMER2,
  .timer32   = true,
  .base      = GD32VW55X_TIMER2_BASE,
  .syscfg    = GD32VW55X_SYSCFG_TIMER2CFG,
  .pclk      = GD32_APB1_TIMER_CLKIN,
  .pincfg    = GPIO_TIMER2_CH0IN,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cap_getreg
 ****************************************************************************/

static uint32_t cap_getreg(struct gd32_cap_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: cap_putreg
 ****************************************************************************/

static void cap_putreg(struct gd32_cap_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: cap_modifyreg
 ****************************************************************************/

static void cap_modifyreg(struct gd32_cap_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  modifyreg32(priv->base + offset, clrbits, setbits);
}

/****************************************************************************
 * Name: cap_setclock
 *
 * Description:
 *   Enable or disable the RCU clock of the timer.  The SYSCFG clock is
 *   needed too because the slave mode controller is configured there.
 *
 ****************************************************************************/

static void cap_setclock(struct gd32_cap_s *priv, bool on)
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

      default:
        return;
    }

  if (on)
    {
      modifyreg32(GD32VW55X_RCU_APB2EN, 0, RCU_APB2EN_SYSCFGEN);
      modifyreg32(regaddr, 0, bit);
    }
  else
    {
      modifyreg32(regaddr, bit, 0);
    }
}

/****************************************************************************
 * Name: cap_interrupt
 *
 * Description:
 *   Channel 0 capture interrupt handler.  A capture on channel 0 means that
 *   a full period has been measured.
 *
 ****************************************************************************/

static int cap_interrupt(int irq, void *context, void *arg)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)arg;
  uint32_t intf;

  intf = cap_getreg(priv, GD32VW55X_TIMER_INTF_OFFSET);

  if ((intf & TIMER_INTF_CH0IF) != 0)
    {
      /* Clear the channel 0 capture and overcapture flags.  The flags of
       * this timer are cleared by writing zero to them.
       */

      cap_modifyreg(priv, GD32VW55X_TIMER_INTF_OFFSET,
                    TIMER_INTF_CH0IF | TIMER_INTF_CH0OF, 0);

      /* Channel 0 holds the period and channel 1 the pulse width.  The
       * counter starts at zero after the slave mode controller resets it,
       * so one tick has to be added to both values.
       */

      priv->period = cap_getreg(priv, GD32VW55X_TIMER_CHCV_OFFSET(0)) + 1;
      priv->pulse  = cap_getreg(priv, GD32VW55X_TIMER_CHCV_OFFSET(1)) + 1;
      priv->edges++;

#ifdef CONFIG_CAPTURE_NOTIFY
      if (priv->cb != NULL)
        {
          priv->cb((struct cap_lowerhalf_s *)priv, priv->priv);
        }
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: cap_start
 *
 * Description:
 *   Configure the timer in PWM input mode and start the capture.
 *
 ****************************************************************************/

static int cap_start(struct cap_lowerhalf_s *lower)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;
  uint32_t prescaler;
  uint32_t chctl0;
  int ret;

  /* Enable the timer clock and configure the input pin */

  cap_setclock(priv, true);

  if (priv->pincfg != 0)
    {
      gd32_gpio_config(priv->pincfg);
    }

  /* Stop the counter while it is being configured */

  cap_putreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_DMAINTEN_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_INTF_OFFSET, 0);

  priv->period = 0;
  priv->pulse  = 0;
  priv->edges  = 0;

  /* Free running counter clocked at GD32_CAP_TICK_FREQ */

  prescaler = priv->pclk / GD32_CAP_TICK_FREQ;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > TIMER_PSC_MAX + 1)
    {
      prescaler = TIMER_PSC_MAX + 1;
    }

  cap_putreg(priv, GD32VW55X_TIMER_PSC_OFFSET, prescaler - 1);
  cap_putreg(priv, GD32VW55X_TIMER_CAR_OFFSET,
             priv->timer32 ? TIMER_CAR_MAX32 : TIMER_CAR_MAX16);
  cap_putreg(priv, GD32VW55X_TIMER_CNT_OFFSET, 0);

  /* PWM input mode:
   *
   *   Channel 0 is mapped directly on CI0 and captures the rising edges,
   *   which gives the period of the signal.
   *   Channel 1 is mapped on the same input (indirect selection) and
   *   captures the falling edges, which gives the width of the pulse.
   */

  chctl0 = (TIMER_CHCTL_CHMS_DIRECTTI << TIMER_CHCTL_SHIFT(0)) |
           (TIMER_CHCTL_CHMS_INDIRECTT << TIMER_CHCTL_SHIFT(1));

  cap_putreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET, chctl0);
  cap_putreg(priv, GD32VW55X_TIMER_CHCTL1_OFFSET, 0);

  /* Channel 0 on the rising edge, channel 1 on the falling edge */

  cap_putreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET,
             TIMER_CHCTL2_CHEN(0) | TIMER_CHCTL2_CHEN(1) |
             TIMER_CHCTL2_CHP(1));

  /* Reset the counter on every rising edge of CI0.  The slave mode of this
   * family is selected from the SYSCFG block, not from TIMER_SMCFG.
   */

  putreg32(SYSCFG_TIMERCFG(TIMER_SLAVE_MODE_RESTART, TIMER_TRGSEL_CI0FE0),
           priv->syscfg);

  /* Attach and enable the capture interrupt */

  ret = irq_attach(priv->irq, cap_interrupt, priv);
  if (ret < 0)
    {
      cperr("ERROR: Failed to attach IRQ %u\n", priv->irq);
      return ret;
    }

  up_enable_irq(priv->irq);

  cap_putreg(priv, GD32VW55X_TIMER_DMAINTEN_OFFSET, TIMER_DMAINTEN_CH0IE);

  /* Start the counter */

  cap_modifyreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, 0, TIMER_CTL0_CEN);

  priv->running = true;

  return OK;
}

/****************************************************************************
 * Name: cap_stop
 *
 * Description:
 *   Stop the capture.
 *
 ****************************************************************************/

static int cap_stop(struct cap_lowerhalf_s *lower)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;

  /* Stop the counter and the captures */

  cap_putreg(priv, GD32VW55X_TIMER_CTL0_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_CHCTL2_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_DMAINTEN_OFFSET, 0);
  cap_putreg(priv, GD32VW55X_TIMER_INTF_OFFSET, 0);

  /* Leave the slave mode controller disabled */

  putreg32(SYSCFG_TIMERCFG(TIMER_SLAVE_MODE_DISABLE, TIMER_TRGSEL_NONE),
           priv->syscfg);

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  cap_setclock(priv, false);

  priv->running = false;

  return OK;
}

/****************************************************************************
 * Name: cap_getduty
 *
 * Description:
 *   Return the duty cycle of the captured signal, in percent.
 *
 ****************************************************************************/

static int cap_getduty(struct cap_lowerhalf_s *lower, uint8_t *duty)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;
  irqstate_t flags;
  uint32_t period;
  uint32_t pulse;

  flags  = enter_critical_section();
  period = priv->period;
  pulse  = priv->pulse;
  leave_critical_section(flags);

  if (period == 0)
    {
      *duty = 0;
    }
  else
    {
      if (pulse > period)
        {
          pulse = period;
        }

      *duty = (uint8_t)((pulse * 100) / period);
    }

  return OK;
}

/****************************************************************************
 * Name: cap_getfreq
 *
 * Description:
 *   Return the frequency of the captured signal, in Hz.
 *
 ****************************************************************************/

static int cap_getfreq(struct cap_lowerhalf_s *lower, uint32_t *freq)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;
  irqstate_t flags;
  uint32_t period;

  flags  = enter_critical_section();
  period = priv->period;
  leave_critical_section(flags);

  if (period == 0)
    {
      *freq = 0;
    }
  else
    {
      *freq = GD32_CAP_TICK_FREQ / period;
    }

  return OK;
}

/****************************************************************************
 * Name: cap_getedges
 *
 * Description:
 *   Return the number of rising edges seen since the capture was started.
 *
 ****************************************************************************/

static int cap_getedges(struct cap_lowerhalf_s *lower, uint32_t *edges)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;

  *edges = priv->edges;

  return OK;
}

/****************************************************************************
 * Name: cap_ioctl
 *
 * Description:
 *   Lower half logic may support platform-specific ioctl commands.
 *
 ****************************************************************************/

static int cap_ioctl(struct cap_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
  struct gd32_cap_s *priv = (struct gd32_cap_s *)lower;
  int ret = OK;

  switch (cmd)
    {
      case CAPIOC_CLR_CNT:
        {
          irqstate_t flags = enter_critical_section();

          priv->edges  = 0;
          priv->period = 0;
          priv->pulse  = 0;

          leave_critical_section(flags);
        }
        break;

      case CAPIOC_PULSES:
        {
          int *count = (int *)((uintptr_t)arg);

          DEBUGASSERT(count != NULL);
          *count = (int)priv->edges;
        }
        break;

      case CAPIOC_FILTER:
        {
          /* The glitch filter samples the input at fDTS, which is the timer
           * clock here.  Convert the requested duration to the number of
           * samples of the input capture filter.
           */

          uint32_t ns  = (uint32_t)arg;
          uint32_t flt = (uint32_t)(((uint64_t)ns * priv->pclk) /
                                    1000000000ull);
          uint32_t chctl0;

          if (flt > 15)
            {
              flt = 15;
            }

          chctl0  = cap_getreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET);
          chctl0 &= ~(TIMER_CHCTL_CHCAPFLT_MASK << TIMER_CHCTL_SHIFT(0));
          chctl0 |= (flt << TIMER_CHCTL_CHCAPFLT_SHIFT) <<
                    TIMER_CHCTL_SHIFT(0);
          cap_putreg(priv, GD32VW55X_TIMER_CHCTL0_OFFSET, chctl0);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifdef CONFIG_CAPTURE_NOTIFY

/****************************************************************************
 * Name: cap_bind
 *
 * Description:
 *   Bind the capture edge notification callback.
 *
 ****************************************************************************/

static int cap_bind(struct cap_lowerhalf_s *lower, enum cap_type_e type,
                    capture_notify_t cb, void *priv)
{
  struct gd32_cap_s *cap = (struct gd32_cap_s *)lower;
  irqstate_t flags;

  /* Only the rising edge of channel 0 is reported: it is the edge that
   * completes the measurement of a period.
   */

  if (type != CAP_TYPE_RISING)
    {
      return -ENOSYS;
    }

  flags     = enter_critical_section();
  cap->cb   = cb;
  cap->priv = priv;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: cap_unbind
 *
 * Description:
 *   Un-bind the capture edge notification callback.
 *
 ****************************************************************************/

static int cap_unbind(struct cap_lowerhalf_s *lower)
{
  struct gd32_cap_s *cap = (struct gd32_cap_s *)lower;
  irqstate_t flags;

  flags     = enter_critical_section();
  cap->cb   = NULL;
  cap->priv = NULL;
  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_cap_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper level capture driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use: {0,1,2}
 *
 * Returned Value:
 *   On success, a pointer to the lower half capture driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct cap_lowerhalf_s *gd32_cap_initialize(int timer)
{
  struct gd32_cap_s *lower;

  cpinfo("TIMER%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_GD32VW55X_TIMER0
      case 0:
        lower = &g_cap0;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER1
      case 1:
        lower = &g_cap1;
        break;
#endif

#ifdef CONFIG_GD32VW55X_TIMER2
      case 2:
        lower = &g_cap2;
        break;
#endif

      /* TIMER5 has no channel and TIMER15/TIMER16 have neither a second
       * channel nor a slave mode controller, so they cannot be used in PWM
       * input mode.
       */

      default:
        cperr("ERROR: No such timer configured: %d\n", timer);
        return NULL;
    }

  return (struct cap_lowerhalf_s *)lower;
}

#endif /* CONFIG_GD32VW55X_CAPTURE */
