/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_timer.c
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
 *
 * A /dev/timerN driver on top of an RP2350 system timer block (TIMER0 or
 * TIMER1).  Each block is a free-running 64-bit counter incremented once per
 * microsecond (the 1 MHz tick is set up by the TICKS block in
 * rp23xx_clock.c) and independent of the ARM SysTick that drives the OS
 * tick, so it is free for application timers.  This driver uses ALARM0 of
 * the selected block, which matches against the low 32 bits of the counter,
 * giving a single-shot or periodic timeout with 1 us resolution and a
 * maximum interval of 2^32 - 1 us (~71.5 minutes).
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/timer.h>

#include "chip.h"
#include "arm_internal.h"
#include "rp23xx_timer.h"
#include "hardware/rp23xx_timer.h"
#include "hardware/rp23xx_memorymap.h"

#ifdef CONFIG_RP23XX_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This driver uses ALARM0 of the block; alarm 0 is bit 0 in the ARMED,
 * INTE, INTF, INTR and INTS registers.
 */

#define TIMER_ALARM0_BIT  (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_timer_lowerhalf_s
{
  const struct timer_ops_s *ops;      /* Lower-half operations (must be 1st) */
  uint32_t                  base;     /* TIMER block base address */
  int                       irq;      /* ALARM0 interrupt number */
  tccb_t                    callback; /* Current user interrupt callback */
  void                     *arg;      /* Argument passed to upper half cb */
  uint32_t                  timeout;  /* The current timeout value (us) */
  bool                      started;  /* True if the alarm is armed */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rp23xx_timer_start(FAR struct timer_lowerhalf_s *lower);
static int  rp23xx_timer_stop(FAR struct timer_lowerhalf_s *lower);
static int  rp23xx_timer_getstatus(FAR struct timer_lowerhalf_s *lower,
                                   FAR struct timer_status_s *status);
static int  rp23xx_timer_settimeout(FAR struct timer_lowerhalf_s *lower,
                                    uint32_t timeout);
static void rp23xx_timer_setcallback(FAR struct timer_lowerhalf_s *lower,
                                     tccb_t callback, FAR void *arg);
static int  rp23xx_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                    FAR uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_rp23xx_timer_ops =
{
  .start       = rp23xx_timer_start,
  .stop        = rp23xx_timer_stop,
  .getstatus   = rp23xx_timer_getstatus,
  .settimeout  = rp23xx_timer_settimeout,
  .setcallback = rp23xx_timer_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = rp23xx_timer_maxtimeout,
};

static struct rp23xx_timer_lowerhalf_s g_rp23xx_timer_lowerhalf[2] =
{
  {
    .ops = &g_rp23xx_timer_ops,
  },
  {
    .ops = &g_rp23xx_timer_ops,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_timer_interrupt
 *
 * Description:
 *   The ALARM0 interrupt handler.  Acknowledges the alarm, invokes the
 *   upper-half callback and, for a periodic timer, re-arms the alarm for the
 *   next expiry.
 *
 ****************************************************************************/

static int rp23xx_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv = arg;
  uint32_t next;
  uint32_t now;
  uint32_t target;

  /* Acknowledge the alarm (write-1-clear of the latched interrupt).  The
   * alarm auto-disarms in hardware when it fires.
   */

  putreg32(TIMER_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);

  if (priv->callback != NULL)
    {
      next = priv->timeout;
      if (priv->callback(&next, priv->arg) && next > 0)
        {
          /* Periodic: schedule the next expiry relative to the previous
           * target so the period does not drift with interrupt latency, but
           * never behind the counter -- an alarm set in the past would not
           * match until the 32-bit microsecond counter wraps ~71 minutes
           * later.
           */

          priv->timeout = next;
          now    = getreg32(priv->base + RP23XX_TIMER_TIMERAWL_OFFSET);
          target = getreg32(priv->base + RP23XX_TIMER_ALARM0_OFFSET) + next;

          if ((int32_t)(target - now) <= 0)
            {
              target = now + next;
            }

          putreg32(target, priv->base + RP23XX_TIMER_ALARM0_OFFSET);
        }
      else
        {
          rp23xx_timer_stop((FAR struct timer_lowerhalf_s *)priv);
        }
    }
  else
    {
      rp23xx_timer_stop((FAR struct timer_lowerhalf_s *)priv);
    }

  return OK;
}

/****************************************************************************
 * Name: rp23xx_timer_start
 ****************************************************************************/

static int rp23xx_timer_start(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv =
    (FAR struct rp23xx_timer_lowerhalf_s *)lower;
  irqstate_t flags;
  uint32_t now;

  if (priv->timeout == 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  now = getreg32(priv->base + RP23XX_TIMER_TIMERAWL_OFFSET);
  putreg32(now + priv->timeout, priv->base + RP23XX_TIMER_ALARM0_OFFSET);
  modifyreg32(priv->base + RP23XX_TIMER_INTE_OFFSET, 0, TIMER_ALARM0_BIT);
  priv->started = true;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_timer_stop
 ****************************************************************************/

static int rp23xx_timer_stop(FAR struct timer_lowerhalf_s *lower)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv =
    (FAR struct rp23xx_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Disable the interrupt, force-disarm the alarm (writing a 1 to the ARMED
   * bit disarms it) and clear any latched status.
   */

  modifyreg32(priv->base + RP23XX_TIMER_INTE_OFFSET, TIMER_ALARM0_BIT, 0);
  putreg32(TIMER_ALARM0_BIT, priv->base + RP23XX_TIMER_ARMED_OFFSET);
  putreg32(TIMER_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);
  priv->started = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_timer_getstatus
 ****************************************************************************/

static int rp23xx_timer_getstatus(FAR struct timer_lowerhalf_s *lower,
                                  FAR struct timer_status_s *status)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv =
    (FAR struct rp23xx_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();

  status->flags = 0;
  if (priv->started)
    {
      status->flags |= TCFLAGS_ACTIVE;
    }

  if (priv->callback != NULL)
    {
      status->flags |= TCFLAGS_HANDLER;
    }

  status->timeout = priv->timeout;

  if (priv->started)
    {
      uint32_t now    = getreg32(priv->base + RP23XX_TIMER_TIMERAWL_OFFSET);
      uint32_t target = getreg32(priv->base + RP23XX_TIMER_ALARM0_OFFSET);

      /* Unsigned subtraction wraps mod 2^32, matching the counter. */

      status->timeleft = target - now;
    }
  else
    {
      status->timeleft = 0;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_timer_settimeout
 ****************************************************************************/

static int rp23xx_timer_settimeout(FAR struct timer_lowerhalf_s *lower,
                                   uint32_t timeout)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv =
    (FAR struct rp23xx_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  if (timeout == 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  priv->timeout = timeout;
  if (priv->started)
    {
      uint32_t now = getreg32(priv->base + RP23XX_TIMER_TIMERAWL_OFFSET);
      putreg32(now + timeout, priv->base + RP23XX_TIMER_ALARM0_OFFSET);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_timer_setcallback
 ****************************************************************************/

static void rp23xx_timer_setcallback(FAR struct timer_lowerhalf_s *lower,
                                     tccb_t callback, FAR void *arg)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv =
    (FAR struct rp23xx_timer_lowerhalf_s *)lower;
  irqstate_t flags;

  flags = enter_critical_section();
  priv->callback = callback;
  priv->arg      = arg;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: rp23xx_timer_maxtimeout
 ****************************************************************************/

static int rp23xx_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower,
                                   FAR uint32_t *maxtimeout)
{
  UNUSED(lower);

  /* ALARM0 matches the low 32 bits of the microsecond counter. */

  *maxtimeout = UINT32_MAX;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_timer_initialize
 *
 * Description:
 *   Bind TIMER0 or TIMER1 to a /dev/timerN device.  See rp23xx_timer.h.
 *
 ****************************************************************************/

int rp23xx_timer_initialize(FAR const char *devpath, int instance)
{
  FAR struct rp23xx_timer_lowerhalf_s *priv;
  FAR void *handle;

  if (instance != 0 && instance != 1)
    {
      return -EINVAL;
    }

  priv = &g_rp23xx_timer_lowerhalf[instance];

  priv->base     = (instance == 0) ? RP23XX_TIMER0_BASE : RP23XX_TIMER1_BASE;
  priv->irq      = (instance == 0) ?
                   RP23XX_TIMER0_IRQ_0 : RP23XX_TIMER1_IRQ_0;
  priv->callback = NULL;
  priv->arg      = NULL;
  priv->timeout  = 0;
  priv->started  = false;

  /* Start from a known-idle state: interrupt disabled, alarm disarmed, any
   * latched status cleared.
   */

  modifyreg32(priv->base + RP23XX_TIMER_INTE_OFFSET, TIMER_ALARM0_BIT, 0);
  putreg32(TIMER_ALARM0_BIT, priv->base + RP23XX_TIMER_ARMED_OFFSET);
  putreg32(TIMER_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);

  irq_attach(priv->irq, rp23xx_timer_interrupt, priv);
  up_enable_irq(priv->irq);

  handle = timer_register(devpath, (FAR struct timer_lowerhalf_s *)priv);
  if (handle == NULL)
    {
      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
      return -EEXIST;
    }

  return OK;
}

#endif /* CONFIG_RP23XX_TIMER */
