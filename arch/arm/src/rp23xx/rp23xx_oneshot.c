/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_oneshot.c
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
 * A oneshot lower-half backed by an RP2350 system timer block (TIMER0 or
 * TIMER1, selected by CONFIG_RP23XX_SYSTIMER_TICKLESS_TIMERn), used to drive
 * the tickless scheduler via up_alarm_set_lowerhalf().  The block is a
 * free-running 64-bit counter incremented once per microsecond (the 1 MHz
 * tick is set up by the TICKS block in rp23xx_clock.c), and serves directly
 * as the monotonic time base returned by current().  The other block is left
 * free for the /dev/timer driver.
 *
 * The one wrinkle versus a full 64-bit compare timer (e.g. the RISC-V mtimer
 * this mirrors) is that the RP2350 ALARM registers match only the low 32
 * bits of the counter.  max_delay() is therefore capped below 2^32 counts so
 * the scheduler never asks for a longer interval, and any deadline that is
 * already due (or that the counter reaches while the alarm is being armed)
 * is raised at once through the INTF force register rather than waiting a
 * full 32-bit wrap (~71.5 minutes) for the compare to match again.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/oneshot.h>

#include "chip.h"
#include "arm_internal.h"
#include "rp23xx_oneshot.h"
#include "hardware/rp23xx_timer.h"
#include "hardware/rp23xx_memorymap.h"

#ifdef CONFIG_RP23XX_SYSTIMER_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This driver uses ALARM0 of TIMER0; alarm 0 is bit 0 in the ARMED, INTE,
 * INTF, INTR and INTS registers.
 */

#define ONESHOT_ALARM0_BIT  (1 << 0)

/* The counter is clocked at 1 MHz (1 us per count). */

#define ONESHOT_FREQ_HZ     1000000

/* The ALARM compare is only 32 bits wide, so never schedule further ahead
 * than this many counts.
 */

#define ONESHOT_MAX_COUNTS  ((clkcnt_t)UINT32_MAX)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Cast-compatible with struct oneshot_lowerhalf_s (embedded first). */

struct rp23xx_oneshot_lowerhalf_s
{
  struct oneshot_lowerhalf_s lower;   /* Lower-half instance (must be first) */
  uint32_t                   base;    /* TIMER block base address */
  int                        irq;     /* ALARM0 interrupt number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t rp23xx_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *l);
static clkcnt_t rp23xx_oneshot_current(FAR struct oneshot_lowerhalf_s *l);
static void rp23xx_oneshot_start_absolute(FAR struct oneshot_lowerhalf_s *l,
                                          clkcnt_t expected);
static void rp23xx_oneshot_start(FAR struct oneshot_lowerhalf_s *l,
                                 clkcnt_t delta);
static void rp23xx_oneshot_cancel(FAR struct oneshot_lowerhalf_s *l);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_rp23xx_oneshot_ops =
{
  .current        = rp23xx_oneshot_current,
  .start          = rp23xx_oneshot_start,
  .start_absolute = rp23xx_oneshot_start_absolute,
  .cancel         = rp23xx_oneshot_cancel,
  .max_delay      = rp23xx_oneshot_max_delay,
};

static struct rp23xx_oneshot_lowerhalf_s g_rp23xx_oneshot =
{
  .lower.ops = &g_rp23xx_oneshot_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_oneshot_count
 *
 * Description:
 *   Read the free-running 64-bit microsecond counter, guarding against a
 *   carry from the low word into the high word between the two reads.
 *
 ****************************************************************************/

static uint64_t
rp23xx_oneshot_count(FAR struct rp23xx_oneshot_lowerhalf_s *p)
{
  uint32_t hi = getreg32(p->base + RP23XX_TIMER_TIMERAWH_OFFSET);
  uint32_t lo;
  uint32_t next;

  for (; ; )
    {
      lo   = getreg32(p->base + RP23XX_TIMER_TIMERAWL_OFFSET);
      next = getreg32(p->base + RP23XX_TIMER_TIMERAWH_OFFSET);
      if (hi == next)
        {
          break;
        }

      hi = next;
    }

  return ((uint64_t)hi << 32) | lo;
}

/****************************************************************************
 * Name: rp23xx_oneshot_setcompare
 *
 * Description:
 *   Arm ALARM0 for the absolute counter value "target".  Must be called with
 *   interrupts disabled.  If the target is already due -- or the counter
 *   reaches it while the alarm is being armed -- the interrupt is forced
 *   immediately through INTF instead of waiting a full 32-bit wrap for the
 *   compare to match.
 *
 ****************************************************************************/

static void
rp23xx_oneshot_setcompare(FAR struct rp23xx_oneshot_lowerhalf_s *priv,
                          uint64_t target)
{
  /* Clear any stale force left from a previous expiry. */

  modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, ONESHOT_ALARM0_BIT, 0);

  if ((int64_t)(target - rp23xx_oneshot_count(priv)) <= 0)
    {
      /* Already due: raise the interrupt now. */

      modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, 0,
                  ONESHOT_ALARM0_BIT);
    }
  else
    {
      /* Writing ALARM0 arms it against the low 32 bits of the counter. */

      putreg32((uint32_t)target, priv->base + RP23XX_TIMER_ALARM0_OFFSET);

      /* Close the race: if the counter reached the target while we were
       * arming, force so we do not wait a full wrap for the match.
       */

      if ((int64_t)(rp23xx_oneshot_count(priv) - target) >= 0)
        {
          modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, 0,
                      ONESHOT_ALARM0_BIT);
        }
    }
}

/****************************************************************************
 * Name: rp23xx_oneshot_interrupt
 ****************************************************************************/

static int rp23xx_oneshot_interrupt(int irq, FAR void *context,
                                    FAR void *arg)
{
  FAR struct rp23xx_oneshot_lowerhalf_s *priv = arg;

  /* Disarm the alarm, drop any force and clear the latched match. */

  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_ARMED_OFFSET);
  modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);

  oneshot_process_callback(&priv->lower);
  return OK;
}

/****************************************************************************
 * Name: rp23xx_oneshot_max_delay
 ****************************************************************************/

static clkcnt_t rp23xx_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *l)
{
  UNUSED(l);
  return ONESHOT_MAX_COUNTS;
}

/****************************************************************************
 * Name: rp23xx_oneshot_current
 ****************************************************************************/

static clkcnt_t rp23xx_oneshot_current(FAR struct oneshot_lowerhalf_s *l)
{
  return rp23xx_oneshot_count((FAR struct rp23xx_oneshot_lowerhalf_s *)l);
}

/****************************************************************************
 * Name: rp23xx_oneshot_start_absolute
 ****************************************************************************/

static void rp23xx_oneshot_start_absolute(FAR struct oneshot_lowerhalf_s *l,
                                          clkcnt_t expected)
{
  FAR struct rp23xx_oneshot_lowerhalf_s *priv =
    (FAR struct rp23xx_oneshot_lowerhalf_s *)l;
  irqstate_t flags = up_irq_save();

  rp23xx_oneshot_setcompare(priv, expected);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rp23xx_oneshot_start
 ****************************************************************************/

static void rp23xx_oneshot_start(FAR struct oneshot_lowerhalf_s *l,
                                 clkcnt_t delta)
{
  FAR struct rp23xx_oneshot_lowerhalf_s *priv =
    (FAR struct rp23xx_oneshot_lowerhalf_s *)l;
  irqstate_t flags = up_irq_save();

  rp23xx_oneshot_setcompare(priv, rp23xx_oneshot_count(priv) + delta);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rp23xx_oneshot_cancel
 ****************************************************************************/

static void rp23xx_oneshot_cancel(FAR struct oneshot_lowerhalf_s *l)
{
  FAR struct rp23xx_oneshot_lowerhalf_s *priv =
    (FAR struct rp23xx_oneshot_lowerhalf_s *)l;
  irqstate_t flags = up_irq_save();

  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_ARMED_OFFSET);
  modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);
  up_irq_restore(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_oneshot_initialize
 *
 * Description:
 *   Initialise the selected system timer block (TIMER0 or TIMER1) as a
 *   oneshot lower-half.  See rp23xx_oneshot.h.
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *rp23xx_oneshot_initialize(void)
{
  FAR struct rp23xx_oneshot_lowerhalf_s *priv = &g_rp23xx_oneshot;

#ifdef CONFIG_RP23XX_SYSTIMER_TICKLESS_TIMER1
  priv->base = RP23XX_TIMER1_BASE;
  priv->irq  = RP23XX_TIMER1_IRQ_0;
#else
  priv->base = RP23XX_TIMER0_BASE;
  priv->irq  = RP23XX_TIMER0_IRQ_0;
#endif

  oneshot_count_init(&priv->lower, ONESHOT_FREQ_HZ);

  /* Start from a known-idle state: alarm disarmed, no force, status clear,
   * then enable the ALARM0 interrupt at the peripheral and the NVIC.
   */

  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_ARMED_OFFSET);
  modifyreg32(priv->base + RP23XX_TIMER_INTF_OFFSET, ONESHOT_ALARM0_BIT, 0);
  putreg32(ONESHOT_ALARM0_BIT, priv->base + RP23XX_TIMER_INTR_OFFSET);
  modifyreg32(priv->base + RP23XX_TIMER_INTE_OFFSET, 0, ONESHOT_ALARM0_BIT);

  irq_attach(priv->irq, rp23xx_oneshot_interrupt, priv);
  up_enable_irq(priv->irq);

  return &priv->lower;
}

#endif /* CONFIG_RP23XX_SYSTIMER_TICKLESS */
