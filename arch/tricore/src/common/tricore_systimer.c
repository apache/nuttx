/****************************************************************************
 * arch/tricore/src/common/tricore_systimer.c
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

#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>

#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "tricore_internal.h"

#include "IfxStm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Since the tricore hardware timer triggers an interrupt only when the
 * compare value is equal to the counter, setting a compare value that has
 * already timed out will not trigger an interrupt. To avoid missing
 * interrupts when setting the timer, we should set a minimum delay.
 * The minimum delay is calculated based on the CPU frequency and the timer
 * frequency. We assume that the worst-case execution time for setting the
 * timer does not exceed 40 CPU cycles, and calculate the minimum timer
 * delay accordingly.
 * 40 CPU cycles (100ns at 400Mhz) ~ 10 timer cycles (for 100 Mhz timer).
 */

#define TRICORE_SYSTIMER_MIN_DELAY \
  (40ull * SCU_FREQUENCY / IFX_CFG_CPU_CLOCK_FREQUENCY)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct tricore_systimer_lowerhalf_s
{
  struct oneshot_lowerhalf_s lower;
  volatile void             *tbase;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t
tricore_systimer_get_time(struct tricore_systimer_lowerhalf_s *priv)
{
  irqstate_t flags;
  uint64_t ticks;

  flags = spin_lock_irqsave(&priv->lock);

  ticks = IfxStm_get(priv->tbase);

  spin_unlock_irqrestore(&priv->lock, flags);

  return ticks;
}

static void
tricore_systimer_set_timecmp(struct tricore_systimer_lowerhalf_s *priv,
                             uint64_t value)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  IfxStm_updateCompare(priv->tbase, IfxStm_Comparator_0, value);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: tricore_systimer_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *
 * Returned Value:
 *   The maximum delay value.
 *
 ****************************************************************************/

static clkcnt_t tricore_systimer_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT32_MAX;
}

/****************************************************************************
 * Name: tricore_systimer_start
 *
 * Description:
 *   Start the oneshot timer. Note that the tricore systimer is special, the
 *   IRQ is only triggered when timecmp == mtime, so we should avoid the case
 *   that we miss the timecmp.
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   delta   Provides the duration of delta count.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tricore_systimer_start(struct oneshot_lowerhalf_s *lower,
                                   clkcnt_t delta)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;
  irqstate_t flags;
  uint64_t   mtime;

  delta = delta < TRICORE_SYSTIMER_MIN_DELAY ?
                  TRICORE_SYSTIMER_MIN_DELAY : delta;
  flags = up_irq_save();
  mtime = tricore_systimer_get_time(priv);

  tricore_systimer_set_timecmp(priv, mtime + delta);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: tricore_systimer_start_absolute
 *
 * Description:
 *   Start the oneshot timer. Note that the tricore systimer is special, the
 *   IRQ is only triggered when timecmp == mtime, so we should avoid the case
 *   that we miss the timecmp.
 *
 * Input Parameters:
 *   lower    An instance of the lower-half oneshot state structure.  This
 *            structure must have been previously initialized via a call to
 *            oneshot_initialize();
 *   expected Target
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void
tricore_systimer_start_absolute(struct oneshot_lowerhalf_s *lower,
                                clkcnt_t expected)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;

  irqstate_t flags = up_irq_save();
  uint64_t min_expected = tricore_systimer_get_time(priv) +
                          TRICORE_SYSTIMER_MIN_DELAY;
  expected = expected < min_expected ? min_expected : expected;
  tricore_systimer_set_timecmp(priv, expected);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: tricore_systimer_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void tricore_systimer_cancel(struct oneshot_lowerhalf_s *lower)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;

  tricore_systimer_set_timecmp(priv, UINT64_MAX);
}

/****************************************************************************
 * Name: tricore_systimer_current
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *
 * Returned Value:
 *   Current timer count.
 *
 ****************************************************************************/

static clkcnt_t tricore_systimer_current(struct oneshot_lowerhalf_s *lower)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;

  return tricore_systimer_get_time(priv);
}

/****************************************************************************
 * Name: tricore_systimer_interrupt
 *
 * Description:
 *   This function is software interrupt handler to proceed
 *   the system timer interrupt.
 *
 ****************************************************************************/

static int tricore_systimer_interrupt(int irq, void *context, void *arg)
{
  struct tricore_systimer_lowerhalf_s *priv = arg;

  /* We do not need to clear the compare register here. */

  oneshot_process_callback(&priv->lower);

  return 0;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_tricore_oneshot_ops =
{
  .current        = tricore_systimer_current,
  .start          = tricore_systimer_start,
  .start_absolute = tricore_systimer_start_absolute,
  .cancel         = tricore_systimer_cancel,
  .max_delay      = tricore_systimer_max_delay
};

static struct tricore_systimer_lowerhalf_s g_tricore_oneshot_lowerhalf =
{
  .lower.ops = &g_tricore_oneshot_ops
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_systimer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *
tricore_systimer_initialize(volatile void *tbase, int irq, uint64_t freq)
{
  struct tricore_systimer_lowerhalf_s *priv = &g_tricore_oneshot_lowerhalf;

  priv->tbase = tbase;

  ASSERT(freq <= UINT32_MAX);

  oneshot_count_init(&priv->lower, (uint32_t)freq);

  IfxStm_setCompareControl(tbase,
      IfxStm_Comparator_0,
      IfxStm_ComparatorOffset_0,
      IfxStm_ComparatorSize_32Bits,
      IfxStm_ComparatorInterrupt_ir0);

  IfxStm_clearCompareFlag(tbase, IfxStm_Comparator_0);
  tricore_systimer_set_timecmp(priv, UINT64_MAX);
  IfxStm_enableComparatorInterrupt(tbase, IfxStm_Comparator_0);

  irq_attach(irq, tricore_systimer_interrupt, priv);
  up_enable_irq(irq);

  return (struct oneshot_lowerhalf_s *)priv;
}
