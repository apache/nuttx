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

#include <nuttx/kmalloc.h>

#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "tricore_internal.h"
#include "tricore_stm.h"

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

#define TRICORE_SYSTIMER_CPU_FREQUENCY 100000000

#define TRICORE_SYSTIMER_MIN_DELAY \
  (40ull * SCU_FREQUENCY / TRICORE_SYSTIMER_CPU_FREQUENCY)

/* STM register of the timer */

#define TRICORE_STM_REG(tbase, offset)  ((tbase) + (offset))

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
  uintptr_t                 tbase;
  uintptr_t                 freerun_tbase;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t tricore_systimer_get_time(uintptr_t tbase)
{
  uint64_t ticks;

#ifdef TRICORE_STM_ABS_OFFSET
  ticks = getreg64(TRICORE_STM_REG(tbase, TRICORE_STM_ABS_OFFSET));
#else
  /* Reading TIM0 latches the upper 32 bits into CAP */

  ticks  = getreg32(TRICORE_STM_REG(tbase, TRICORE_STM_TIM0_OFFSET));
  ticks |= (uint64_t)getreg32(TRICORE_STM_REG(tbase,
                                              TRICORE_STM_CAP_OFFSET)) << 32;
#endif

  return ticks;
}

static void tricore_systimer_set_timecmp(uintptr_t tbase, uint64_t value)
{
  /* CMP0 compares the timer bits [31:0] */

  putreg32((uint32_t)value, TRICORE_STM_REG(tbase, TRICORE_STM_CMP0_OFFSET));
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
 * Name: tricore_systimer_update
 ****************************************************************************/

static void tricore_systimer_update(struct oneshot_lowerhalf_s *lower,
                                    clkcnt_t clkcnt, bool relative)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;
  irqstate_t flags;
  uint64_t   expected;
  uint64_t   current;

  flags = up_irq_save();

  current = tricore_systimer_get_time(priv->tbase);
  expected = relative ? clkcnt + current : clkcnt;

  /* Time synchronization between cores */

  if (priv->freerun_tbase)
    {
      expected = expected + current -
                 tricore_systimer_get_time(priv->freerun_tbase);
    }

  /* Mini_delay time processing */

  expected = expected < current + TRICORE_SYSTIMER_MIN_DELAY ?
             current + TRICORE_SYSTIMER_MIN_DELAY : expected;

  /* The comparator register is 32-bits. */

  DEBUGASSERT(expected <= current + UINT32_MAX);

  tricore_systimer_set_timecmp(priv->tbase, expected);

  up_irq_restore(flags);
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
  tricore_systimer_update(lower, delta, true);
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
  tricore_systimer_update(lower, expected, false);
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

  tricore_systimer_set_timecmp(priv->tbase, UINT64_MAX);
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

  if (priv->freerun_tbase)
    {
      return tricore_systimer_get_time(priv->freerun_tbase);
    }
  else
    {
      return tricore_systimer_get_time(priv->tbase);
    }
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
tricore_systimer_initialize(uintptr_t tbase, uintptr_t fr_tbase,
                            int irq, uint64_t freq)
{
  struct tricore_systimer_lowerhalf_s *priv = &g_tricore_oneshot_lowerhalf;
  uint32_t regval;

  priv->tbase = tbase;
  priv->freerun_tbase = fr_tbase;

  ASSERT(freq <= UINT32_MAX);

  oneshot_count_init(&priv->lower, (uint32_t)freq);

  /* Stop the timer while the core is halted by the debugger */

#ifdef TRICORE_CBS_OSTATE
  if ((getreg32(TRICORE_CBS_OSTATE) & TRICORE_CBS_OSTATE_OEN) != 0)
    {
      putreg32(TRICORE_STM_OCS_SUS_P | TRICORE_STM_OCS_SUS_HARD,
               TRICORE_STM_REG(tbase, TRICORE_STM_OCS_OFFSET));
      regval = getreg32(TRICORE_STM_REG(tbase, TRICORE_STM_OCS_OFFSET));
      putreg32(regval & ~TRICORE_STM_OCS_SUS_P,
               TRICORE_STM_REG(tbase, TRICORE_STM_OCS_OFFSET));
    }
#else
  putreg32(TRICORE_STM_OCS_SUS_P | TRICORE_STM_OCS_SUS_HARD,
           TRICORE_STM_REG(tbase, TRICORE_STM_OCS_OFFSET));
#endif

  /* CMP0 compares the timer bits [31:0] and raises STMIR0 */

  regval  = getreg32(TRICORE_STM_REG(tbase, TRICORE_STM_CMCON_OFFSET));
  regval &= ~(TRICORE_STM_CMCON_MSIZE0 | TRICORE_STM_CMCON_MSTART0);
  regval |= 31 & TRICORE_STM_CMCON_MSIZE0;
  putreg32(regval, TRICORE_STM_REG(tbase, TRICORE_STM_CMCON_OFFSET));

  regval  = getreg32(TRICORE_STM_REG(tbase, TRICORE_STM_ICR_OFFSET));
  putreg32(regval & ~TRICORE_STM_ICR_CMP0OS,
           TRICORE_STM_REG(tbase, TRICORE_STM_ICR_OFFSET));

  putreg32(TRICORE_STM_ISCR_CMP0IRR,
           TRICORE_STM_REG(tbase, TRICORE_STM_ISCR_OFFSET));
  tricore_systimer_set_timecmp(tbase, UINT64_MAX);

  regval = getreg32(TRICORE_STM_REG(tbase, TRICORE_STM_ICR_OFFSET));
  putreg32(regval | TRICORE_STM_ICR_CMP0EN,
           TRICORE_STM_REG(tbase, TRICORE_STM_ICR_OFFSET));

  irq_attach(irq, tricore_systimer_interrupt, priv);
  up_enable_irq(irq);

  return (struct oneshot_lowerhalf_s *)priv;
}
