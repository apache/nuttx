/****************************************************************************
 * arch/tricore/src/common/tricore_systimer.c
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "tricore_internal.h"

#include "IfxStm.h"

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
  void                      *tbase;
  uint64_t                   freq;
  uint64_t                   alarm;
  oneshot_callback_t         callback;
  void                       *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tricore_systimer_max_delay(struct oneshot_lowerhalf_s *lower,
                                      struct timespec *ts);
static int tricore_systimer_start(struct oneshot_lowerhalf_s *lower,
                                  oneshot_callback_t callback, void *arg,
                                  const struct timespec *ts);
static int tricore_systimer_cancel(struct oneshot_lowerhalf_s *lower,
                                   struct timespec *ts);
static int tricore_systimer_current(struct oneshot_lowerhalf_s *lower,
                                    struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_tricore_systimer_ops =
{
  .max_delay = tricore_systimer_max_delay,
  .start     = tricore_systimer_start,
  .cancel    = tricore_systimer_cancel,
  .current   = tricore_systimer_current,
};

static struct tricore_systimer_lowerhalf_s g_systimer_lower =
{
  .lower.ops = &g_tricore_systimer_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t
tricore_systimer_get_time(struct tricore_systimer_lowerhalf_s *priv)
{
  irqstate_t flags;
  uint64_t ticks;

  flags = enter_critical_section();

  ticks = IfxStm_get(priv->tbase);

  leave_critical_section(flags);

  return ticks;
}

static void
tricore_systimer_set_timecmp(struct tricore_systimer_lowerhalf_s *priv,
                             uint64_t value)
{
  irqstate_t flags;

  flags = enter_critical_section();

  IfxStm_updateCompare(priv->tbase, IfxStm_Comparator_0, value);

  leave_critical_section(flags);
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
 *   ts      The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int tricore_systimer_max_delay(struct oneshot_lowerhalf_s *lower,
                                      struct timespec *ts)
{
  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;

  return 0;
}

/****************************************************************************
 * Name: tricore_systimer_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   handler The function to call when when the oneshot timer expires.
 *   arg     An opaque argument that will accompany the callback.
 *   ts      Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int tricore_systimer_start(struct oneshot_lowerhalf_s *lower,
                                  oneshot_callback_t callback, void *arg,
                                  const struct timespec *ts)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;
  uint64_t mtime = tricore_systimer_get_time(priv);

  priv->alarm = mtime + ts->tv_sec * priv->freq +
                ts->tv_nsec * priv->freq / NSEC_PER_SEC;
  if (priv->alarm < mtime)
    {
      priv->alarm = UINT64_MAX;
    }

  priv->callback = callback;
  priv->arg      = arg;

  tricore_systimer_set_timecmp(priv, priv->alarm);
  return 0;
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
 *   ts      The location in which to return the time remaining on the
 *           oneshot timer.  A time of zero is returned if the timer is
 *           not running.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int tricore_systimer_cancel(struct oneshot_lowerhalf_s *lower,
                                   struct timespec *ts)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;
  uint64_t mtime;

  tricore_systimer_set_timecmp(priv, UINT64_MAX);

  mtime = tricore_systimer_get_time(priv);
  if (priv->alarm > mtime)
    {
      uint64_t nsec = (priv->alarm - mtime) *
                      NSEC_PER_SEC / priv->freq;

      ts->tv_sec  = nsec / NSEC_PER_SEC;
      ts->tv_nsec = nsec % NSEC_PER_SEC;
    }
  else
    {
      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
    }

  priv->alarm    = 0;
  priv->callback = NULL;
  priv->arg      = NULL;

  return 0;
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
 *   ts      The location in which to return the current time. A time of zero
 *           is returned for the initialization moment.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int tricore_systimer_current(struct oneshot_lowerhalf_s *lower,
                                    struct timespec *ts)
{
  struct tricore_systimer_lowerhalf_s *priv =
    (struct tricore_systimer_lowerhalf_s *)lower;
  uint64_t mtime = tricore_systimer_get_time(priv);
  uint64_t nsec = mtime / (priv->freq / USEC_PER_SEC) * NSEC_PER_USEC;

  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  return 0;
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

  tricore_systimer_set_timecmp(priv, UINT64_MAX);
  if (priv->callback != NULL)
    {
      priv->callback(&priv->lower, priv->arg);
    }

  return 0;
}

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
tricore_systimer_initialize(void *tbase, int irq, uint64_t freq)
{
  struct tricore_systimer_lowerhalf_s *priv = &g_systimer_lower;

  priv->tbase = tbase;
  priv->freq  = freq;

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
