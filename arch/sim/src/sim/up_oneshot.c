/****************************************************************************
 * arch/sim/src/sim/up_oneshot.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <queue.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/arch_alarm.h>

#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver.
 */

struct sim_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct sim_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;  /* Common lower-half driver fields */

  /* Private lower half data follows */

  sq_entry_t link;
  struct timespec alarm;

  oneshot_callback_t callback;    /* internal handler that receives callback */
  FAR void *arg;                  /* Argument that is passed to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sim_process_tick(sq_entry_t *entry);

static int sim_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                         FAR struct timespec *ts);
static int sim_start(FAR struct oneshot_lowerhalf_s *lower,
                     oneshot_callback_t callback, FAR void *arg,
                     FAR const struct timespec *ts);
static int sim_cancel(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts);
static int sim_current(FAR struct oneshot_lowerhalf_s *lower,
                       FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timespec g_current;
static sq_queue_t g_oneshot_list;

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = sim_max_delay,
  .start     = sim_start,
  .cancel    = sim_cancel,
  .current   = sim_current,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_timer_update
 *
 * Description:
 *   Ths function is called periodically to deliver the tick events to the
 *   NuttX simulation.
 *
 ****************************************************************************/

static void sim_timer_update(void)
{
  static const struct timespec tick =
  {
    .tv_sec  = 0,
    .tv_nsec = NSEC_PER_TICK,
  };

  FAR sq_entry_t *entry;

  clock_timespec_add(&g_current, &tick, &g_current);

  for (entry = sq_peek(&g_oneshot_list); entry; entry = sq_next(entry))
    {
      sim_process_tick(entry);
    }
}

/****************************************************************************
 * Name: sim_process_tick
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   entry - Point to the link field of sim_oneshot_lowerhalf_s.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sim_process_tick(sq_entry_t *entry)
{
  FAR struct sim_oneshot_lowerhalf_s *priv =
    container_of(entry, struct sim_oneshot_lowerhalf_s, link);
  oneshot_callback_t callback;
  FAR void *cbarg;

  DEBUGASSERT(priv != NULL);

  if (priv->callback)
    {
      if (clock_timespec_compare(&priv->alarm, &g_current) > 0)
        {
          return; /* Alarm doesn't expire yet */
        }

      /* Sample and nullify BEFORE executing callback (in case the callback
       * restarts the oneshot).
       */

      callback       = priv->callback;
      cbarg          = priv->arg;
      priv->callback = NULL;
      priv->arg      = NULL;

      /* Then perform the callback */

      callback(&priv->lh, cbarg);
    }
}

/****************************************************************************
 * Name: sim_max_delay
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

static int sim_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                         FAR struct timespec *ts)
{
  DEBUGASSERT(lower != NULL && ts != NULL);

  ts->tv_sec  = UINT_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;
  return OK;
}

/****************************************************************************
 * Name: sim_start
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

static int sim_start(FAR struct oneshot_lowerhalf_s *lower,
                     oneshot_callback_t callback, FAR void *arg,
                     FAR const struct timespec *ts)
{
  FAR struct sim_oneshot_lowerhalf_s *priv =
    (FAR struct sim_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  clock_timespec_add(&g_current, ts, &priv->alarm);

  priv->callback = callback;
  priv->arg      = arg;

  return OK;
}

/****************************************************************************
 * Name: sim_cancel
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

static int sim_cancel(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts)
{
  FAR struct sim_oneshot_lowerhalf_s *priv =
    (FAR struct sim_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && ts != NULL);

  clock_timespec_subtract(&priv->alarm, &g_current, ts);

  priv->callback = NULL;
  priv->arg      = NULL;

  return OK;
}

/****************************************************************************
 * Name: sim_current
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

static int sim_current(FAR struct oneshot_lowerhalf_s *lower,
                       FAR struct timespec *ts)
{
#ifdef CONFIG_DEBUG_ASSERTIONS
  FAR struct sim_oneshot_lowerhalf_s *priv =
    (FAR struct sim_oneshot_lowerhalf_s *)lower;
#endif

  DEBUGASSERT(priv != NULL && ts != NULL);

  *ts = g_current;
  return OK;
}

#ifdef CONFIG_SIM_WALLTIME_SIGNAL
/****************************************************************************
 * Name: sim_alarm_handler
 *
 * Description:
 *   The signal handler is called periodically and is used to deliver TICK
 *   events to the OS.
 *
 * Input Parameters:
 *   sig - the signal number
 *   si  - the signal information
 *   old_ucontext - the previous context
 *
 ****************************************************************************/

static int sim_alarm_handler(int irq, FAR void *context, FAR void *arg)
{
  sim_timer_update();
  return OK;
}
#endif /* CONFIG_SIM_WALLTIME_SIGNAL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds.  NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                                   uint16_t resolution)
{
  FAR struct sim_oneshot_lowerhalf_s *priv;

  /* Allocate an instance of the lower half driver */

  priv = (FAR struct sim_oneshot_lowerhalf_s *)
    kmm_zalloc(sizeof(struct sim_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");
      return NULL;
    }

  /* Initialize the lower-half driver structure */

  sq_addlast(&priv->link, &g_oneshot_list);
  priv->lh.ops = &g_oneshot_ops;

  return &priv->lh;
}

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer hardware.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
#ifdef CONFIG_SIM_WALLTIME_SIGNAL
  int host_alarm_irq;

  host_settimer(&host_alarm_irq);

  /* Enable the alarm handler and attach the interrupt to the NuttX logic */

  up_enable_irq(host_alarm_irq);
  irq_attach(host_alarm_irq, sim_alarm_handler, NULL);
#endif

  up_alarm_set_lowerhalf(oneshot_initialize(0, 0));
}

/****************************************************************************
 * Name: up_timer_update
 *
 * Description:
 *   Called from the IDLE loop to fake one timer tick.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_timer_update(void)
{
#ifdef CONFIG_SIM_WALLTIME_SLEEP

  /* Wait a bit so that the timing is close to the correct rate. */

  host_sleepuntil(g_current.tv_nsec +
    (uint64_t)g_current.tv_sec * NSEC_PER_SEC);
#endif

#ifndef CONFIG_SIM_WALLTIME_SIGNAL
  sim_timer_update();
#endif
}
