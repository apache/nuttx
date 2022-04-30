/****************************************************************************
 * arch/sim/src/sim/up_oneshot.c
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

  oneshot_callback_t callback; /* internal handler that receives callback */
  void *arg;                   /* Argument that is passed to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sim_process_tick(sq_entry_t *entry);

static int sim_max_delay(struct oneshot_lowerhalf_s *lower,
                         struct timespec *ts);
static int sim_start(struct oneshot_lowerhalf_s *lower,
                     oneshot_callback_t callback, void *arg,
                     const struct timespec *ts);
static int sim_cancel(struct oneshot_lowerhalf_s *lower,
                      struct timespec *ts);
static int sim_current(struct oneshot_lowerhalf_s *lower,
                       struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
 * Name: sim_timer_current
 *
 * Description:
 *   Get current time from host.
 *
 ****************************************************************************/

static inline void sim_timer_current(struct timespec *ts)
{
  uint64_t nsec;
  time_t sec;

  nsec  = host_gettime(false);
  sec   = nsec / NSEC_PER_SEC;
  nsec -= sec * NSEC_PER_SEC;

  ts->tv_sec  = sec;
  ts->tv_nsec = nsec;
}

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
  sq_entry_t *entry;

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
  struct sim_oneshot_lowerhalf_s *priv =
    container_of(entry, struct sim_oneshot_lowerhalf_s, link);
  oneshot_callback_t callback;
  void *cbarg;

  DEBUGASSERT(priv != NULL);

  if (priv->callback)
    {
      struct timespec current;

      sim_timer_current(&current);
      if (clock_timespec_compare(&priv->alarm, &current) > 0)
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

static int sim_max_delay(struct oneshot_lowerhalf_s *lower,
                         struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

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

static int sim_start(struct oneshot_lowerhalf_s *lower,
                     oneshot_callback_t callback, void *arg,
                     const struct timespec *ts)
{
  struct sim_oneshot_lowerhalf_s *priv =
    (struct sim_oneshot_lowerhalf_s *)lower;
  struct timespec current;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  sim_timer_current(&current);
  clock_timespec_add(&current, ts, &priv->alarm);

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

static int sim_cancel(struct oneshot_lowerhalf_s *lower,
                      struct timespec *ts)
{
  struct sim_oneshot_lowerhalf_s *priv =
    (struct sim_oneshot_lowerhalf_s *)lower;
  struct timespec current;

  DEBUGASSERT(priv != NULL && ts != NULL);

  sim_timer_current(&current);
  clock_timespec_subtract(&priv->alarm, &current, ts);

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

static int sim_current(struct oneshot_lowerhalf_s *lower,
                       struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

  sim_timer_current(ts);

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

static int sim_alarm_handler(int irq, void *context, void *arg)
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

struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                               uint16_t resolution)
{
  struct sim_oneshot_lowerhalf_s *priv;

  /* Allocate an instance of the lower half driver */

  priv = (struct sim_oneshot_lowerhalf_s *)
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
  static uint64_t until;

  /* Wait a bit so that the timing is close to the correct rate. */

  until += NSEC_PER_TICK;
  host_sleepuntil(until);

#ifdef CONFIG_SIM_WALLTIME_SLEEP
  sim_timer_update();
#endif
}
