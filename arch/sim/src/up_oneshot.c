/****************************************************************************
 *  arch/sim/src/up_oneshot.c
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

#include <stdint.h>
#include <time.h>
#include <limits.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver */

struct sim_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct sim_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;  /* Common lower-half driver fields */

  /* Private lower half data follows */

  WDOG_ID wdog;                   /* Simulates oneshot timer */
  oneshot_callback_t callback;    /* internal handler that receives callback */
  FAR void *arg;                  /* Argument that is passed to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sim_oneshot_handler(int argc, wdparm_t arg1, ...);

static int sim_max_delay(FAR struct oneshot_lowerhalf_s *lower,
                         FAR struct timespec *ts);
static int sim_start(FAR struct oneshot_lowerhalf_s *lower,
                     oneshot_callback_t callback, FAR void *arg,
                     FAR const struct timespec *ts);
static int sim_cancel(FAR struct oneshot_lowerhalf_s *lower,
                      FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = sim_max_delay,
  .start     = sim_start,
  .cancel    = sim_cancel,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_oneshot_handler
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when sim_oneshot_start()
 *         was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sim_oneshot_handler(int argc, wdparm_t arg1, ...)
{
  FAR struct sim_oneshot_lowerhalf_s *priv =
    (FAR struct sim_oneshot_lowerhalf_s *)arg1;
  oneshot_callback_t callback;
  FAR void *cbarg;

  DEBUGASSERT(argc == 1 && priv != NULL);

  /* Perhaps the callback was nullified in a race condition with
   * sim_cancel?
   */

  if (priv->callback)
    {
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
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the maxumum delay.
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

  ts->tv_sec  = INT_MAX;
  ts->tv_nsec = 1000000000ul - 1;
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
  clock_t ticks;
  int64_t nsec;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  /* Convert time to ticks */

  nsec = (int64_t)ts->tv_sec * NSEC_PER_SEC +
         (int64_t)ts->tv_nsec;
  ticks = (clock_t)((nsec + NSEC_PER_TICK - 1) / NSEC_PER_TICK);

  /* Save the callback information and start the timer */

  priv->callback = callback;
  priv->arg      = arg;

  return wd_start(priv->wdog, ticks, (wdentry_t)sim_oneshot_handler,
                  1, (wdparm_t)priv);
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
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  ret            = wd_cancel(priv->wdog);
  priv->callback = NULL;
  priv->arg      = NULL;

  return ret;
}

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

  priv->lh.ops = &g_oneshot_ops;

  /* Initialize the contained watchdog timer */

  priv->wdog = wd_create();
  if (priv->wdog == NULL)
    {
      tmrerr("ERROR: Failed to create wdog\n");
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}