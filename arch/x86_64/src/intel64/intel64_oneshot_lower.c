/****************************************************************************
 * arch/x86_64/src/intel64/intel64_oneshot_lower.c
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
#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/oneshot.h>

#include "intel64_oneshot.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver
 */

struct intel64_oneshot_lowerhalf_s
{
  struct oneshot_lowerhalf_s  lh;
  struct intel64_oneshot_s    oneshot;
  oneshot_callback_t          callback;
  void                       *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void intel64_oneshot_handler(void *arg);

static int intel64_max_delay(struct oneshot_lowerhalf_s *lower,
                           struct timespec *ts);
static int intel64_start(struct oneshot_lowerhalf_s *lower,
                       oneshot_callback_t callback, void *arg,
                       const struct timespec *ts);
static int intel64_cancel(struct oneshot_lowerhalf_s *lower,
                        struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = intel64_max_delay,
  .start     = intel64_start,
  .cancel    = intel64_cancel,
};

static spinlock_t g_oneshotlow_spin;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: intel64_oneshot_handler
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when intel64_oneshot_start()
 *         was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void intel64_oneshot_handler(void *arg)
{
  struct intel64_oneshot_lowerhalf_s *priv = arg;
  oneshot_callback_t  callback;
  void               *cbarg;

  DEBUGASSERT(priv != NULL);

  /* Perhaps the callback was nullified in a race condition with
   * intel64_cancel?
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
 * Name: intel64_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
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

static int intel64_max_delay(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  uint64_t usecs;
  uint64_t sec;
  int      ret;

  DEBUGASSERT(priv != NULL && ts != NULL);
  ret = intel64_oneshot_max_delay(&priv->oneshot, &usecs);
  if (ret >= 0)
    {
      sec = usecs / 1000000;
      usecs -= 1000000 * sec;

      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (long)(usecs * 1000);
    }

  return ret;
}

/****************************************************************************
 * Name: intel64_start
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

static int intel64_start(struct oneshot_lowerhalf_s *lower,
                         oneshot_callback_t callback, void *arg,
                         const struct timespec *ts)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int        ret;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  /* Save the callback information and start the timer */

  flags          = spin_lock_irqsave(&g_oneshotlow_spin);
  priv->callback = callback;
  priv->arg      = arg;
  ret            = intel64_oneshot_start(&priv->oneshot,
                                         intel64_oneshot_handler,
                                         priv, ts);
  spin_unlock_irqrestore(&g_oneshotlow_spin, flags);

  if (ret < 0)
    {
      tmrerr("ERROR: intel64_oneshot_start failed\n");
    }

  return ret;
}

/****************************************************************************
 * Name: intel64_cancel
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

static int intel64_cancel(struct oneshot_lowerhalf_s *lower,
                          struct timespec *ts)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int        ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags          = spin_lock_irqsave(&g_oneshotlow_spin);
  ret            = intel64_oneshot_cancel(&priv->oneshot, ts);
  priv->callback = NULL;
  priv->arg      = NULL;
  spin_unlock_irqrestore(&g_oneshotlow_spin, flags);

  if (ret < 0)
    {
      tmrerr("ERROR: intel64_oneshot_cancel failed\n");
    }

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

struct oneshot_lowerhalf_s *oneshot_initialize(int chan, uint16_t resolution)
{
  struct intel64_oneshot_lowerhalf_s *priv;
  int                                 ret;

  /* Allocate an instance of the lower half driver */

  priv = (struct intel64_oneshot_lowerhalf_s *)
    kmm_zalloc(sizeof(struct intel64_oneshot_lowerhalf_s));
  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");
      return NULL;
    }

  /* Initialize the lower-half driver structure */

  priv->lh.ops = &g_oneshot_ops;

  /* Initialize the contained INTEL64 oneshot timer */

  ret = intel64_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("ERROR: intel64_oneshot_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
