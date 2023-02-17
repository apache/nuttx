/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_oneshot_lowerhalf.c
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
#include <nuttx/timers/oneshot.h>

#include "bm3803_oneshot.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half
 * driver
 */

struct bm3803_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct bm3803_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh; /* Common lower-half driver fields */

  /* Private lower half data follows */

  /* STM32-specific oneshot state */

  struct bm3803_oneshot_s oneshot;
  oneshot_callback_t callback;   /* internal handler that receives callback */
  void *arg;                     /* Argument that is passed to the handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void bm3803_oneshot_handler(void *arg);

static int bm3803_max_delay(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts);
static int bm3803_start(struct oneshot_lowerhalf_s *lower,
                         oneshot_callback_t callback, void *arg,
                         const struct timespec *ts);
static int bm3803_cancel(struct oneshot_lowerhalf_s *lower,
                          struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .max_delay = bm3803_max_delay,
  .start     = bm3803_start,
  .cancel    = bm3803_cancel,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_oneshot_handler
 *
 * Description:
 *   Timer expiration handler
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when bm3803_oneshot_start()
 *         was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void bm3803_oneshot_handler(void *arg)
{
  struct bm3803_oneshot_lowerhalf_s *priv =
    (struct bm3803_oneshot_lowerhalf_s *)arg;
  oneshot_callback_t callback;
  void *cbarg;

  DEBUGASSERT(priv != NULL);

  /* Perhaps the callback was nullified in a race condition with
   * bm3803_cancel?
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
 * Name: bm3803_max_delay
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

static int bm3803_max_delay(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts)
{
  struct bm3803_oneshot_lowerhalf_s *priv =
    (struct bm3803_oneshot_lowerhalf_s *)lower;
  uint64_t usecs;
  int ret;

  DEBUGASSERT(priv != NULL && ts != NULL);
  ret = bm3803_oneshot_max_delay(&priv->oneshot, &usecs);
  if (ret >= 0)
    {
      uint64_t sec = usecs / 1000000;
      usecs -= 1000000 * sec;

      ts->tv_sec  = (time_t)sec;
      ts->tv_nsec = (long)(usecs * 1000);
    }

  return ret;
}

/****************************************************************************
 * Name: bm3803_start
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

static int bm3803_start(struct oneshot_lowerhalf_s *lower,
                       oneshot_callback_t callback, void *arg,
                       const struct timespec *ts)
{
  struct bm3803_oneshot_lowerhalf_s *priv =
    (struct bm3803_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL && callback != NULL && ts != NULL);

  /* Save the callback information and start the timer */

  flags          = enter_critical_section();
  priv->callback = callback;
  priv->arg      = arg;
  ret            = bm3803_oneshot_start(&priv->oneshot,
                                       bm3803_oneshot_handler, priv, ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_oneshot_start failed: %d\n", flags);
    }

  return ret;
}

/****************************************************************************
 * Name: bm3803_cancel
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

static int bm3803_cancel(struct oneshot_lowerhalf_s *lower,
                        struct timespec *ts)
{
  struct bm3803_oneshot_lowerhalf_s *priv =
    (struct bm3803_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags          = enter_critical_section();
  ret            = bm3803_oneshot_cancel(&priv->oneshot, ts);
  priv->callback = NULL;
  priv->arg      = NULL;
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_oneshot_cancel failed: %d\n", flags);
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

struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                               uint16_t resolution)
{
  struct bm3803_oneshot_lowerhalf_s *priv;
  int ret;

  /* Allocate an instance of the lower half driver */

  priv = (struct bm3803_oneshot_lowerhalf_s *)
    kmm_zalloc(sizeof(struct bm3803_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");
      return NULL;
    }

  /* Initialize the lower-half driver structure */

  priv->lh.ops = &g_oneshot_ops;

  /* Initialize the contained STM32 oneshot timer */

  ret = bm3803_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_oneshot_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
