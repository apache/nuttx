/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_oneshot_lowerhalf.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/kmalloc.h>

#include "esp32s3_oneshot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_oneshot_lowerhalf_s
{
  /* This is the part of the lower-half driver that is visible to the upper-
   * half client of the driver. This must be the first thing in this
   * struct so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct esp32s3_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;    /* Lower-half instance */
  struct esp32s3_oneshot_s oneshot; /* ESP32-S3-specific oneshot state */
  oneshot_callback_t callback;      /* Upper-half Interrupt callback */
  void *arg;                        /* Argument passed to handler */
  uint16_t resolution;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void oneshot_lh_handler(void *arg);

/* "Lower-half" driver methods **********************************************/

static int oneshot_lh_max_delay(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts);
static int oneshot_lh_start(struct oneshot_lowerhalf_s *lower,
                            oneshot_callback_t callback,
                            void *arg,
                            const struct timespec *ts);
static int oneshot_lh_cancel(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts);
static int oneshot_lh_current(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower-half" driver methods */

static const struct oneshot_operations_s g_esp32s3_timer_ops =
{
  .max_delay = oneshot_lh_max_delay,
  .start     = oneshot_lh_start,
  .cancel    = oneshot_lh_cancel,
  .current   = oneshot_lh_current
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_lh_handler
 *
 * Description:
 *   Timer expiration handler.
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when esp32s3_oneshot_start()
 *         was called.
 *
 ****************************************************************************/

static void oneshot_lh_handler(void *arg)
{
  struct esp32s3_oneshot_lowerhalf_s *priv =
    (struct esp32s3_oneshot_lowerhalf_s *)arg;
  oneshot_callback_t callback;
  FAR void *cb_arg;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->callback != NULL);

  tmrinfo("Oneshot LH handler triggered\n");

  /* Sample and nullify BEFORE executing callback (in case the callback
   * restarts the oneshot).
   */

  callback       = priv->callback;
  cb_arg         = priv->arg;
  priv->callback = NULL;
  priv->arg      = NULL;

  /* Then perform the callback */

  callback(&priv->lh, cb_arg);
}

/****************************************************************************
 * Name: oneshot_lh_max_delay
 *
 * Description:
 *   Determine the maximum delay of the oneshot timer (in microseconds).
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure. This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize().
 *   ts      The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int oneshot_lh_max_delay(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

  /* The real maximum delay surpass the limit that timespec can represent.
   * Even using the better case: a resolution of 1 us.
   * Therefore, here, set the timespec with the maximum value it represent.
   */

  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;

  tmrinfo("max sec=%" PRIu32 "\n", ts->tv_sec);
  tmrinfo("max nsec=%ld\n", ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: oneshot_lh_start
 *
 * Description:
 *   Start the oneshot timer.
 *
 * Input Parameters:
 *   lower    An instance of the lower-half oneshot state structure. This
 *            structure must have been previously initialized via a call to
 *            oneshot_initialize().
 *   callback The function to call when when the oneshot timer expires.
 *            Inside the handler scope.
 *   arg      A pointer to the argument that will accompany the callback.
 *   ts       Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int oneshot_lh_start(struct oneshot_lowerhalf_s *lower,
                            oneshot_callback_t callback,
                            void *arg,
                            const struct timespec *ts)
{
  struct esp32s3_oneshot_lowerhalf_s *priv =
    (struct esp32s3_oneshot_lowerhalf_s *)lower;
  int ret;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(callback != NULL);
  DEBUGASSERT(ts != NULL);

  /* Save the callback information and start the timer */

  flags          = enter_critical_section();
  priv->callback = callback;
  priv->arg      = arg;
  ret            = esp32s3_oneshot_start(&priv->oneshot, oneshot_lh_handler,
                                         priv, ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("Failed to start oneshot timer: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: oneshot_lh_cancel
 *
 * Description:
 *   Cancel the oneshot timer and return the time remaining on the timer.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure. This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize().
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

static int oneshot_lh_cancel(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts)
{
  struct esp32s3_oneshot_lowerhalf_s *priv =
    (struct esp32s3_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags          = enter_critical_section();
  ret            = esp32s3_oneshot_cancel(&priv->oneshot, ts);
  priv->callback = NULL;
  priv->arg      = NULL;
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("Failed to cancel oneshot timer: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: oneshot_lh_current
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure. This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize().
 *   ts      The location in which to return the current time. A time of zero
 *           is returned for the initialization moment.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int oneshot_lh_current(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts)
{
  struct esp32s3_oneshot_lowerhalf_s *priv =
    (struct esp32s3_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(ts != NULL);

  esp32s3_oneshot_current(&priv->oneshot, &current_us);
  ts->tv_sec  = current_us / USEC_PER_SEC;
  current_us  = current_us - ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec = current_us * NSEC_PER_USEC;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: oneshot_initialize
 *
 * Description:
 *   Initialize the oneshot timer and return a oneshot lower-half driver
 *   instance.
 *
 * Input Parameters:
 *   chan       Timer counter channel to be used.
 *   resolution The required resolution of the timer in units of
 *              microseconds. NOTE that the range is restricted to the
 *              range of uint16_t (excluding zero).
 *
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned. NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *oneshot_initialize(int chan, uint16_t resolution)
{
  struct esp32s3_oneshot_lowerhalf_s *priv;
  int ret;

  /* Allocate an instance of the lower-half driver */

  priv = (struct esp32s3_oneshot_lowerhalf_s *)kmm_zalloc(
          sizeof(struct esp32s3_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("Failed to allocate oneshot state structure\n");
      return NULL;
    }

  priv->lh.ops     = &g_esp32s3_timer_ops; /* Pointer to the LH operations */
  priv->callback   = NULL;                 /* No callback yet */
  priv->arg        = NULL;                 /* No arg yet */
  priv->resolution = resolution;           /* Configured resolution */

  /* Initialize esp32s3_oneshot_s structure */

  ret = esp32s3_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("Failed to initialize oneshot timer driver: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
