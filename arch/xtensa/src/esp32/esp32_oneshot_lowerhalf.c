/****************************************************************************
 * arch/xtensa/src/esp32/esp32_oneshot_lowerhalf.c
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
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/kmalloc.h>

#include "esp32_oneshot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct esp32_oneshot_lowerhalf_s and vice versa.
   * That means, opaque pointers.
   */

  struct oneshot_lowerhalf_s        lh;  /* Lower half instance */
  struct esp32_oneshot_s       oneshot;  /* ESP32-specific oneshot state */
  oneshot_callback_t          callback;  /* Upper half Interrupt callback */
  FAR void                        *arg;  /* Argument passed to handler */
  uint16_t                  resolution;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_oneshot_lh_handler(void *arg);

/* "Lower half" driver methods **********************************************/

static int esp32_max_lh_delay(FAR struct oneshot_lowerhalf_s *lower,
                              FAR struct timespec *ts);
static int esp32_lh_start(FAR struct oneshot_lowerhalf_s *lower,
                          oneshot_callback_t callback,
                          FAR void *arg,
                          FAR const struct timespec *ts);
static int esp32_lh_cancel(FAR struct oneshot_lowerhalf_s *lower,
                           FAR struct timespec *ts);
static int esp32_lh_current(FAR struct oneshot_lowerhalf_s *lower,
                            FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct oneshot_operations_s g_esp32_timer_ops =
{
  .max_delay = esp32_max_lh_delay,
  .start     = esp32_lh_start,
  .cancel    = esp32_lh_cancel,
  .current   = esp32_lh_current
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_oneshot_lh_handler
 *
 * Description:
 *   Timer expiration handler.
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when esp32_oneshot_start()
 *         was called.
 *
 ****************************************************************************/

static void esp32_oneshot_lh_handler(void *arg)
{
  FAR struct esp32_oneshot_lowerhalf_s *priv =
    (FAR struct esp32_oneshot_lowerhalf_s *)arg;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->callback != NULL);

  tmrinfo("Oneshot LH handler triggered\n");

  /* Call the callback */

  priv->callback(&priv->lh, priv->arg);

  /* Restore state */

  priv->callback = NULL;
  priv->arg = NULL;
}

/****************************************************************************
 * Name: esp32_max_lh_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds).
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

static int esp32_max_lh_delay(FAR struct oneshot_lowerhalf_s *lower,
                              FAR struct timespec *ts)
{
  DEBUGASSERT(ts != NULL);

  /* The real maximum delay surpass the limit that timespec can
   * reprent. Even using the better case: a resolution of
   * 1 us.
   * Therefore, here, fulfill the timespec with the
   * maximum value it can represent.
   */

  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;

  tmrinfo("max sec=%" PRIu32 "\n", ts->tv_sec);
  tmrinfo("max nsec=%ld\n", ts->tv_nsec);

  return OK;
}

/****************************************************************************
 * Name: esp32_lh_start
 *
 * Description:
 *   Start the oneshot timer.
 *
 * Input Parameters:
 *   lower    An instance of the lower-half oneshot state structure.  This
 *            structure must have been previously initialized via a call to
 *            oneshot_initialize();
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

static int esp32_lh_start(FAR struct oneshot_lowerhalf_s *lower,
                          oneshot_callback_t callback,
                          FAR void *arg,
                          FAR const struct timespec *ts)
{
  FAR struct esp32_oneshot_lowerhalf_s *priv =
    (FAR struct esp32_oneshot_lowerhalf_s *)lower;
  int ret;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(callback != NULL);
  DEBUGASSERT(arg != NULL);
  DEBUGASSERT(ts != NULL);

  /* Save the callback information and start the timer */

  flags          = enter_critical_section();
  priv->callback = callback;
  priv->arg      = arg;
  ret            = esp32_oneshot_start(&priv->oneshot,
                                       esp32_oneshot_lh_handler, priv, ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: esp32_oneshot_start failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_lh_cancel
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

static int esp32_lh_cancel(FAR struct oneshot_lowerhalf_s *lower,
                           FAR struct timespec *ts)
{
  FAR struct esp32_oneshot_lowerhalf_s *priv =
    (FAR struct esp32_oneshot_lowerhalf_s *)lower;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags          = enter_critical_section();
  ret            = esp32_oneshot_cancel(&priv->oneshot, ts);
  priv->callback = NULL;
  priv->arg      = NULL;
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: esp32_oneshot_cancel failed: %d\n", flags);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_lh_current
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

static int esp32_lh_current(FAR struct oneshot_lowerhalf_s *lower,
                            FAR struct timespec *ts)
{
  FAR struct esp32_oneshot_lowerhalf_s *priv =
    (FAR struct esp32_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(ts != NULL);

  esp32_oneshot_current(&priv->oneshot, &current_us);
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
 *   Initialize the oneshot timer and return a oneshot lower half driver
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

FAR struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                                   uint16_t resolution)
{
  FAR struct esp32_oneshot_lowerhalf_s *priv;
  int ret;

  /* Allocate an instance of the lower half driver */

  priv = (FAR struct esp32_oneshot_lowerhalf_s *)kmm_zalloc(
          sizeof(struct esp32_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialize oneshot state structure\n");
      return NULL;
    }

  priv->lh.ops     = &g_esp32_timer_ops; /* Pointer to the LH operations */
  priv->callback   = NULL;               /* No callback yet */
  priv->arg        = NULL;               /* No arg yet */
  priv->resolution = resolution;         /* Configured resolution */

  /* Initialize esp32_oneshot_s structure */

  ret = esp32_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("ERROR: esp32_oneshot_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
