/****************************************************************************
 * arch/x86_64/src/intel64/intel64_oneshot_lower.c
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
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void intel64_oneshot_handler(void *arg);

static clkcnt_t intel64_timer_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t intel64_timer_current(struct oneshot_lowerhalf_s *lower);
static void intel64_timer_start_absolute(struct oneshot_lowerhalf_s *lower,
                                         clkcnt_t expected);
static void intel64_timer_start(struct oneshot_lowerhalf_s *lower,
                                clkcnt_t delta);
static void intel64_timer_cancel(struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_oneshot_ops =
{
  .current        = intel64_timer_current,
  .start          = intel64_timer_start,
  .start_absolute = intel64_timer_start_absolute,
  .cancel         = intel64_timer_cancel,
  .max_delay      = intel64_timer_max_delay
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

  DEBUGASSERT(priv != NULL);

  /* Perhaps the callback was nullified in a race condition with
   * intel64_cancel?
   */

  oneshot_process_callback(&priv->lh);
}

static clkcnt_t intel64_timer_max_delay(struct oneshot_lowerhalf_s *lower)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  uint64_t usecs;
  int      ret;

  ret = intel64_oneshot_max_delay(&priv->oneshot, &usecs);

  DEBUGASSERT(ret == OK);

  return usecs;
}

static clkcnt_t intel64_timer_current(struct oneshot_lowerhalf_s *lower)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;
  int ret = intel64_oneshot_current(&priv->oneshot, &current_us);

  DEBUGASSERT(ret == OK);

  return current_us;
}

static void intel64_timer_start_absolute(struct oneshot_lowerhalf_s *lower,
                                         clkcnt_t expected)
{
  struct timespec ts;
  uint64_t current_us;
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  irqstate_t flags    = spin_lock_irqsave(&g_oneshotlow_spin);
  int        ret      = intel64_oneshot_current(&priv->oneshot, &current_us);
  uint64_t   delta_us = expected - current_us;

  DEBUGASSERT(ret == OK);

  ts.tv_sec  = delta_us / USEC_PER_SEC;
  ts.tv_nsec = delta_us % USEC_PER_SEC * 1000ull;
  ret = intel64_oneshot_start(&priv->oneshot, intel64_oneshot_handler,
                              priv, &ts);

  DEBUGASSERT(ret == OK);

  spin_unlock_irqrestore(&g_oneshotlow_spin, flags);
}

static void intel64_timer_start(struct oneshot_lowerhalf_s *lower,
                                clkcnt_t delta)
{
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  struct timespec ts =
    {
      delta / USEC_PER_SEC,
      (delta % USEC_PER_SEC) * 1000ull
    };

  irqstate_t flags = spin_lock_irqsave(&g_oneshotlow_spin);
  int ret = intel64_oneshot_start(&priv->oneshot, intel64_oneshot_handler,
                                  priv, &ts);
  DEBUGASSERT(ret == OK);
  spin_unlock_irqrestore(&g_oneshotlow_spin, flags);
}

static void intel64_timer_cancel(struct oneshot_lowerhalf_s *lower)
{
  struct timespec ts;
  struct intel64_oneshot_lowerhalf_s *priv =
    (struct intel64_oneshot_lowerhalf_s *)lower;
  irqstate_t flags = spin_lock_irqsave(&g_oneshotlow_spin);
  int ret = intel64_oneshot_cancel(&priv->oneshot, &ts);
  DEBUGASSERT(ret == OK);
  spin_unlock_irqrestore(&g_oneshotlow_spin, flags);
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

  oneshot_count_init(&priv->lh, USEC_PER_SEC);

  return &priv->lh;
}
