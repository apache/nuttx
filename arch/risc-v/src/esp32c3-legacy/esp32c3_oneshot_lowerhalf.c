/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/esp32c3_oneshot_lowerhalf.c
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
#include <nuttx/spinlock.h>

#include "esp32c3_oneshot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct esp32c3_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s        lh;   /* Lower half instance */
  struct esp32c3_oneshot_s     oneshot;   /* ESP32-C3-specific oneshot state */
  uint16_t                  resolution;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32c3_lh_handler(void *arg);

/* "Lower half" driver methods **********************************************/

static clkcnt_t esp32c3_lh_max_delay(struct oneshot_lowerhalf_s *lower);
static clkcnt_t esp32c3_lh_current(struct oneshot_lowerhalf_s *lower);
static void esp32c3_lh_start_absolute(struct oneshot_lowerhalf_s *lower,
                                              clkcnt_t expected);
static void esp32c3_lh_start(struct oneshot_lowerhalf_s *lower,
                                  clkcnt_t delta);
static void esp32c3_lh_cancel(struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct oneshot_operations_s g_esp32c3_timer_ops =
{
  .current        = esp32c3_lh_current,
  .start          = esp32c3_lh_start,
  .start_absolute = esp32c3_lh_start_absolute,
  .cancel         = esp32c3_lh_cancel,
  .max_delay      = esp32c3_lh_max_delay
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_lh_handler
 *
 * Description:
 *   Timer expiration handler.
 *
 * Input Parameters:
 *   arg - Should be the same argument provided when esp32c3_oneshot_start()
 *         was called.
 *
 ****************************************************************************/

static void esp32c3_lh_handler(void *arg)
{
  struct esp32c3_oneshot_lowerhalf_s *priv =
    (struct esp32c3_oneshot_lowerhalf_s *)arg;

  DEBUGASSERT(priv != NULL);

  tmrinfo("Oneshot handler triggered\n");

  /* Sample and nullify BEFORE executing callback (in case the callback
   * restarts the oneshot).
   */

  oneshot_process_callback(&priv->lh);
}

static clkcnt_t esp32c3_lh_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t esp32c3_lh_current(struct oneshot_lowerhalf_s *lower)
{
  struct esp32c3_oneshot_lowerhalf_s *priv =
    (struct esp32c3_oneshot_lowerhalf_s *)lower;
  uint64_t current_us;

  DEBUGASSERT(priv != NULL);

  esp32c3_oneshot_current(&priv->oneshot, &current_us);

  return current_us;
}

static void esp32c3_lh_start_absolute(struct oneshot_lowerhalf_s *lower,
                                      clkcnt_t expected)
{
  struct esp32c3_oneshot_lowerhalf_s *priv =
    (struct esp32c3_oneshot_lowerhalf_s *)lower;
  struct timespec ts;
  irqstate_t flags;
  uint64_t delta;
  clkcnt_t curr;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Save the callback information and start the timer */

  flags = enter_critical_section();

  curr  = esp32c3_lh_current(lower);
  delta = expected < curr ? 0 : expected - curr;

  ts.tv_sec  = delta / USEC_PER_SEC;
  ts.tv_nsec = delta % USEC_PER_SEC * NSEC_PER_USEC;
  ret   = esp32c3_oneshot_start(&priv->oneshot, esp32c3_lh_handler,
                                priv, &ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: esp32c3_oneshot_start_absolute failed: %d\n", ret);
    }
}

static void esp32c3_lh_start(struct oneshot_lowerhalf_s *lower,
                             clkcnt_t delta)
{
  struct esp32c3_oneshot_lowerhalf_s *priv =
    (struct esp32c3_oneshot_lowerhalf_s *)lower;
  struct timespec ts;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Save the callback information and start the timer */

  ts.tv_sec  = delta / USEC_PER_SEC;
  ts.tv_nsec = delta % USEC_PER_SEC * NSEC_PER_USEC;

  flags = enter_critical_section();
  ret   = esp32c3_oneshot_start(&priv->oneshot, esp32c3_lh_handler,
                                priv, &ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: esp32c3_oneshot_start failed: %d\n", ret);
    }
}

static void esp32c3_lh_cancel(struct oneshot_lowerhalf_s *lower)
{
  struct esp32c3_oneshot_lowerhalf_s *priv =
    (struct esp32c3_oneshot_lowerhalf_s *)lower;
  struct timespec ts;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Cancel the timer */

  flags = enter_critical_section();
  ret   = esp32c3_oneshot_cancel(&priv->oneshot, &ts);
  leave_critical_section(flags);

  if (ret < 0)
    {
      tmrerr("ERROR: esp32c3_oneshot_cancel failed: %d\n", flags);
    }
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

struct oneshot_lowerhalf_s *oneshot_initialize(int chan,
                                               uint16_t resolution)
{
  struct esp32c3_oneshot_lowerhalf_s *priv;
  int ret;

  /* Allocate an instance of the lower half driver */

  priv = kmm_zalloc(sizeof(struct esp32c3_oneshot_lowerhalf_s));
  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialize oneshot state structure\n");
      return NULL;
    }

  priv->lh.ops     = &g_esp32c3_timer_ops; /* Pointer to the LH operations */
  priv->resolution = resolution;           /* Configured resolution */

  oneshot_count_init(&priv->lh, USEC_PER_SEC);

  /* Initialize esp32c3_oneshot_s structure */

  ret = esp32c3_oneshot_initialize(&priv->oneshot, chan, resolution);
  if (ret < 0)
    {
      tmrerr("ERROR: esp32c3_oneshot_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  return &priv->lh;
}
