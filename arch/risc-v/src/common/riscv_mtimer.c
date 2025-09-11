/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "riscv_mtimer.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct riscv_mtimer_lowerhalf_s
{
  struct oneshot_lowerhalf_s lower;
  uintreg_t                  mtime;
  uintreg_t                  mtimecmp;
  uint64_t                   freq;
  uint64_t                   alarm;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int riscv_mtimer_max_delay(struct oneshot_lowerhalf_s *lower,
                                  struct timespec *ts);
static int riscv_mtimer_start(struct oneshot_lowerhalf_s *lower,
                              const struct timespec *ts);
static int riscv_mtimer_cancel(struct oneshot_lowerhalf_s *lower,
                               struct timespec *ts);
static int riscv_mtimer_current(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_riscv_mtimer_ops =
{
  .max_delay      = riscv_mtimer_max_delay,
  .start          = riscv_mtimer_start,
  .cancel         = riscv_mtimer_cancel,
  .current        = riscv_mtimer_current,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint64_t riscv_mtimer_get_mtime(struct riscv_mtimer_lowerhalf_s *priv)
{
  return riscv_mtimer_get(priv->mtime);
}

static void riscv_mtimer_set_mtimecmp(struct riscv_mtimer_lowerhalf_s *priv,
                                      uint64_t value)
{
  riscv_mtimer_set(priv->mtime, priv->mtimecmp, value);
}

/****************************************************************************
 * Name: riscv_mtimer_max_delay
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

static int riscv_mtimer_max_delay(struct oneshot_lowerhalf_s *lower,
                                  struct timespec *ts)
{
  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = NSEC_PER_SEC - 1;

  return 0;
}

/****************************************************************************
 * Name: riscv_mtimer_start
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

static int riscv_mtimer_start(struct oneshot_lowerhalf_s *lower,
                              const struct timespec *ts)
{
  struct riscv_mtimer_lowerhalf_s *priv =
    (struct riscv_mtimer_lowerhalf_s *)lower;
  irqstate_t flags;
  uint64_t mtime;
  uint64_t alarm;

  flags = up_irq_save();

  mtime = riscv_mtimer_get_mtime(priv);
  alarm = mtime + ts->tv_sec * priv->freq +
          ts->tv_nsec * priv->freq / NSEC_PER_SEC;

  priv->alarm = alarm;

  riscv_mtimer_set_mtimecmp(priv, priv->alarm);

  up_irq_restore(flags);
  return 0;
}

/****************************************************************************
 * Name: riscv_mtimer_cancel
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

static int riscv_mtimer_cancel(struct oneshot_lowerhalf_s *lower,
                               struct timespec *ts)
{
  struct riscv_mtimer_lowerhalf_s *priv =
    (struct riscv_mtimer_lowerhalf_s *)lower;
  uint64_t mtime;
  uint64_t alarm;
  uint64_t nsec;
  irqstate_t flags;

  flags = up_irq_save();

  alarm = priv->alarm;

  mtime = riscv_mtimer_get_mtime(priv);

  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);

  nsec = (alarm - mtime) * NSEC_PER_SEC / priv->freq;
  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  priv->alarm = 0;

  up_irq_restore(flags);

  return 0;
}

/****************************************************************************
 * Name: riscv_mtimer_current
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

static int riscv_mtimer_current(struct oneshot_lowerhalf_s *lower,
                                struct timespec *ts)
{
  struct riscv_mtimer_lowerhalf_s *priv =
    (struct riscv_mtimer_lowerhalf_s *)lower;
  uint64_t mtime = riscv_mtimer_get_mtime(priv);
  uint64_t left;

  ts->tv_sec  = mtime / priv->freq;
  left        = mtime - ts->tv_sec * priv->freq;
  ts->tv_nsec = NSEC_PER_SEC * left / priv->freq;

  return 0;
}

static int riscv_mtimer_interrupt(int irq, void *context, void *arg)
{
  struct riscv_mtimer_lowerhalf_s *priv = arg;

  oneshot_process_callback(&priv->lower);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oneshot_lowerhalf_s *
riscv_mtimer_initialize(uintreg_t mtime, uintreg_t mtimecmp,
                        int irq, uint64_t freq)
{
  struct riscv_mtimer_lowerhalf_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv != NULL)
    {
      priv->lower.ops = &g_riscv_mtimer_ops;
      priv->mtime     = mtime;
      priv->mtimecmp  = mtimecmp;
      priv->freq      = freq;
      priv->alarm     = UINT64_MAX;

      riscv_mtimer_set_mtimecmp(priv, priv->alarm);
      irq_attach(irq, riscv_mtimer_interrupt, priv);
      up_enable_irq(irq);
    }

  return (struct oneshot_lowerhalf_s *)priv;
}
