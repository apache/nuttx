/****************************************************************************
 * arch/risc-v/src/common/riscv_mtimer.c
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/barriers.h>

#include "riscv_mtimer.h"
#include "riscv_internal.h"

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
  uintptr_t                  mtime;
  uintptr_t                  mtimecmp;
  uint64_t                   freq;
  uint64_t                   alarm;
  oneshot_callback_t         callback;
  void                       *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int riscv_mtimer_max_delay(struct oneshot_lowerhalf_s *lower,
                                  struct timespec *ts);
static int riscv_mtimer_start(struct oneshot_lowerhalf_s *lower,
                              oneshot_callback_t callback, void *arg,
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
  .max_delay = riscv_mtimer_max_delay,
  .start     = riscv_mtimer_start,
  .cancel    = riscv_mtimer_cancel,
  .current   = riscv_mtimer_current,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_USE_S_MODE
static uint64_t riscv_mtimer_get_mtime(struct riscv_mtimer_lowerhalf_s *priv)
{
#ifdef CONFIG_ARCH_RV64
  /* priv->mtime is -1, means this SoC:
   * 1. does NOT support 64bit/DWORD write for the mtimer compare value regs,
   * 2. has NO memory mapped regs which hold the value of mtimer counter,
   *    it could be read from the CSR "time".
   */

  return -1 == priv->mtime ? READ_CSR(time) : getreg64(priv->mtime);
#else
  uint32_t hi;
  uint32_t lo;

  do
    {
      hi = getreg32(priv->mtime + 4);
      lo = getreg32(priv->mtime);
    }
  while (getreg32(priv->mtime + 4) != hi);

  return ((uint64_t)hi << 32) | lo;
#endif
}

static void riscv_mtimer_set_mtimecmp(struct riscv_mtimer_lowerhalf_s *priv,
                                      uint64_t value)
{
#ifdef CONFIG_ARCH_RV64
  if (-1 != priv->mtime)
    {
      putreg64(value, priv->mtimecmp);
    }
  else
#endif
    {
      putreg32(UINT32_MAX, priv->mtimecmp + 4);
      putreg32(value, priv->mtimecmp);
      putreg32(value >> 32, priv->mtimecmp + 4);
    }

  /* Make sure it sticks */

  __MB();
}
#else
static uint64_t riscv_mtimer_get_mtime(struct riscv_mtimer_lowerhalf_s *priv)
{
  UNUSED(priv);
  return riscv_sbi_get_time();
}

static void riscv_mtimer_set_mtimecmp(struct riscv_mtimer_lowerhalf_s *priv,
                                      uint64_t value)
{
  UNUSED(priv);
  riscv_sbi_set_timer(value);
}
#endif

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
                              oneshot_callback_t callback, void *arg,
                              const struct timespec *ts)
{
  struct riscv_mtimer_lowerhalf_s *priv =
    (struct riscv_mtimer_lowerhalf_s *)lower;
  uint64_t mtime = riscv_mtimer_get_mtime(priv);

  priv->alarm = mtime + ts->tv_sec * priv->freq +
                ts->tv_nsec * priv->freq / NSEC_PER_SEC;
  if (priv->alarm < mtime)
    {
      priv->alarm = UINT64_MAX;
    }

  priv->callback = callback;
  priv->arg      = arg;

  riscv_mtimer_set_mtimecmp(priv, priv->alarm);
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

  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);

  mtime = riscv_mtimer_get_mtime(priv);
  if (priv->alarm > mtime)
    {
      uint64_t nsec = (priv->alarm - mtime) *
                      NSEC_PER_SEC / priv->freq;

      ts->tv_sec  = nsec / NSEC_PER_SEC;
      ts->tv_nsec = nsec % NSEC_PER_SEC;
    }
  else
    {
      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
    }

  priv->alarm    = 0;
  priv->callback = NULL;
  priv->arg      = NULL;

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
  uint64_t nsec = mtime / (priv->freq / USEC_PER_SEC) * NSEC_PER_USEC;

  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  return 0;
}

static int riscv_mtimer_interrupt(int irq, void *context, void *arg)
{
  struct riscv_mtimer_lowerhalf_s *priv = arg;

  riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);
  if (priv->callback != NULL)
    {
      priv->callback(&priv->lower, priv->arg);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oneshot_lowerhalf_s *
riscv_mtimer_initialize(uintptr_t mtime, uintptr_t mtimecmp,
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

      riscv_mtimer_set_mtimecmp(priv, UINT64_MAX);
      irq_attach(irq, riscv_mtimer_interrupt, priv);
      up_enable_irq(irq);
    }

  return (struct oneshot_lowerhalf_s *)priv;
}
