/****************************************************************************
 * arch/arm64/src/common/arm64_arch_timer.c
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
#include <debug.h>
#include <assert.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_arch_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_ARM_TIMER_SECURE_IRQ         (GIC_PPI_INT_BASE + 13)
#define CONFIG_ARM_TIMER_NON_SECURE_IRQ     (GIC_PPI_INT_BASE + 14)
#define CONFIG_ARM_TIMER_VIRTUAL_IRQ        (GIC_PPI_INT_BASE + 11)
#define CONFIG_ARM_TIMER_HYP_IRQ            (GIC_PPI_INT_BASE + 10)

#define ARM_ARCH_TIMER_IRQ                  CONFIG_ARM_TIMER_VIRTUAL_IRQ
#define ARM_ARCH_TIMER_PRIO                 IRQ_DEFAULT_PRIORITY
#define ARM_ARCH_TIMER_FLAGS                IRQ_TYPE_LEVEL

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arm64_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct arm64_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;      /* Common lower-half driver fields */

  /* Private lower half data follows */

  void *arg;                          /* Argument that is passed to the handler */
  uint64_t frequency;                 /* Frequency in cycle per second */
  oneshot_callback_t callback;        /* Internal handler that receives callback */

  /* which cpu timer is running, -1 indicate timer stoppd */

  int running;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm64_arch_timer_set_compare(uint64_t value)
{
  write_sysreg(value, cntv_cval_el0);
}

static inline void arm64_arch_timer_enable(bool enable)
{
  uint64_t value;

  value = read_sysreg(cntv_ctl_el0);

  if (enable)
    {
      value |= CNTV_CTL_ENABLE_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_ENABLE_BIT;
    }

  write_sysreg(value, cntv_ctl_el0);
}

static inline void arm64_arch_timer_set_irq_mask(bool mask)
{
  uint64_t value;

  value = read_sysreg(cntv_ctl_el0);

  if (mask)
    {
      value |= CNTV_CTL_IMASK_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_IMASK_BIT;
    }

  write_sysreg(value, cntv_ctl_el0);
}

static inline uint64_t arm64_arch_timer_count(void)
{
  return read_sysreg(cntvct_el0);
}

static inline uint64_t arm64_arch_timer_get_cntfrq(void)
{
  return read_sysreg(cntfrq_el0);
}

static inline uint64_t arm64_arch_cnt2tick(uint64_t count, uint64_t freq)
{
  uint64_t multiply_safe_count = UINT64_MAX / TICK_PER_SEC;
  uint64_t result_ticks = 0;

  /* We convert count to ticks via
   *   ticks = count / cycle_per_tick.
   * Concretely, we have:
   *   ticks = count / (freq / TICK_PER_SEC).
   * However, the `freq / TICK_PER_SEC` might be inaccurate
   * due to the integer division.
   * So we transform it to:
   *   ticks = count * TICK_PER_SEC / freq.
   */

  if (count > multiply_safe_count)
    {
      /* In case of count * TICK_PER_SEC overflow.
       * We divide the count into two parts:
       * The multiply overflow part and non-overflow part.
       * We convert the overflow part to ticks first,
       * and then add the non-overflow part.
       */

      result_ticks += count / multiply_safe_count *
                      (multiply_safe_count * TICK_PER_SEC / freq);
      count         = count % multiply_safe_count;
    }

  /* Here we convert the non-overflow part to ticks. */

  result_ticks += count * TICK_PER_SEC / freq;

  return result_ticks;
}

static inline uint64_t arm64_arch_tick2cnt(uint64_t ticks, uint64_t freq)
{
  uint64_t multiply_safe_ticks = UINT64_MAX / freq;
  uint64_t result_count = 0;

  if (ticks > multiply_safe_ticks)
    {
      /* In case of count * freq overflow.
       * We divide the ticks into two parts:
       * The multiply overflow part and non-overflow part.
       * We convert the overflow part to count first,
       * and then add the non-overflow part.
       */

      result_count += ticks / multiply_safe_ticks *
                      (multiply_safe_ticks * freq / TICK_PER_SEC);
      ticks         = ticks % multiply_safe_ticks;
    }

  /* Here we convert the non-overflow part to count. */

  result_count += ticks * freq / TICK_PER_SEC;

  return result_count;
}

/****************************************************************************
 * Name: arm64_arch_timer_compare_isr
 *
 * Description:
 *   Common timer interrupt callback.  When any oneshot timer interrupt
 *   expires, this function will be called.  It will forward the call to
 *   the next level up.
 *
 * Input Parameters:
 *   oneshot - The state associated with the expired timer
 *
 * Returned Value:
 *   Always returns OK
 *
 ****************************************************************************/

static int arm64_arch_timer_compare_isr(int irq, void *regs, void *arg)
{
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)arg;

  arm64_arch_timer_set_irq_mask(true);

  if (priv->callback && priv->running == this_cpu())
    {
      /* Then perform the callback */

      priv->callback(&priv->lh, priv->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: arm64_tick_max_delay
 *
 * Description:
 *   Determine the maximum delay of the one-shot timer (in microseconds)
 *
 * Input Parameters:
 *   lower   An instance of the lower-half oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ticks   The location in which to return the maximum delay.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int arm64_tick_max_delay(struct oneshot_lowerhalf_s *lower,
                                clock_t *ticks)
{
  DEBUGASSERT(ticks != NULL);

  *ticks = (clock_t)UINT32_MAX;

  return OK;
}

/****************************************************************************
 * Name: arm64_tick_cancel
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
 *   ticks   The location in which to return the time remaining on the
 *           oneshot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int arm64_tick_cancel(struct oneshot_lowerhalf_s *lower,
                             clock_t *ticks)
{
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && ticks != NULL);

  /* Disable int */

  priv->running = -1;
  arm64_arch_timer_set_irq_mask(true);

  return OK;
}

/****************************************************************************
 * Name: arm64_tick_start
 *
 * Description:
 *   Start the oneshot timer
 *
 * Input Parameters:
 *   lower    An instance of the lower-half oneshot state structure.  This
 *            structure must have been previously initialized via a call to
 *            oneshot_initialize();
 *   handler  The function to call when when the oneshot timer expires.
 *   arg      An opaque argument that will accompany the callback.
 *   ticks    Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int arm64_tick_start(struct oneshot_lowerhalf_s *lower,
                            oneshot_callback_t callback, void *arg,
                            clock_t ticks)
{
  uint64_t next_cnt;
  uint64_t next_tick;
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;
  uint64_t freq = priv->frequency;

  DEBUGASSERT(priv != NULL && callback != NULL);

  /* Save the new handler and its argument */

  priv->callback = callback;
  priv->arg = arg;

  priv->running = this_cpu();

  /* Align the timer count to the tick boundary */

  next_tick = arm64_arch_cnt2tick(arm64_arch_timer_count(), freq) + ticks;
  next_cnt  = arm64_arch_tick2cnt(next_tick, freq);

  arm64_arch_timer_set_compare(next_cnt);

  arm64_arch_timer_set_irq_mask(false);

  return OK;
}

/****************************************************************************
 * Name: arm64_tick_current
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ticks   The location in which to return the current time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int arm64_tick_current(struct oneshot_lowerhalf_s *lower,
                              clock_t *ticks)
{
  uint64_t count;
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(ticks != NULL);

  count = arm64_arch_timer_count();

  *ticks = arm64_arch_cnt2tick(count, priv->frequency);

  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_oneshot_ops =
{
  .tick_start     = arm64_tick_start,
  .tick_current   = arm64_tick_current,
  .tick_max_delay = arm64_tick_max_delay,
  .tick_cancel    = arm64_tick_cancel,
};

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
 * Returned Value:
 *   On success, a non-NULL instance of the oneshot lower-half driver is
 *   returned.  NULL is return on any failure.
 *
 ****************************************************************************/

struct oneshot_lowerhalf_s *arm64_oneshot_initialize(void)
{
  struct arm64_oneshot_lowerhalf_s *priv;

  tmrinfo("oneshot_initialize\n");

  /* Allocate an instance of the lower half driver */

  priv = (struct arm64_oneshot_lowerhalf_s *)
    kmm_zalloc(sizeof(struct arm64_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");

      return NULL;
    }

  /* Initialize the lower-half driver structure */

  priv->lh.ops = &g_oneshot_ops;
  priv->running = -1;
  priv->frequency = arm64_arch_timer_get_cntfrq();

  /* Attach handler */

  irq_attach(ARM_ARCH_TIMER_IRQ,
             arm64_arch_timer_compare_isr, priv);

  arm64_oneshot_secondary_init();

  tmrinfo("oneshot_initialize ok %p \n", &priv->lh);

  return &priv->lh;
}

/****************************************************************************
 * Name: arm64_arch_timer_secondary_init
 *
 * Description:
 *   Initialize the ARM generic timer for secondary CPUs.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm64_oneshot_secondary_init(void)
{
  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  /* Start timer */

  arm64_arch_timer_enable(true);
}
