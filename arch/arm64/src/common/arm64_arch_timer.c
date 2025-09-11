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

  uint64_t frequency;                 /* Frequency in cycle per second */

  /* which cpu timer is running, -1 indicate timer stoppd */

  int running;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void arm64_arch_timer_set_compare(uint64_t value)
{
  write_sysreg(value, cntv_cval_el0);
}

static inline void arm64_arch_timer_enable(bool enable)
{
  modify_sysreg(enable ? CNTV_CTL_ENABLE_BIT : 0u,
                CNTV_CTL_ENABLE_BIT, cntv_ctl_el0);
}

static inline void arm64_arch_timer_set_irq_mask(bool mask)
{
  modify_sysreg(mask ? CNTV_CTL_IMASK_BIT : 0u,
                CNTV_CTL_IMASK_BIT, cntv_ctl_el0);
}

static inline uint64_t arm64_arch_timer_count(void)
{
  return read_sysreg(cntvct_el0);
}

static inline uint64_t arm64_arch_timer_get_cntfrq(void)
{
  return read_sysreg(cntfrq_el0);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  if (priv->running == this_cpu())
    {
      /* Then perform the callback */

      oneshot_process_callback(&priv->lh);
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

static int arm64_max_delay(struct oneshot_lowerhalf_s *lower,
                           struct timespec *ts)
{
  uint64_t freq = arm64_arch_timer_get_cntfrq();

  DEBUGASSERT(ts != NULL);

  ts->tv_sec  = UINT64_MAX / freq;
  ts->tv_nsec = UINT64_MAX % freq * NSEC_PER_SEC / freq;

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

static int arm64_cancel(struct oneshot_lowerhalf_s *lower,
                        struct timespec *ts)
{
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && ts != NULL);

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

static int arm64_start(struct oneshot_lowerhalf_s *lower,
                       const struct timespec *ts)
{
  uint64_t count;
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;
  uint64_t freq = priv->frequency;

  DEBUGASSERT(priv && ts);

  priv->running = this_cpu();

  count  = arm64_arch_timer_count();
  count += (uint64_t)ts->tv_sec * freq +
           (uint64_t)ts->tv_nsec * freq / NSEC_PER_SEC;

  arm64_arch_timer_set_compare(count);

  arm64_arch_timer_set_irq_mask(false);

  return OK;
}

/****************************************************************************
 * Name: arm64_tick_current
 *
 * Description:
 *  Get the current time.
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ticks   The location in which to return the current time.
 *
 * Returned Value:
 *   any failure.
 *
 ****************************************************************************/

static int arm64_current(struct oneshot_lowerhalf_s *lower,
                         struct timespec *ts)
{
  uint64_t count;
  uint64_t freq;
  struct arm64_oneshot_lowerhalf_s *priv =
    (struct arm64_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(ts != NULL);

  freq  = priv->frequency;
  count = arm64_arch_timer_count();

  ts->tv_sec  = count / freq;
  ts->tv_nsec = (count % freq) * NSEC_PER_SEC / freq;

  return OK;
}

/****************************************************************************
 * Name: arm64_oneshot_initialize_per_cpu
 *
 * Description:
 *   Initialize the ARM generic timer for secondary CPUs.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void arm64_oneshot_initialize_per_cpu(void)
{
  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  /* Start timer */

  arm64_arch_timer_enable(true);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_oneshot_ops =
{
  .start     = arm64_start,
  .current   = arm64_current,
  .max_delay = arm64_max_delay,
  .cancel    = arm64_cancel,
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

  arm64_oneshot_initialize_per_cpu();

  tmrinfo("oneshot_initialize ok %p \n", &priv->lh);

  return &priv->lh;
}

void up_timer_initialize(void)
{
  up_alarm_set_lowerhalf(arm64_oneshot_initialize());
}

void arm64_timer_secondary_init(void)
{
  arm64_oneshot_initialize_per_cpu();
}
