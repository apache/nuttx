/****************************************************************************
 * arch/arm/src/armv8-r/arm_arch_timer.c
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

#include "barriers.h"
#include "cp15.h"
#include "arm_gic.h"
#include "arm_arch_timer.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arm_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.  This must be the first thing in this
   * structure so that pointers to struct oneshot_lowerhalf_s are cast
   * compatible to struct arm64_oneshot_lowerhalf_s and vice versa.
   */

  struct oneshot_lowerhalf_s lh;      /* Common lower-half driver fields */

  /* Private lower half data follows */

  void *arg;                          /* Argument that is passed to the handler */
  uint64_t cycle_per_tick;            /* cycle per tick */
  oneshot_callback_t callback;        /* Internal handler that receives callback */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm_arch_timer_set_compare(uint64_t value)
{
  CP15_SET64(CNTV_CVAL, value);
}

static inline uint64_t arm_arch_timer_get_compare(void)
{
  return CP15_GET64(CNTV_CVAL);
}

static inline void arm_arch_timer_enable(bool enable)
{
  uint64_t value;

  value = CP15_GET(CNTV_CTL);

  if (enable)
    {
      value |= CNTV_CTL_ENABLE_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_ENABLE_BIT;
    }

  CP15_SET(CNTV_CTL, value);
}

static inline void arm_arch_timer_set_irq_mask(bool mask)
{
  uint64_t value;

  value = CP15_GET(CNTV_CTL);

  if (mask)
    {
      value |= CNTV_CTL_IMASK_BIT;
    }
  else
    {
      value &= ~CNTV_CTL_IMASK_BIT;
    }

  CP15_SET(CNTV_CTL, value);
}

static inline uint64_t arm_arch_timer_count(void)
{
  return CP15_GET64(CNTVCT);
}

static inline uint64_t arm_arch_timer_get_cntfrq(void)
{
  return CP15_GET(CNTFRQ);
}

/****************************************************************************
 * Name: arm_arch_timer_compare_isr
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

static int arm_arch_timer_compare_isr(int irq, void *regs, void *arg)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)arg;

  arm_arch_timer_set_irq_mask(true);

  if (priv->callback)
    {
      /* Then perform the callback */

      priv->callback(&priv->lh, priv->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: arm_tick_max_delay
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

static int arm_tick_max_delay(struct oneshot_lowerhalf_s *lower,
                                clock_t *ticks)
{
  DEBUGASSERT(ticks != NULL);

  *ticks = (clock_t)UINT64_MAX;

  return OK;
}

/****************************************************************************
 * Name: arm_tick_cancel
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

static int arm_tick_cancel(struct oneshot_lowerhalf_s *lower,
                             clock_t *ticks)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && ticks != NULL);

  /* Disable int */

  arm_arch_timer_set_irq_mask(true);

  return OK;
}

/****************************************************************************
 * Name: arm_tick_start
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

static int arm_tick_start(struct oneshot_lowerhalf_s *lower,
                            oneshot_callback_t callback, void *arg,
                            clock_t ticks)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && callback != NULL);

  /* Save the new handler and its argument */

  priv->callback = callback;
  priv->arg = arg;

  /* Set the timeout */

  arm_arch_timer_set_compare(arm_arch_timer_count() +
                               priv->cycle_per_tick * ticks);
  arm_arch_timer_set_irq_mask(false);

  return OK;
}

/****************************************************************************
 * Name: arm_tick_current
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

static int arm_tick_current(struct oneshot_lowerhalf_s *lower,
                              clock_t *ticks)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(ticks != NULL);

  *ticks = arm_arch_timer_count() / priv->cycle_per_tick;

  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_oneshot_ops =
{
  .tick_start     = arm_tick_start,
  .tick_current   = arm_tick_current,
  .tick_max_delay = arm_tick_max_delay,
  .tick_cancel    = arm_tick_cancel,
};

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

static struct oneshot_lowerhalf_s *arm_oneshot_initialize(void)
{
  struct arm_oneshot_lowerhalf_s *priv;

  tmrinfo("oneshot_initialize\n");

  /* Allocate an instance of the lower half driver */

  priv = (struct arm_oneshot_lowerhalf_s *)
    kmm_zalloc(sizeof(struct arm_oneshot_lowerhalf_s));

  if (priv == NULL)
    {
      tmrerr("ERROR: Failed to initialized state structure\n");

      return NULL;
    }

  /* Initialize the lower-half driver structure */

  priv->lh.ops = &g_oneshot_ops;
  priv->cycle_per_tick = arm_arch_timer_get_cntfrq() / TICK_PER_SEC;
  tmrinfo("cycle_per_tick %" PRIu64 "\n", priv->cycle_per_tick);

  /* Attach handler */

  irq_attach(ARM_ARCH_TIMER_IRQ,
             arm_arch_timer_compare_isr, priv);

  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  /* Start timer */

  arm_arch_timer_enable(true);

  tmrinfo("oneshot_initialize ok %p \n", &priv->lh);

  return &priv->lh;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint64_t freq;

  freq = arm_arch_timer_get_cntfrq();
  tmrinfo("%s: cp15 timer(s) running at %" PRIu64 ".%" PRIu64 "MHz\n",
          __func__, freq / 1000000, (freq / 10000) % 100);

  up_alarm_set_lowerhalf(arm_oneshot_initialize());
}

#ifdef CONFIG_SMP
/****************************************************************************
 * Function:  arm_arch_timer_secondary_init
 *
 * Description:
 *   This function is called during start-up to initialize the system timer
 *   interrupt for smp.
 *
 * Notes:
 * The origin design for ARMv8-A timer is assigned private timer to
 * every PE(CPU core), the ARM_ARCH_TIMER_IRQ is a PPI so it's
 * should be enable at every core.
 *
 * But for NuttX, it's design only for primary core to handle timer
 * interrupt and call nxsched_process_timer at timer tick mode.
 * So we need only enable timer for primary core
 *
 * IMX6 use GPT which is a SPI rather than generic timer to handle
 * timer interrupt
 ****************************************************************************/

void arm_arch_timer_secondary_init()
{
#ifdef CONFIG_SCHED_TICKLESS
  tmrinfo("arm_arch_timer_secondary_init\n");

  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  /* Start timer */

  arm_arch_timer_enable(true);
#endif
}
#endif