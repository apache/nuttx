/****************************************************************************
 * arch/arm/src/armv8-r/arm_timer.c
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
#include <nuttx/bits.h>
#include <arch/barriers.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/kmalloc.h>

#include "arm_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CNT_CTL_ENABLE_BIT       0
#define CNT_CTL_IMASK_BIT        1
#define CNT_CTL_ISTATUS_BIT      2

#define GIC_IRQ_HYP_TIMER        26 /* Hypervisor Timer (HTM) IRQ */
#define GIC_IRQ_VIRT_TIMER       27 /* Virtual Timer (VTM) IRQ */
#define GIC_IRQ_SEC_PHY_TIMER    29 /* Secure Physical Timer IRQ */
#define GIC_IRQ_NONSEC_PHY_TIMER 30 /* Non-secure Physical Timer IRQ */

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
#  define GIC_IRQ_PHY_TIMER      GIC_IRQ_SEC_PHY_TIMER
#else
#  define GIC_IRQ_PHY_TIMER      GIC_IRQ_NONSEC_PHY_TIMER
#endif

#define ARM_ARCH_TIMER_IRQ       GIC_IRQ_PHY_TIMER
#define ARM_ARCH_TIMER_PRIO      IRQ_DEFAULT_PRIORITY
#define ARM_ARCH_TIMER_FLAGS     IRQ_TYPE_LEVEL

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

  uint32_t           frequency;       /* Frequency */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm_timer_set_freq(uint32_t freq)
{
  CP15_SET(CNTFRQ, freq);
}

static inline uint64_t arm_timer_phy_count(void)
{
  return CP15_GET64(CNTPCT);
}

static inline void arm_timer_phy_set_relative(uint32_t tval)
{
  CP15_SET(CNTP_TVAL, tval);
}

static inline void arm_timer_phy_set_absolute(uint64_t cval)
{
  CP15_SET64(CNTP_CVAL, cval);
}

static inline void arm_timer_phy_enable(bool enable)
{
  CP15_MODIFY((uint32_t)enable << CNT_CTL_ENABLE_BIT,
              BIT(CNT_CTL_ENABLE_BIT), CNTP_CTL);
}

static inline void arm_timer_phy_set_irq_mask(bool mask)
{
  CP15_MODIFY((uint32_t)mask << CNT_CTL_IMASK_BIT,
              BIT(CNT_CTL_IMASK_BIT), CNTP_CTL);
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

  /* Suspend the timer irq, restart again when call tick_start */

  arm_timer_phy_set_irq_mask(true);

  /* Then perform the callback */

  oneshot_process_callback(&priv->lh);

  return OK;
}

/****************************************************************************
 * Name: arm_max_delay
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

static int arm_max_delay(struct oneshot_lowerhalf_s *lower,
                         struct timespec *ts)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;
  uint32_t freq = priv->frequency;

  DEBUGASSERT(ts != NULL);

  ts->tv_sec  = UINT64_MAX / freq;
  ts->tv_nsec = UINT64_MAX % freq * NSEC_PER_SEC / freq;

  return OK;
}

/****************************************************************************
 * Name: arm_cancel
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
 *           oneshot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

static int arm_cancel(struct oneshot_lowerhalf_s *lower,
                      struct timespec *ts)
{
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(priv != NULL && ts != NULL);

  /* Disable int */

  arm_timer_phy_set_irq_mask(true);

  return OK;
}

/****************************************************************************
 * Name: arm_start
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
 *   ts       Provides the duration of the one shot timer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

static int arm_start(struct oneshot_lowerhalf_s *lower,
                     const struct timespec *ts)
{
  uint64_t count;
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;
  uint64_t freq = priv->frequency;

  DEBUGASSERT(priv && ts);

  /* Set the timeout */

  count  = arm_timer_phy_count();
  count += (uint64_t)ts->tv_sec * freq +
           (uint64_t)ts->tv_nsec * freq / NSEC_PER_SEC;

  arm_timer_phy_set_absolute(count);

  /* Try to unmask the timer irq in timer controller
   * in case of arm_tick_cancel is called.
   */

  arm_timer_phy_set_irq_mask(false);

  return OK;
}

/****************************************************************************
 * Name: arm_current
 *
 * Description:
 *  Get the current time.
 *
 * Input Parameters:
 *   lower   Caller allocated instance of the oneshot state structure.  This
 *           structure must have been previously initialized via a call to
 *           oneshot_initialize();
 *   ts      The location in which to return the current time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success, a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int arm_current(struct oneshot_lowerhalf_s *lower,
                       struct timespec *ts)
{
  uint64_t count;
  uint32_t freq;
  struct arm_oneshot_lowerhalf_s *priv =
    (struct arm_oneshot_lowerhalf_s *)lower;

  DEBUGASSERT(ts != NULL);

  freq  = priv->frequency;
  count = arm_timer_phy_count();

  ts->tv_sec  = count / freq;
  ts->tv_nsec = (count % freq) * NSEC_PER_SEC / freq;

  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_oneshot_ops =
{
  .start     = arm_start,
  .current   = arm_current,
  .max_delay = arm_max_delay,
  .cancel    = arm_cancel,
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

  DEBUGASSERT(arm_timer_get_freq() <= UINT32_MAX);

  priv->lh.ops = &g_oneshot_ops;
  priv->frequency = arm_timer_get_freq();

  /* Attach handler */

  irq_attach(ARM_ARCH_TIMER_IRQ,
             arm_arch_timer_compare_isr, priv);

  /* Avoid early timer irq cause abort. */

  arm_timer_phy_set_irq_mask(true);

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

  freq = arm_timer_get_freq();
  tmrinfo("%s: cp15 timer(s) running at %" PRIu64 ".%" PRIu64 "MHz\n",
          __func__, freq / 1000000, (freq / 10000) % 100);

  up_alarm_set_lowerhalf(arm_oneshot_initialize());
  up_enable_irq(ARM_ARCH_TIMER_IRQ);
  arm_timer_phy_enable(true);
}

#ifdef CONFIG_SMP
/****************************************************************************
 * Function: arm_timer_secondary_init
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

void arm_timer_secondary_init(unsigned int freq)
{
#ifdef CONFIG_SCHED_TICKLESS
  tmrinfo("arm_arch_timer_secondary_init\n");

  /* Enable int */

  up_enable_irq(ARM_ARCH_TIMER_IRQ);

  /* Start timer */

  arm_timer_phy_enable(true);
#endif
}
#endif
