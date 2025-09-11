/****************************************************************************
 * arch/arm/src/armv7-a/arm_timer.c
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
#include <nuttx/arch.h>
#include <nuttx/bits.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/barriers.h>
#include <arch/irq.h>

#include "arm_timer.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CNT_CTL_ENABLE_BIT      0
#define CNT_CTL_IMASK_BIT       1
#define CNT_CTL_ISTATUS_BIT     2

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#  define GIC_IRQ_TIMER         GIC_IRQ_STM
#else
#  define GIC_IRQ_TIMER         GIC_IRQ_PTM
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct arm_timer_lowerhalf_s
{
  struct oneshot_lowerhalf_s lh;        /* Lower half operations */
  uint32_t                   freq;      /* Timer working clock frequency(Hz) */

  /* which cpu timer is running, -1 indicate timer stoppd */

  int running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int arm_timer_maxdelay(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts);
static int arm_timer_start(struct oneshot_lowerhalf_s *lower,
                           const struct timespec *ts);
static int arm_timer_cancel(struct oneshot_lowerhalf_s *lower,
                            struct timespec *ts);
static int arm_timer_current(struct oneshot_lowerhalf_s *lower,
                             struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_arm_timer_ops =
{
  .max_delay = arm_timer_maxdelay,
  .start     = arm_timer_start,
  .cancel    = arm_timer_cancel,
  .current   = arm_timer_current,
};

static struct arm_timer_lowerhalf_s g_arm_timer_lowerhalf;

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

static inline void arm_timer_virt_set_irq_mask(bool mask)
{
  CP15_MODIFY((uint32_t)mask << CNT_CTL_IMASK_BIT,
              BIT(CNT_CTL_IMASK_BIT), CNTV_CTL);
}

static inline uint64_t nsec_from_count(uint64_t count, uint32_t freq)
{
  uint64_t sec = count / freq;
  uint64_t nsec = (count % freq) * NSEC_PER_SEC / freq;
  return sec * NSEC_PER_SEC + nsec;
}

static inline uint64_t nsec_to_count(uint32_t nsec, uint32_t freq)
{
  return (uint64_t)nsec * freq / NSEC_PER_SEC;
}

static inline uint64_t sec_to_count(uint32_t sec, uint32_t freq)
{
  return (uint64_t)sec * freq;
}

static int arm_timer_maxdelay(struct oneshot_lowerhalf_s *lower_,
                              struct timespec *ts)
{
  uint64_t maxnsec = nsec_from_count(UINT64_MAX, arm_timer_get_freq());

  ts->tv_sec  = maxnsec / NSEC_PER_SEC;
  ts->tv_nsec = maxnsec % NSEC_PER_SEC;

  return 0;
}

static int arm_timer_start(struct oneshot_lowerhalf_s *lower_,
                           const struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint64_t count;

  flags = up_irq_save();

  lower->running = this_cpu();

  count = sec_to_count(ts->tv_sec, lower->freq) +
          nsec_to_count(ts->tv_nsec, lower->freq);

  arm_timer_phy_set_relative(count > UINT32_MAX ? UINT32_MAX : count);

  arm_timer_phy_set_irq_mask(false);

  up_irq_restore(flags);

  return 0;
}

static int arm_timer_cancel(struct oneshot_lowerhalf_s *lower_,
                            struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;
  irqstate_t flags;

  flags = up_irq_save();

  lower->running  = -1;

  arm_timer_phy_set_irq_mask(true);

  up_irq_restore(flags);

  return 0;
}

static int arm_timer_current(struct oneshot_lowerhalf_s *lower_,
                             struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;
  uint64_t nsec = nsec_from_count(arm_timer_phy_count(),
                                  lower->freq);

  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  return 0;
}

static int arm_timer_interrupt(int irq, void *context, void *arg)
{
  struct arm_timer_lowerhalf_s *lower = arg;

  DEBUGASSERT(lower != NULL);

  arm_timer_phy_set_irq_mask(true);

  if (lower->running == this_cpu())
    {
      oneshot_process_callback(&lower->lh);
    }

  return 0;
}

static void arm_timer_initialize_per_cpu(unsigned int freq)
{
  /* Enable timer */

  if (freq)
    {
      arm_timer_set_freq(freq);
    }

  arm_timer_phy_set_irq_mask(true);
  arm_timer_phy_enable(true);

  up_enable_irq(GIC_IRQ_TIMER);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct oneshot_lowerhalf_s *arm_timer_initialize(unsigned int freq)
{
  struct arm_timer_lowerhalf_s *lower = &g_arm_timer_lowerhalf;

  arm_timer_initialize_per_cpu(freq);

  lower->freq    = arm_timer_get_freq();
  lower->lh.ops  = &g_arm_timer_ops;
  lower->running = -1;

  irq_attach(GIC_IRQ_TIMER, arm_timer_interrupt, lower);

  return (struct oneshot_lowerhalf_s *)lower;
}

void arm_timer_secondary_init(unsigned int freq)
{
  arm_timer_initialize_per_cpu(freq);
}
