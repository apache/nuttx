/****************************************************************************
 * arch/arm/src/armv7-a/arm_timer.c
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
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <arch/irq.h>

#include "arm_timer.h"
#include "barriers.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARM_TIMER_CTRL_ENABLE       (1 << 0)
#define ARM_TIMER_CTRL_INT_MASK     (1 << 1)
#define ARM_TIMER_CTRL_INT_STAT     (1 << 2)

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
  oneshot_callback_t         callback;  /* Current user interrupt callback */
  void                       *arg;      /* Argument passed to upper half callback */
  bool init[CONFIG_SMP_NCPUS];          /* True: timer is init */

  /* which cpu timer is running, -1 indicate timer stoppd */

  int running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int arm_timer_maxdelay(struct oneshot_lowerhalf_s *lower,
                              struct timespec *ts);
static int arm_timer_start(struct oneshot_lowerhalf_s *lower,
                           oneshot_callback_t callback, void *arg,
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void arm_timer_set_freq(uint32_t freq)
{
  CP15_SET(CNTFRQ, freq);
  ARM_ISB();
}

static inline uint64_t arm_timer_get_count(void)
{
  ARM_ISB();
  return CP15_GET64(CNTPCT);
}

static inline uint32_t arm_timer_get_ctrl(void)
{
  ARM_ISB();
  return CP15_GET(CNTP_CTL);
}

static inline void arm_timer_set_ctrl(uint32_t ctrl)
{
  CP15_SET(CNTP_CTL, ctrl);
  ARM_ISB();
}

static inline uint32_t arm_timer_get_tval(void)
{
  ARM_ISB();
  return CP15_GET(CNTP_TVAL);
}

static inline void arm_timer_set_tval(uint32_t tval)
{
  CP15_SET(CNTP_TVAL, tval);
  ARM_ISB();
}

static inline uint64_t arm_timer_get_cval(void)
{
  ARM_ISB();
  return CP15_GET64(CNTP_CVAL);
}

static inline void arm_timer_set_cval(uint64_t cval)
{
  CP15_SET64(CNTP_CVAL, cval);
  ARM_ISB();
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
                           oneshot_callback_t callback, void *arg,
                           const struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint64_t count;
  uint32_t ctrl;

  flags = up_irq_save();

  lower->callback = callback;
  lower->arg      = arg;

  if (!lower->init[this_cpu()])
    {
      if (lower->freq)
        {
          arm_timer_set_freq(lower->freq);
        }

      /* Enable timer */

      ctrl = arm_timer_get_ctrl();
      ctrl |= ARM_TIMER_CTRL_ENABLE | ARM_TIMER_CTRL_INT_MASK;
      arm_timer_set_ctrl(ctrl);
#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
      up_enable_irq(GIC_IRQ_STM);
#else
      up_enable_irq(GIC_IRQ_PTM);
#endif

      lower->init[this_cpu()] = true;
    }

  lower->running = this_cpu();

  count = sec_to_count(ts->tv_sec, arm_timer_get_freq()) +
          nsec_to_count(ts->tv_nsec, arm_timer_get_freq());
  arm_timer_set_cval(arm_timer_get_count() + count);

  ctrl = arm_timer_get_ctrl();
  ctrl &= ~ARM_TIMER_CTRL_INT_MASK;
  arm_timer_set_ctrl(ctrl);

  up_irq_restore(flags);

  return 0;
}

static int arm_timer_cancel(struct oneshot_lowerhalf_s *lower_,
                            struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint32_t ctrl;

  flags = up_irq_save();

  lower->callback = NULL;
  lower->arg      = NULL;
  lower->running  = -1;

  ctrl = arm_timer_get_ctrl();
  ctrl |= ARM_TIMER_CTRL_INT_MASK;
  arm_timer_set_ctrl(ctrl);

  up_irq_restore(flags);

  return 0;
}

static int arm_timer_current(struct oneshot_lowerhalf_s *lower_,
                             struct timespec *ts)
{
  uint64_t nsec = nsec_from_count(arm_timer_get_count(),
                                  arm_timer_get_freq());

  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  return 0;
}

static int arm_timer_interrupt(int irq, void *context, void *arg)
{
  struct arm_timer_lowerhalf_s *lower = arg;
  oneshot_callback_t callback;
  void *cbarg;

  DEBUGASSERT(lower != NULL);

  arm_timer_set_ctrl(arm_timer_get_ctrl() | ARM_TIMER_CTRL_INT_MASK);

  if (lower->callback != NULL && lower->running == this_cpu())
    {
      callback        = lower->callback;
      cbarg           = lower->arg;
      lower->callback = NULL;
      lower->arg      = NULL;

      /* Then perform the callback */

      callback(&lower->lh, cbarg);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t arm_timer_get_freq(void)
{
  ARM_ISB();
  return CP15_GET(CNTFRQ);
}

struct oneshot_lowerhalf_s *arm_timer_initialize(unsigned int freq)
{
  struct arm_timer_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower == NULL)
    {
      return NULL;
    }

  lower->lh.ops  = &g_arm_timer_ops;
  lower->freq    = freq;
  lower->running = -1;

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  irq_attach(GIC_IRQ_STM, arm_timer_interrupt, lower);
#else
  irq_attach(GIC_IRQ_PTM, arm_timer_interrupt, lower);
#endif

  return (struct oneshot_lowerhalf_s *)lower;
}
