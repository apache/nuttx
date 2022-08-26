/****************************************************************************
 * arch/arm/src/armv7-r/arm_timer.c
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

#include "arm_timer.h"
#include "barriers.h"
#include "gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define	ARM_TIMER_CTRL_ENABLE       (1 << 0)
#define	ARM_TIMER_CTRL_INT_MASK     (1 << 1)
#define	ARM_TIMER_CTRL_INT_STAT     (1 << 2)

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

static inline uint32_t arm_timer_get_freq(void)
{
  uint32_t freq;

  ARM_ISB();

  __asm__ __volatile__
  (
    "\tmrc   p15, 0, %0, c14, c0, 0\n"  /* Read CNTFRQ */
    : "=r"(freq)
    :
    :
  );

  return freq;
}

static inline void arm_timer_set_freq(uint32_t freq)
{
  __asm__ __volatile__
  (
    "\tmcr   p15, 0, %0, c14, c0, 0\n"  /* Write CNTFRQ */
    :
    : "r"(freq)
    :
  );

  ARM_ISB();
}

static inline uint64_t arm_timer_get_count(void)
{
  uint64_t count;

  ARM_ISB();

  __asm__ __volatile__
  (
    "\tmrrc   p15, 0, %Q0, %R0, c14\n"  /* Read CNTPCT */
    : "=r"(count)
    :
    :
  );

  return count;
}

static inline uint32_t arm_timer_get_ctrl(void)
{
  uint32_t ctrl;

  ARM_ISB();

  __asm__ __volatile__
  (
    "\tmrc   p15, 0, %0, c14, c2, 1\n"  /* Read CNTP_CTL */
    : "=r"(ctrl)
    :
    :
  );

  return ctrl;
}

static inline void arm_timer_set_ctrl(uint32_t ctrl)
{
  __asm__ __volatile__
  (
    "\tmcr   p15, 0, %0, c14, c2, 1\n"  /* Write CNTP_CTL */
    :
    : "r"(ctrl)
    :
  );

  ARM_ISB();
}

static inline uint32_t arm_timer_get_tval(void)
{
  uint32_t tval;

  ARM_ISB();

  __asm__ __volatile__
  (
    "\tmrc   p15, 0, %0, c14, c2, 0\n"  /* Read CNTP_TVAL */
    : "=r"(tval)
    :
    :
  );

  return tval;
}

static inline void arm_timer_set_tval(uint32_t tval)
{
  __asm__ __volatile__
  (
    "\tmcr   p15, 0, %0, c14, c2, 0\n"  /* Write CNTP_TVAL */
    :
    : "r"(tval)
    :
  );

  ARM_ISB();
}

static inline uint64_t nsec_from_count(uint64_t count, uint32_t freq)
{
  return (uint64_t)count * NSEC_PER_SEC / freq;
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
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;

  uint64_t maxnsec = nsec_from_count(UINT32_MAX, lower->freq);

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
  uint32_t count;
  uint32_t ctrl;

  flags = up_irq_save();

  lower->callback = callback;
  lower->arg      = arg;

  count = sec_to_count(ts->tv_sec, lower->freq) +
          nsec_to_count(ts->tv_nsec, lower->freq);
  arm_timer_set_tval(count);

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

  lower->callback  = NULL;
  lower->arg       = NULL;

  ctrl = arm_timer_get_ctrl();
  ctrl |= ARM_TIMER_CTRL_INT_MASK;
  arm_timer_set_ctrl(ctrl);

  up_irq_restore(flags);

  return 0;
}

static int arm_timer_current(struct oneshot_lowerhalf_s *lower_,
                             struct timespec *ts)
{
  struct arm_timer_lowerhalf_s *lower =
    (struct arm_timer_lowerhalf_s *)lower_;

  uint64_t nsec = nsec_from_count(arm_timer_get_count(), lower->freq);

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

  if (lower->callback != NULL)
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

struct oneshot_lowerhalf_s *arm_timer_initialize(unsigned int freq)
{
  struct arm_timer_lowerhalf_s *lower;
  uint32_t ctrl;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower == NULL)
    {
      return NULL;
    }

  if (freq == 0)
    {
      freq = arm_timer_get_freq();
    }

  lower->lh.ops = &g_arm_timer_ops;
  lower->freq   = freq;
  arm_timer_set_freq(freq);

  /* Enable timer, but disable interrupt */

  ctrl = arm_timer_get_ctrl();
  ctrl |= ARM_TIMER_CTRL_ENABLE | ARM_TIMER_CTRL_INT_MASK;
  arm_timer_set_ctrl(ctrl);

  irq_attach(GIC_IRQ_PTM, arm_timer_interrupt, lower);
  up_enable_irq(GIC_IRQ_PTM);

  return (struct oneshot_lowerhalf_s *)lower;
}
