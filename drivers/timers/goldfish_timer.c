/****************************************************************************
 * drivers/timers/goldfish_timer.c
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
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef getreg32
#define getreg32(a)      (*(volatile uint32_t *)(a))
#endif

#ifndef putreg32
#define putreg32(v,a)    (*(volatile uint32_t *)(a) = (v))
#endif

#define GOLDFISH_TIMER_TIME_LOW         0x0  /* Get current time, then return low-order 32-bits. */
#define GOLDFISH_TIMER_TIME_HIGH        0x4  /* Return high 32-bits from previous TIME_LOW read. */
#define GOLDFISH_TIMER_ALARM_LOW        0x8  /* Set low 32-bit value of alarm, then arm it. */
#define GOLDFISH_TIMER_ALARM_HIGH       0xc  /* Set high 32-bit value of alarm. */
#define GOLDFISH_TIMER_IRQ_ENABLED      0x10 /* Enable interrupts. */
#define GOLDFISH_TIMER_CLEAR_ALARM      0x14 /* Clear alarm. */
#define GOLDFISH_TIMER_ALARM_STATUS     0x18 /* Return 1 if alarm is armed, 0 if not. */
#define GOLDFISH_TIMER_CLEAR_INTERRUPT  0x1c /* Clear interrupt. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct goldfish_timer_lowerhalf_s
{
  struct oneshot_lowerhalf_s lh;        /* Lower half operations */
  uintptr_t                  base;      /* Base address of registers */
  spinlock_t                 lock;      /* Lock for interrupt handling */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function uint64_t goldfish_timer_get_nsec(uintptr_t base)
{
  uint32_t l32;
  uint32_t h32;

  l32 = getreg32(base + GOLDFISH_TIMER_TIME_LOW);
  h32 = getreg32(base + GOLDFISH_TIMER_TIME_HIGH);

  return ((uint64_t)h32 << 32) | l32;
}

static inline_function
void goldfish_timer_set_nsec(uintptr_t base, uint64_t expected)
{
  putreg32(1, base + GOLDFISH_TIMER_CLEAR_ALARM);
  putreg32(expected >> 32, base + GOLDFISH_TIMER_ALARM_HIGH);
  putreg32(expected, base + GOLDFISH_TIMER_ALARM_LOW);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_timer_interrupt(int irq,
                                    FAR void *context,
                                    FAR void *arg)
{
  FAR struct goldfish_timer_lowerhalf_s *lower = arg;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_INTERRUPT);

  spin_unlock_irqrestore(&lower->lock, flags);

  /* Then perform the callback */

  oneshot_process_callback(&lower->lh);

  return 0;
}

static clkcnt_t goldfish_timer_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t goldfish_timer_current(struct oneshot_lowerhalf_s *lower_)
{
  FAR struct goldfish_timer_lowerhalf_s *lower =
    (FAR struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint64_t nsec;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  nsec = goldfish_timer_get_nsec(lower->base);

  spin_unlock_irqrestore(&lower->lock, flags);

  return nsec;
}

static void goldfish_timer_start_absolute(struct oneshot_lowerhalf_s *lower_,
                                          clkcnt_t expected)
{
  FAR struct goldfish_timer_lowerhalf_s *lower =
    (FAR struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  goldfish_timer_set_nsec(lower->base, expected);

  spin_unlock_irqrestore(&lower->lock, flags);
}

static void goldfish_timer_start(struct oneshot_lowerhalf_s *lower_,
                                 clkcnt_t delta)
{
  FAR struct goldfish_timer_lowerhalf_s *lower =
    (FAR struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  delta += goldfish_timer_get_nsec(lower->base);

  goldfish_timer_set_nsec(lower->base, delta);

  spin_unlock_irqrestore(&lower->lock, flags);
}

static void goldfish_timer_cancel(struct oneshot_lowerhalf_s *lower_)
{
  struct goldfish_timer_lowerhalf_s *lower =
    (struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  goldfish_timer_set_nsec(lower->base, UINT64_MAX);

  spin_unlock_irqrestore(&lower->lock, flags);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_goldfish_timer_ops =
{
  .current        = goldfish_timer_current,
  .start          = goldfish_timer_start,
  .start_absolute = goldfish_timer_start_absolute,
  .cancel         = goldfish_timer_cancel,
  .max_delay      = goldfish_timer_max_delay,
};

static struct goldfish_timer_lowerhalf_s g_goldfish_timer_lowerhalf =
{
  .lh.ops = &g_goldfish_timer_ops
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *
goldfish_timer_initialize(uintptr_t base, int irq)
{
  FAR struct goldfish_timer_lowerhalf_s *lower = &g_goldfish_timer_lowerhalf;

  lower->base = base;

  spin_lock_init(&lower->lock);

  /* Enable timer, but disable interrupt */

  oneshot_count_init(&lower->lh, NSEC_PER_SEC);

  putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_INTERRUPT);
  goldfish_timer_set_nsec(lower->base, UINT64_MAX);
  putreg32(1, base + GOLDFISH_TIMER_IRQ_ENABLED);

  irq_attach(irq, goldfish_timer_interrupt, lower);
  up_enable_irq(irq);

  return (struct oneshot_lowerhalf_s *)lower;
}
