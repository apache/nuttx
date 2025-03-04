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
  oneshot_callback_t         callback;  /* Current user interrupt callback */
  FAR void                   *arg;      /* Argument passed to upper half callback */
  uintptr_t                  base;      /* Base address of registers */
  spinlock_t                 lock;      /* Lock for interrupt handling */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_timer_maxdelay(FAR struct oneshot_lowerhalf_s *lower,
                                   FAR struct timespec *ts);
static int goldfish_timer_start(FAR struct oneshot_lowerhalf_s *lower,
                                FAR oneshot_callback_t callback,
                                FAR void *arg,
                                FAR const struct timespec *ts);
static int goldfish_timer_cancel(FAR struct oneshot_lowerhalf_s *lower,
                                 FAR struct timespec *ts);
static int goldfish_timer_current(FAR struct oneshot_lowerhalf_s *lower,
                                  FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_goldfish_timer_ops =
{
  .max_delay = goldfish_timer_maxdelay,
  .start     = goldfish_timer_start,
  .cancel    = goldfish_timer_cancel,
  .current   = goldfish_timer_current,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_timer_maxdelay(FAR struct oneshot_lowerhalf_s *lower_,
                                   FAR struct timespec *ts)
{
  ts->tv_sec  = UINT32_MAX;
  ts->tv_nsec = UINT32_MAX;

  return 0;
}

static int goldfish_timer_start(FAR struct oneshot_lowerhalf_s *lower_,
                                FAR oneshot_callback_t callback,
                                FAR void *arg,
                                FAR const struct timespec *ts)
{
  FAR struct goldfish_timer_lowerhalf_s *lower =
    (FAR struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint64_t nsec;
  uint32_t l32;
  uint32_t h32;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  lower->callback = callback;
  lower->arg      = arg;

  nsec  = ts->tv_sec * 1000000000 + ts->tv_nsec;
  l32   = getreg32(lower->base + GOLDFISH_TIMER_TIME_LOW);
  h32   = getreg32(lower->base + GOLDFISH_TIMER_TIME_HIGH);
  nsec += ((uint64_t)h32 << 32) | l32;

  putreg32(1, lower->base + GOLDFISH_TIMER_IRQ_ENABLED);
  putreg32(nsec >> 32, lower->base + GOLDFISH_TIMER_ALARM_HIGH);
  putreg32(nsec, lower->base + GOLDFISH_TIMER_ALARM_LOW);

  spin_unlock_irqrestore(&lower->lock, flags);

  return 0;
}

static int goldfish_timer_cancel(FAR struct oneshot_lowerhalf_s *lower_,
                                 FAR struct timespec *ts)
{
  struct goldfish_timer_lowerhalf_s *lower =
    (struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  lower->callback  = NULL;
  lower->arg       = NULL;

  putreg32(0, lower->base + GOLDFISH_TIMER_IRQ_ENABLED);
  putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_ALARM);

  spin_unlock_irqrestore(&lower->lock, flags);

  return 0;
}

static int goldfish_timer_current(FAR struct oneshot_lowerhalf_s *lower_,
                                  FAR struct timespec *ts)
{
  FAR struct goldfish_timer_lowerhalf_s *lower =
    (FAR struct goldfish_timer_lowerhalf_s *)lower_;
  irqstate_t flags;
  uint32_t l32;
  uint32_t h32;
  uint64_t nsec;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  l32 = getreg32(lower->base + GOLDFISH_TIMER_TIME_LOW);
  h32 = getreg32(lower->base + GOLDFISH_TIMER_TIME_HIGH);
  nsec = ((uint64_t)h32 << 32) | l32;

  ts->tv_sec  = nsec / NSEC_PER_SEC;
  ts->tv_nsec = nsec % NSEC_PER_SEC;

  spin_unlock_irqrestore(&lower->lock, flags);

  return 0;
}

static int goldfish_timer_interrupt(int irq,
                                    FAR void *context,
                                    FAR void *arg)
{
  FAR struct goldfish_timer_lowerhalf_s *lower = arg;
  oneshot_callback_t callback = NULL;
  irqstate_t flags;
  void *cbarg;

  DEBUGASSERT(lower != NULL);

  flags = spin_lock_irqsave(&lower->lock);

  putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_ALARM);
  putreg32(1, lower->base + GOLDFISH_TIMER_CLEAR_INTERRUPT);

  if (lower->callback != NULL)
    {
      callback        = lower->callback;
      cbarg           = lower->arg;
      lower->callback = NULL;
      lower->arg      = NULL;
    }

  spin_unlock_irqrestore(&lower->lock, flags);

  /* Then perform the callback */

  if (callback)
    {
      callback(&lower->lh, cbarg);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *
goldfish_timer_initialize(uintptr_t base, int irq)
{
  FAR struct goldfish_timer_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower == NULL)
    {
      return NULL;
    }

  lower->lh.ops = &g_goldfish_timer_ops;
  lower->base = base;

  spin_lock_init(&lower->lock);

  /* Enable timer, but disable interrupt */

  irq_attach(irq, goldfish_timer_interrupt, lower);
  up_enable_irq(irq);
  putreg32(0, base + GOLDFISH_TIMER_IRQ_ENABLED);

  return (struct oneshot_lowerhalf_s *)lower;
}
