/****************************************************************************
 * arch/xtensa/src/common/xtensa_oneshot.c
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

#include <assert.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/oneshot.h>

#include "xtensa_counter.h"
#include "xtensa.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * oneshot_lowerhalf_s structure.
 */

struct xoneshot_lowerhalf_s
{
  struct oneshot_lowerhalf_s lh;       /* Lower half operations */
  uint32_t                   freq;     /* Timer working clock frequency(Hz) */
  oneshot_callback_t         callback; /* Current user interrupt callback */
  void                      *arg;      /* Argument passed to upper half callback */
  uint32_t                   irq;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int xtensa_oneshot_maxdelay(struct oneshot_lowerhalf_s *lower,
                                   struct timespec *ts);
static int xtensa_oneshot_start(struct oneshot_lowerhalf_s *lower,
                                oneshot_callback_t callback, void *arg,
                                const struct timespec *ts);
static int xtensa_oneshot_cancel(struct oneshot_lowerhalf_s *lower,
                                 struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_xtensa_oneshot_ops =
{
  .max_delay = xtensa_oneshot_maxdelay,
  .start     = xtensa_oneshot_start,
  .cancel    = xtensa_oneshot_cancel,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint64_t nsec_from_count(uint32_t count, uint32_t freq)
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

static int xtensa_oneshot_start(struct oneshot_lowerhalf_s *lower_,
                                oneshot_callback_t callback, void *arg,
                                const struct timespec *ts)
{
  struct xoneshot_lowerhalf_s *lower =
    (struct xoneshot_lowerhalf_s *)lower_;
  uint32_t count;
  irqstate_t flags;

  flags = enter_critical_section();

  lower->callback = callback;
  lower->arg      = arg;

  count = sec_to_count((uint64_t)ts->tv_sec, lower->freq) +
          nsec_to_count((uint64_t)ts->tv_nsec, lower->freq);

  count = xtensa_getcount() + count;
  xtensa_setcompare(count);

  up_enable_irq(lower->irq);

  leave_critical_section(flags);

  return 0;
}

static int xtensa_oneshot_cancel(struct oneshot_lowerhalf_s *lower_,
                                 struct timespec *ts)
{
  struct xoneshot_lowerhalf_s *lower =
    (struct xoneshot_lowerhalf_s *)lower_;
  irqstate_t flags;

  flags = enter_critical_section();

  lower->callback  = NULL;
  lower->arg       = NULL;

  up_disable_irq(lower->irq);

  leave_critical_section(flags);

  return 0;
}

static int xtensa_oneshot_maxdelay(struct oneshot_lowerhalf_s *lower_,
                                   struct timespec *ts)
{
  struct xoneshot_lowerhalf_s *lower =
    (struct xoneshot_lowerhalf_s *)lower_;

  uint64_t maxnsec = nsec_from_count(UINT32_MAX, lower->freq);

  ts->tv_sec  = maxnsec / NSEC_PER_SEC;
  ts->tv_nsec = maxnsec % NSEC_PER_SEC;

  return 0;
}

static int xtensa_oneshot_interrupt(int irq, void *context, void *arg)
{
  struct xoneshot_lowerhalf_s *lower = arg;
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

struct oneshot_lowerhalf_s *
xtensa_oneshot_initialize(uint32_t irq, uint32_t freq)
{
  struct xoneshot_lowerhalf_s *lower =
      (struct xoneshot_lowerhalf_s *)kmm_zalloc(sizeof(*lower));

  if (lower == NULL)
    {
      return NULL;
    }

  lower->lh.ops = &g_xtensa_oneshot_ops;
  lower->freq   = freq;
  lower->irq    = irq;

  irq_attach(irq, xtensa_oneshot_interrupt, lower);

  return (struct oneshot_lowerhalf_s *)lower;
}
