/****************************************************************************
 * include/nuttx/seqlock.h
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

#ifndef __INCLUDE_NUTTX_SEQLOCK_H
#define __INCLUDE_NUTTX_SEQLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock_type.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: seqlock_init
 *
 * Description:
 *   init seqlock
 *
 * Input Parameters:
 *   seqcount_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline_function void seqlock_init(FAR seqcount_t *s)
{
  s->sequence = 0u;
#ifdef CONFIG_SMP
  SMP_WMB();
#endif
}

/****************************************************************************
 * Name: read_seqbegin
 *
 * Description:
 *   This is a primitive counting synchronization mechanism
 *   that enables lock-free reading.
 *
 * Input Parameters:
 *   seqcount_t
 *
 * Returned Value:
 *   seq - Used to determine whether the state has changed during reading.
 *
 ****************************************************************************/

static inline_function
uint32_t read_seqbegin(FAR const seqcount_t *s)
{
  uint32_t seq;

#ifdef CONFIG_SMP
  seq = atomic_read_acquire((FAR atomic_t *)&s->sequence) & (~1u);
#else
  seq = s->sequence;
  SMP_RMB();
#endif
  return seq;
}

/****************************************************************************
 * Name: read_seqretry
 *
 * Description:
 *   This is a primitive counting synchronization mechanism
 *   that enables lock-free reading.
 *
 * Input Parameters:
 *   seqcount_t
 *   start - Used to determine whether the state has changed during reading.
 *
 * Returned Value:
 *   0 indicate need retry
 *
 ****************************************************************************/

static inline_function
uint32_t read_seqretry(FAR const seqcount_t *s, uint32_t start)
{
  uint32_t seq;

  /* Ensure all load operations before are completed. */

  SMP_RMB();

#ifdef CONFIG_SMP
  seq = atomic_read((FAR atomic_t *)&s->sequence);
#else
  seq = s->sequence;
#endif

  return predict_false(seq != start);
}

/****************************************************************************
 * Name: write_seqlock_irqsave
 *
 * Description:
 *   This is a primitive counting synchronization mechanism
 *   that enables lock-free reading. write need spinlock to protect
 *
 * Input Parameters:
 *   seqcount_t
 *
 * Returned Value:
 *   irqstate
 *
 ****************************************************************************/

static inline_function
irqstate_t write_seqlock_irqsave(FAR seqcount_t *s)
{
  irqstate_t flags = up_irq_save();

#ifdef CONFIG_SMP
  for (; ; )
    {
      uint32_t sequence = atomic_read((FAR atomic_t *)&s->sequence);

      if (predict_true((sequence & 1u) == 0u))
        {
          /* Try to acquire the lock ownership. */

          if (atomic_cmpxchg_acquire((FAR atomic_t *)&s->sequence,
                                     (FAR atomic_t *)&sequence,
                                     sequence + 1u))
            {
              break;
            }
        }

      /* CPU Relax and retry. */
    }
#else
  s->sequence++;
#endif

  return flags;
}

/****************************************************************************
 * Name: write_sequnlock_irqrestore
 *
 * Description:
 *   This is a primitive counting synchronization mechanism
 *   that enables lock-free reading. write need spinlock to protect
 *
 * Input Parameters:
 *   seqcount_t
 *   irqstate_t - irqstate used to restore
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline_function
void write_sequnlock_irqrestore(FAR seqcount_t *s, irqstate_t flags)
{
#ifdef CONFIG_SMP
  atomic_set_release((FAR atomic_t *)&s->sequence, s->sequence + 1u);
#endif
  up_irq_restore(flags);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SEQLOCK_H */
