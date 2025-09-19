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

#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define SEQLOCK_INITIALIZER {0, SP_UNLOCKED}

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/****************************************************************************
 * Sequence counters (seqcount_t)
 ****************************************************************************/

typedef struct seqclock
{
  volatile unsigned int sequence;
  spinlock_t lock;
} seqcount_t;

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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
  s->sequence = 0;
  spin_lock_init(&s->lock);
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
unsigned int read_seqbegin(FAR const seqcount_t *s)
{
  unsigned int seq;

  while (predict_false((seq = s->sequence) & 1));

#ifdef CONFIG_SMP
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
unsigned int read_seqretry(FAR const seqcount_t *s, unsigned int start)
{
  unsigned int seq;

#ifdef CONFIG_SMP
  UP_DMB();
#endif

  seq = s->sequence;
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
  irqstate_t flags = spin_lock_irqsave(&s->lock);

  s->sequence++;
  SMP_WMB();
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
void write_sequnlock_irqrestore(seqcount_t *s, irqstate_t flags)
{
  SMP_WMB();
  s->sequence++;
  spin_unlock_irqrestore(&s->lock, flags);
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SEQLOCK_H */
