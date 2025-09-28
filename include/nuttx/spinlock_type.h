/****************************************************************************
 * include/nuttx/spinlock_type.h
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

#ifndef __INCLUDE_NUTTX_SPINLOCK_TYPE_H
#define __INCLUDE_NUTTX_SPINLOCK_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/atomic.h>
#if defined(CONFIG_SPINLOCK_DEBUG)
#  include <nuttx/queue.h>
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

#if defined(CONFIG_SPINLOCK_DEBUG)
typedef struct spinlock_debug_s
{
  sq_entry_t flink;
  FAR struct tcb_s *holder;
#  if defined(CONFIG_SPINLOCK_BACKTRACE)
  FAR void *stack;
#  endif
} spinlock_debug_t;
#  define SPINLOCK_DEBUG_INFO spinlock_debug_t info;
#  if defined(CONFIG_SPINLOCK_BACKTRACE)
#    define SPINLOCK_DEBUG_INIT {{NULL}, NULL, NULL},
#  else
#    define SPINLOCK_DEBUG_INIT {{NULL}, NULL},
#  endif
#else
#  define SPINLOCK_DEBUG_INFO
#  define SPINLOCK_DEBUG_INIT
#endif

#if defined(CONFIG_RW_SPINLOCK)
typedef struct rwlock_s
{
  atomic_t lock;
  SPINLOCK_DEBUG_INFO
} rwlock_t;
#  define RW_SP_UNLOCKED      0
#  define RW_SP_READ_LOCKED   1
#  define RW_SP_WRITE_LOCKED -1
#endif

#ifndef CONFIG_SPINLOCK
#  define SP_UNLOCKED {0, SPINLOCK_DEBUG_INIT} /* The Un-locked state */
#  define SP_LOCKED   {1, SPINLOCK_DEBUG_INIT} /* The Locked state */

typedef struct spinlock_s
{
  atomic_t lock;
  SPINLOCK_DEBUG_INFO
} spinlock_t;
#elif defined(CONFIG_TICKET_SPINLOCK)

typedef struct spinlock_s
{
  atomic_t owner;
  atomic_t next;
  SPINLOCK_DEBUG_INFO
} spinlock_t;

#  define SP_UNLOCKED {0, 0, SPINLOCK_DEBUG_INIT}
#  define SP_LOCKED   {0, 1, SPINLOCK_DEBUG_INIT}

#else

typedef struct spinlock_s
{
  atomic_t lock;
  SPINLOCK_DEBUG_INFO
} spinlock_t;

#  define SP_UNLOCKED {0, SPINLOCK_DEBUG_INIT}
#  define SP_LOCKED   {1, SPINLOCK_DEBUG_INIT}

#endif /* CONFIG_SPINLOCK */

#define RSPINLOCK_CPU_INVALID (-1)
#define RSPINLOCK_INITIALIZER {{0}, SPINLOCK_DEBUG_INIT}

typedef struct rspinlock_s
{
  union
  {
    /* Which cpu is holding spinlock, and taking recursive count */

    atomic_t val;       /* Shared lock value. */
    uint32_t local_val; /* Local copy to avoid atomic read/write. */
  };

  SPINLOCK_DEBUG_INFO
} rspinlock_t;

#define RSPINLOCK_OWNER(val)          ((uint32_t)(val) >> 16u)    /* The high 16 bits are the owner */
#define RSPINLOCK_COUNT(val)          ((uint32_t)(val) & 0xffffu) /* The low 16 bits are the count */
#define RSPINLOCK_VALUE(owner, count) (((uint32_t)(owner) << 16u) | (count))

#define SEQLOCK_INITIALIZER { 0u }

typedef struct seqclock
{
#ifdef CONFIG_SMP
  atomic_t sequence;
#else
  volatile uint32_t sequence;
#endif
} seqcount_t;

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SPINLOCK_TYPE_H */
