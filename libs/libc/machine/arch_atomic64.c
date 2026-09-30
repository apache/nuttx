/****************************************************************************
 * libs/libc/machine/arch_atomic64.c
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

/* 8 byte atomics are not lock free on every target.  An arch may only have a
 * 32 bit atomic instruction (TriCore swap.w/cmpswap.w for instance) and a
 * toolchain without 64 bit support emits calls to the __atomic_*_8 helpers
 * that would otherwise come from libatomic, which NuttX does not link.
 *
 * The helpers are implemented here on top of a single spinlock.  A spinlock
 * rather than a plain up_irq_save() is needed because disabling interrupts
 * only excludes the local CPU: on SMP another CPU could still enter the same
 * critical section and corrupt the 64 bit value.  The interrupt state is
 * still saved (spin_lock_irqsave) so that an ISR on this CPU cannot deadlock
 * against a holder it interrupted.
 *
 * <arch/atomic.h> is deliberately not reused: its macros are built around
 * the native word size and a 64 bit access would be silently truncated.  All
 * symbols are weak, so a toolchain or arch with a native 64 bit
 * implementation still wins at link time.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Every 64 bit atomic serializes on this lock.  The granularity is coarse,
 * but 64 bit atomics are rare enough that a single lock is not a bottleneck.
 */

static spinlock_t g_atomic64_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline irqstate_t atomic64_lock(void)
{
  return spin_lock_irqsave(&g_atomic64_lock);
}

static inline void atomic64_unlock(irqstate_t flags)
{
  spin_unlock_irqrestore(&g_atomic64_lock, flags);
}

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ATOMIC64_STORE(func, t)                                       \
  weak_function                                                       \
  void func(FAR volatile void *ptr, t value, int memorder)            \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
                                                                      \
    *(FAR t *)ptr = value;                                            \
                                                                      \
    atomic64_unlock(irqstate);                                        \
  }

#define ATOMIC64_LOAD(func, t)                                        \
  weak_function                                                       \
  t func(FAR const volatile void *ptr, int memorder)                  \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
                                                                      \
    t ret = *(FAR t *)ptr;                                            \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_EXCHANGE(func, t)                                    \
  weak_function                                                       \
  t func(FAR volatile void *ptr, t value, int memorder)               \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
                                                                      \
    t ret = *tmp;                                                     \
    *tmp = value;                                                     \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_COMPARE_EXCHANGE(func, t)                            \
  weak_function                                                       \
  bool func(FAR volatile void *mem, FAR volatile void *expect,        \
            t desired, bool weak, int success, int failure)           \
  {                                                                   \
    bool ret = false;                                                 \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmpmem = (FAR t *)mem;                                     \
    FAR t *tmpexp = (FAR t *)expect;                                  \
                                                                      \
    if (*tmpmem == *tmpexp)                                           \
      {                                                               \
        ret = true;                                                   \
        *tmpmem = desired;                                            \
      }                                                               \
    else                                                              \
      {                                                               \
        *tmpexp = *tmpmem;                                            \
      }                                                               \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_FLAGS_TEST_AND_SET(func, t)                          \
  weak_function                                                       \
  t func(FAR volatile void *ptr, int memorder)                        \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
    t ret = *tmp;                                                     \
                                                                      \
    *tmp = 1;                                                         \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_FETCH_OP(func, t, op)                                \
  weak_function                                                       \
  t func(FAR volatile void *ptr, t value, int memorder)               \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
    t ret = *tmp;                                                     \
                                                                      \
    *tmp = *tmp op value;                                             \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_OP_FETCH(func, t, op)                                \
  weak_function                                                       \
  t func(FAR volatile void *ptr, t value)                             \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
    t ret;                                                            \
                                                                      \
    *tmp = *tmp op value;                                             \
    ret = *tmp;                                                       \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_NAND_FETCH(func, t)                                  \
  weak_function                                                       \
  t func(FAR volatile void *ptr, t value)                             \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
    t ret;                                                            \
                                                                      \
    *tmp = ~(*tmp & value);                                           \
    ret = *tmp;                                                       \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_BOOL_CMP_SWAP(func, t)                               \
  weak_function                                                       \
  bool func(FAR volatile void *ptr, t oldvalue, t newvalue)           \
  {                                                                   \
    bool ret = false;                                                 \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        ret = true;                                                   \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_VAL_CMP_SWAP(func, t)                                \
  weak_function                                                       \
  t func(FAR volatile void *ptr, t oldvalue, t newvalue)              \
  {                                                                   \
    irqstate_t irqstate = atomic64_lock();                            \
    FAR t *tmp = (FAR t *)ptr;                                        \
    t ret = *tmp;                                                     \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic64_unlock(irqstate);                                        \
    return ret;                                                       \
  }

#define ATOMIC64_DEFINE(prefix, t, n)                                 \
  ATOMIC64_STORE(prefix ## _store_ ## n, t)                           \
  ATOMIC64_LOAD(prefix ## _load_ ## n, t)                             \
  ATOMIC64_EXCHANGE(prefix ## _exchange_ ## n, t)                     \
  ATOMIC64_COMPARE_EXCHANGE(prefix ## _compare_exchange_ ## n, t)     \
  ATOMIC64_FLAGS_TEST_AND_SET(prefix ## _flags_test_and_set_ ## n, t) \
  ATOMIC64_FETCH_OP(prefix ## _fetch_add_ ## n, t, +)                 \
  ATOMIC64_FETCH_OP(prefix ## _fetch_sub_ ## n, t, -)                 \
  ATOMIC64_FETCH_OP(prefix ## _fetch_and_ ## n, t, &)                 \
  ATOMIC64_FETCH_OP(prefix ## _fetch_or_ ## n, t, |)                  \
  ATOMIC64_FETCH_OP(prefix ## _fetch_xor_ ## n, t, ^)

#define SYNC64_DEFINE(prefix, t, n)                                   \
  ATOMIC64_OP_FETCH(prefix ## _add_and_fetch_ ## n, t, +)             \
  ATOMIC64_OP_FETCH(prefix ## _sub_and_fetch_ ## n, t, -)             \
  ATOMIC64_OP_FETCH(prefix ## _or_and_fetch_ ## n, t, |)              \
  ATOMIC64_OP_FETCH(prefix ## _and_and_fetch_ ## n, t, &)             \
  ATOMIC64_OP_FETCH(prefix ## _xor_and_fetch_ ## n, t, ^)             \
  ATOMIC64_NAND_FETCH(prefix ## _nand_and_fetch_ ## n, t)             \
  ATOMIC64_BOOL_CMP_SWAP(prefix ## _bool_compare_and_swap_ ## n, t)   \
  ATOMIC64_VAL_CMP_SWAP(prefix ## _val_compare_and_swap_ ## n, t)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: atomic_*_8 and __atomic_*_8
 ****************************************************************************/

#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
ATOMIC64_DEFINE(atomic, int64_t, 8)
#endif

ATOMIC64_DEFINE(__atomic, uint64_t, 8)

/* Clang define the __sync builtins, add #ifndef to avoid
 * redefined/redeclared problem.
 */

#ifndef __clang__

/****************************************************************************
 * Name: sync_*_8 and __sync_*_8
 ****************************************************************************/

#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
SYNC64_DEFINE(sync, uint64_t, 8)
#endif

SYNC64_DEFINE(__sync, uint64_t, 8)

#endif /* __clang__ */
