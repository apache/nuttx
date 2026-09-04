/****************************************************************************
 * include/nuttx/lib/arch_atomic.h
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

#ifndef __INCLUDE_NUTTX_LIB_ARCH_ATOMIC_H
#define __INCLUDE_NUTTX_LIB_ARCH_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>

#include <stdbool.h>
#include <stdint.h>

#if !defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__)
#  include <sched.h>
#elif defined(CONFIG_LIBC_ATOMIC_HWSPINLOCK)
#  include <nuttx/hwspinlock/hwspinlock.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef ARCH_ATOMIC_SPECIFIER
#  define ARCH_ATOMIC_SPECIFIER static always_inline_function
#endif

#if !defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__)
#  define atomic_lock() (sched_lock(), (irqstate_t)0)
#  define atomic_unlock(flags) ((void)(flags), sched_unlock())
#elif defined(CONFIG_LIBC_ATOMIC_HWSPINLOCK)
extern struct hwspinlock_dev_s g_atomic_hwspinlock;
#  define atomic_lock() \
     hwspin_lock_irqsave(&g_atomic_hwspinlock)
#  define atomic_unlock(flags) \
     hwspin_unlock_restore(&g_atomic_hwspinlock, flags)
#else
#  define atomic_lock() up_irq_save()
#  define atomic_unlock(flags) up_irq_restore(flags)
#endif

#undef ARCH_ATOMIC_STORE
#undef ARCH_ATOMIC_LOAD
#undef ARCH_ATOMIC_EXCHANGE
#undef ARCH_ATOMIC_COMPARE_EXCHANGE
#undef ARCH_ATOMIC_FLAGS_TEST_AND_SET
#undef ARCH_ATOMIC_FETCH_OP
#undef ARCH_ATOMIC_DEFINE
#undef ARCH_SYNC_OP_FETCH
#undef ARCH_SYNC_NAND_FETCH
#undef ARCH_SYNC_BOOL_CMP_SWAP
#undef ARCH_SYNC_VAL_CMP_SWAP
#undef ARCH_SYNC_SYNCHRONIZE
#undef ARCH_SYNC_DEFINE

#define ARCH_ATOMIC_STORE(func, t)                                       \
  ARCH_ATOMIC_SPECIFIER                                                  \
  void func(FAR volatile void *ptr, t value, int memorder)               \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
                                                                         \
    *(FAR t *)ptr = value;                                               \
                                                                         \
    atomic_unlock(irqstate);                                             \
  }

#define ARCH_ATOMIC_LOAD(func, t)                                        \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR const volatile void *ptr, int memorder)                     \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
                                                                         \
    t ret = *(FAR t *)ptr;                                               \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_ATOMIC_EXCHANGE(func, t)                                    \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, t value, int memorder)                  \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
                                                                         \
    t ret = *tmp;                                                        \
    *tmp = value;                                                        \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_ATOMIC_COMPARE_EXCHANGE(func, t)                            \
  ARCH_ATOMIC_SPECIFIER                                                  \
  bool func(FAR volatile void *mem, FAR volatile void *expect,           \
            t desired, bool weak, int success, int failure)              \
  {                                                                      \
    bool ret = false;                                                    \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmpmem = (FAR t *)mem;                                        \
    FAR t *tmpexp = (FAR t *)expect;                                     \
                                                                         \
    if (*tmpmem == *tmpexp)                                              \
      {                                                                  \
        ret = true;                                                      \
        *tmpmem = desired;                                               \
      }                                                                  \
    else                                                                 \
      {                                                                  \
        *tmpexp = *tmpmem;                                               \
      }                                                                  \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_ATOMIC_FLAGS_TEST_AND_SET(func, t)                          \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, int memorder)                           \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
    t ret = *tmp;                                                        \
                                                                         \
    *(FAR t *)ptr = 1;                                                   \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_ATOMIC_FETCH_OP(func, t, op)                                \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, t value, int memorder)                  \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
    t ret = *tmp;                                                        \
                                                                         \
    *tmp = *tmp op value;                                                \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_ATOMIC_DEFINE(prefix, t, n)                                 \
  ARCH_ATOMIC_STORE(prefix ## _store_ ## n, t)                           \
  ARCH_ATOMIC_LOAD(prefix ## _load_ ## n, t)                             \
  ARCH_ATOMIC_EXCHANGE(prefix ## _exchange_ ## n, t)                     \
  ARCH_ATOMIC_COMPARE_EXCHANGE(prefix ## _compare_exchange_ ## n, t)     \
  ARCH_ATOMIC_FLAGS_TEST_AND_SET(prefix ## _flags_test_and_set_ ## n, t) \
  ARCH_ATOMIC_FETCH_OP(prefix ## _fetch_add_ ## n, t, +)                 \
  ARCH_ATOMIC_FETCH_OP(prefix ## _fetch_sub_ ## n, t, -)                 \
  ARCH_ATOMIC_FETCH_OP(prefix ## _fetch_and_ ## n, t, &)                 \
  ARCH_ATOMIC_FETCH_OP(prefix ## _fetch_or_ ## n, t, |)                  \
  ARCH_ATOMIC_FETCH_OP(prefix ## _fetch_xor_ ## n, t, ^)

#define ARCH_SYNC_OP_FETCH(func, t, op)                                  \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, t value)                                \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
    t ret;                                                               \
                                                                         \
    *tmp = *tmp op value;                                                \
    ret = *tmp;                                                          \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_SYNC_NAND_FETCH(func, t)                                    \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, t value)                                \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
    t ret;                                                               \
                                                                         \
    *tmp = ~(*tmp & value);                                              \
    ret = *tmp;                                                          \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_SYNC_BOOL_CMP_SWAP(func, t)                                 \
  ARCH_ATOMIC_SPECIFIER                                                  \
  bool func(FAR volatile void *ptr, t oldvalue, t newvalue)              \
  {                                                                      \
    bool ret = false;                                                    \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
                                                                         \
    if (*tmp == oldvalue)                                                \
      {                                                                  \
        ret = true;                                                      \
        *tmp = newvalue;                                                 \
      }                                                                  \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#define ARCH_SYNC_VAL_CMP_SWAP(func, t)                                  \
  ARCH_ATOMIC_SPECIFIER                                                  \
  t func(FAR volatile void *ptr, t oldvalue, t newvalue)                 \
  {                                                                      \
    irqstate_t irqstate = atomic_lock();                                 \
    FAR t *tmp = (FAR t *)ptr;                                           \
    t ret = *tmp;                                                        \
                                                                         \
    if (*tmp == oldvalue)                                                \
      {                                                                  \
        *tmp = newvalue;                                                 \
      }                                                                  \
                                                                         \
    atomic_unlock(irqstate);                                             \
    return ret;                                                          \
  }

#ifdef UP_DMB
#  define ARCH_SYNC_BARRIER() UP_DMB()
#else
#  define ARCH_SYNC_BARRIER()
#endif

#define ARCH_SYNC_SYNCHRONIZE(prefix)                                    \
  ARCH_ATOMIC_SPECIFIER                                                  \
  void prefix ## _synchronize(void)                                      \
  {                                                                      \
    ARCH_SYNC_BARRIER();                                                 \
  }

#define ARCH_SYNC_DEFINE(prefix, t, n)                                   \
  ARCH_SYNC_OP_FETCH(prefix ## _add_and_fetch_ ## n, t, +)               \
  ARCH_SYNC_OP_FETCH(prefix ## _sub_and_fetch_ ## n, t, -)               \
  ARCH_SYNC_OP_FETCH(prefix ## _or_and_fetch_ ## n, t, |)                \
  ARCH_SYNC_OP_FETCH(prefix ## _and_and_fetch_ ## n, t, &)               \
  ARCH_SYNC_OP_FETCH(prefix ## _xor_and_fetch_ ## n, t, ^)               \
  ARCH_SYNC_NAND_FETCH(prefix ## _nand_and_fetch_ ## n, t)               \
  ARCH_SYNC_BOOL_CMP_SWAP(prefix ## _bool_compare_and_swap_ ## n, t)     \
  ARCH_SYNC_VAL_CMP_SWAP(prefix ## _val_compare_and_swap_ ## n, t)

#if defined(CONFIG_LIBC_ATOMIC_HWSPINLOCK) || \
    defined(CONFIG_LIBC_ATOMIC_IRQ)
#  define ARCH_HAVE_ATOMIC_1
#  define ARCH_HAVE_ATOMIC_2
#  define ARCH_HAVE_ATOMIC_4
#  define ARCH_HAVE_ATOMIC_8

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

ARCH_ATOMIC_DEFINE(atomic, uint8_t, 1)
ARCH_ATOMIC_DEFINE(atomic, uint16_t, 2)
ARCH_ATOMIC_DEFINE(atomic, int32_t, 4)
ARCH_ATOMIC_DEFINE(atomic, int64_t, 8)

ARCH_SYNC_DEFINE(sync, uint8_t, 1)
ARCH_SYNC_DEFINE(sync, uint16_t, 2)
ARCH_SYNC_DEFINE(sync, uint32_t, 4)
ARCH_SYNC_DEFINE(sync, uint64_t, 8)
#endif

#endif /* __INCLUDE_NUTTX_LIB_ARCH_ATOMIC_H */
