/****************************************************************************
 * include/nuttx/atomic.h
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

#ifndef __INCLUDE_NUTTX_ATOMIC_H
#define __INCLUDE_NUTTX_ATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#if defined(__has_include) && !defined(CONFIG_LIBC_ARCH_ATOMIC)
#  if __has_include(<atomic>) && defined(__cplusplus)
extern "C++"
{
#    include <atomic>
#    define ATOMIC_FUNC(f, n) atomic_##f##_explicit

  using std::atomic_load_explicit;
  using std::atomic_store_explicit;
  using std::atomic_exchange_explicit;
  using std::atomic_compare_exchange_strong_explicit;
  using std::atomic_compare_exchange_weak_explicit;
  using std::atomic_fetch_add_explicit;
  using std::atomic_fetch_sub_explicit;
  using std::atomic_fetch_and_explicit;
  using std::atomic_fetch_or_explicit;
  using std::atomic_fetch_xor_explicit;

  typedef volatile int32_t atomic_t;
  typedef volatile int64_t atomic64_t;
}
#  elif __has_include(<stdatomic.h>) && \
        ((defined(__cplusplus) && __cplusplus >= 201103L) || \
         (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L)) && \
         !defined(__STDC_NO_ATOMICS__)
#    if !defined(__clang__) && defined(__cplusplus)
#      define _Atomic
#    endif
#    include <stdatomic.h>
#    define ATOMIC_FUNC(f, n) atomic_##f##_explicit

  typedef volatile _Atomic int32_t atomic_t;
  typedef volatile _Atomic int64_t atomic64_t;
#  endif
#endif

#ifndef ATOMIC_FUNC
#  define __ATOMIC_RELAXED 0
#  define __ATOMIC_CONSUME 1
#  define __ATOMIC_ACQUIRE 2
#  define __ATOMIC_RELEASE 3
#  define __ATOMIC_ACQ_REL 4
#  define __ATOMIC_SEQ_CST 5

#  define USE_ARCH_ATOMIC  1

#  define ATOMIC_FUNC(f, n) nx_atomic_##f##_##n

#  define nx_atomic_compare_exchange_weak_4(obj, expect, desired, success, failure) \
     nx_atomic_compare_exchange_4(obj, expect, desired, true, success, failure)
#  define nx_atomic_compare_exchange_weak_8(obj, expect, desired, success, failure) \
     nx_atomic_compare_exchange_8(obj, expect, desired, true, success, failure)
#  define nx_atomic_compare_exchange_strong_4(obj, expect, desired, success, failure) \
     nx_atomic_compare_exchange_4(obj, expect, desired, false, success, failure)
#  define nx_atomic_compare_exchange_strong_8(obj, expect, desired, success, failure) \
     nx_atomic_compare_exchange_8(obj, expect, desired, false, success, failure)

typedef volatile int32_t atomic_t;
typedef volatile int64_t atomic64_t;
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define atomic_set(obj, val)                  ATOMIC_FUNC(store, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic_set_release(obj, val)          ATOMIC_FUNC(store, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic64_set(obj, val)                ATOMIC_FUNC(store, 8)(obj, val, __ATOMIC_RELAXED)
#define atomic64_set_release(obj, val)        ATOMIC_FUNC(store, 8)(obj, val, __ATOMIC_RELEASE)

#define atomic_read(obj)                      ATOMIC_FUNC(load, 4)(obj, __ATOMIC_RELAXED)
#define atomic_read_acquire(obj)              ATOMIC_FUNC(load, 4)(obj, __ATOMIC_ACQUIRE)
#define atomic64_read(obj)                    ATOMIC_FUNC(load, 8)(obj, __ATOMIC_RELAXED)
#define atomic64_read_acquire(obj)            ATOMIC_FUNC(load, 8)(obj, __ATOMIC_ACQUIRE)

#define atomic_fetch_add_acquire(obj, val)    ATOMIC_FUNC(fetch_add, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_fetch_add_release(obj, val)    ATOMIC_FUNC(fetch_add, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_fetch_add_relaxed(obj, val)    ATOMIC_FUNC(fetch_add, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_fetch_add(obj, val)          ATOMIC_FUNC(fetch_add, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_fetch_add_acquire(obj, val)  ATOMIC_FUNC(fetch_add, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_fetch_add_release(obj, val)  ATOMIC_FUNC(fetch_add, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_fetch_add_relaxed(obj, val)  ATOMIC_FUNC(fetch_add, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_fetch_sub_acquire(obj, val)    ATOMIC_FUNC(fetch_sub, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_fetch_sub_release(obj, val)    ATOMIC_FUNC(fetch_sub, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_fetch_sub_relaxed(obj, val)    ATOMIC_FUNC(fetch_sub, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_fetch_sub(obj, val)          ATOMIC_FUNC(fetch_sub, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_fetch_sub_acquire(obj, val)  ATOMIC_FUNC(fetch_sub, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_fetch_sub_release(obj, val)  ATOMIC_FUNC(fetch_sub, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_fetch_sub_relaxed(obj, val)  ATOMIC_FUNC(fetch_sub, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_fetch_and_acquire(obj, val)    ATOMIC_FUNC(fetch_and, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_fetch_and_release(obj, val)    ATOMIC_FUNC(fetch_and, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_fetch_and_relaxed(obj, val)    ATOMIC_FUNC(fetch_and, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_fetch_and(obj, val)          ATOMIC_FUNC(fetch_and, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_fetch_and_acquire(obj, val)  ATOMIC_FUNC(fetch_and, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_fetch_and_release(obj, val)  ATOMIC_FUNC(fetch_and, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_fetch_and_relaxed(obj, val)  ATOMIC_FUNC(fetch_and, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_fetch_or_acquire(obj, val)     ATOMIC_FUNC(fetch_or, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_fetch_or_release(obj, val)     ATOMIC_FUNC(fetch_or, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_fetch_or_relaxed(obj, val)     ATOMIC_FUNC(fetch_or, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_fetch_or(obj, val)           ATOMIC_FUNC(fetch_or, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_fetch_or_acquire(obj, val)   ATOMIC_FUNC(fetch_or, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_fetch_or_release(obj, val)   ATOMIC_FUNC(fetch_or, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_fetch_or_relaxed(obj, val)   ATOMIC_FUNC(fetch_or, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_fetch_xor_acquire(obj, val)    ATOMIC_FUNC(fetch_xor, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_fetch_xor_release(obj, val)    ATOMIC_FUNC(fetch_xor, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_fetch_xor_relaxed(obj, val)    ATOMIC_FUNC(fetch_xor, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_fetch_xor(obj, val)          ATOMIC_FUNC(fetch_xor, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_fetch_xor_acquire(obj, val)  ATOMIC_FUNC(fetch_xor, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_fetch_xor_release(obj, val)  ATOMIC_FUNC(fetch_xor, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_fetch_xor_relaxed(obj, val)  ATOMIC_FUNC(fetch_xor, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_xchg(obj, val)                 ATOMIC_FUNC(exchange, 4)(obj, val, __ATOMIC_ACQ_REL)
#define atomic_xchg_acquire(obj, val)         ATOMIC_FUNC(exchange, 4)(obj, val, __ATOMIC_ACQUIRE)
#define atomic_xchg_release(obj, val)         ATOMIC_FUNC(exchange, 4)(obj, val, __ATOMIC_RELEASE)
#define atomic_xchg_relaxed(obj, val)         ATOMIC_FUNC(exchange, 4)(obj, val, __ATOMIC_RELAXED)
#define atomic64_xchg(obj, val)               ATOMIC_FUNC(exchange, 8)(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_xchg_acquire(obj, val)       ATOMIC_FUNC(exchange, 8)(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_xchg_release(obj, val)       ATOMIC_FUNC(exchange, 8)(obj, val, __ATOMIC_RELEASE)
#define atomic64_xchg_relaxed(obj, val)       ATOMIC_FUNC(exchange, 8)(obj, val, __ATOMIC_RELAXED)

#define atomic_cmpxchg(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 4)(obj, expected, desired, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic_cmpxchg_acquire(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 4)(obj, expected, desired, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic_cmpxchg_release(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 4)(obj, expected, desired, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic_cmpxchg_relaxed(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 4)(obj, expected, desired, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 4)(obj, expected, desired, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_acquire(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 4)(obj, expected, desired, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_release(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 4)(obj, expected, desired, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_relaxed(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 4)(obj, expected, desired, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic64_cmpxchg(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 8)(obj, expected, desired, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_acquire(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 8)(obj, expected, desired, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_release(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 8)(obj, expected, desired, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_relaxed(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_strong, 8)(obj, expected, desired, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 8)(obj, expected, desired, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_acquire(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 8)(obj, expected, desired, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_release(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 8)(obj, expected, desired, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_relaxed(obj, expected, desired) \
  ATOMIC_FUNC(compare_exchange_weak, 8)(obj, expected, desired, __ATOMIC_RELAXED, __ATOMIC_RELAXED)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

void nx_atomic_store_4(FAR volatile void *ptr, int32_t value, int memorder);
void nx_atomic_store_8(FAR volatile void *ptr, int64_t value, int memorder);
int32_t nx_atomic_load_4(FAR const volatile void *ptr, int memorder);
int64_t nx_atomic_load_8(FAR const volatile void *ptr, int memorder);
int32_t nx_atomic_exchange_4(FAR volatile void *ptr, int32_t value,
                             int memorder);
int64_t nx_atomic_exchange_8(FAR volatile void *ptr, int64_t value,
                             int memorder);
bool nx_atomic_compare_exchange_4(FAR volatile void *ptr, FAR void *expect,
                                  int32_t desired, bool weak,
                                  int success, int failure);
bool nx_atomic_compare_exchange_8(FAR volatile void *ptr, FAR void *expect,
                                  int64_t desired, bool weak,
                                  int success, int failure);
int32_t nx_atomic_fetch_add_4(FAR volatile void *ptr, int32_t value,
                              int memorder);
int64_t nx_atomic_fetch_add_8(FAR volatile void *ptr, int64_t value,
                              int memorder);
int32_t nx_atomic_fetch_sub_4(FAR volatile void *ptr, int32_t value,
                              int memorder);
int64_t nx_atomic_fetch_sub_8(FAR volatile void *ptr, int64_t value,
                              int memorder);
int32_t nx_atomic_fetch_and_4(FAR volatile void *ptr, int32_t value,
                              int memorder);
int64_t nx_atomic_fetch_and_8(FAR volatile void *ptr, int64_t value,
                              int memorder);
int32_t nx_atomic_fetch_or_4(FAR volatile void *ptr, int32_t value,
                             int memorder);
int64_t nx_atomic_fetch_or_8(FAR volatile void *ptr, int64_t value,
                             int memorder);
int32_t nx_atomic_fetch_xor_4(FAR volatile void *ptr, int32_t value,
                              int memorder);
int64_t nx_atomic_fetch_xor_8(FAR volatile void *ptr, int64_t value,
                              int memorder);

#ifdef USE_ARCH_ATOMIC
static inline int32_t atomic_fetch_add(FAR volatile void *obj, int32_t val)
{
  return nx_atomic_fetch_add_4(obj, val, __ATOMIC_ACQ_REL);
}

static inline int32_t atomic_fetch_sub(FAR volatile void *obj, int32_t val)
{
  return nx_atomic_fetch_sub_4(obj, val, __ATOMIC_ACQ_REL);
}

static inline int32_t atomic_fetch_and(FAR volatile void *obj, int32_t val)
{
  return nx_atomic_fetch_and_4(obj, val, __ATOMIC_ACQ_REL);
}

static inline int32_t atomic_fetch_or(FAR volatile void *obj, int32_t val)
{
  return nx_atomic_fetch_or_4(obj, val, __ATOMIC_ACQ_REL);
}

static inline int32_t atomic_fetch_xor(FAR volatile void *obj, int32_t val)
{
  return nx_atomic_fetch_xor_4(obj, val, __ATOMIC_ACQ_REL);
}
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ATOMIC_H */
