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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ATOMIC_RELAXED
#  define __ATOMIC_RELAXED 0
#endif

#ifndef __ATOMIC_CONSUME
#  define __ATOMIC_CONSUME 1
#endif

#ifndef __ATOMIC_ACQUIRE
#  define __ATOMIC_ACQUIRE 2
#endif

#ifndef __ATOMIC_RELEASE
#  define __ATOMIC_RELEASE 3
#endif

#ifndef __ATOMIC_ACQ_REL
#  define __ATOMIC_ACQ_REL 4
#endif

#ifndef __ATOMIC_SEQ_CST
#  define __ATOMIC_SEQ_CST 5
#endif

#define atomic_set(obj, val)            atomic_store_4(obj, val, __ATOMIC_RELAXED)
#define atomic_set_release(obj, val)    atomic_store_4(obj, val, __ATOMIC_RELEASE)
#define atomic64_set(obj, val)          atomic_store_8(obj, val, __ATOMIC_RELAXED)
#define atomic64_set_release(obj, val)  atomic_store_8(obj, val, __ATOMIC_RELEASE)

#define atomic_read(obj)                atomic_load_4(obj, __ATOMIC_RELAXED)
#define atomic_read_acquire(obj)        atomic_load_4(obj, __ATOMIC_ACQUIRE)
#define atomic64_read(obj)              atomic_load_8(obj, __ATOMIC_RELAXED)
#define atomic64_read_acquire(obj)      atomic_load_8(obj, __ATOMIC_ACQUIRE)

#define atomic_add(obj, val)            atomic_fetch_add_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_add_acquire(obj, val)    atomic_fetch_add_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_add_release(obj, val)    atomic_fetch_add_4(obj, val, __ATOMIC_RELEASE)
#define atomic_add_relaxed(obj, val)    atomic_fetch_add_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_add(obj, val)          atomic_fetch_add_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_add_acquire(obj, val)  atomic_fetch_add_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_add_release(obj, val)  atomic_fetch_add_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_add_relaxed(obj, val)  atomic_fetch_add_8(obj, val, __ATOMIC_RELAXED)

#define atomic_sub(obj, val)            atomic_fetch_sub_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_sub_acquire(obj, val)    atomic_fetch_sub_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_sub_release(obj, val)    atomic_fetch_sub_4(obj, val, __ATOMIC_RELEASE)
#define atomic_sub_relaxed(obj, val)    atomic_fetch_sub_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_sub(obj, val)          atomic_fetch_sub_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_sub_acquire(obj, val)  atomic_fetch_sub_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_sub_release(obj, val)  atomic_fetch_sub_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_sub_relaxed(obj, val)  atomic_fetch_sub_8(obj, val, __ATOMIC_RELAXED)

#define atomic_and(obj, val)            atomic_fetch_and_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_and_acquire(obj, val)    atomic_fetch_and_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_and_release(obj, val)    atomic_fetch_and_4(obj, val, __ATOMIC_RELEASE)
#define atomic_and_relaxed(obj, val)    atomic_fetch_and_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_and(obj, val)          atomic_fetch_and_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_and_acquire(obj, val)  atomic_fetch_and_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_and_release(obj, val)  atomic_fetch_and_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_and_relaxed(obj, val)  atomic_fetch_and_8(obj, val, __ATOMIC_RELAXED)

#define atomic_or(obj, val)             atomic_fetch_or_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_or_acquire(obj, val)     atomic_fetch_or_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_or_release(obj, val)     atomic_fetch_or_4(obj, val, __ATOMIC_RELEASE)
#define atomic_or_relaxed(obj, val)     atomic_fetch_or_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_or(obj, val)           atomic_fetch_or_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_or_acquire(obj, val)   atomic_fetch_or_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_or_release(obj, val)   atomic_fetch_or_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_or_relaxed(obj, val)   atomic_fetch_or_8(obj, val, __ATOMIC_RELAXED)

#define atomic_xor(obj, val)            atomic_fetch_xor_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_xor_acquire(obj, val)    atomic_fetch_xor_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_xor_release(obj, val)    atomic_fetch_xor_4(obj, val, __ATOMIC_RELEASE)
#define atomic_xor_relaxed(obj, val)    atomic_fetch_xor_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_xor(obj, val)          atomic_fetch_xor_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_xor_acquire(obj, val)  atomic_fetch_xor_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_xor_release(obj, val)  atomic_fetch_xor_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_xor_relaxed(obj, val)  atomic_fetch_xor_8(obj, val, __ATOMIC_RELAXED)

#define atomic_xchg(obj, val)           atomic_exchange_4(obj, val, __ATOMIC_ACQ_REL)
#define atomic_xchg_acquire(obj, val)   atomic_exchange_4(obj, val, __ATOMIC_ACQUIRE)
#define atomic_xchg_release(obj, val)   atomic_exchange_4(obj, val, __ATOMIC_RELEASE)
#define atomic_xchg_relaxed(obj, val)   atomic_exchange_4(obj, val, __ATOMIC_RELAXED)
#define atomic64_xchg(obj, val)         atomic_exchange_8(obj, val, __ATOMIC_ACQ_REL)
#define atomic64_xchg_acquire(obj, val) atomic_exchange_8(obj, val, __ATOMIC_ACQUIRE)
#define atomic64_xchg_release(obj, val) atomic_exchange_8(obj, val, __ATOMIC_RELEASE)
#define atomic64_xchg_relaxed(obj, val) atomic_exchange_8(obj, val, __ATOMIC_RELAXED)

#define atomic_cmpxchg(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic_cmpxchg_acquire(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic_cmpxchg_release(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic_cmpxchg_relaxed(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic64_cmpxchg(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, false, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_acquire(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_release(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, false, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic64_cmpxchg_relaxed(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)

#define atomic_try_cmpxchg(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, true, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_acquire(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_release(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, true, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic_try_cmpxchg_relaxed(obj, expected, desired) \
  atomic_compare_exchange_4(obj, (FAR int32_t *)expected, desired, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, true, __ATOMIC_ACQ_REL, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_acquire(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_release(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, true, __ATOMIC_RELEASE, __ATOMIC_RELAXED)
#define atomic64_try_cmpxchg_relaxed(obj, expected, desired) \
  atomic_compare_exchange_8(obj, (FAR int64_t *)expected, desired, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef __Atomic(int32_t) atomic_t;
typedef __Atomic(int64_t) atomic64_t;

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

#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
void atomic_store_4(FAR volatile void *ptr, int32_t value, int memorder);
void atomic_store_8(FAR volatile void *ptr, int64_t value, int memorder);
int32_t atomic_load_4(FAR const volatile void *ptr, int memorder);
int64_t atomic_load_8(FAR const volatile void *ptr, int memorder);
int32_t atomic_exchange_4(FAR volatile void *ptr, int32_t value,
                          int memorder);
int64_t atomic_exchange_8(FAR volatile void *ptr, int64_t value,
                          int memorder);
bool atomic_compare_exchange_4(FAR volatile void *ptr,
                               FAR volatile void *expect,
                               int32_t desired, bool weak,
                               int success, int failure);
bool atomic_compare_exchange_8(FAR volatile void *ptr,
                               FAR volatile void *expect,
                               int64_t desired, bool weak,
                               int success, int failure);
int32_t atomic_fetch_add_4(FAR volatile void *ptr, int32_t value,
                           int memorder);
int64_t atomic_fetch_add_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int32_t atomic_fetch_sub_4(FAR volatile void *ptr, int32_t value,
                           int memorder);
int64_t atomic_fetch_sub_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int32_t atomic_fetch_and_4(FAR volatile void *ptr, int32_t value,
                           int memorder);
int64_t atomic_fetch_and_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
int32_t atomic_fetch_or_4(FAR volatile void *ptr, int32_t value,
                          int memorder);
int64_t atomic_fetch_or_8(FAR volatile void *ptr, int64_t value,
                          int memorder);
int32_t atomic_fetch_xor_4(FAR volatile void *ptr, int32_t value,
                           int memorder);
int64_t atomic_fetch_xor_8(FAR volatile void *ptr, int64_t value,
                           int memorder);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ATOMIC_H */
