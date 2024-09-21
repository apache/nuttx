/****************************************************************************
 * include/nuttx/lib/stdatomic.h
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

#ifndef __INCLUDE_NUTTX_LIB_STDATOMIC_H
#define __INCLUDE_NUTTX_LIB_STDATOMIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ATOMIC_RELAXED
#  define __ATOMIC_RELAXED 0
#endif

#ifndef __ATOMIC_CONSUM
#  define __ATOMIC_CONSUME 1
#endif

#ifndef __ATOMIC_ACQUIR
#  define __ATOMIC_ACQUIRE 2
#endif

#ifndef __ATOMIC_RELEAS
#  define __ATOMIC_RELEASE 3
#endif

#ifndef __ATOMIC_ACQ_REL
#  define __ATOMIC_ACQ_REL 4
#endif

#ifndef __ATOMIC_SEQ_CS
#  define __ATOMIC_SEQ_CST 5
#endif

#define ATOMIC_FLAG_INIT 0
#define ATOMIC_VAR_INIT(value) (value)

#define atomic_store_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_store_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_store_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_store_4(obj, val, type) : \
                         __atomic_store_8(obj, val, type))

#define atomic_store(obj, val) atomic_store_n(obj, val, __ATOMIC_RELAXED)
#define atomic_store_explicit(obj, val, type) atomic_store_n(obj, val, type)
#define atomic_init(obj, val) atomic_store(obj, val)

#define atomic_load_n(obj, type) \
  (sizeof(*(obj)) == 1 ? __atomic_load_1(obj, type) : \
   sizeof(*(obj)) == 2 ? __atomic_load_2(obj, type) : \
   sizeof(*(obj)) == 4 ? __atomic_load_4(obj, type) : \
                         __atomic_load_8(obj, type))

#define atomic_load(obj) atomic_load_n(obj, __ATOMIC_RELAXED)
#define atomic_load_explicit(obj, type) atomic_load_n(obj, type)

#define atomic_exchange_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_exchange_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_exchange_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_exchange_4(obj, val, type) : \
                         __atomic_exchange_8(obj, val, type))

#define atomic_exchange(obj, val) atomic_exchange_n(obj, val, __ATOMIC_RELAXED)
#define atomic_exchange_explicit(obj, val, type) atomic_exchange_n(obj, val, type)

#define atomic_compare_exchange_n(obj, expected, desired, weak, success, failure) \
  (sizeof(*(obj)) == 1 ? __atomic_compare_exchange_1(obj, expected, desired, weak, success, failure) : \
   sizeof(*(obj)) == 2 ? __atomic_compare_exchange_2(obj, expected, desired, weak, success, failure) : \
   sizeof(*(obj)) == 4 ? __atomic_compare_exchange_4(obj, expected, desired, weak, success, failure) : \
                         __atomic_compare_exchange_8(obj, expected, desired, weak, success, failure))

#define atomic_compare_exchange_strong(obj, expected, desired) \
  atomic_compare_exchange_n(obj, expected, desired, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic_compare_exchange_strong_explicit(obj, expected, desired, success, failure) \
  atomic_compare_exchange_n(obj, expected, desired, false, success, failure)
#define atomic_compare_exchange_weak(obj, expected, desired) \
  atomic_compare_exchange_n(obj, expected, desired, true, __ATOMIC_RELAXED, __ATOMIC_RELAXED)
#define atomic_compare_exchange_weak_explicit(obj, expected, desired, success, failure) \
  atomic_compare_exchange_n(obj, expected, desired, true, success, failure)

#define atomic_flag_test_and_set_n(obj, type) \
  (sizeof(*(obj)) == 1 ? __atomic_flag_test_and_set_1(obj, type) : \
   sizeof(*(obj)) == 2 ? __atomic_flag_test_and_set_2(obj, type) : \
   sizeof(*(obj)) == 4 ? __atomic_flag_test_and_set_4(obj, type) : \
                         __atomic_flag_test_and_set_8(obj, type))

#define atomic_flag_test_and_set(obj) atomic_flag_test_and_set_n(obj, __ATOMIC_RELAXED)
#define atomic_flag_test_and_set_explicit(obj, type) atomic_flag_test_and_set_n(obj, 1, type)
#define atomic_flag_clear(obj) atomic_store(obj, 0)
#define atomic_flag_clear_explicit(obj, type) atomic_store_explicit(obj, 0, type)

#define atomic_fetch_and_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_fetch_and_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_fetch_and_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_fetch_and_4(obj, val, type) : \
                         __atomic_fetch_and_8(obj, val, type))

#define atomic_fetch_and(obj, val) atomic_fetch_and_n(obj, val, __ATOMIC_RELAXED)
#define atomic_fetch_and_explicit(obj, val, type) atomic_fetch_and_n(obj, val, type)

#define atomic_fetch_or_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_fetch_or_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_fetch_or_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_fetch_or_4(obj, val, type) : \
                         __atomic_fetch_or_8(obj, val, type))

#define atomic_fetch_or(obj, val) atomic_fetch_or_n(obj, val, __ATOMIC_RELAXED)
#define atomic_fetch_or_explicit(obj, val, type) atomic_fetch_or_n(obj, val, type)

#define atomic_fetch_xor_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_fetch_xor_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_fetch_xor_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_fetch_xor_4(obj, val, type) : \
                         __atomic_fetch_xor_8(obj, val, type))

#define atomic_fetch_xor(obj, val) atomic_fetch_xor_n(obj, val, __ATOMIC_RELAXED)
#define atomic_fetch_xor_explicit(obj, val, type) atomic_fetch_xor_n(obj, val, type)

#define atomic_fetch_add_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_fetch_add_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_fetch_add_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_fetch_add_4(obj, val, type) : \
                         __atomic_fetch_add_8(obj, val, type))

#define atomic_fetch_add(obj, val) atomic_fetch_add_n(obj, val, __ATOMIC_RELAXED)
#define atomic_fetch_add_explicit(obj, val, type) atomic_fetch_add_n(obj, val, type)

#define atomic_fetch_sub_n(obj, val, type) \
  (sizeof(*(obj)) == 1 ? __atomic_fetch_sub_1(obj, val, type) : \
   sizeof(*(obj)) == 2 ? __atomic_fetch_sub_2(obj, val, type) : \
   sizeof(*(obj)) == 4 ? __atomic_fetch_sub_4(obj, val, type) : \
                         __atomic_fetch_sub_8(obj, val, type))

#define atomic_fetch_sub(obj, val) atomic_fetch_sub_n(obj, val, __ATOMIC_RELAXED)
#define atomic_fetch_sub_explicit(obj, val, type) atomic_fetch_sub_n(obj, val, type)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
    memory_order_relaxed = __ATOMIC_RELAXED,
    memory_order_consume = __ATOMIC_CONSUME,
    memory_order_acquire = __ATOMIC_ACQUIRE,
    memory_order_release = __ATOMIC_RELEASE,
    memory_order_acq_rel = __ATOMIC_ACQ_REL,
    memory_order_seq_cst = __ATOMIC_SEQ_CST
} memory_order;

typedef volatile int atomic_flag;
typedef volatile bool atomic_bool;
typedef volatile char atomic_char;
typedef volatile signed char atomic_schar;
typedef volatile unsigned char atomic_uchar;
typedef volatile short atomic_short;
typedef volatile unsigned short atomic_ushort;
typedef volatile int atomic_int;
typedef volatile unsigned int atomic_uint;
typedef volatile long atomic_long;
typedef volatile unsigned long atomic_ulong;
typedef volatile long long atomic_llong;
typedef volatile unsigned long long atomic_ullong;
typedef volatile wchar_t atomic_wchar_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void __atomic_store_1(FAR volatile void *ptr, uint8_t value, int memorder);
void __atomic_store_2(FAR volatile void *ptr, uint16_t value, int memorder);
void __atomic_store_4(FAR volatile void *ptr, uint32_t value, int memorder);
void __atomic_store_8(FAR volatile void *ptr, uint64_t value, int memorder);
uint8_t __atomic_load_1(FAR const volatile void *ptr, int memorder);
uint16_t __atomic_load_2(FAR const volatile void *ptr, int memorder);
uint32_t __atomic_load_4(FAR const volatile void *ptr, int memorder);
uint64_t __atomic_load_8(FAR const volatile void *ptr, int memorder);
uint8_t __atomic_exchange_1(FAR volatile void *ptr, uint8_t value,
                            int memorder);
uint16_t __atomic_exchange_2(FAR volatile void *ptr, uint16_t value,
                             int memorder);
uint32_t __atomic_exchange_4(FAR volatile void *ptr, uint32_t value,
                             int memorder);
uint64_t __atomic_exchange_8(FAR volatile void *ptr, uint64_t value,
                             int memorder);
bool __atomic_compare_exchange_1(FAR volatile void *mem, FAR void *expect,
                                 uint8_t desired, bool weak, int success,
                                 int failure);
bool __atomic_compare_exchange_2(FAR volatile void *mem, FAR void *expect,
                                 uint16_t desired, bool weak, int success,
                                 int failure);
bool __atomic_compare_exchange_4(FAR volatile void *mem, FAR void *expect,
                                 uint32_t desired, bool weak, int success,
                                 int failure);
bool __atomic_compare_exchange_8(FAR volatile void *mem, FAR void *expect,
                                 uint64_t desired, bool weak, int success,
                                 int failure);
uint8_t __atomic_flag_test_and_set_1(FAR const volatile void *ptr,
                                     int memorder);
uint16_t __atomic_flag_test_and_set_2(FAR const volatile void *ptr,
                                      int memorder);
uint32_t __atomic_flag_test_and_set_4(FAR const volatile void *ptr,
                                      int memorder);
uint64_t __atomic_flag_test_and_set_8(FAR const volatile void *ptr,
                                      int memorder);
uint8_t __atomic_fetch_add_1(FAR volatile void *ptr, uint8_t value,
                             int memorder);
uint16_t __atomic_fetch_add_2(FAR volatile void *ptr, uint16_t value,
                              int memorder);
uint32_t __atomic_fetch_add_4(FAR volatile void *ptr, uint32_t value,
                              int memorder);
uint64_t __atomic_fetch_add_8(FAR volatile void *ptr, uint64_t value,
                              int memorder);
uint8_t __atomic_fetch_sub_1(FAR volatile void *ptr, uint8_t value,
                             int memorder);
uint16_t __atomic_fetch_sub_2(FAR volatile void *ptr, uint16_t value,
                              int memorder);
uint32_t __atomic_fetch_sub_4(FAR volatile void *ptr, uint32_t value,
                              int memorder);
uint64_t __atomic_fetch_sub_8(FAR volatile void *ptr, uint64_t value,
                              int memorder);
uint8_t __atomic_fetch_and_1(FAR volatile void *ptr, uint8_t value,
                             int memorder);
uint16_t __atomic_fetch_and_2(FAR volatile void *ptr, uint16_t value,
                              int memorder);
uint32_t __atomic_fetch_and_4(FAR volatile void *ptr, uint32_t value,
                              int memorder);
uint64_t __atomic_fetch_and_8(FAR volatile void *ptr, uint64_t value,
                              int memorder);
uint8_t __atomic_fetch_or_1(FAR volatile void *ptr, uint8_t value,
                            int memorder);
uint16_t __atomic_fetch_or_2(FAR volatile void *ptr, uint16_t value,
                             int memorder);
uint32_t __atomic_fetch_or_4(FAR volatile void *ptr, uint32_t value,
                             int memorder);
uint64_t __atomic_fetch_or_8(FAR volatile void *ptr, uint64_t value,
                             int memorder);
uint8_t __atomic_fetch_xor_1(FAR volatile void *ptr, uint8_t value,
                             int memorder);
uint16_t __atomic_fetch_xor_2(FAR volatile void *ptr, uint16_t value,
                              int memorder);
uint32_t __atomic_fetch_xor_4(FAR volatile void *ptr, uint32_t value,
                              int memorder);
uint64_t __atomic_fetch_xor_8(FAR volatile void *ptr, uint64_t value,
                              int memorder);

#endif /* __INCLUDE_NUTTX_LIB_STDATOMIC_H */
