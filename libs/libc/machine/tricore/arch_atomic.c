/****************************************************************************
 * libs/libc/machine/tricore/arch_atomic.c
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

#include <nuttx/atomic.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARCH_ATOMIC_STORE_4(func)                                       \
                                                                        \
  void func(volatile void *ptr, int32_t value, int memorder)            \
  {                                                                     \
    tricore_atomic_swap(ptr, value);                                    \
  }

#define ARCH_ATOMIC_LOAD_4(func)                                        \
                                                                        \
  int32_t func(const volatile void *ptr, int memorder)                  \
  {                                                                     \
    return *(volatile uint32_t *)ptr;                                   \
  }

#define ARCH_ATOMIC_EXCHANGE_4(func)                                    \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    return tricore_atomic_swap(ptr, value);                             \
  }

#define ARCH_ATOMIC_COMPARE_EXCHANGE_4(func)                            \
                                                                        \
  bool func(volatile void *ptr, volatile void *expect,                  \
            int32_t desired, bool weak, int success, int failure)       \
  {                                                                     \
    int32_t old;                                                        \
                                                                        \
    old = tricore_atomic_cmpswap(ptr, desired, *(int32_t *)expect);     \
    if (old == *(int32_t *)expect)                                      \
      {                                                                 \
        return true;                                                    \
      }                                                                 \
                                                                        \
    *(int32_t *)expect = old;                                           \
                                                                        \
    return false;                                                       \
  }

#define ARCH_ATOMIC_FLAGS_TEST_AND_SET_4(func)                          \
                                                                        \
  int32_t func(volatile void *ptr, int memorder)                        \
  {                                                                     \
    return tricore_atomic_swap(ptr, 1);                                 \
  }

#define ARCH_ATOMIC_FETCH_ADD_4(func)                                   \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    int32_t old_val;                                                    \
                                                                        \
    do                                                                  \
      {                                                                 \
        old_val = atomic_load_4(ptr, memorder);                         \
      }                                                                 \
    while (tricore_atomic_cmpswap(ptr, old_val + value, old_val)        \
           != old_val);                                                 \
                                                                        \
    return old_val;                                                     \
  }

#define ARCH_ATOMIC_FETCH_SUB_4(func)                                   \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    int32_t old_val;                                                    \
                                                                        \
    do                                                                  \
      {                                                                 \
        old_val = atomic_load_4(ptr, memorder);                         \
      }                                                                 \
    while (tricore_atomic_cmpswap(ptr, old_val - value, old_val)        \
           != old_val);                                                 \
                                                                        \
    return old_val;                                                     \
  }

#define ARCH_ATOMIC_FETCH_AND_4(func)                                   \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    int32_t old_val;                                                    \
                                                                        \
    do                                                                  \
      {                                                                 \
        old_val = atomic_load_4(ptr, memorder);                         \
      }                                                                 \
    while (tricore_atomic_cmpswap(ptr, old_val & value, old_val)        \
           != old_val);                                                 \
                                                                        \
    return old_val;                                                     \
  }

#define ARCH_ATOMIC_FETCH_OR_4(func)                                    \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    int32_t old_val;                                                    \
                                                                        \
    do                                                                  \
      {                                                                 \
        old_val = atomic_load_4(ptr, memorder);                         \
      }                                                                 \
    while (tricore_atomic_cmpswap(ptr, old_val | value, old_val)        \
           != old_val);                                                 \
                                                                        \
    return old_val;                                                     \
  }

#define ARCH_ATOMIC_FETCH_XOR_4(func)                                   \
                                                                        \
  int32_t func(volatile void *ptr, int32_t value, int memorder)         \
  {                                                                     \
    int32_t old_val;                                                    \
                                                                        \
    do                                                                  \
      {                                                                 \
        old_val = atomic_load_4(ptr, memorder);                         \
      }                                                                 \
    while (tricore_atomic_cmpswap(ptr, old_val ^ value, old_val)        \
           != old_val);                                                 \
                                                                        \
    return old_val;                                                     \
  }

/****************************************************************************
 * Private Functions
 ****************************************************************************/

always_inline_function
static uint32_t tricore_atomic_swap(volatile void *addr, uint32_t value)
{
  uint32_t res;

  __asm__ volatile ("swap.w [%1]0, %2"
                    : "=d"(res) : "a"(addr), "0"(value));
  return res;
}

always_inline_function
static uint32_t tricore_atomic_cmpswap(volatile void *addr, uint32_t value,
                                       uint32_t condition)
{
  uint64_t reg64 = value | ((uint64_t)condition << 32);

  __asm__ __volatile__ ("cmpswap.w [%1]0, %A0"
                        : "+d" (reg64)
                        : "a" (addr)
                        : "memory");
  return (uint32_t)reg64;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: atomic_store_4
 ****************************************************************************/

ARCH_ATOMIC_STORE_4(__atomic_store_4)
ARCH_ATOMIC_STORE_4(atomic_store_4)

/****************************************************************************
 * Name: atomic_load_4
 ****************************************************************************/

ARCH_ATOMIC_LOAD_4(__atomic_load_4)
ARCH_ATOMIC_LOAD_4(atomic_load_4)

/****************************************************************************
 * Name: atomic_exchange_4
 ****************************************************************************/

ARCH_ATOMIC_EXCHANGE_4(__atomic_exchange_4)
ARCH_ATOMIC_EXCHANGE_4(atomic_exchange_4)

/****************************************************************************
 * Name: atomic_compare_exchange_4
 ****************************************************************************/

ARCH_ATOMIC_COMPARE_EXCHANGE_4(__atomic_compare_exchange_4)
ARCH_ATOMIC_COMPARE_EXCHANGE_4(atomic_compare_exchange_4)

/****************************************************************************
 * Name: atomic_flag_test_and_set_4
 ****************************************************************************/

ARCH_ATOMIC_FLAGS_TEST_AND_SET_4(__atomic_flags_test_and_set_4)
ARCH_ATOMIC_FLAGS_TEST_AND_SET_4(atomic_flags_test_and_set_4)

/****************************************************************************
 * Name: atomic_fetch_add_4
 ****************************************************************************/

ARCH_ATOMIC_FETCH_ADD_4(__atomic_fetch_add_4)
ARCH_ATOMIC_FETCH_ADD_4(atomic_fetch_add_4)

/****************************************************************************
 * Name: atomic_fetch_sub_4
 ****************************************************************************/

ARCH_ATOMIC_FETCH_SUB_4(__atomic_fetch_sub_4)
ARCH_ATOMIC_FETCH_SUB_4(atomic_fetch_sub_4)

/****************************************************************************
 * Name: atomic_fetch_and_4
 ****************************************************************************/

ARCH_ATOMIC_FETCH_AND_4(__atomic_fetch_and_4)
ARCH_ATOMIC_FETCH_AND_4(atomic_fetch_and_4)

/****************************************************************************
 * Name: atomic_fetch_or_4
 ****************************************************************************/

ARCH_ATOMIC_FETCH_OR_4(__atomic_fetch_or_4)
ARCH_ATOMIC_FETCH_OR_4(atomic_fetch_or_4)

/****************************************************************************
 * Name: atomic_fetch_xor_4
 ****************************************************************************/

ARCH_ATOMIC_FETCH_XOR_4(__atomic_fetch_xor_4)
ARCH_ATOMIC_FETCH_XOR_4(atomic_fetch_xor_4)
