/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_std_atomic.c
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

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMP_EXCHANGE(n, type)                             \
                                                          \
  bool __atomic_compare_exchange_ ## n (type *mem,        \
                                        type *expect,     \
                                        type desired,     \
                                        int success,      \
                                        int failure)      \
  {                                                       \
    bool ret = false;                                     \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
                                                          \
    if (*mem == *expect)                                  \
      {                                                   \
        ret = true;                                       \
        *mem = desired;                                   \
      }                                                   \
    else                                                  \
      {                                                   \
        *expect = *mem;                                   \
      }                                                   \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret; \
  }

#define FETCH_ADD(n, type)                                \
                                                          \
  type __atomic_fetch_add_ ## n (type *ptr,               \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    type ret = *ptr;                                      \
                                                          \
    *ptr = *ptr + value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_SUB(n, type)                                \
                                                          \
  type __atomic_fetch_sub_ ## n (type *ptr,               \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    type ret = *ptr;                                      \
                                                          \
    *ptr = *ptr - value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_AND(n, type)                                \
                                                          \
  type __atomic_fetch_and_ ## n (type *ptr,               \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    type ret = *ptr;                                      \
                                                          \
    *ptr = *ptr & value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_OR(n, type)                                 \
                                                          \
  type __atomic_fetch_or_ ## n (type *ptr,                \
                                type value,               \
                                int memorder)             \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    type ret = *ptr;                                      \
                                                          \
    *ptr = *ptr | value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_XOR(n, type)                                \
                                                          \
  type __atomic_fetch_xor_ ## n (type *ptr,               \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    type ret = *ptr;                                      \
                                                          \
    *ptr = *ptr ^ value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#pragma GCC diagnostic ignored "-Wbuiltin-declaration-mismatch"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __atomic_compare_exchange_1
 ****************************************************************************/

CMP_EXCHANGE(1, uint8_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_2
 ****************************************************************************/

CMP_EXCHANGE(2, uint16_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_4
 ****************************************************************************/

CMP_EXCHANGE(4, uint32_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_8
 ****************************************************************************/

CMP_EXCHANGE(8, uint64_t)

/****************************************************************************
 * Name: __atomic_fetch_add_1
 ****************************************************************************/

FETCH_ADD(1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_add_2
 ****************************************************************************/

FETCH_ADD(2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_add_4
 ****************************************************************************/

FETCH_ADD(4, uint32_t)

/****************************************************************************
 * Name: __atomic_fetch_add_8
 ****************************************************************************/

FETCH_ADD(8, uint64_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_1
 ****************************************************************************/

FETCH_SUB(1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_2
 ****************************************************************************/

FETCH_SUB(2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_4
 ****************************************************************************/

FETCH_SUB(4, uint32_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_8
 ****************************************************************************/

FETCH_SUB(8, uint64_t)

/****************************************************************************
 * Name: __atomic_fetch_and_1
 ****************************************************************************/

FETCH_AND(1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_and_2
 ****************************************************************************/

FETCH_AND(2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_and_4
 ****************************************************************************/

FETCH_AND(4, uint32_t)

/****************************************************************************
 * Name: __atomic_fetch_and_8
 ****************************************************************************/

FETCH_AND(8, uint64_t)

/****************************************************************************
 * Name: __atomic_fetch_or_1
 ****************************************************************************/

FETCH_OR(1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_or_2
 ****************************************************************************/

FETCH_OR(2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_or_4
 ****************************************************************************/

FETCH_OR(4, uint32_t)

/****************************************************************************
 * Name: __atomic_fetch_or_4
 ****************************************************************************/

FETCH_OR(8, uint64_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_1
 ****************************************************************************/

FETCH_XOR(1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_2
 ****************************************************************************/

FETCH_XOR(2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_4
 ****************************************************************************/

FETCH_XOR(4, uint32_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_8
 ****************************************************************************/

FETCH_XOR(8, uint64_t)
