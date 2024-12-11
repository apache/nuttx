/****************************************************************************
 * libs/libc/machine/arch_atomic.c
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

#include <stdbool.h>
#include <stdint.h>
#include <nuttx/spinlock.h>
#include <nuttx/macro.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_atomic_lock = SP_UNLOCKED;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STORE(fn, n, type)                                           \
                                                                     \
  void weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
                                                                     \
    *(FAR type *)ptr = value;                                        \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
  }

#define LOAD(fn, n, type)                                             \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR const volatile void *ptr, \
                                        int memorder)                 \
  {                                                                   \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock);  \
                                                                      \
    type ret = *(FAR type *)ptr;                                      \
                                                                      \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);         \
    return ret;                                                       \
  }

#define EXCHANGE(fn, n, type)                                        \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    type ret = *tmp;                                                 \
    *tmp = value;                                                    \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define CMP_EXCHANGE(fn, n, type)                                    \
                                                                     \
  bool weak_function CONCATENATE(fn, n)(FAR volatile void *mem,      \
                                        FAR void *expect,            \
                                        type desired, bool weak,     \
                                        int success, int failure)    \
  {                                                                  \
    bool ret = false;                                                \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmpmem = (FAR type *)mem;                              \
    FAR type *tmpexp = (FAR type *)expect;                           \
                                                                     \
    if (*tmpmem == *tmpexp)                                          \
      {                                                              \
        ret = true;                                                  \
        *tmpmem = desired;                                           \
      }                                                              \
    else                                                             \
      {                                                              \
        *tmpexp = *tmpmem;                                           \
      }                                                              \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FLAG_TEST_AND_SET(fn, n, type)                               \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        int memorder)                \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *(FAR type *)ptr = 1;                                            \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FETCH_ADD(fn, n, type)                                       \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *tmp = *tmp + value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FETCH_SUB(fn, n, type)                                       \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *tmp = *tmp - value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FETCH_AND(fn, n, type)                                       \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *tmp = *tmp & value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FETCH_OR(fn, n, type)                                        \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *tmp = *tmp | value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define FETCH_XOR(fn, n, type)                                       \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value, int memorder)    \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    *tmp = *tmp ^ value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define SYNC_ADD_FETCH(fn, n, type)                                  \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = *tmp + value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_SUB_FETCH(fn, n, type)                                  \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = *tmp - value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_OR_FETCH(fn, n, type)                                   \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = *tmp | value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_AND_FETCH(fn, n, type)                                  \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = *tmp & value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_XOR_FETCH(fn, n, type)                                  \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = *tmp ^ value;                                             \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_NAND_FETCH(fn, n, type)                                 \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type value)                  \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    *tmp = ~(*tmp & value);                                          \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return *tmp;                                                     \
  }

#define SYNC_BOOL_CMP_SWAP(fn, n, type)                              \
                                                                     \
  bool weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type oldvalue,               \
                                        type newvalue)               \
  {                                                                  \
    bool ret = false;                                                \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
                                                                     \
    if (*tmp == oldvalue)                                            \
      {                                                              \
        ret = true;                                                  \
        *tmp = newvalue;                                             \
      }                                                              \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

#define SYNC_VAL_CMP_SWAP(fn, n, type)                               \
                                                                     \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,      \
                                        type oldvalue,               \
                                        type newvalue)               \
  {                                                                  \
    irqstate_t irqstate = spin_lock_irqsave_wo_note(&g_atomic_lock); \
    FAR type *tmp = (FAR type *)ptr;                                 \
    type ret = *tmp;                                                 \
                                                                     \
    if (*tmp == oldvalue)                                            \
      {                                                              \
        *tmp = newvalue;                                             \
      }                                                              \
                                                                     \
    spin_unlock_irqrestore_wo_note(&g_atomic_lock, irqstate);        \
    return ret;                                                      \
  }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __atomic_store_1
 ****************************************************************************/

STORE(__atomic_store_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_store_2
 ****************************************************************************/

STORE(__atomic_store_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_store_4
 ****************************************************************************/

STORE(__atomic_store_, 4, uint32_t)
STORE(nx_atomic_store_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_store_8
 ****************************************************************************/

STORE(__atomic_store_, 8, uint64_t)
STORE(nx_atomic_store_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_load_1
 ****************************************************************************/

LOAD(__atomic_load_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_load__2
 ****************************************************************************/

LOAD(__atomic_load_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_load__4
 ****************************************************************************/

LOAD(__atomic_load_, 4, uint32_t)
LOAD(nx_atomic_load_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_load__8
 ****************************************************************************/

LOAD(__atomic_load_, 8, uint64_t)
LOAD(nx_atomic_load_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_exchange_1
 ****************************************************************************/

EXCHANGE(__atomic_exchange_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_exchange__2
 ****************************************************************************/

EXCHANGE(__atomic_exchange_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_exchange__4
 ****************************************************************************/

EXCHANGE(__atomic_exchange_, 4, uint32_t)
EXCHANGE(nx_atomic_exchange_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_exchange__8
 ****************************************************************************/

EXCHANGE(__atomic_exchange_, 8, uint64_t)
EXCHANGE(nx_atomic_exchange_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_1
 ****************************************************************************/

CMP_EXCHANGE(__atomic_compare_exchange_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_2
 ****************************************************************************/

CMP_EXCHANGE(__atomic_compare_exchange_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_4
 ****************************************************************************/

CMP_EXCHANGE(__atomic_compare_exchange_, 4, uint32_t)
CMP_EXCHANGE(nx_atomic_compare_exchange_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_compare_exchange_8
 ****************************************************************************/

CMP_EXCHANGE(__atomic_compare_exchange_, 8, uint64_t)
CMP_EXCHANGE(nx_atomic_compare_exchange_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_flag_test_and_set_1
 ****************************************************************************/

FLAG_TEST_AND_SET(__atomic_flags_test_and_set_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_flag_test_and_set_2
 ****************************************************************************/

FLAG_TEST_AND_SET(__atomic_flags_test_and_set_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_flag_test_and_set_4
 ****************************************************************************/

FLAG_TEST_AND_SET(__atomic_flags_test_and_set_, 4, uint32_t)
FLAG_TEST_AND_SET(nx_atomic_flags_test_and_set_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_flag_test_and_set_8
 ****************************************************************************/

FLAG_TEST_AND_SET(__atomic_flags_test_and_set_, 8, uint64_t)
FLAG_TEST_AND_SET(nx_atomic_flags_test_and_set_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_fetch_add_1
 ****************************************************************************/

FETCH_ADD(__atomic_fetch_add_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_add_2
 ****************************************************************************/

FETCH_ADD(__atomic_fetch_add_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_add_4
 ****************************************************************************/

FETCH_ADD(__atomic_fetch_add_, 4, uint32_t)
FETCH_ADD(nx_atomic_fetch_add_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_fetch_add_8
 ****************************************************************************/

FETCH_ADD(__atomic_fetch_add_, 8, uint64_t)
FETCH_ADD(nx_atomic_fetch_add_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_1
 ****************************************************************************/

FETCH_SUB(__atomic_fetch_sub_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_2
 ****************************************************************************/

FETCH_SUB(__atomic_fetch_sub_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_4
 ****************************************************************************/

FETCH_SUB(__atomic_fetch_sub_, 4, uint32_t)
FETCH_SUB(nx_atomic_fetch_sub_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_fetch_sub_8
 ****************************************************************************/

FETCH_SUB(__atomic_fetch_sub_, 8, uint64_t)
FETCH_SUB(nx_atomic_fetch_sub_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_fetch_and_1
 ****************************************************************************/

FETCH_AND(__atomic_fetch_and_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_and_2
 ****************************************************************************/

FETCH_AND(__atomic_fetch_and_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_and_4
 ****************************************************************************/

FETCH_AND(__atomic_fetch_and_, 4, uint32_t)
FETCH_AND(nx_atomic_fetch_and_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_fetch_and_8
 ****************************************************************************/

FETCH_AND(__atomic_fetch_and_, 8, uint64_t)
FETCH_AND(nx_atomic_fetch_and_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_fetch_or_1
 ****************************************************************************/

FETCH_OR(__atomic_fetch_or_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_or_2
 ****************************************************************************/

FETCH_OR(__atomic_fetch_or_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_or_4
 ****************************************************************************/

FETCH_OR(__atomic_fetch_or_, 4, uint32_t)
FETCH_OR(nx_atomic_fetch_or_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_fetch_or_4
 ****************************************************************************/

FETCH_OR(__atomic_fetch_or_, 8, uint64_t)
FETCH_OR(nx_atomic_fetch_or_, 8, int64_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_1
 ****************************************************************************/

FETCH_XOR(__atomic_fetch_xor_, 1, uint8_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_2
 ****************************************************************************/

FETCH_XOR(__atomic_fetch_xor_, 2, uint16_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_4
 ****************************************************************************/

FETCH_XOR(__atomic_fetch_xor_, 4, uint32_t)
FETCH_XOR(nx_atomic_fetch_xor_, 4, int32_t)

/****************************************************************************
 * Name: __atomic_fetch_xor_8
 ****************************************************************************/

FETCH_XOR(__atomic_fetch_xor_, 8, uint64_t)
FETCH_XOR(nx_atomic_fetch_xor_, 8, int64_t)

/* Clang define the __sync builtins, add #ifndef to avoid
 * redefined/redeclared problem.
 */

#ifndef __clang__

/****************************************************************************
 * Name: __sync_add_and_fetch_1
 ****************************************************************************/

SYNC_ADD_FETCH(__sync_add_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_2
 ****************************************************************************/

SYNC_ADD_FETCH(__sync_add_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_4
 ****************************************************************************/

SYNC_ADD_FETCH(__sync_add_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_8
 ****************************************************************************/

SYNC_ADD_FETCH(__sync_add_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_1
 ****************************************************************************/

SYNC_SUB_FETCH(__sync_sub_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_2
 ****************************************************************************/

SYNC_SUB_FETCH(__sync_sub_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_4
 ****************************************************************************/

SYNC_SUB_FETCH(__sync_sub_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_8
 ****************************************************************************/

SYNC_SUB_FETCH(__sync_sub_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_1
 ****************************************************************************/

SYNC_OR_FETCH(__sync_or_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_2
 ****************************************************************************/

SYNC_OR_FETCH(__sync_or_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_4
 ****************************************************************************/

SYNC_OR_FETCH(__sync_or_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_8
 ****************************************************************************/

SYNC_OR_FETCH(__sync_or_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_1
 ****************************************************************************/

SYNC_AND_FETCH(__sync_and_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_2
 ****************************************************************************/

SYNC_AND_FETCH(__sync_and_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_4
 ****************************************************************************/

SYNC_AND_FETCH(__sync_and_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_8
 ****************************************************************************/

SYNC_AND_FETCH(__sync_and_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_1
 ****************************************************************************/

SYNC_XOR_FETCH(__sync_xor_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_2
 ****************************************************************************/

SYNC_XOR_FETCH(__sync_xor_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_4
 ****************************************************************************/

SYNC_XOR_FETCH(__sync_xor_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_8
 ****************************************************************************/

SYNC_XOR_FETCH(__sync_xor_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_1
 ****************************************************************************/

SYNC_NAND_FETCH(__sync_nand_and_fetch_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_2
 ****************************************************************************/

SYNC_NAND_FETCH(__sync_nand_and_fetch_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_4
 ****************************************************************************/

SYNC_NAND_FETCH(__sync_nand_and_fetch_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_8
 ****************************************************************************/

SYNC_NAND_FETCH(__sync_nand_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_1
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(__sync_bool_compare_and_swap_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_2
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(__sync_bool_compare_and_swap_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_4
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(__sync_bool_compare_and_swap_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_8
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(__sync_bool_compare_and_swap_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_1
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(__sync_val_compare_and_swap_, 1, uint8_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_2
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(__sync_val_compare_and_swap_, 2, uint16_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_4
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(__sync_val_compare_and_swap_, 4, uint32_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_8
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(__sync_val_compare_and_swap_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_synchronize
 ****************************************************************************/

void weak_function __sync_synchronize(void)
{
#ifdef UP_DMB
  UP_DMB();
#endif
}

#endif /* __clang__ */
