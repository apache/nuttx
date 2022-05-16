/****************************************************************************
 * libs/libc/machine/arch_atomic.c
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

#define STORE(n, type)                                    \
                                                          \
  void __atomic_store_ ## n (FAR volatile void *ptr,      \
                             type value,                  \
                             int memorder)                \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
                                                          \
    *(FAR type *)ptr = value;                             \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
  }

#define LOAD(n, type)                                     \
                                                          \
  type __atomic_load_ ## n (FAR const volatile void *ptr, \
                            int memorder)                 \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
                                                          \
    type ret = *(FAR type *)ptr;                          \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define EXCHANGE(n, type)                                 \
                                                          \
  type __atomic_exchange_ ## n (FAR volatile void *ptr,   \
                                type value,               \
                                int memorder)             \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    type ret = *tmp;                                      \
    *tmp = value;                                         \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define CMP_EXCHANGE(n, type)                             \
                                                          \
  bool __atomic_compare_exchange_ ## n (                  \
                                FAR volatile void *mem,   \
                                FAR void *expect,         \
                                type desired,             \
                                bool weak,                \
                                int success,              \
                                int failure)              \
  {                                                       \
    bool ret = false;                                     \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmpmem = (FAR type *)mem;                   \
    FAR type *tmpexp = (FAR type *)expect;                \
                                                          \
    if (*tmpmem == *tmpexp)                               \
      {                                                   \
        ret = true;                                       \
        *tmpmem = desired;                                \
      }                                                   \
    else                                                  \
      {                                                   \
        *tmpexp = *tmpmem;                                \
      }                                                   \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_ADD(n, type)                                \
                                                          \
  type __atomic_fetch_add_ ## n (FAR volatile void *ptr,  \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    *tmp = *tmp + value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_SUB(n, type)                                \
                                                          \
  type __atomic_fetch_sub_ ## n (FAR volatile void *ptr,  \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    *tmp = *tmp - value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_AND(n, type)                                \
                                                          \
  type __atomic_fetch_and_ ## n (FAR volatile void *ptr,  \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    *tmp = *tmp & value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_OR(n, type)                                 \
                                                          \
  type __atomic_fetch_or_ ## n (FAR volatile void *ptr,   \
                                type value,               \
                                int memorder)             \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    *tmp = *tmp | value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define FETCH_XOR(n, type)                                \
                                                          \
  type __atomic_fetch_xor_ ## n (FAR volatile void *ptr,  \
                                 type value,              \
                                 int memorder)            \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    *tmp = *tmp ^ value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define SYNC_ADD_FETCH(n, type)                           \
                                                          \
  type __sync_add_and_fetch_ ## n (                       \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = *tmp + value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_SUB_FETCH(n, type)                           \
                                                          \
  type __sync_sub_and_fetch_ ## n (                       \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = *tmp - value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_OR_FETCH(n, type)                            \
                                                          \
  type __sync_or_and_fetch_ ## n (                        \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = *tmp | value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_AND_FETCH(n, type)                           \
                                                          \
  type __sync_and_and_fetch_ ## n (                       \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = *tmp & value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_XOR_FETCH(n, type)                           \
                                                          \
  type __sync_xor_and_fetch_ ## n (                       \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = *tmp ^ value;                                  \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_NAND_FETCH(n, type)                          \
                                                          \
  type __sync_nand_and_fetch_ ## n (                      \
                                FAR volatile void *ptr,   \
                                type value)               \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    *tmp = ~(*tmp & value);                               \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return *tmp;                                          \
  }

#define SYNC_BOOL_CMP_SWAP(n, type)                       \
                                                          \
  bool __sync_bool_compare_and_swap_ ## n (               \
                                  FAR volatile void *ptr, \
                                  type oldvalue,          \
                                  type newvalue)          \
  {                                                       \
    bool ret = false;                                     \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
                                                          \
    if (*tmp == oldvalue)                                 \
      {                                                   \
        ret = true;                                       \
        *tmp = newvalue;                                  \
      }                                                   \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

#define SYNC_VAL_CMP_SWAP(n, type)                        \
                                                          \
  type __sync_val_compare_and_swap_ ## n (                \
                                  FAR volatile void *ptr, \
                                  type oldvalue,          \
                                  type newvalue)          \
  {                                                       \
    irqstate_t irqstate = spin_lock_irqsave(NULL);        \
    FAR type *tmp = (FAR type *)ptr;                      \
    type ret = *tmp;                                      \
                                                          \
    if (*tmp == oldvalue)                                 \
      {                                                   \
        *tmp = newvalue;                                  \
      }                                                   \
                                                          \
    spin_unlock_irqrestore(NULL, irqstate);               \
    return ret;                                           \
  }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __atomic_store_1
 ****************************************************************************/

STORE(1, uint8_t)

/****************************************************************************
 * Name: __atomic_store_2
 ****************************************************************************/

STORE(2, uint16_t)

/****************************************************************************
 * Name: __atomic_store_4
 ****************************************************************************/

STORE(4, uint32_t)

/****************************************************************************
 * Name: __atomic_store_8
 ****************************************************************************/

STORE(8, uint64_t)

/****************************************************************************
 * Name: __atomic_load_1
 ****************************************************************************/

LOAD(1, uint8_t)

/****************************************************************************
 * Name: __atomic_load__2
 ****************************************************************************/

LOAD(2, uint16_t)

/****************************************************************************
 * Name: __atomic_load__4
 ****************************************************************************/

LOAD(4, uint32_t)

/****************************************************************************
 * Name: __atomic_load__8
 ****************************************************************************/

LOAD(8, uint64_t)

/****************************************************************************
 * Name: __atomic_exchange_1
 ****************************************************************************/

EXCHANGE(1, uint8_t)

/****************************************************************************
 * Name: __atomic_exchange__2
 ****************************************************************************/

EXCHANGE(2, uint16_t)

/****************************************************************************
 * Name: __atomic_exchange__4
 ****************************************************************************/

EXCHANGE(4, uint32_t)

/****************************************************************************
 * Name: __atomic_exchange__8
 ****************************************************************************/

EXCHANGE(8, uint64_t)

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

/* Clang define the __sync builtins, add #ifndef to avoid
 * redefined/redeclared problem.
 */

#ifndef __clang__

/****************************************************************************
 * Name: __sync_add_and_fetch_1
 ****************************************************************************/

SYNC_ADD_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_2
 ****************************************************************************/

SYNC_ADD_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_4
 ****************************************************************************/

SYNC_ADD_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_add_and_fetch_8
 ****************************************************************************/

SYNC_ADD_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_1
 ****************************************************************************/

SYNC_SUB_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_2
 ****************************************************************************/

SYNC_SUB_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_4
 ****************************************************************************/

SYNC_SUB_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_8
 ****************************************************************************/

SYNC_SUB_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_1
 ****************************************************************************/

SYNC_OR_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_2
 ****************************************************************************/

SYNC_OR_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_4
 ****************************************************************************/

SYNC_OR_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_8
 ****************************************************************************/

SYNC_OR_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_1
 ****************************************************************************/

SYNC_AND_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_2
 ****************************************************************************/

SYNC_AND_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_4
 ****************************************************************************/

SYNC_AND_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_8
 ****************************************************************************/

SYNC_AND_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_1
 ****************************************************************************/

SYNC_XOR_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_2
 ****************************************************************************/

SYNC_XOR_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_4
 ****************************************************************************/

SYNC_XOR_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_8
 ****************************************************************************/

SYNC_XOR_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_1
 ****************************************************************************/

SYNC_NAND_FETCH(1, uint8_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_2
 ****************************************************************************/

SYNC_NAND_FETCH(2, uint16_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_4
 ****************************************************************************/

SYNC_NAND_FETCH(4, uint32_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_8
 ****************************************************************************/

SYNC_NAND_FETCH(8, uint64_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_1
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(1, uint8_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_2
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(2, uint16_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_4
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(4, uint32_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_8
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(8, uint64_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_1
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(1, uint8_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_2
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(2, uint16_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_4
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(4, uint32_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_8
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(8, uint64_t)

#endif /* __clang__ */

/****************************************************************************
 * Name: up_testset
 ****************************************************************************/

#if defined(CONFIG_SPINLOCK) && !defined(CONFIG_ARCH_HAVE_TESTSET)
spinlock_t up_testset(volatile FAR spinlock_t *lock)
{
  irqstate_t flags;
  spinlock_t ret;

  flags = spin_lock_irqsave(NULL);

  ret = *lock;
  if (ret == SP_UNLOCKED)
    {
      *lock = SP_LOCKED;
    }

  spin_unlock_irqrestore(NULL, flags);
  return ret;
}
#endif
