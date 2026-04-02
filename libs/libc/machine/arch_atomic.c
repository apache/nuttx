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

#define ARCH_ATOMIC_SPECIFIER

#include <nuttx/atomic.h>
#include <nuttx/macro.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYNC_ADD_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = *tmp + value;                                              \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_SUB_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = *tmp - value;                                              \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_OR_FETCH(fn, n, type)                                    \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = *tmp | value;                                              \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_AND_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = *tmp & value;                                              \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_XOR_FETCH(fn, n, type)                                   \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = *tmp ^ value;                                              \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_NAND_FETCH(fn, n, type)                                  \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type value)                   \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret;                                                         \
                                                                      \
    *tmp = ~(*tmp & value);                                           \
    ret = *tmp;                                                       \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_BOOL_CMP_SWAP(fn, n, type)                               \
                                                                      \
  bool weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type oldvalue,                \
                                        type newvalue)                \
  {                                                                   \
    bool ret = false;                                                 \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        ret = true;                                                   \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

#define SYNC_VAL_CMP_SWAP(fn, n, type)                                \
                                                                      \
  type weak_function CONCATENATE(fn, n)(FAR volatile void *ptr,       \
                                        type oldvalue,                \
                                        type newvalue)                \
  {                                                                   \
    irqstate_t irqstate = atomic_lock();                              \
    FAR type *tmp = (FAR type *)ptr;                                  \
    type ret = *tmp;                                                  \
                                                                      \
    if (*tmp == oldvalue)                                             \
      {                                                               \
        *tmp = newvalue;                                              \
      }                                                               \
                                                                      \
    atomic_unlock(irqstate);                                          \
    return ret;                                                       \
  }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __atomic_*_{1,2,4,8}
 ****************************************************************************/

ARCH_ATOMIC_DEFINE(__atomic, uint8_t, 1)
ARCH_ATOMIC_DEFINE(__atomic, uint16_t, 2)
ARCH_ATOMIC_DEFINE(__atomic, uint32_t, 4)
ARCH_ATOMIC_DEFINE(__atomic, uint64_t, 8)

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
