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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spinlock.h>

#include "arch_atomic.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_atomic_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline irqstate_t atomic_lock(void)
{
  return spin_lock_irqsave(&g_atomic_lock);
}

static inline void atomic_unlock(irqstate_t flags)
{
  spin_unlock_irqrestore(&g_atomic_lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __atomic_store_8
 ****************************************************************************/

STORE(__atomic_store_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
STORE(atomic_store_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_load_8
 ****************************************************************************/

LOAD(__atomic_load_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
LOAD(atomic_load_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_exchange_8
 ****************************************************************************/

EXCHANGE(__atomic_exchange_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
EXCHANGE(atomic_exchange_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_compare_exchange_8
 ****************************************************************************/

CMP_EXCHANGE(__atomic_compare_exchange_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
CMP_EXCHANGE(atomic_compare_exchange_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_flag_test_and_set_8
 ****************************************************************************/

FLAG_TEST_AND_SET(__atomic_flags_test_and_set_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FLAG_TEST_AND_SET(atomic_flags_test_and_set_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_fetch_add_8
 ****************************************************************************/

FETCH_ADD(__atomic_fetch_add_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FETCH_ADD(atomic_fetch_add_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_fetch_sub_8
 ****************************************************************************/

FETCH_SUB(__atomic_fetch_sub_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FETCH_SUB(atomic_fetch_sub_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_fetch_and_8
 ****************************************************************************/

FETCH_AND(__atomic_fetch_and_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FETCH_AND(atomic_fetch_and_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_fetch_or_8
 ****************************************************************************/

FETCH_OR(__atomic_fetch_or_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FETCH_OR(atomic_fetch_or_, 8, int64_t)
#endif

/****************************************************************************
 * Name: __atomic_fetch_xor_8
 ****************************************************************************/

FETCH_XOR(__atomic_fetch_xor_, 8, uint64_t)
#ifndef CONFIG_LIBC_ATOMIC_TOOLCHAIN
FETCH_XOR(atomic_fetch_xor_, 8, int64_t)
#endif

/* Clang define the __sync builtins, add #ifndef to avoid
 * redefined/redeclared problem.
 */

#ifndef __clang__

/****************************************************************************
 * Name: __sync_add_and_fetch_8
 ****************************************************************************/

SYNC_ADD_FETCH(__sync_add_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_sub_and_fetch_8
 ****************************************************************************/

SYNC_SUB_FETCH(__sync_sub_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_or_and_fetch_8
 ****************************************************************************/

SYNC_OR_FETCH(__sync_or_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_and_and_fetch_8
 ****************************************************************************/

SYNC_AND_FETCH(__sync_and_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_xor_and_fetch_8
 ****************************************************************************/

SYNC_XOR_FETCH(__sync_xor_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_nand_and_fetch_8
 ****************************************************************************/

SYNC_NAND_FETCH(__sync_nand_and_fetch_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_bool_compare_and_swap_8
 ****************************************************************************/

SYNC_BOOL_CMP_SWAP(__sync_bool_compare_and_swap_, 8, uint64_t)

/****************************************************************************
 * Name: __sync_val_compare_and_swap_8
 ****************************************************************************/

SYNC_VAL_CMP_SWAP(__sync_val_compare_and_swap_, 8, uint64_t)

#endif /* __clang__ */
