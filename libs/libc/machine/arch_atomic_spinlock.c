/****************************************************************************
 * libs/libc/machine/arch_atomic_spinlock.c
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

#include <sched.h>

#include <nuttx/spinlock.h>
#include <nuttx/lib/arch_atomic.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef ARCH_ATOMIC_SPECIFIER
#define ARCH_ATOMIC_SPECIFIER weak_function

#undef atomic_lock
#undef atomic_unlock

#if !defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__)
#  define atomic_lock() \
     atomic_lock_sched()
#  define atomic_unlock(flags) \
     atomic_unlock_sched(flags)
#else
#  define atomic_lock() \
     spin_lock_irqsave_notrace(&g_atomic_spinlock)
#  define atomic_unlock(flags) \
     spin_unlock_irqrestore_notrace(&g_atomic_spinlock, flags)
#endif

#ifdef CONFIG_LIBC_ATOMIC_ARCH
#  define ARCH_ATOMIC_NOPREFIX_DEFINE  ARCH_ATOMIC_DEFINE
#  define ARCH_SYNC_NOPREFIX_DEFINE    ARCH_SYNC_DEFINE
#else
#  define ARCH_ATOMIC_NOPREFIX_DEFINE(prefix, t, n)
#  define ARCH_SYNC_NOPREFIX_DEFINE(prefix, t, n)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if !defined(ARCH_HAVE_ATOMIC_1) || !defined(ARCH_HAVE_ATOMIC_2) || \
    !defined(ARCH_HAVE_ATOMIC_8)
static spinlock_t g_atomic_spinlock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#  if !defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__)

/* User code cannot disable the interrupts. Lock the scheduler instead. */

static inline irqstate_t atomic_lock_sched(void)
{
  sched_lock();
  while (!spin_trylock_notrace(&g_atomic_spinlock));

  return 0;
}

static inline void atomic_unlock_sched(irqstate_t flags)
{
  spin_unlock_notrace(&g_atomic_spinlock);
  sched_unlock();
}
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef ARCH_HAVE_ATOMIC_1
ARCH_ATOMIC_NOPREFIX_DEFINE(atomic, uint8_t, 1)
ARCH_ATOMIC_DEFINE(__atomic, uint8_t, 1)
#endif

#ifndef ARCH_HAVE_ATOMIC_2
ARCH_ATOMIC_NOPREFIX_DEFINE(atomic, uint16_t, 2)
ARCH_ATOMIC_DEFINE(__atomic, uint16_t, 2)
#endif

#ifndef ARCH_HAVE_ATOMIC_8
ARCH_ATOMIC_NOPREFIX_DEFINE(atomic, int64_t, 8)
ARCH_ATOMIC_DEFINE(__atomic, uint64_t, 8)
#endif

#ifndef __clang__
#  ifdef ARCH_SYNC_DEFINE
#    ifndef ARCH_HAVE_ATOMIC_1
ARCH_SYNC_NOPREFIX_DEFINE(sync, uint8_t, 1)
ARCH_SYNC_DEFINE(__sync, uint8_t, 1)
#    endif
#    ifndef ARCH_HAVE_ATOMIC_2
ARCH_SYNC_NOPREFIX_DEFINE(sync, uint16_t, 2)
ARCH_SYNC_DEFINE(__sync, uint16_t, 2)
#    endif
#    ifndef ARCH_HAVE_ATOMIC_8
ARCH_SYNC_NOPREFIX_DEFINE(sync, uint64_t, 8)
ARCH_SYNC_DEFINE(__sync, uint64_t, 8)
#    endif
#  endif
#endif
