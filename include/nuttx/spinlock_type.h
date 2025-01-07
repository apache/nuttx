/****************************************************************************
 * include/nuttx/spinlock_type.h
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

#ifndef __INCLUDE_NUTTX_SPINLOCK_TYPE_H
#define __INCLUDE_NUTTX_SPINLOCK_TYPE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_RW_SPINLOCK) || defined(CONFIG_TICKET_SPINLOCK)
#include <nuttx/atomic.h>
#endif

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if defined(CONFIG_RW_SPINLOCK)
typedef atomic_t rwlock_t;
#  define RW_SP_UNLOCKED      0
#  define RW_SP_READ_LOCKED   1
#  define RW_SP_WRITE_LOCKED -1
#endif

#ifndef CONFIG_SPINLOCK
#  define SP_UNLOCKED 0  /* The Un-locked state */
#  define SP_LOCKED   1  /* The Locked state */

typedef uint8_t spinlock_t;
#elif defined(CONFIG_TICKET_SPINLOCK)

typedef struct spinlock_s
{
  atomic_t owner;
  atomic_t next;
} spinlock_t;

#  define SP_UNLOCKED (spinlock_t){0, 0}
#  define SP_LOCKED   (spinlock_t){0, 1}

#else

/* The architecture specific spinlock.h header file must also provide the
 * following:
 *
 *   SP_LOCKED   - A definition of the locked state value (usually 1)
 *   SP_UNLOCKED - A definition of the unlocked state value (usually 0)
 *   spinlock_t  - The type of a spinlock memory object.
 *
 * SP_LOCKED and SP_UNLOCKED must be constants of type spinlock_t.
 */

#include <arch/spinlock.h>

#endif /* CONFIG_SPINLOCK */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SPINLOCK_TYPE_H */
