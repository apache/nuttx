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
#include <nuttx/arch.h>

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
 * Name: __sync_*_{1,2,4,8}
 ****************************************************************************/

#ifdef ARCH_SYNC_DEFINE
ARCH_SYNC_DEFINE(__sync, uint8_t, 1)
ARCH_SYNC_DEFINE(__sync, uint16_t, 2)
ARCH_SYNC_DEFINE(__sync, uint32_t, 4)
ARCH_SYNC_DEFINE(__sync, uint64_t, 8)
#endif

/****************************************************************************
 * Name: __sync_synchronize
 ****************************************************************************/

#ifdef ARCH_SYNC_SYNCHRONIZE
ARCH_SYNC_SYNCHRONIZE(__sync)
#endif

#endif /* __clang__ */
