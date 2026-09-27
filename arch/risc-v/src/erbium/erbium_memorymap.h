/****************************************************************************
 * arch/risc-v/src/erbium/erbium_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MEMORYMAP_H
#define __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_common_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  define ERBIUM_IDLESTACK_BASE  (uintptr_t)_ebss
#else
#  define ERBIUM_IDLESTACK_BASE  _ebss
#endif

#define ERBIUM_IDLESTACK_SIZE    SMP_STACK_SIZE
#define ERBIUM_IDLESTACK_TOP     (ERBIUM_IDLESTACK_BASE + ERBIUM_IDLESTACK_SIZE)

#endif /* __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MEMORYMAP_H */
