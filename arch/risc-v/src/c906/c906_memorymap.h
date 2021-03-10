/****************************************************************************
 * arch/risc-v/src/c906/c906_memorymap.h
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

#ifndef _ARCH_RISCV_SRC_C906_C906_MEMORYMAP_H
#define _ARCH_RISCV_SRC_C906_C906_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/c906_memorymap.h"
#include "hardware/c906_uart.h"
#include "hardware/c906_clint.h"
#include "hardware/c906_plic.h"
#include "hardware/c906_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _default_stack_limit */

#ifndef __ASSEMBLY__
extern uintptr_t *_default_stack_limit;
#define C906_IDLESTACK_BASE  (uintptr_t)&_default_stack_limit
#else
#define C906_IDLESTACK_BASE  _default_stack_limit
#endif

#define C906_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~7)

#define C906_IDLESTACK0_TOP  (C906_IDLESTACK_BASE + C906_IDLESTACK_SIZE)

#define C906_IDLESTACK_TOP   (C906_IDLESTACK0_TOP)

#endif /* _ARCH_RISCV_SRC_C906_C906_MEMORYMAP_H */
