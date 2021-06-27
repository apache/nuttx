/****************************************************************************
 * arch/risc-v/src/k210/k210_memorymap.h
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

#ifndef _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H
#define _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/k210_memorymap.h"
#include "hardware/k210_uart.h"
#include "hardware/k210_clint.h"
#include "hardware/k210_plic.h"
#include "hardware/k210_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
extern uintptr_t *_default_stack_limit;
#define K210_IDLESTACK_BASE  (uintptr_t)&_default_stack_limit
#else
#define K210_IDLESTACK_BASE  _default_stack_limit
#endif

#define K210_IDLESTACK0_BASE (K210_IDLESTACK_BASE)
#define K210_IDLESTACK0_TOP  (K210_IDLESTACK0_BASE + CONFIG_IDLETHREAD_STACKSIZE)

#define K210_IDLESTACK1_BASE (K210_IDLESTACK0_TOP)
#define K210_IDLESTACK1_TOP  (K210_IDLESTACK1_BASE + CONFIG_IDLETHREAD_STACKSIZE)

#if defined(CONFIG_SMP) && (CONFIG_SMP_NCPUS > 1)
#define K210_HEAP_START   (K210_IDLESTACK1_TOP)
#else
#define K210_HEAP_START   (K210_IDLESTACK0_TOP)
#endif

#endif /* _ARCH_RISCV_SRC_K210_K210_MEMORYMAP_H */
