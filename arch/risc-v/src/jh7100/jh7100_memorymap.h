/****************************************************************************
 * arch/risc-v/src/jh7100/jh7100_memorymap.h
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

#ifndef _ARCH_RISCV_SRC_JH7100_JH7100_MEMORYMAP_H
#define _ARCH_RISCV_SRC_JH7100_JH7100_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/jh7100_memorymap.h"
#include "hardware/jh7100_uart.h"
#include "hardware/jh7100_clint.h"
#include "hardware/jh7100_plic.h"
#include "hardware/jh7100_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _default_stack_limit */

#ifndef __ASSEMBLY__
extern uintptr_t *_default_stack_limit;
#define JH7100_IDLESTACK_BASE  (uintptr_t)&_default_stack_limit
#else
#define JH7100_IDLESTACK_BASE  _default_stack_limit
#endif

#define JH7100_IDLESTACK0_TOP  (JH7100_IDLESTACK_BASE + CONFIG_IDLETHREAD_STACKSIZE)
#define JH7100_IDLESTACK_TOP   (JH7100_IDLESTACK0_TOP)

#endif /* _ARCH_RISCV_SRC_JH7100_JH7100_MEMORYMAP_H */
