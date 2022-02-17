/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_MEMORYMAP_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_internal.h"

#include "hardware/mpfs_clint.h"
#include "hardware/mpfs_memorymap.h"
#include "hardware/mpfs_plic.h"
#include "hardware/mpfs_sysreg.h"
#include "hardware/mpfs_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define MPFS_IDLESTACK_BASE  (uintptr_t)&_ebss
#else
#define MPFS_IDLESTACK_BASE  _ebss
#endif

#define MPFS_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~15)

#define MPFS_IDLESTACK0_TOP  (MPFS_IDLESTACK_BASE + MPFS_IDLESTACK_SIZE)

#define MPFS_IDLESTACK_TOP   (MPFS_IDLESTACK0_TOP)

#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_MEMORYMAP_H */
