/****************************************************************************
 * arch/risc-v/src/mpfs/chip.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_CHIP_H
#define __ARCH_RISCV_SRC_MPFS_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "mpfs_memorymap.h"

#include "riscv_percpu.h"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the correct interrupt stack
 *   for the current CPU.
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
#if defined(CONFIG_SMP) || defined(CONFIG_ARCH_USE_S_MODE)
.macro  setintstack tmp0, tmp1
  csrr    \tmp0, CSR_SCRATCH
  REGLOAD sp, RISCV_PERCPU_IRQSTACK(\tmp0)
.endm
#endif /* defined(CONFIG_SMP) || defined(CONFIG_ARCH_USE_S_MODE) */
#endif /* CONFIG_ARCH_INTERRUPTSTACK > 15 */

#endif /* __ASSEMBLY__  */
#endif /* __ARCH_RISCV_SRC_MPFS_CHIP_H */
