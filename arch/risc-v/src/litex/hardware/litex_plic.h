/****************************************************************************
 * arch/risc-v/src/litex/hardware/litex_plic.h
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

#ifndef __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H
#define __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "litex_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LITEX_CORE_VEXRISCV_SMP
#  define LITEX_PLIC_PRIORITY    (LITEX_PLIC_BASE + 0x000000)
#  define LITEX_PLIC_PENDING1    (LITEX_PLIC_BASE + 0x001000)

#  define LITEX_PLIC_ENABLE1     (LITEX_PLIC_BASE + 0x002080)
#  define LITEX_PLIC_ENABLE2     (LITEX_PLIC_BASE + 0x002084)
#  define LITEX_PLIC_THRESHOLD   (LITEX_PLIC_BASE + 0x201000)
#  define LITEX_PLIC_CLAIM       (LITEX_PLIC_BASE + 0x201004)
#else

/* litex vexRiscv does not follow RISC-V privileged specification and
 * uses two additional CSRs: mask and pending.
 */
#define LITEX_MMASK_CSR     0xBC0
#define LITEX_MPENDING_CSR     0xFC0

#endif

#endif /* __ARCH_RISCV_SRC_LITEX_HARDWARE_LITEX_PLIC_H */
