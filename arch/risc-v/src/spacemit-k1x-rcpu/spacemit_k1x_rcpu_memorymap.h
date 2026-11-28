/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_MEMORYMAP_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The K1 RCPU is a Nuclei N308 core with its own address map.  All addresses
 * here are from the point of view of the RCPU, not of the application
 * processor.
 */

/* Nuclei IRegion.
 *
 * The core reports its capabilities in CSR_MCFG_INFO.  When
 * MCFG_INFO_IREGION_EXIST is set, CSR_MIRGB_INFO holds the base of the
 * IRegion (1 KB aligned) and the ECLIC and the machine timer live at fixed
 * offsets inside it.
 */

#define SPACEMIT_K1X_RCPU_IREGION_ALIGN_MASK      0xfffffc00u
#define SPACEMIT_K1X_RCPU_IREGION_ECLIC_OFFSET    0x00020000u
#define SPACEMIT_K1X_RCPU_IREGION_TIMER_OFFSET    0x00030000u

/* Nuclei CSR_MCFG_INFO bit used to detect the IRegion */

#define SPACEMIT_K1X_RCPU_MCFG_INFO_IREGION_EXIST (1 << 16)

/* Fallback addresses, used only when the core reports that no IRegion is
 * present.  These are the Nuclei default values.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_BASE_FALLBACK     0x0c000000u
#define SPACEMIT_K1X_RCPU_SYSTIMER_BASE_FALLBACK  0x02000000u

/* RCPU UART (r_uart0).  The block is 16550 compatible with a 4 byte register
 * stride, configured through CONFIG_16550_REGWIDTH=32 with the default
 * CONFIG_16550_REGINCR=1.  It is the PXA variant, so the interrupt output is
 * gated by MCR.OUT2, which CONFIG_16550_SET_MCR_OUT2 takes care of.
 */

#define SPACEMIT_K1X_RCPU_RUART0_BASE             0xc0881000u

/* Boot stack.
 *
 * The image is linked for the address it runs at, so the boot stack
 * is simply placed above the cleared BSS.  _ebss comes from the linker;
 * SMP_STACK_SIZE comes from the common RISC-V memory map and rounds
 * CONFIG_IDLETHREAD_STACKSIZE up to a 16 byte boundary.
 */

#ifndef __ASSEMBLY__
#define SPACEMIT_K1X_RCPU_IDLESTACK_BASE          (uintptr_t)_ebss
#else
#define SPACEMIT_K1X_RCPU_IDLESTACK_BASE          _ebss
#endif
#define SPACEMIT_K1X_RCPU_IDLESTACK_TOP           (SPACEMIT_K1X_RCPU_IDLESTACK_BASE + SMP_STACK_SIZE)

#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_SPACEMIT_K1X_RCPU_MEMORYMAP_H */
