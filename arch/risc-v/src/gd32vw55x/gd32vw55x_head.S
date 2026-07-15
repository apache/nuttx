/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gd32vw55x_head.S
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

#include "chip.h"
#include "gd32vw55x_memorymap.h"
#include "hardware/gd32vw55x_eclic.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

  /* Disable all interrupts */

  csrw CSR_MIE, zero
  csrci CSR_MSTATUS, MSTATUS_MIE

  /* Set stack pointer to the idle thread stack */

  la   sp, GD32VW55X_IDLESTACK_TOP

  /* Enable the I-cache: executing from flash without it is too slow to
   * meet interrupt latency (UART RX overruns at 115200)
   */

  csrsi CSR_MCACHE_CTL, MCACHE_CTL_IC_EN

  /* NMI vector shares mtvec (Nuclei MMISC_CTL bit 9) */

  li   t0, MMISC_CTL_NMI_CAUSE_FFF
  csrs CSR_MMISC_CTL, t0

  /* ECLIC vector table base (used only by hardware-vectored irqs) */

  la   t0, __trap_vec
  csrw CSR_MTVT, t0

  /* Non-vectored interrupt entry: __trap_vec, MTVT2 enable bit set */

  la   t0, __trap_vec
  csrw CSR_MTVT2, t0
  csrsi CSR_MTVT2, 0x1

  /* Exception entry in ECLIC mode: mtvec = __trap_vec | 0b000011 */

  la   t0, __trap_vec
  ori  t0, t0, ECLIC_MTVEC_MODE
  csrw CSR_MTVEC, t0

#ifdef CONFIG_ARCH_FPU
  /* Enable the FPU (mstatus.FS = initial) */

  li   t0, MSTATUS_FS_INIT
  csrs CSR_MSTATUS, t0
  fscsr zero
#endif

  /* Jump to __gd32vw55x_start */

  jal  x1, __gd32vw55x_start

  /* We shouldn't return from __gd32vw55x_start */

1:
  wfi
  j    1b

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
