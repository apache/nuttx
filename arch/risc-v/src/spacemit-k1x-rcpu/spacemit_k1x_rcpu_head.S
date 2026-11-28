/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_head.S
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
#include "spacemit_k1x_rcpu_memorymap.h"
#include "hardware/spacemit_k1x_rcpu_eclic.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start
  .type   __start, function

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__start:

  /* The K1 RCPU resets with interrupts disabled and it is normally entered
   * either from the boot chain or, during bring-up, from a debugger that has
   * just loaded this image.  Do not assume any of that: mask interrupts and
   * set up the trap vector from scratch so that a warm entry is as safe as a
   * cold one.
   */

  csrw CSR_MIE, zero
  csrci CSR_MSTATUS, MSTATUS_MIE

  /* Boot stack: the linker script puts it directly above .bss */

  la   sp, SPACEMIT_K1X_RCPU_IDLESTACK_TOP

  /* Enable the instruction cache.
   *
   * The RCPU reports MCFG_INFO_ICACHE and MCFG_INFO_DCACHE, so both caches
   * exist.  The Nuclei firmware turns the I-cache on before it does anything
   * else; without it every fetch costs several cycles at 245 MHz and the
   * interrupt latency of a UART at 115200 baud is not met.
   *
   * The D-cache is deliberately left in its reset state for now: enabling it
   * changes the visibility rules for the shared DDR that the remoteproc
    * stage will use, and that has to be decided together with the cache
    * maintenance calls rather than switched on blindly here.
   */

  csrsi CSR_MCACHE_CTL, SPACEMIT_K1X_RCPU_MCACHE_CTL_IC_EN

  /* Let the NMI vector share mtvec with the ordinary trap vector
   * (MMISC_CTL bit 9).  The K1 RCPU drives an NMI from the audio subsystem,
   * so it must not land somewhere uninitialised.
   */

  li   t0, SPACEMIT_K1X_RCPU_MMISC_CTL_NMI_CAUSE_FFF
  csrs CSR_MMISC_CTL, t0

  /* ECLIC mode.
   *
   * In CLIC mode mtvec holds the non-vectored entry point with the low bits
    * set to the CLIC mode selector, mtvt holds the base of the per-source
    * vector table, and mtvt2 holds the entry used when the vector table is
    * not consulted.  These three must agree or the first interrupt goes to
    * the wrong place.
   */

  la   t0, __trap_vec
  csrw CSR_MTVT, t0

  la   t0, __trap_vec
  csrw CSR_MTVT2, t0
  csrsi CSR_MTVT2, 0x1

  la   t0, __trap_vec
  ori  t0, t0, SPACEMIT_K1X_RCPU_ECLIC_MTVEC_MODE
  csrw CSR_MTVEC, t0

#ifdef CONFIG_ARCH_FPU

  /* Enable the FPU.  The RCPU implements the F extension (ILP32F), and the
   * core requires mstatus.FS to be non-zero before any floating point
   * instruction is issued.
   */

  li   t0, MSTATUS_FS_INIT
  csrs CSR_MSTATUS, t0
  fscsr zero

#endif

  /* Jump to the C part of the start up */

  jal  x1, __spacemit_k1x_rcpu_start

  /* We should not return from __spacemit_k1x_rcpu_start */

1:
  wfi
  j    1b

  .global _init
  .global _fini

_init:
_fini:

  /* These do not have to do anything: init_array/fini_array are used. */

  ret
