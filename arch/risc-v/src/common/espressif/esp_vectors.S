/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_vectors.S
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

#include <arch/irq.h>

#include "chip.h"

#include "sdkconfig.h"
#include "soc/soc.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global  _vector_table

#ifdef CONFIG_SOC_INT_CLIC_SUPPORTED
  .global _mtvt_table
#endif

/****************************************************************************
 * Section: .exception_vectors.text
 ****************************************************************************/

  .section .exception_vectors.text

  /* Prevent the compiler from generating 2-byte instruction in the vector tables */

  .option push
  .option norvc

/****************************************************************************
 * Name: _vector_table
 ****************************************************************************/

#ifdef CONFIG_SOC_INT_CLIC_SUPPORTED

  /* Non-hardware vectored interrupt entry. MTVEC CSR points here.
   *
   * On targets that use CLIC as their interrupt controller, when an exception occurs, the CPU
   * jumps to the address stored in MTVEC[31:6] << 6. The CPU will also jump to this location
   * if an interrupt is configured as non-vectored (CLIC_INT_ATTR.shv = 0).
   *
   * Because of the left-shift `<< 6`, this entry must be aligned on 64.
   */

  .balign 0x40
  .type     _vector_table, @function

_vector_table:
  j exception_common
  nop

#else /* CONFIG_SOC_INT_PLIC_SUPPORTED */

  /* This is the vector table. MTVEC points here.
   *
   * Use 4-byte instructions here. 1 instruction = 1 entry of the table.
   * The CPU jumps to MTVEC (i.e. the first entry) in case of an exception,
   * and (MTVEC & 0xfffffffc) + (mcause & 0x7fffffff) * 4, in case of an
   * interrupt.
   *
   * Note: for our CPU, we need to place this on a 256-byte boundary, as CPU
   * only uses the 24 MSBs of the MTVEC, i.e. (MTVEC & 0xffffff00).
   */

  .balign 0x100
  .type     _vector_table, @function

_vector_table:

  j _panic_handler            /* 0: Exception entry */
  j exception_common        /* 1: Free interrupt number */
  j exception_common        /* 2: Free interrupt number */
  j exception_common        /* 3: Free interrupt number */
  j exception_common        /* 4: Free interrupt number */
  j exception_common        /* 5: Free interrupt number */
  j exception_common        /* 6: Free interrupt number */
  j exception_common        /* 7: Free interrupt number */
  j exception_common        /* 8: Free interrupt number */
  j exception_common        /* 9: Free interrupt number */
  j exception_common        /* 10: Free interrupt number */
  j exception_common        /* 11: Free interrupt number */
  j exception_common        /* 12: Free interrupt number */
  j exception_common        /* 13: Free interrupt number */
  j exception_common        /* 14: Free interrupt number */
  j exception_common        /* 15: Free interrupt number */
  j exception_common        /* 16: Free interrupt number */
  j exception_common        /* 17: Free interrupt number */
  j exception_common        /* 18: Free interrupt number */
  j exception_common        /* 19: Free interrupt number */
  j exception_common        /* 20: Free interrupt number */
  j exception_common        /* 21: Free interrupt number */
  j exception_common        /* 22: Free interrupt number */
  j exception_common        /* 23: Free interrupt number */
  j _panic_handler            /* 24: ETS_INT_WDT_INUM panic-interrupt (soc-level panic) */
  j _panic_handler            /* 25: ETS_CACHEERR_INUM panic-interrupt (soc-level panic) */
  j _panic_handler               /* 26: ETS_MEMPROT_ERR_INUM handler (soc-level panic) */
  j _panic_handler                /* 27: ETS_ASSIST_DEBUG_INUM handler (soc-level panic) */
  j exception_common        /* 28: Free interrupt number */
  j exception_common        /* 29: Free interrupt number */
  j exception_common        /* 30: Free interrupt number */
  j exception_common        /* 31: Free interrupt number */

#endif

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   Handles interrupts. If kernel is in S-mode, handles delegated interrupts
 *   in S-mode interrupt handler.
 *
 ****************************************************************************/

  .global _panic_handler

_panic_handler:
  j exception_common /* Dispatch the system call */#endif

/****************************************************************************
 * Name: _mtvt_table
 ****************************************************************************/

#ifdef CONFIG_SOC_INT_CLIC_SUPPORTED
  /* Vectored interrupt table. MTVT CSR points here.
   *
   * If an interrupt occurs and is configured as (hardware) vectored, the CPU
   * will jump to MTVT[31:0] + 4 * interrupt_id
   *
   * In the case of the ESP32P4, the interrupt matrix, between the CPU
   * interrupt lines and the peripherals, offers 32 lines. As such, the
   * interrupt_id between 0 and 31.
   *
   * Since the interrupts are initialized as vectored on CPU start, we can
   * manage the special interrupts ETS_T1_WDT_INUM, ETS_CACHEERR_INUM and
   * ETS_MEMPROT_ERR_INUM here.
   */

  .balign 0x40
  .type _mtvt_table, @function

_mtvt_table:
  .word exception_common        /* 0: System interrupt number. Exceptions are non-vectored, won't load this. */
  .word exception_common        /* 1: System interrupt number */
  .word exception_common        /* 2: System interrupt number */
  .word exception_common        /* 3: System interrupt number */
  .word exception_common        /* 4: System interrupt number */
  .word exception_common        /* 5: System interrupt number */
  .word exception_common        /* 6: System interrupt number */
  .word exception_common        /* 7: System interrupt number */
  .word exception_common        /* 8: System interrupt number */
  .word exception_common        /* 9: System interrupt number */
  .word exception_common        /* 10: System interrupt number */
  .word exception_common        /* 11: System interrupt number */
  .word exception_common        /* 12: System interrupt number */
  .word exception_common        /* 13: System interrupt number */
  .word exception_common        /* 14: System interrupt number */
  .word exception_common        /* 15: System interrupt number */
  .word exception_common        /* 16: Free interrupt number */
  .word exception_common        /* 17: Free interrupt number */
  .word exception_common        /* 18: Free interrupt number */
  .word exception_common        /* 19: Free interrupt number */
  .word exception_common        /* 20: Free interrupt number */
  .word exception_common        /* 21: Free interrupt number */
  .word exception_common        /* 22: Free interrupt number */
  .word exception_common        /* 23: Free interrupt number */
  .word exception_common        /* 24: Free interrupt number */
  .word exception_common        /* 25: Free interrupt number */
  .word exception_common        /* 26: Free interrupt number */
  .word exception_common        /* 27: Free interrupt number */
  .word exception_common        /* 28: Free interrupt number */
  .word exception_common        /* 29: Free interrupt number */
  .word exception_common        /* 30: Free interrupt number */
  .word exception_common        /* 31: Free interrupt number */
  .word exception_common        /* 32: Free interrupt number */
  .word exception_common        /* 33: Free interrupt number */
  .word exception_common        /* 34: Free interrupt number */
  .word exception_common        /* 35: Free interrupt number */
  .word exception_common        /* 36: Free interrupt number */
  .word exception_common        /* 37: Free interrupt number */
  .word exception_common        /* 38: Free interrupt number */
  .word exception_common        /* 39: Free interrupt number */
  .word exception_common        /* 40: ETS_INT_WDT_INUM (+16) panic-interrupt (soc-level panic) */
  .word exception_common        /* 41: ETS_CACHEERR_INUM (+16) panic-interrupt (soc-level panic) */
  .word exception_common        /* 42: ETS_MEMPROT_ERR_INUM (+16) handler (soc-level panic) */
  .word exception_common        /* 43: ETS_ASSIST_DEBUG_INUM (+16) handler (soc-level panic) */
  .word exception_common        /* 44: ETS_IPC_ISR_INUM (+16) handler*/
  .word exception_common        /* 45: Free interrupt number */
  .word exception_common        /* 46: Free interrupt number */
  .word exception_common        /* 47: Free interrupt number */

  .size _mtvt_table, .-_mtvt_table

#endif

  /* Re-enable the compressed instruction set it is was enabled before */
  .option pop
