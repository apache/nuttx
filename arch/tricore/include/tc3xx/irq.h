/****************************************************************************
 * arch/tricore/include/tc3xx/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H
#define __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <IfxCpu_Intrinsics.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Upper CSA */

#define REG_UPCXI        0
#define REG_PSW          1
#define REG_A10          2
#define REG_UA11         3
#define REG_D8           4
#define REG_D9           5
#define REG_D10          6
#define REG_D11          7
#define REG_A12          8
#define REG_A13          9
#define REG_A14          10
#define REG_A15          11
#define REG_D12          12
#define REG_D13          13
#define REG_D14          14
#define REG_D15          15

/* Lower CSA */

#define REG_LPCXI        0
#define REG_LA11         1
#define REG_A2           2
#define REG_A3           3
#define REG_D0           4
#define REG_D1           5
#define REG_D2           6
#define REG_D3           7
#define REG_A4           8
#define REG_A5           9
#define REG_A6           10
#define REG_A7           11
#define REG_D4           12
#define REG_D5           13
#define REG_D6           14
#define REG_D7           15

#define REG_RA           REG_UA11
#define REG_SP           REG_A10
#define REG_UPC          REG_UA11

#define REG_LPC          REG_LA11

#define TC_CONTEXT_REGS  (16)

#define XCPTCONTEXT_REGS (TC_CONTEXT_REGS)
#define XCPTCONTEXT_SIZE (sizeof(void *) * TC_CONTEXT_REGS)

#define NR_IRQS          (255)

/* PSW: Program Status Word Register */

#define PSW_CDE         (1 << 7) /* Bits 7: Call Depth Count Enable */
#define PSW_IS          (1 << 9) /* Bits 9: Interrupt Stack Control */
#define PSW_IO          (10)     /* Bits 10-11: Access Privilege Level Control (I/O Privilege) */
#  define PSW_IO_USER0      (0 << PSW_IO)
#  define PSW_IO_USER1      (1 << PSW_IO)
#  define PSW_IO_SUPERVISOR (2 << PSW_IO)

/* PCXI: Previous Context Information and Pointer Register */

#define PCXI_UL         (1 << 20) /* Bits 20: Upper or Lower Context Tag */
#define PCXI_PIE        (1 << 21) /* Bits 21: Previous Interrupt Enable */

/* FCX: Free CSA List Head Pointer Register */

#define FCX_FCXO        (0)       /* Bits 0-15: FCX Offset Address */
#define FCX_FCXS        (16)      /* Bits 16-19: FCX Segment Address */
#define FCX_FCXO_MASK   (0xffff << FCX_FCXO)
#define FCX_FCXS_MASK   (0xf    << FCX_FCXS)
#define FCX_FREE        (FCX_FCXS_MASK | FCX_FCXO_MASK) /* Free CSA manipulation */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct xcptcontext
{
  /* These are saved copies of the context used during
   * signal processing.
   */

  uintptr_t *saved_regs;

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uintptr_t *regs;
};
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H */
