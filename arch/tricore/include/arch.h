/****************************************************************************
 * arch/tricore/include/arch.h
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

#ifndef __ARCH_TRICORE_INCLUDE_ARCH_H
#define __ARCH_TRICORE_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stddef.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRICORE_CPU_VCON0         0xb000
#define TRICORE_CPU_PCXI          0xfe00
#define TRICORE_CPU_PSW         0xfe04
#define TRICORE_CPU_CORE_ID         0xfe1c
#define TRICORE_CPU_BIV         0xfe20
#define TRICORE_CPU_BTV         0xfe24
#define TRICORE_CPU_ISP         0xfe28
#define TRICORE_CPU_FCX         0xfe38
#define TRICORE_CPU_LCX         0xfe3c

#define TRICORE_CFG_SSW_PSW_DEFAULT     0x980
#define TRICORE_CPU_CORE_ID_MASK GENMASK(2, 0)

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
#define REG_PC           REG_UA11

#define REG_LPC          REG_LA11

#define TC_CONTEXT_REGS  (16)

#define XCPTCONTEXT_REGS (TC_CONTEXT_REGS)
#define XCPTCONTEXT_SIZE (sizeof(void *) * TC_CONTEXT_REGS)

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

/* Address <--> Context Save Areas */

#define tricore_csa2addr(csa) ((uintptr_t *)((((csa) & 0x000f0000) << 12) \
                                             | (((csa) & 0x0000ffff) << 6)))
#define tricore_addr2csa(addr) ((uintptr_t)(((((uintptr_t)(addr)) & 0xf0000000) >> 12) \
                                            | (((uintptr_t)(addr) & 0x003fffc0) >> 6)))

#ifndef __ASSEMBLY__
struct xcptcontext
{
  uintptr_t *saved_regs;
  uintptr_t *regs;
};
#endif /* __ASSEMBLY__ */

#define TRICORE_CFG_SSW_CSA_BOOT_PTR_START    (0x7010ec00)
#define TRICORE_CFG_SSW_CSA_USTACK_PTR    (0x7010eb00)
#define TRICORE_CFG_DSPR0_START     (0x70000000)

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#  define TRICORE_IR_LASR_ADDR          (0xf4430c20)
#  define TRICORE_IR_LASR_TOS_STRIDE    (0x34)
#  define TRICORE_IR_LASR_PIPN_WIDTH    (11)
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  /* TC3x INT_CHx_LASR: 0xf0037204 + CH_index * 0x10. */

#  define TRICORE_IR_LASR_ADDR          (0xf0037204)
#  define TRICORE_IR_LASR_TOS_STRIDE    (0x10)
#  define TRICORE_IR_LASR_PIPN_WIDTH    (10)
#endif

#ifndef __ASSEMBLY__
#define TRICORE_MTCR(reg, val)  { __asm__ __volatile__ ("dsync" : : : "memory"); \
            __asm__ __volatile__ ("mtcr %0,%1"::"i"(reg),"d"(val): "memory"); \
            __asm__ __volatile__ ("isync" : : : "memory");}

#define TRICORE_MFCR(reg, val)  {__asm__ __volatile__ ("mfcr %0,%1": "=d"(val) :"i"(reg): "memory");}
#define TRICORE_IRQ_ENABLE()  { __asm__ __volatile__ ("enable" : : : "memory"); }
#define TRICORE_IRQ_DISABLE()  { __asm__ __volatile__ ("disable" : : : "memory"); }
#endif /* __ASSEMBLY__ */

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_ARCH_H */
