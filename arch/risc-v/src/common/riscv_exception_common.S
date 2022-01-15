/****************************************************************************
 * arch/risc-v/src/common/riscv_exception_common.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

#ifdef CONFIG_ARCH_RV32
#  define REGLOAD   lw
#  define REGSTORE  sw
#else
#  define REGLOAD   ld
#  define REGSTORE  sd
#endif

#ifdef CONFIG_IRQ_NSTACKS
#  define IRQ_NSTACKS CONFIG_IRQ_NSTACKS
#elif defined CONFIG_SMP
#  define IRQ_NSTACKS CONFIG_SMP_NCPUS
#else
#  define IRQ_NSTACKS 1
#endif

/****************************************************************************
 * Name: exception_common
 ****************************************************************************/

  .section .text
  .global exception_common
  .align  8

exception_common:

  addi       sp, sp, -XCPTCONTEXT_SIZE

  REGSTORE   x1,  REG_X1(sp)   /* ra */
#ifdef RISCV_SAVE_GP
  REGSTORE   x3,  REG_X3(sp)   /* gp */
#endif
  REGSTORE   x4,  REG_X4(sp)   /* tp */
  REGSTORE   x5,  REG_X5(sp)   /* t0 */
  REGSTORE   x6,  REG_X6(sp)   /* t1 */
  REGSTORE   x7,  REG_X7(sp)   /* t2 */
  REGSTORE   x8,  REG_X8(sp)   /* s0 */
  REGSTORE   x9,  REG_X9(sp)   /* s1 */
  REGSTORE   x10, REG_X10(sp)  /* a0 */
  REGSTORE   x11, REG_X11(sp)  /* a1 */
  REGSTORE   x12, REG_X12(sp)  /* a2 */
  REGSTORE   x13, REG_X13(sp)  /* a3 */
  REGSTORE   x14, REG_X14(sp)  /* a4 */
  REGSTORE   x15, REG_X15(sp)  /* a5 */
  REGSTORE   x16, REG_X16(sp)  /* a6 */
  REGSTORE   x17, REG_X17(sp)  /* a7 */
  REGSTORE   x18, REG_X18(sp)  /* s2 */
  REGSTORE   x19, REG_X19(sp)  /* s3 */
  REGSTORE   x20, REG_X20(sp)  /* s4 */
  REGSTORE   x21, REG_X21(sp)  /* s5 */
  REGSTORE   x22, REG_X22(sp)  /* s6 */
  REGSTORE   x23, REG_X23(sp)  /* s7 */
  REGSTORE   x24, REG_X24(sp)  /* s8 */
  REGSTORE   x25, REG_X25(sp)  /* s9 */
  REGSTORE   x26, REG_X26(sp)  /* s10 */
  REGSTORE   x27, REG_X27(sp)  /* s11 */
  REGSTORE   x28, REG_X28(sp)  /* t3 */
  REGSTORE   x29, REG_X29(sp)  /* t4 */
  REGSTORE   x30, REG_X30(sp)  /* t5 */
  REGSTORE   x31, REG_X31(sp)  /* t6 */

  csrr       s0, mstatus
  REGSTORE   s0, REG_INT_CTX(sp)  /* mstatus          */

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_X2(sp)       /* original SP      */

  /* Setup arg0(exception cause), arg1(context) */

  csrr       a0, mcause           /* exception cause  */
  csrr       s0, mepc
  REGSTORE   s0, REG_EPC(sp)      /* exception PC     */

  mv         a1, sp      /* context = sp */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Load mhartid (cpuid) */

  csrr s0, mhartid

  /* Switch to interrupt stack */
#if IRQ_NSTACKS > 1
  li t0, (CONFIG_ARCH_INTERRUPTSTACK & ~15)
  mul t0, s0, t0
  la s0, g_intstacktop
  sub sp, s0, t0
#else
  la sp, g_intstacktop
#endif

#endif

  /* Call interrupt handler in C */

  jal        x1, riscv_dispatch_irq

  /* If context switch is needed, return a new sp     */

  mv         sp, a0
  REGLOAD    s0, REG_EPC(sp)     /* restore mepc      */
  csrw       mepc, s0

  REGLOAD    s0, REG_INT_CTX(sp) /* restore mstatus   */
  csrw       mstatus, s0

#ifdef RISCV_SAVE_GP
  REGLOAD    x3,  REG_X3(sp)   /* gp */
#endif
  REGLOAD    x4,  REG_X4(sp)   /* tp */
  REGLOAD    x5,  REG_X5(sp)   /* t0 */
  REGLOAD    x6,  REG_X6(sp)   /* t1 */
  REGLOAD    x7,  REG_X7(sp)   /* t2 */
  REGLOAD    x8,  REG_X8(sp)   /* s0 */
  REGLOAD    x9,  REG_X9(sp)   /* s1 */
  REGLOAD    x10, REG_X10(sp)  /* a0 */
  REGLOAD    x11, REG_X11(sp)  /* a1 */
  REGLOAD    x12, REG_X12(sp)  /* a2 */
  REGLOAD    x13, REG_X13(sp)  /* a3 */
  REGLOAD    x14, REG_X14(sp)  /* a4 */
  REGLOAD    x15, REG_X15(sp)  /* a5 */
  REGLOAD    x16, REG_X16(sp)  /* a6 */
  REGLOAD    x17, REG_X17(sp)  /* a7 */
  REGLOAD    x18, REG_X18(sp)  /* s2 */
  REGLOAD    x19, REG_X19(sp)  /* s3 */
  REGLOAD    x20, REG_X20(sp)  /* s4 */
  REGLOAD    x21, REG_X21(sp)  /* s5 */
  REGLOAD    x22, REG_X22(sp)  /* s6 */
  REGLOAD    x23, REG_X23(sp)  /* s7 */
  REGLOAD    x24, REG_X24(sp)  /* s8 */
  REGLOAD    x25, REG_X25(sp)  /* s9 */
  REGLOAD    x26, REG_X26(sp)  /* s10 */
  REGLOAD    x27, REG_X27(sp)  /* s11 */
  REGLOAD    x28, REG_X28(sp)  /* t3 */
  REGLOAD    x29, REG_X29(sp)  /* t4 */
  REGLOAD    x30, REG_X30(sp)  /* t5 */
  REGLOAD    x31, REG_X31(sp)  /* t6 */

  REGLOAD    x1,  REG_X1(sp)   /* ra */

  REGLOAD    sp,  REG_X2(sp)   /* restore original sp */

  /* Return from Machine Interrupt */

  mret

/************************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ************************************************************************************/

/************************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
  .skip  (((CONFIG_ARCH_INTERRUPTSTACK * IRQ_NSTACKS) + 8) & ~15)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, ((CONFIG_ARCH_INTERRUPTSTACK * IRQ_NSTACKS) & ~15)
#endif
