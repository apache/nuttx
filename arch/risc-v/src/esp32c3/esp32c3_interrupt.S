/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_interrupt.S
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
#include <arch/rv32im/irq.h>

#include "chip.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global   _interrupt_handler
  .global   _exception_handler
  .global   g_intstackalloc
  .global   g_intstacktop

/****************************************************************************
 * Section: .noinit
 ****************************************************************************/

  .section  .noinit

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .balign   16
  .type     g_intstackalloc, @object
  .type     g_intstacktop,   @object
g_intstackalloc:
  .skip     ((CONFIG_ARCH_INTERRUPTSTACK + 8) & ~15)
g_intstacktop:
  .size     g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~15)
#endif

/****************************************************************************
 * Section: .iram1
 ****************************************************************************/

  .section  .iram1

/****************************************************************************
 * Name: _interrupt_handler
 ****************************************************************************/

  .type     _interrupt_handler, @function

_exception_handler:
_interrupt_handler:
  addi sp,  sp, -XCPTCONTEXT_SIZE

  sw   x1,  1*4(sp)   /* ra */
  sw   x3,  3*4(sp)   /* gp (For register dumping on exception handler) */
  sw   x4,  4*4(sp)   /* tp */
  sw   x5,  5*4(sp)   /* t0 */
  sw   x6,  6*4(sp)   /* t1 */
  sw   x7,  7*4(sp)   /* t2 */
  sw   x8,  8*4(sp)   /* s0 */
  sw   x9,  9*4(sp)   /* s1 */
  sw   x10, 10*4(sp)  /* a0 */
  sw   x11, 11*4(sp)  /* a1 */
  sw   x12, 12*4(sp)  /* a2 */
  sw   x13, 13*4(sp)  /* a3 */
  sw   x14, 14*4(sp)  /* a4 */
  sw   x15, 15*4(sp)  /* a5 */
  sw   x16, 16*4(sp)  /* a6 */
  sw   x17, 17*4(sp)  /* a7 */
  sw   x18, 18*4(sp)  /* s2 */
  sw   x19, 19*4(sp)  /* s3 */
  sw   x20, 20*4(sp)  /* s4 */
  sw   x21, 21*4(sp)  /* s5 */
  sw   x22, 22*4(sp)  /* s6 */
  sw   x23, 23*4(sp)  /* s7 */
  sw   x24, 24*4(sp)  /* s8 */
  sw   x25, 25*4(sp)  /* s9 */
  sw   x26, 26*4(sp)  /* s10 */
  sw   x27, 27*4(sp)  /* s11 */
  sw   x28, 28*4(sp)  /* t3 */
  sw   x29, 29*4(sp)  /* t4 */
  sw   x30, 30*4(sp)  /* t5 */
  sw   x31, 31*4(sp)  /* t6 */

  addi s0,  sp, XCPTCONTEXT_SIZE
  sw   s0,  2*4(sp)   /* Save original SP */

  /* Save MSTATUS (Machine Status Register) */

  csrr s0,  mstatus
  sw   s0,  32*4(sp)

  /* Save MEPC (Machine Exception Program Counter) */

  csrr s0,  mepc
  sw   s0,  0(sp)

  csrr a0,  mcause    /* Handler arg0: Exception cause */
  mv   a1,  sp        /* Handler arg1: Context (saved registers on stack) */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  lui  sp,  %hi(g_intstacktop)
  addi sp,  sp, %lo(g_intstacktop)
#endif

  /* Call interrupt/exception handler in C */

  jal  x1, esp32c3_dispatch_irq

  /* If context switch is needed, return a new SP */

  mv   sp,  a0

  /* Restore MEPC (Machine Exception Program Counter) */

  lw   s0,  0(sp)
  csrw mepc, s0

  /* Restore MSTATUS (Machine Status Register) */

  lw   s0,  32*4(sp)
  csrw mstatus, s0

  lw   x1,  1*4(sp)   /* ra */

  /* GP must not be changed after start-up due to relaxing optimization */

  lw   x4,  4*4(sp)   /* tp */
  lw   x5,  5*4(sp)   /* t0 */
  lw   x6,  6*4(sp)   /* t1 */
  lw   x7,  7*4(sp)   /* t2 */
  lw   x8,  8*4(sp)   /* s0 */
  lw   x9,  9*4(sp)   /* s1 */
  lw   x10, 10*4(sp)  /* a0 */
  lw   x11, 11*4(sp)  /* a1 */
  lw   x12, 12*4(sp)  /* a2 */
  lw   x13, 13*4(sp)  /* a3 */
  lw   x14, 14*4(sp)  /* a4 */
  lw   x15, 15*4(sp)  /* a5 */
  lw   x16, 16*4(sp)  /* a6 */
  lw   x17, 17*4(sp)  /* a7 */
  lw   x18, 18*4(sp)  /* s2 */
  lw   x19, 19*4(sp)  /* s3 */
  lw   x20, 20*4(sp)  /* s4 */
  lw   x21, 21*4(sp)  /* s5 */
  lw   x22, 22*4(sp)  /* s6 */
  lw   x23, 23*4(sp)  /* s7 */
  lw   x24, 24*4(sp)  /* s8 */
  lw   x25, 25*4(sp)  /* s9 */
  lw   x26, 26*4(sp)  /* s10 */
  lw   x27, 27*4(sp)  /* s11 */
  lw   x28, 28*4(sp)  /* t3 */
  lw   x29, 29*4(sp)  /* t4 */
  lw   x30, 30*4(sp)  /* t5 */
  lw   x31, 31*4(sp)  /* t6 */

  lw   sp,  2*4(sp)   /* Restore original SP */

  /* Return from Machine Interrupt */

  mret

