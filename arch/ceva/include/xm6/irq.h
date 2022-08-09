/****************************************************************************
 * arch/ceva/include/xm6/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CEVA_INCLUDE_XM6_IRQ_H
#define __ARCH_CEVA_INCLUDE_XM6_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Included implementation-dependent register save structure layouts */

#include <arch/xm6/reg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If this is kernel build, how many nested system calls should we support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* Alternate register names *************************************************/

#define REG_A0              REG_R0
#define REG_A1              REG_R1
#define REG_A2              REG_R2
#define REG_A3              REG_R3
#define REG_A4              REG_R4
#define REG_A5              REG_R5
#define REG_A6              REG_R6
#define REG_FP              REG_R8
#define REG_LR              REG_RETREG
#define REG_PC              REG_RETREGI
#define REG_OM              REG_MODC  /* Operation Mode */

/* MODA: IRQ enable/disable */

#define REG_MODA_DEFAULT    0x07f0

#define REG_MODA_ENABLE     0x07f0
#define REG_MODA_DISABLE    0x0010

/* MODC: Operation mode */

#define REG_OM_DEFAULT      0x20      /* PI and Supervisor */

/* Note: this is POM field not OM field */

#define REG_OM_KERNEL       0x00      /* Supervisor Mode */
#define REG_OM_USER         0x08      /* User0 Mode */
#define REG_OM_MASK         0x18      /* Mode Mask */

/* First Level Interrupt (vectors 0-15) */

#define IRQ_RESET           0x00 /* Vector  0: Reset(not handler as an IRQ) */
#define IRQ_BOOT            0x01 /* Vector  1: Boot(not handler as an IRQ) */
#define IRQ_TRAP            0x02 /* Vector  2: Software Interrupt */
#define IRQ_TRAPE           0x03 /* Vector  3: Emulation Software Interrupt */
#define IRQ_BI              0x03 /* Vector  3: Breakpoint Interrupt */
#define IRQ_NMI             0x04 /* Vector  4: Non-Maskable Interrupt */
#define IRQ_INT0            0x05 /* Vector  5: Maskable Interrupt 0 */
#define IRQ_INT1            0x06 /* Vector  6: Maskable Interrupt 1 */
#define IRQ_INT2            0x07 /* Vector  7: Maskable Interrupt 2 */
#define IRQ_INT3            0x08 /* Vector  8: Maskable Interrupt 3 */
#define IRQ_INT4            0x09 /* Vector  9: Maskable Interrupt 4 */
#define IRQ_VINT            0x0a /* Vector 10: Vectored Interrupt */
#define IRQ_TRAP0           0x0b /* Vector 11: Software Interrupt 0 */
#define IRQ_TRAP1           0x0c /* Vector 12: Software Interrupt 1 */
#define IRQ_TRAP2           0x0d /* Vector 13: Software Interrupt 2 */
#define IRQ_TRAP3           0x0e /* Vector 14: Software Interrupt 3 */
#define IRQ_PABP            0x03 /* Vector  3: Program Address Breakpoint */

/* Second Level interrupts (vectors >= 16).
 * These definitions are chip-specific.
 */

#define IRQ_VINT_FIRST      16 /* Vector number of the first VINT interrupt */

/****************************************************************************
 * Public Types
 ****************************************************************************/
#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_LIB_SYSCALL
struct xcpt_syscall_s
{
  uint32_t saved_pc;
  uint32_t saved_om;
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
#ifndef CONFIG_DISABLE_SIGNALS
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of the context used during
   * signal processing.
   */

  uint32_t *saved_regs;

# ifdef CONFIG_BUILD_PROTECTED
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;

# endif
#endif

#ifdef CONFIG_LIB_SYSCALL
  /* The following array holds the return address and operation mode
   * needed to return from each nested system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];

#endif

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uint32_t *regs;
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Get/set the MODA register, here is the irq related bits:
 *   Bit  [4] Interrupt enable            (RW)
 *   Bit  [5] Interrupt mask for INT0     (RW)
 *   Bit  [6] Interrupt mask for INT1     (RW)
 *   Bit  [7] Interrupt mask for INT2     (RW)
 *   Bit  [8] Interrupt mask for INT3     (RW)
 *   Bit  [9] Interrupt mask for INT4     (RW)
 *   Bit [10] Interrupt mask for VINT     (RW)
 *   Bit [11] Interrupt pending for INT0  (RO)
 *   Bit [12] Interrupt pending for INT1  (RO)
 *   Bit [13] Interrupt pending for INT2  (RO)
 *   Bit [14] Interrupt pending for INT3  (RO)
 *   Bit [15] Interrupt pending for INT4  (RO)
 *   Bit [16] Interrupt pending for INTV  (RO)
 * All writable bits are clear by hardware during reset.
 *
 * We manipulate the individual mask bits instead of global enable bit since:
 * 1.Global IE not only mask INTX request but also mask TRAPX instruction.
 * 2.Hardware always enable global IE after the interrupt return.
 * Both behavior don't match the nuttx requirement.
 */

static inline uint32_t getmoda(void)
{
  uint32_t moda;
  __asm__ __volatile__("mov moda.ui, %0.ui\nnop #0x02" : "=r"(moda));
  return moda;
}

static inline void setmoda(uint32_t moda)
{
  __asm__ __volatile__("nop #0x04\nnop\nmovp %0.ui, moda.ui" : : "r"(moda));
}

/* Return the current value of the stack pointer */

static inline uint32_t up_getsp(void)
{
  uint32_t sp;
  __asm__ __volatile__("mov sp.ui, %0.ui" : "=r"(sp));
  return sp;
}

static inline void up_irq_disable(void)
{
  setmoda(REG_MODA_DISABLE);
}

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags = getmoda();
  up_irq_disable();
  return flags;
}

static inline void up_irq_enable(void)
{
  setmoda(REG_MODA_ENABLE);
}

static inline void up_irq_restore(irqstate_t flags)
{
  setmoda(flags);
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XM6_IRQ_H */
