/****************************************************************************
 * arch/arm64/include/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM64_INCLUDE_IRQ_H
#define __ARCH_ARM64_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define up_getsp()              (uintptr_t)__builtin_frame_address(0)

/****************************************************************************
 * Exception stack frame format:
 *
 * x0 ~ x18, x30 (lr), spsr and elr
 *    Corruptible Registers and exception context
 *    reference to Armv8-A Instruction Set Architecture
 *    (ARM062-948681440-3280, Issue 1.1), chapter 11 PCS
 *    need to be saved in all exception
 *
 * x19 ~ x29, sp_el0, sp_elx
 *    Callee-saved Registers and SP pointer
 *    reference to Armv8-A Instruction Set Architecture
 *    (ARM062-948681440-3280, Issue 1.1), chapter 11 PCS
 *    These registers frame is allocated on stack frame
 *    when a exception is occurred and saved at task switch
 *    or crash exception
 *    check arm64_vectors.S for detail
 *
 ****************************************************************************/

/****************************************************************************
 * Registers and exception context
 * Note:
 * REG_EXEC_DEPTH indicate the task's exception depth
 *
 ****************************************************************************/

#define REG_X0              (0)
#define REG_X1              (1)
#define REG_X2              (2)
#define REG_X3              (3)
#define REG_X4              (4)
#define REG_X5              (5)
#define REG_X6              (6)
#define REG_X7              (7)
#define REG_X8              (8)
#define REG_X9              (9)
#define REG_X10             (10)
#define REG_X11             (11)
#define REG_X12             (12)
#define REG_X13             (13)
#define REG_X14             (14)
#define REG_X15             (15)
#define REG_X16             (16)
#define REG_X17             (17)
#define REG_X18             (18)
#define REG_X19             (19)
#define REG_X20             (20)
#define REG_X21             (21)
#define REG_X22             (22)
#define REG_X23             (23)
#define REG_X24             (24)
#define REG_X25             (25)
#define REG_X26             (26)
#define REG_X27             (27)
#define REG_X28             (28)
#define REG_X29             (29)
#define REG_X30             (30)
#define REG_SP_ELX          (31)
#define REG_ELR             (32)
#define REG_SPSR            (33)
#define REG_SP_EL0          (34)
#define REG_EXE_DEPTH       (35)
#define REG_TPIDR_EL0       (36)
#define REG_TPIDR_EL1       (37)

/* In Armv8-A Architecture, the stack must align with 16 byte */

#define XCPTCONTEXT_GP_REGS (38)
#define XCPTCONTEXT_GP_SIZE (8 * XCPTCONTEXT_GP_REGS)

#ifdef CONFIG_ARCH_FPU

/****************************************************************************
 * q0 ~ q31(128bit), fpsr, fpcr
 *    armv8 fpu registers and context
 *    With CONFIG_ARCH_FPU is enabled, armv8 fpu registers context
 *    is allocated on stack frame at exception and store/restore
 *    when switching FPU context
 *    check arm64_fpu.c for detail
 *
 ****************************************************************************/

/* 128bit registers */

#define FPU_REG_Q0          (0)
#define FPU_REG_Q1          (1)
#define FPU_REG_Q2          (2)
#define FPU_REG_Q3          (3)
#define FPU_REG_Q4          (4)
#define FPU_REG_Q5          (5)
#define FPU_REG_Q6          (6)
#define FPU_REG_Q7          (7)
#define FPU_REG_Q8          (8)
#define FPU_REG_Q9          (9)
#define FPU_REG_Q10         (10)
#define FPU_REG_Q11         (11)
#define FPU_REG_Q12         (12)
#define FPU_REG_Q13         (13)
#define FPU_REG_Q14         (14)
#define FPU_REG_Q15         (15)
#define FPU_REG_Q16         (16)
#define FPU_REG_Q17         (17)
#define FPU_REG_Q18         (18)
#define FPU_REG_Q19         (19)
#define FPU_REG_Q20         (20)
#define FPU_REG_Q21         (21)
#define FPU_REG_Q22         (22)
#define FPU_REG_Q23         (23)
#define FPU_REG_Q24         (24)
#define FPU_REG_Q25         (25)
#define FPU_REG_Q26         (26)
#define FPU_REG_Q27         (27)
#define FPU_REG_Q28         (28)
#define FPU_REG_Q29         (29)
#define FPU_REG_Q30         (30)
#define FPU_REG_Q31         (31)

/* 32 bit registers
 */
#define FPU_REG_FPSR        (0)
#define FPU_REG_FPCR        (1)

/* FPU registers(Q0~Q31, 128bit): 32x2 = 64
 * FPU FPSR/SPSR(32 bit) : 1
 * FPU TRAP: 1
 * 64 + 1 + 1 = 66
 */
#define XCPTCONTEXT_FPU_REGS      (66)
#else
#define XCPTCONTEXT_FPU_REGS      (0)
#endif

#define FPUCONTEXT_SIZE     (8 * XCPTCONTEXT_FPU_REGS)

#define XCPTCONTEXT_REGS    (XCPTCONTEXT_GP_REGS + XCPTCONTEXT_FPU_REGS)
#define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uint64_t *g_current_regs[CONFIG_SMP_NCPUS];
#define CURRENT_REGS (g_current_regs[up_cpu_index()])

struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

#ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uintptr_t sigreturn;

#endif
  /* task stack reg context */

  uint64_t *regs;

  /* task context, for signal process */

  uint64_t *saved_reg;

#ifdef CONFIG_ARCH_FPU
  uint64_t *fpu_regs;
  uint64_t *saved_fpu_regs;
#endif

  /* Extra fault address register saved for common paging logic.  In the
   * case of the pre-fetch abort, this value is the same as regs[REG_ELR];
   * For the case of the data abort, this value is the value of the fault
   * address register (FAR) at the time of data abort exception.
   */

#ifdef CONFIG_PAGING
  uintptr_t far;
#endif

#ifdef CONFIG_ARCH_ADDRENV
#  ifdef CONFIG_ARCH_STACK_DYNAMIC
  /* This array holds the physical address of the level 2 page table used
   * to map the thread's stack memory.  This array will be initially of
   * zeroed and would be back-up up with pages during page fault exception
   * handling to support dynamically sized stacks for each thread.
   */

  uintptr_t *ustack[ARCH_STACK_NSECTS];
#  endif

#  ifdef CONFIG_ARCH_KERNEL_STACK
  /* In this configuration, all syscalls execute from an internal kernel
   * stack.  Why?  Because when we instantiate and initialize the address
   * environment of the new user process, we will temporarily lose the
   * address environment of the old user process, including its stack
   * contents.  The kernel C logic will crash immediately with no valid
   * stack in place.
   */

  uintptr_t *ustkptr;  /* Saved user stack pointer */
  uintptr_t *kstack;   /* Allocate base of the (aligned) kernel stack */
  uintptr_t *kstkptr;  /* Saved kernel stack pointer */
#  endif
#endif
};

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Return the current IRQ state */

static inline irqstate_t irqstate(void)
{
  irqstate_t flags;

  __asm__ __volatile__("mrs %0, daif" : "=r" (flags):: "memory");

  return flags;
}

/* Disable IRQs and return the previous IRQ state */

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  __asm__ __volatile__
    (
      "mrs %0, daif\n"
      "msr daifset, #2\n"
      : "=r" (flags)
      :
      : "memory"
    );

  return flags;
}

/* Enable IRQs and return the previous IRQ state */

static inline irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  __asm__ __volatile__
    (
      "mrs %0, daif\n"
      "msr daifclr, #2\n"
      : "=r" (flags)
      :
      : "memory"
    );
  return flags;
}

/* Restore saved IRQ & FIQ state */

static inline void up_irq_restore(irqstate_t flags)
{
  __asm__ __volatile__("msr daif, %0" :: "r" (flags): "memory");
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_index(void);
#else
#  define up_cpu_index() (0)
#endif

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description: Return true is we are currently executing in
 * the interrupt handler context.
 *
 ****************************************************************************/

static inline bool up_interrupt_context(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
#endif

  bool ret = (CURRENT_REGS != NULL);

#ifdef CONFIG_SMP
  up_irq_restore(flags);
#endif

  return ret;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_IRQ_H */
