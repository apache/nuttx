/****************************************************************************
 * arch/arm64/include/irq.h
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
#  include <arch/syscall.h>
#  include <nuttx/macro.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef __ghs__
#  define __ARM_ARCH 8
#endif

#define up_getsp()          (uintptr_t)__builtin_frame_address(0)

/* MPIDR_EL1, Multiprocessor Affinity Register */

#define MPIDR_AFFLVL_MASK   (0xff)

#define MPIDR_AFF0_SHIFT    (0)
#define MPIDR_AFF1_SHIFT    (8)
#define MPIDR_AFF2_SHIFT    (16)
#define MPIDR_AFF3_SHIFT    (32)

/* mpidr_el1 register, the register is define:
 *   - bit 0~7:   Aff0
 *   - bit 8~15:  Aff1
 *   - bit 16~23: Aff2
 *   - bit 24:    MT, multithreading
 *   - bit 25~29: RES0
 *   - bit 30:    U, multiprocessor/Uniprocessor
 *   - bit 31:    RES1
 *   - bit 32~39: Aff3
 *   - bit 40~63: RES0
 *   Different ARM64 Core will use different Affn define, the mpidr_el1
 *  value is not CPU number, So we need to change CPU number to mpid
 *  and vice versa
 */

#define GET_MPIDR()                              \
  ({                                             \
    uint64_t __val;                              \
    __asm__ volatile ("mrs %0, mpidr_el1"        \
                    : "=r" (__val) :: "memory"); \
    __val;                                       \
  })

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
#define REG_SCTLR_EL1       (36)

/* In Armv8-A Architecture, the stack must align with 16 byte */

#define ARM64_CONTEXT_REGS  (37)
#define ARM64_CONTEXT_SIZE  (8 * ARM64_CONTEXT_REGS)

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

#define REG_Q0              (0)
#define REG_Q1              (1)
#define REG_Q2              (2)
#define REG_Q3              (3)
#define REG_Q4              (4)
#define REG_Q5              (5)
#define REG_Q6              (6)
#define REG_Q7              (7)
#define REG_Q8              (8)
#define REG_Q9              (9)
#define REG_Q10             (10)
#define REG_Q11             (11)
#define REG_Q12             (12)
#define REG_Q13             (13)
#define REG_Q14             (14)
#define REG_Q15             (15)
#define REG_Q16             (16)
#define REG_Q17             (17)
#define REG_Q18             (18)
#define REG_Q19             (19)
#define REG_Q20             (20)
#define REG_Q21             (21)
#define REG_Q22             (22)
#define REG_Q23             (23)
#define REG_Q24             (24)
#define REG_Q25             (25)
#define REG_Q26             (26)
#define REG_Q27             (27)
#define REG_Q28             (28)
#define REG_Q29             (29)
#define REG_Q30             (30)
#define REG_Q31             (31)

/* 32 bit registers
 */
#define REG_FPSR            (0)
#define REG_FPCR            (1)

/* FPU registers(Q0~Q31, 128bit): 32x2 = 64
 * FPU FPSR/SPSR(32 bit) : 1
 * FPU TRAP: 1
 * 64 + 1 + 1 = 66
 */
#define FPU_CONTEXT_REGS    (66)
#else
#define FPU_CONTEXT_REGS    (0)
#endif

#define FPU_CONTEXT_SIZE    (8 * FPU_CONTEXT_REGS)

#define XCPTCONTEXT_REGS    (ARM64_CONTEXT_REGS + FPU_CONTEXT_REGS)
#define XCPTCONTEXT_SIZE    (8 * XCPTCONTEXT_REGS)

/* Friendly register names */

#define REG_FP              REG_X29
#define REG_LR              REG_X30

#ifdef CONFIG_ARM64_DECODEFIQ
#  define IRQ_DAIF_MASK (3)
#else
#  define IRQ_DAIF_MASK (2)
#endif

#define IRQ_SPSR_MASK (IRQ_DAIF_MASK << 6)

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

struct xcptcontext
{
#ifdef CONFIG_BUILD_KERNEL
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uintptr_t sigreturn;

#endif
  /* task stack reg context */

  uint64_t *regs;
#ifndef CONFIG_BUILD_FLAT
  uint64_t *initregs;
#endif

  /* task context, for signal process */

  uint64_t *saved_regs;

#ifdef CONFIG_ARCH_FPU
  uint64_t *fpu_regs;
  uint64_t *saved_fpu_regs;
#endif

  /* Extra fault address register saved for common paging logic.  In the
   * case of the pre-fetch abort, this value is the same as regs[REG_ELR];
   * For the case of the data abort, this value is the value of the fault
   * address register (FAR) at the time of data abort exception.
   */

#ifdef CONFIG_LEGACY_PAGING
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

static inline_function irqstate_t irqstate(void)
{
  irqstate_t flags;

  __asm__ __volatile__("mrs %0, daif" : "=r" (flags):: "memory");

  return flags;
}

/* Disable IRQs and return the previous IRQ state */

static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  __asm__ __volatile__
    (
      "mrs %0, daif\n"
      "msr daifset, %1\n"
      : "=r" (flags)
      : "i" (IRQ_DAIF_MASK)
      : "memory"
    );

  return flags;
}

/* Enable IRQs and return the previous IRQ state */

static inline_function irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  __asm__ __volatile__
    (
      "mrs %0, daif\n"
      "msr daifclr, %1\n"
      : "=r" (flags)
      : "i" (IRQ_DAIF_MASK)
      : "memory"
    );
  return flags;
}

/* Restore saved IRQ & FIQ state */

static inline_function void up_irq_restore(irqstate_t flags)
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
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_MULTICPU
#  ifndef MPID_TO_CORE
#    define MPID_TO_CORE(mpid) \
            (((mpid) >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK)
#  endif
#  define up_cpu_index() ((int)MPID_TO_CORE(GET_MPIDR()))
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

/****************************************************************************
 * Name:
 *   read_/write_/zero_/modify_ sysreg
 *
 * Description:
 *
 *   ARMv8 Architecture Registers access method
 *   All the macros need a memory clobber
 *
 ****************************************************************************/

#define read_sysreg(reg)                            \
  ({                                                \
    uint64_t __val;                                 \
    __asm__ volatile ("mrs %0, " STRINGIFY(reg)     \
                    : "=r" (__val) :: "memory");    \
    __val;                                          \
  })

#define write_sysreg(__val, reg)                    \
  ({                                                \
    __asm__ volatile ("msr " STRINGIFY(reg) ", %0"  \
                      : : "r" (__val) : "memory");  \
  })

#define zero_sysreg(reg)                            \
  ({                                                \
    __asm__ volatile ("msr " STRINGIFY(reg) ", xzr" \
                      ::: "memory");                \
  })

#define modify_sysreg(v,m,a)                        \
  write_sysreg((read_sysreg(a) & ~(m)) |            \
               ((uintptr_t)(v) & (m)), a)

/****************************************************************************
 * Schedule acceleration macros
 *
 * The lsbit of tpidr_el1 stores information about whether the current
 * execution is in an interrupt context, where 1 indicates being in an
 * interrupt context and 0 indicates being in a thread context.
 ****************************************************************************/

#define up_this_task()         ((struct tcb_s *)(read_sysreg(tpidr_el1) & ~1ul))
#define up_update_task(t)      modify_sysreg(t, ~1ul, tpidr_el1)
#define up_interrupt_context() (read_sysreg(tpidr_el1) & 1)

#define up_switch_context(tcb, rtcb)                                      \
  do                                                                      \
    {                                                                     \
      if (!up_interrupt_context())                                        \
        {                                                                 \
          sys_call0(SYS_switch_context);                                  \
        }                                                                 \
      UNUSED(rtcb);                                                       \
    }                                                                     \
  while (0)

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uintptr_t *)((regs) ? (regs) : running_regs()))[REG_ELR])

#ifndef CONFIG_BUILD_KERNEL
#  define up_getusrsp(regs) \
    ((uintptr_t)((uint64_t *)(regs))[REG_SP_ELX])
#else
#  define up_getusrsp(regs) \
    ((uintptr_t)((uint64_t *)(regs))[REG_SP_EL0])
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_IRQ_H */
