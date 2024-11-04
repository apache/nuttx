/****************************************************************************
 * arch/arm/include/armv6-m/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV6_M_IRQ_H
#define __ARCH_ARM_INCLUDE_ARMV6_M_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <stdint.h>
#endif

#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef __ghs__
#  define __ARM_ARCH 6
#endif

/* Configuration ************************************************************/

/* If this is a kernel build,
 * how many nested system calls should we support?
 */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* IRQ Stack Frame Format ***************************************************
 *
 * The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_R13             (0)  /* R13 = SP at time of interrupt */
#define REG_PRIMASK         (1)  /* PRIMASK */
#define REG_R4              (2)  /* R4 */
#define REG_R5              (3)  /* R5 */
#define REG_R6              (4)  /* R6 */
#define REG_R7              (5)  /* R7 */
#define REG_R8              (6)  /* R8 */
#define REG_R9              (7)  /* R9 */
#define REG_R10             (8)  /* R10 */
#define REG_R11             (9)  /* R11 */
#define REG_CONTROL         (10) /* CONTROL */
#define REG_EXC_RETURN      (11) /* EXC_RETURN */
#define SW_XCPT_REGS        (12)

/* The total number of registers saved by software */

#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_R0              (SW_XCPT_REGS+0) /* R0 */
#define REG_R1              (SW_XCPT_REGS+1) /* R1 */
#define REG_R2              (SW_XCPT_REGS+2) /* R2 */
#define REG_R3              (SW_XCPT_REGS+3) /* R3 */
#define REG_R12             (SW_XCPT_REGS+4) /* R12 */
#define REG_R14             (SW_XCPT_REGS+5) /* R14 = LR */
#define REG_R15             (SW_XCPT_REGS+6) /* R15 = PC */
#define REG_XPSR            (SW_XCPT_REGS+7) /* xPSR */

#define HW_XCPT_REGS        (8)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/* Alternate register names */

#define REG_A1              REG_R0
#define REG_A2              REG_R1
#define REG_A3              REG_R2
#define REG_A4              REG_R3
#define REG_V1              REG_R4
#define REG_V2              REG_R5
#define REG_V3              REG_R6
#define REG_V4              REG_R7
#define REG_V5              REG_R8
#define REG_V6              REG_R9
#define REG_V7              REG_R10
#define REG_SB              REG_R9
#define REG_SL              REG_R10
#define REG_FP              REG_R7
#define REG_IP              REG_R12
#define REG_SP              REG_R13
#define REG_LR              REG_R14
#define REG_PC              REG_R15

/* The PIC register is usually R10. It can be R9 is stack checking is enabled
 * or if the user changes it with -mpic-register on the GCC command line.
 */

#define REG_PIC             REG_R10

/* CONTROL register */

#define CONTROL_FPCA        (1 << 2) /* Bit 2: Floating-point context active */
#define CONTROL_SPSEL       (1 << 1) /* Bit 1: Stack-pointer select */
#define CONTROL_NPRIV       (1 << 0) /* Bit 0: Not privileged */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_LIB_SYSCALL
struct xcpt_syscall_s
{
  uint32_t excreturn;   /* The EXC_RETURN value */
  uint32_t sysreturn;   /* The return PC */
  uint32_t ctrlreturn;  /* The return CONTROL value */
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
  /* These are saved copies of the context used during
   * signal processing.
   */

  uint32_t *saved_regs;

#ifdef CONFIG_BUILD_PROTECTED
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;
#endif

#ifdef CONFIG_LIB_SYSCALL
  /* The following array holds the return address and the exc_return value
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the
 * [get/set]_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

extern volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Get/set the PRIMASK register */

static inline uint8_t getprimask(void) always_inline_function;
static inline uint8_t getprimask(void)
{
  uint32_t primask;
  __asm__ __volatile__
    (
     "\tmrs  %0, primask\n"
     : "=r" (primask)
     :
     : "memory");

  return (uint8_t)primask;
}

static inline void setprimask(uint32_t primask) always_inline_function;
static inline void setprimask(uint32_t primask)
{
  __asm__ __volatile__
    (
      "\tmsr primask, %0\n"
      :
      : "r" (primask)
      : "memory");
}

/* Disable IRQs */

static inline void up_irq_disable(void) always_inline_function;
static inline void up_irq_disable(void)
{
  __asm__ __volatile__ ("\tcpsid  i\n");
}

/* Save the current primask state & disable IRQs */

static inline irqstate_t up_irq_save(void) always_inline_function;
static inline irqstate_t up_irq_save(void)
{
  unsigned short primask;

  /* Return the current value of primask register and set
   * bit 0 of the primask register to disable interrupts
   */

  __asm__ __volatile__
    (
     "\tmrs    %0, primask\n"
     "\tcpsid  i\n"
     : "=r" (primask)
     :
     : "memory");

  return primask;
}

/* Enable IRQs */

static inline void up_irq_enable(void) always_inline_function;
static inline void up_irq_enable(void)
{
  __asm__ __volatile__ ("\tcpsie  i\n");
}

/* Restore saved primask state */

static inline void up_irq_restore(irqstate_t flags) always_inline_function;
static inline void up_irq_restore(irqstate_t flags)
{
  /* If bit 0 of the primask is 0, then we need to restore
   * interrupts.
   */

  __asm__ __volatile__
    (
      "\tmsr primask, %0\n"
      :
      : "r" (flags)
      : "memory");
}

/* Get/set IPSR */

static inline uint32_t getipsr(void) always_inline_function;
static inline uint32_t getipsr(void)
{
  uint32_t ipsr;
  __asm__ __volatile__
    (
     "\tmrs  %0, ipsr\n"
     : "=r" (ipsr)
     :
     : "memory");

  return ipsr;
}

/* Get/set CONTROL */

static inline uint32_t getcontrol(void) always_inline_function;
static inline uint32_t getcontrol(void)
{
  uint32_t control;
  __asm__ __volatile__
    (
     "\tmrs  %0, control\n"
     : "=r" (control)
     :
     : "memory");

  return control;
}

static inline void setcontrol(uint32_t control) always_inline_function;
static inline void setcontrol(uint32_t control)
{
  __asm__ __volatile__
    (
      "\tmsr control, %0\n"
      :
      : "r" (control)
      : "memory");
}

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_MULTICPU
int up_cpu_index(void) noinstrument_function;
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

static inline_function uint32_t up_getsp(void)
{
  register uint32_t sp;

  __asm__ __volatile__
  (
    "mov %0, sp\n"
    : "=r" (sp)
  );

  return sp;
}

noinstrument_function
static inline_function uint32_t *up_current_regs(void)
{
#ifdef CONFIG_SMP
  return (uint32_t *)g_current_regs[up_cpu_index()];
#else
  return (uint32_t *)g_current_regs[0];
#endif
}

noinstrument_function
static inline_function void up_set_current_regs(uint32_t *regs)
{
#ifdef CONFIG_SMP
  g_current_regs[up_cpu_index()] = regs;
#else
  g_current_regs[0] = regs;
#endif
}

noinstrument_function
static inline_function bool up_interrupt_context(void)
{
  return getipsr() != 0;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_INCLUDE_ARMV6_M_IRQ_H */
