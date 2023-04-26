/****************************************************************************
 * arch/arm/include/armv7-m/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H
#define __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <arch/armv7-m/nvicpri.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Configuration ************************************************************/

/* If this is a kernel build, how many nested system calls should we
 * support?
 */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* IRQ Stack Frame Format: */

/* The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_R13             (0)  /* R13 = SP at time of interrupt */
#ifdef CONFIG_ARMV7M_USEBASEPRI
#  define REG_BASEPRI       (1)  /* BASEPRI */
#else
#  define REG_PRIMASK       (1)  /* PRIMASK */
#endif
#define REG_R4              (2)  /* R4 */
#define REG_R5              (3)  /* R5 */
#define REG_R6              (4)  /* R6 */
#define REG_R7              (5)  /* R7 */
#define REG_R8              (6)  /* R8 */
#define REG_R9              (7)  /* R9 */
#define REG_R10             (8)  /* R10 */
#define REG_R11             (9)  /* R11 */
#define REG_EXC_RETURN      (10) /* EXC_RETURN */
#define SW_INT_REGS         (11)

#ifdef CONFIG_ARCH_FPU

/* If the MCU supports a floating point unit, then it will be necessary
 * to save the state of the non-volatile registers before calling code
 * that may save and overwrite them.
 */

#  define REG_S16           (SW_INT_REGS + 0)  /* S16 */
#  define REG_S17           (SW_INT_REGS + 1)  /* S17 */
#  define REG_S18           (SW_INT_REGS + 2)  /* S18 */
#  define REG_S19           (SW_INT_REGS + 3)  /* S19 */
#  define REG_S20           (SW_INT_REGS + 4)  /* S20 */
#  define REG_S21           (SW_INT_REGS + 5)  /* S21 */
#  define REG_S22           (SW_INT_REGS + 6)  /* S22 */
#  define REG_S23           (SW_INT_REGS + 7)  /* S23 */
#  define REG_S24           (SW_INT_REGS + 8)  /* S24 */
#  define REG_S25           (SW_INT_REGS + 9)  /* S25 */
#  define REG_S26           (SW_INT_REGS + 10) /* S26 */
#  define REG_S27           (SW_INT_REGS + 11) /* S27 */
#  define REG_S28           (SW_INT_REGS + 12) /* S28 */
#  define REG_S29           (SW_INT_REGS + 13) /* S29 */
#  define REG_S30           (SW_INT_REGS + 14) /* S30 */
#  define REG_S31           (SW_INT_REGS + 15) /* S31 */
#  define SW_FPU_REGS       (16)
#else
#  define SW_FPU_REGS       (0)
#endif

/* The total number of registers saved by software */

#define SW_XCPT_REGS        (SW_INT_REGS + SW_FPU_REGS)
#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_R0              (SW_XCPT_REGS + 0) /* R0 */
#define REG_R1              (SW_XCPT_REGS + 1) /* R1 */
#define REG_R2              (SW_XCPT_REGS + 2) /* R2 */
#define REG_R3              (SW_XCPT_REGS + 3) /* R3 */
#define REG_R12             (SW_XCPT_REGS + 4) /* R12 */
#define REG_R14             (SW_XCPT_REGS + 5) /* R14 = LR */
#define REG_R15             (SW_XCPT_REGS + 6) /* R15 = PC */
#define REG_XPSR            (SW_XCPT_REGS + 7) /* xPSR */
#define HW_INT_REGS         (8)

#ifdef CONFIG_ARCH_FPU

/* If the FPU is enabled, the hardware also saves the volatile FP registers.
 */

#  define REG_S0            (SW_XCPT_REGS + 8)  /* S0 */
#  define REG_S1            (SW_XCPT_REGS + 9)  /* S1 */
#  define REG_S2            (SW_XCPT_REGS + 10) /* S2 */
#  define REG_S3            (SW_XCPT_REGS + 11) /* S3 */
#  define REG_S4            (SW_XCPT_REGS + 12) /* S4 */
#  define REG_S5            (SW_XCPT_REGS + 13) /* S5 */
#  define REG_S6            (SW_XCPT_REGS + 14) /* S6 */
#  define REG_S7            (SW_XCPT_REGS + 15) /* S7 */
#  define REG_S8            (SW_XCPT_REGS + 16) /* S8 */
#  define REG_S9            (SW_XCPT_REGS + 17) /* S9 */
#  define REG_S10           (SW_XCPT_REGS + 18) /* S10 */
#  define REG_S11           (SW_XCPT_REGS + 19) /* S11 */
#  define REG_S12           (SW_XCPT_REGS + 20) /* S12 */
#  define REG_S13           (SW_XCPT_REGS + 21) /* S13 */
#  define REG_S14           (SW_XCPT_REGS + 22) /* S14 */
#  define REG_S15           (SW_XCPT_REGS + 23) /* S15 */
#  define REG_FPSCR         (SW_XCPT_REGS + 24) /* FPSCR */
#  define REG_FP_RESERVED   (SW_XCPT_REGS + 25) /* Reserved */
#  define HW_FPU_REGS       (18)
#else
#  define HW_FPU_REGS       (0)
#endif

#define HW_XCPT_REGS        (HW_INT_REGS + HW_FPU_REGS)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/* Alternate register names *************************************************/

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
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

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

static inline void cpsie(void) always_inline_function;
static inline void cpsie(void)
{
  __asm__ __volatile__ ("\tcpsie  i\n");
}

static inline void cpsid(void) always_inline_function;
static inline void cpsid(void)
{
  __asm__ __volatile__ ("\tcpsid  i\n");
}

/* Get/set the BASEPRI register.  The BASEPRI register defines the minimum
 * priority for exception processing. When BASEPRI is set to a nonzero
 * value, it prevents the activation of all exceptions with the same or
 * lower priority level as the BASEPRI value.
 */

static inline uint8_t getbasepri(void) always_inline_function;
static inline uint8_t getbasepri(void)
{
  uint32_t basepri;

  __asm__ __volatile__
    (
     "\tmrs  %0, basepri\n"
     : "=r" (basepri)
     :
     : "memory");

  return (uint8_t)basepri;
}

static inline void setbasepri(uint32_t basepri) always_inline_function;
static inline void setbasepri(uint32_t basepri)
{
  __asm__ __volatile__
    (
      "\tmsr basepri, %0\n"
      :
      : "r" (basepri)
      : "memory");
}

#ifdef CONFIG_ARMV7M_BASEPRI_WAR  /* Cortex-M7 r0p1 Errata 837070 Workaround */
/* Set the BASEPRI register (possibly increasing the priority).
 *
 * This may be retaining or raising priority.  Cortex-M7 r0p1 Errata
 * 837070 Workaround may be required if we are raising the priority.
 */

static inline void raisebasepri(uint32_t basepri) always_inline_function;
static inline void raisebasepri(uint32_t basepri)
{
  register uint32_t primask;

  /* 1. Retain the previous value of the PRIMASK register,
   * 2  Disable all interrupts via the PRIMASK register.  NOTE:  They
   *    could possibly already be disabled.
   * 3. Set the BASEPRI register as requested (possibly increasing the
   *    priority)
   * 4. Restore the original value of the PRIMASK register, probably re-
   *    enabling interrupts.  This avoids the possibly undesirable side-
   *    effect of unconditionally re-enabling interrupts.
   */

#pragma GCC diagnostic push /* primask is initialized in ASM */
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  __asm__ __volatile__
    (
     "\tmrs   %0, primask\n"
     "\tcpsid i\n"
     "\tmsr   basepri, %1\n"
     "\tmsr   primask, %0\n"
     : "+r" (primask)
     : "r"  (basepri)
     : "memory");
#pragma GCC diagnostic pop
}
#else
#  define raisebasepri(b) setbasepri(b);
#endif

/* Disable IRQs */

static inline void up_irq_disable(void) always_inline_function;
static inline void up_irq_disable(void)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  /* Probably raising priority */

  raisebasepri(NVIC_SYSH_DISABLE_PRIORITY);
#else
  __asm__ __volatile__ ("\tcpsid  i\n");
#endif
}

/* Save the current primask state & disable IRQs */

static inline irqstate_t up_irq_save(void) always_inline_function;
static inline irqstate_t up_irq_save(void)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  /* Probably raising priority */

  uint8_t basepri = getbasepri();
  raisebasepri(NVIC_SYSH_DISABLE_PRIORITY);
  return (irqstate_t)basepri;

#else

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
#endif
}

/* Enable IRQs */

static inline void up_irq_enable(void) always_inline_function;
static inline void up_irq_enable(void)
{
  /* In this case, we are always retaining or lowering the priority value */

  setbasepri(NVIC_SYSH_PRIORITY_MIN);
  __asm__ __volatile__ ("\tcpsie  i\n");
}

/* Restore saved primask state */

static inline void up_irq_restore(irqstate_t flags) always_inline_function;
static inline void up_irq_restore(irqstate_t flags)
{
#ifdef CONFIG_ARMV7M_USEBASEPRI
  /* In this case, we are always retaining or lowering the priority value */

  setbasepri((uint32_t)flags);

#else
  /* If bit 0 of the primask is 0, then we need to restore
   * interrupts.
   */

  __asm__ __volatile__
    (
      "\ttst    %0, #1\n"
      "\tbne.n  1f\n"
      "\tcpsie  i\n"
      "1:\n"
      :
      : "r" (flags)
      : "cc", "memory");

#endif
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

/* Get/set FAULTMASK */

static inline uint32_t getfaultmask(void) always_inline_function;
static inline uint32_t getfaultmask(void)
{
  uint32_t faultmask;
  __asm__ __volatile__
    (
     "\tmrs  %0, faultmask\n"
     : "=r" (faultmask)
     :
     : "memory");

  return faultmask;
}

static inline void setfaultmask(uint32_t faultmask) always_inline_function;
static inline void setfaultmask(uint32_t faultmask)
{
  __asm__ __volatile__
    (
      "\tmsr faultmask, %0\n"
      :
      : "r" (faultmask)
      : "memory");
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

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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
#endif

#endif /* __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_H */
