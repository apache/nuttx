/****************************************************************************
 * arch/xtensa/include/irq.h
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

#ifndef __ARCH_XTENSA_INCLUDE_IRQ_H
#define __ARCH_XTENSA_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include NuttX-specific IRQ definitions */

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <arch/types.h>
#include <arch/chip/tie.h>
#include <arch/chip/core-isa.h>

#include <arch/xtensa/xtensa_specregs.h>
#include <arch/xtensa/xtensa_corebits.h>
#include <arch/xtensa/xtensa_coproc.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include architecture-specific IRQ definitions */

#ifdef CONFIG_ARCH_FAMILY_LX6
#  include <arch/lx6/irq.h>

#elif CONFIG_ARCH_FAMILY_LX7
#  include <arch/lx7/irq.h>

#else
#  error Unknown XTENSA architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format.  Each value is a uint32_t register index */

#define REG_PC              (0)  /* Return PC */
#define REG_PS              (1)  /* Return PS */
#define REG_A0              (2)
#define REG_A1              (3)  /* Stack pointer before interrupt */
#define REG_A2              (4)
#define REG_A3              (5)
#define REG_A4              (6)
#define REG_A5              (7)
#define REG_A6              (8)
#define REG_A7              (9)
#define REG_A8              (10)
#define REG_A9              (11)
#define REG_A10             (12)
#define REG_A11             (13)
#define REG_A12             (14)
#define REG_A13             (15)
#define REG_A14             (16)
#define REG_A15             (17)
#define REG_SAR             (18)
#define REG_EXCCAUSE        (19)
#define REG_EXCVADDR        (20)

#define _REG_EXTRA_START    (21)

#if XCHAL_HAVE_S32C1I != 0
#  define REG_SCOMPARE1       (_REG_EXTRA_START + 0)
#  define _REG_LOOPS_START    (_REG_EXTRA_START + 1)
#else
#  define _REG_LOOPS_START    _REG_EXTRA_START
#endif

#if XCHAL_HAVE_LOOPS != 0
#  define REG_LBEG          (_REG_LOOPS_START + 0)
#  define REG_LEND          (_REG_LOOPS_START + 1)
#  define REG_LCOUNT        (_REG_LOOPS_START + 2)
#  define _REG_WINDOW_TMPS  (_REG_LOOPS_START + 3)
#else
#  define _REG_WINDOW_TMPS  _REG_LOOPS_START
#endif

#ifndef __XTENSA_CALL0_ABI__
  /* Temporary space for saving stuff during window spill.
   * REVISIT: I don't think that we need so many temporaries.
   */

#  define REG_TMP0          (_REG_WINDOW_TMPS + 0)
#  define REG_TMP1          (_REG_WINDOW_TMPS + 1)
#  define _REG_OVLY_START   (_REG_WINDOW_TMPS + 2)
#else
#  define _REG_OVLY_START   _REG_WINDOW_TMPS
#endif

#ifdef CONFIG_XTENSA_USE_OVLY
/* Storage for overlay state */

#  error Overlays not supported
#  define XCPTCONTEXT_REGS  _REG_OVLY_START
#else
#  define XCPTCONTEXT_REGS  _REG_OVLY_START
#endif

#define XCPTCONTEXT_SIZE    ((4 * XCPTCONTEXT_REGS) + 0x20)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This struct defines the way the registers are stored. */

struct xcptcontext
{
  /* The following function pointer is non-zero if there are pending signals
   * to be processed.
   */

  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of registers used during signal processing.
   *
   * REVISIT:  Because there is only one copy of these save areas,
   * only a single signal handler can be active.  This precludes
   * queuing of signal actions.  As a result, signals received while
   * another signal handler is executing will be ignored!
   */

  uint32_t saved_pc;
  uint32_t saved_ps;

  /* Register save area */

  uint32_t regs[XCPTCONTEXT_REGS];

#if XCHAL_CP_NUM > 0
  /* Co-processor save area */

  struct xtensa_cpstate_s cpstate;
#endif

#ifdef CONFIG_LIB_SYSCALL
  /* The following array holds the return address and the exc_return value
   * needed to return from each nested system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];
#endif
};

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Return the current value of the PS register */

static inline uint32_t xtensa_getps(void)
{
  uint32_t ps;

  __asm__ __volatile__
  (
    "rsr %0, PS"  : "=r"(ps)
  );

  return ps;
}

/* Set the value of the PS register */

static inline void xtensa_setps(uint32_t ps)
{
  __asm__ __volatile__
  (
    "wsr %0, PS \n"
    "rsync \n"
    :
    : "r"(ps)
    : "memory"
  );
}

/* Restore the value of the PS register */

static inline void up_irq_restore(uint32_t ps)
{
  __asm__ __volatile__
  (
    "wsr %0, PS \n"
    "rsync \n"
    :
    : "r"(ps)
    : "memory"
  );
}

/* Disable interrupts and return the previous value of the PS register */

static inline uint32_t up_irq_save(void)
{
  uint32_t ps;

  /* Disable all low- and medium-priority interrupts.  High priority
   * interrupts should not interfere with ongoing RTOS operations and
   * are not disabled.
   */

  __asm__ __volatile__
  (
    "rsil %0, %1" : "=r"(ps) : "i"(XCHAL_EXCM_LEVEL)
  );

  /* Return the previous PS value so that it can be restored with
   * up_irq_restore().
   */

  return ps;
}

/* Enable interrupts at all levels */

static inline void up_irq_enable(void)
{
#ifdef __XTENSA_CALL0_ABI__
  xtensa_setps(PS_INTLEVEL(0) | PS_UM);
#else
  xtensa_setps(PS_INTLEVEL(0) | PS_UM | PS_WOE);
#endif
}

/* Disable low- and medium- priority interrupts */

static inline void up_irq_disable(void)
{
#ifdef __XTENSA_CALL0_ABI__
  xtensa_setps(PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM);
#else
  xtensa_setps(PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE);
#endif
}

/****************************************************************************
 * Name: xtensa_disable_all
 ****************************************************************************/

static inline void xtensa_disable_all(void)
{
  __asm__ __volatile__
  (
    "movi a2, 0\n"
    "xsr a2, INTENABLE\n"
    : : : "a2"
  );
}

/****************************************************************************
 * Name: xtensa_intclear
 ****************************************************************************/

static inline void xtensa_intclear(uint32_t mask)
{
  __asm__ __volatile__
  (
    "wsr %0, INTCLEAR\n"
    :
    : "r"(mask)
    :
  );
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_enable_interrupts
 *
 * Description:
 *   Enables a set of interrupts. Does not simply set INTENABLE directly,
 *   but computes it as a function of the current virtual priority.
 *   Can be called from interrupt handlers.
 *
 ****************************************************************************/

irqstate_t xtensa_enable_interrupts(irqstate_t mask);

/****************************************************************************
 * Name: xtensa_disable_interrupts
 *
 * Description:
 *   Disables a set of interrupts. Does not simply clear INTENABLE directly,
 *   but computes it as a function of the current virtual priority.
 *   Can be called from interrupt handlers.
 *
 ****************************************************************************/

irqstate_t xtensa_disable_interrupts(irqstate_t mask);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_INCLUDE_IRQ_H */
