/****************************************************************************
 * arch/xtensa/include/irq.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This file should never be included directed but, rather, only indirectly
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

/* Include architecture-specific IRQ definitions */

#ifdef CONFIG_ARCH_FAMILY_LX6
#  include <arch/lx6/irq.h>

/* Include implementation-specific IRQ definitions (including IRQ numbers) */

#  ifdef CONFIG_ARCH_CHIP_ESP32
#    include <arch/esp32/irq.h>
#  else
#    error Unknown LX6 implementation
#  endif

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

#define _REG_LOOPS_START    (21)

#ifdef XCHAL_HAVE_LOOPS
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

#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

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

#ifndef CONFIG_DISABLE_SIGNALS
  void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of registers used during signal processing. */

  uint32_t saved_pc;
  uint32_t saved_ps;
#endif

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
    "wsr %0, PS"  : : "r"(ps)
  );
}

/* Restore the value of the PS register */

static inline void up_irq_restore(uint32_t ps)
{
  __asm__ __volatile__
  (
    "wsr %0, PS"  : : "r"(ps)
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
    "rsil %0, %1" : "=r"(ps) : "I"(XCHAL_EXCM_LEVEL)
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
 *   Disables a set of interrupts. Does not simply set INTENABLE directly,
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
