/****************************************************************************
 * arch/tricore/include/irq.h
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

#ifndef __ARCH_TRICORE_INCLUDE_IRQ_H
#define __ARCH_TRICORE_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#  include <syscall.h>
#endif

#include <arch/arch.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Upper CSA */

#define REG_UPCXI           0
#define REG_PSW             1
#define REG_A10             2
#define REG_UA11            3
#define REG_D8              4
#define REG_D9              5
#define REG_D10             6
#define REG_D11             7
#define REG_A12             8
#define REG_A13             9
#define REG_A14             10
#define REG_A15             11
#define REG_D12             12
#define REG_D13             13
#define REG_D14             14
#define REG_D15             15

/* Lower CSA */

#define REG_LPCXI           0
#define REG_LA11            1
#define REG_A2              2
#define REG_A3              3
#define REG_D0              4
#define REG_D1              5
#define REG_D2              6
#define REG_D3              7
#define REG_A4              8
#define REG_A5              9
#define REG_A6              10
#define REG_A7              11
#define REG_D4              12
#define REG_D5              13
#define REG_D6              14
#define REG_D7              15

#define REG_RA              REG_UA11
#define REG_SP              REG_A10
#define REG_UPC             REG_UA11
#define REG_LPC             REG_LA11

#define TC_CONTEXT_REGS     (16)
#define TC_CONTEXT_SIZE     (4 * TC_CONTEXT_REGS)
#define TC_CONTEXT_ALIGNED(addr) \
  (((uintptr_t)(addr) & (TC_CONTEXT_SIZE - 1)) == 0)

#define XCPTCONTEXT_REGS    (TC_CONTEXT_REGS * 2)
#define XCPTCONTEXT_SIZE    (sizeof(void *) * XCPTCONTEXT_REGS)

#define FCD_RESERVE_SIZE    (TC_CONTEXT_SIZE * 4)

/* PSW: Program Status Word Register */

#define PSW_CDE             (1 << 7) /* Bits 7: Call Depth Count Enable */
#define PSW_GW              (1 << 8) /* Global address register write permission */
#define PSW_IS              (1 << 9) /* Bits 9: Interrupt Stack Control */
#define PSW_IO              (10)     /* Bits 10-11: Access Privilege Level Control (I/O Privilege) */
#  define PSW_MODE_MASK     (3 << PSW_IO)
#  define PSW_IO_USER0      (0 << PSW_IO)
#  define PSW_IO_USER1      (1 << PSW_IO)
#  define PSW_IO_SUPERVISOR (2 << PSW_IO)
#define PSW_STATUS_BITS     (31 << 27) /* Bits 27-31: Execution Effect Status Bits */

#define PSW_PRS_KERNEL_SET  (0)
#define PSW_PRS_USER_SET    (1)

#ifdef CONFIG_ARCH_HAVE_PPRS
  #define PSW_PRS_MASK                           (0)
  #define PSW_PRS_USER                           (0)
  #define tricore_change_pprs(tcb, val)          (tcb)->xcp.pprs = (val)
  #define tricore_load_pprs(tcb)                 (tcb)->xcp.pprs
  #define tricore_restore_pprs(tcb)              tricore_mtcr(TRICORE_CPU_PPRS, (tcb)->xcp.pprs)
  #define tricore_store_pprs(tcb)                (tcb)->xcp.pprs = tricore_mfcr(TRICORE_CPU_PPRS)
  #define tricore_syscall_load_pprs(tcb ,index)  (tcb)->xcp.syscall_pprs[index]
  #define tricore_syscall_store_pprs(tcb ,index) (tcb)->xcp.syscall_pprs[index] = tricore_mfcr(TRICORE_CPU_PPRS)
  #define tricore_sig_change_pprs(tcb ,val)      (tcb)->xcp.saved_pprs = (val)
  #define tricore_sig_load_pprs(tcb)             (tcb)->xcp.saved_pprs
  #define tricore_pprs_to_prs(pprs)              (((pprs & 3) << 12) | ((pprs & 4) << 13))
  #define tricore_prs_to_pprs(prs)               (((prs >> 12) & 3) | ((prs >> 13) & 4))
#else
  #define PSW_PRS_MASK                           (3 << 12) | (1 << 15)
  #define PSW_PRS_USER                           (((PSW_PRS_USER_SET & 3) << 12) | ((PSW_PRS_USER_SET & 4) << 13))
  #define tricore_change_pprs(tcb, val)          do { } while (0)
  #define tricore_load_pprs(tcb)                 (0)
  #define tricore_restore_pprs(tcb)              do { } while (0)
  #define tricore_store_pprs(tcb)                do { } while (0)
  #define tricore_syscall_load_pprs(tcb ,index)  (0)
  #define tricore_syscall_store_pprs(tcb ,index) do { } while (0)
  #define tricore_sig_change_pprs(tcb ,val)      do { } while (0)
  #define tricore_sig_load_pprs(tcb)             (0)
  #define tricore_pprs_to_prs(pprs)              (0)
  #define tricore_prs_to_pprs(prs)               (0)
#endif

/* PCXI: Previous Context Information and Pointer Register */

#define PCXI_UL         (1 << 20) /* Bits 20: Upper or Lower Context Tag */
#define PCXI_PIE        (1 << 21) /* Bits 21: Previous Interrupt Enable */

/* FCX: Free CSA List Head Pointer Register */

#define FCX_FCXO        (0)       /* Bits 0-15: FCX Offset Address */
#define FCX_FCXS        (16)      /* Bits 16-19: FCX Segment Address */
#define FCX_FCXO_MASK   (0xffff << FCX_FCXO)
#define FCX_FCXS_MASK   (0xf    << FCX_FCXS)
#define FCX_FREE        (FCX_FCXS_MASK | FCX_FCXO_MASK) /* Free CSA manipulation */

#define TRICORE_SRCNUM_PER_GPSR 8
#define TRICORE_SRC_SPACING     4

#define TRICORE_SRC2IRQ(src) \
  (((uintptr_t)(src) - TRICORE_SRC_BASE) / TRICORE_SRC_SPACING)
#define TRICORE_IRQ2SRC(irq) \
  (TRICORE_SRC_BASE + (irq) * TRICORE_SRC_SPACING)
#define TRICORE_GPSR2IRQ(src_cpu, dest_cpu) \
  TRICORE_SRC2IRQ(TRICORE_SRC_GPSR_BASE + \
                  (src_cpu) * TRICORE_SRCNUM_PER_GPSR * TRICORE_SRC_SPACING + \
                  (dest_cpu) * TRICORE_SRC_SPACING)

#ifdef CONFIG_ARCH_IRQPRIO
#  define TRICORE_SCHEDIRQ TRICORE_GPSR2IRQ(up_this_cpu(), up_this_cpu())
#endif

/* TriCore interrupt priority range, higher value means higher priority. */

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  #define TRICORE_IRQ_DISABLE_PRIORITY CONFIG_ARCH_IRQ_DISABLE_PRIORITY
#endif

#define TRICORE_IRQ_MIN_PRIORITY 1
#define TRICORE_IRQ_MAX_PRIORITY 255

/* For use with EABI and floating point, the stack must be aligned to 8-byte
 * addresses.
 */

#define STACKFRAME_ALIGN 8

/* FPU registers */

#ifdef CONFIG_ARCH_TC1V6
#  define FPU_SYNC_TRAP_REG   0xA000
#  define FPU_ASYNC_TRAP_REG  0xA000
#else
#  define FPU_SYNC_TRAP_REG   0xA030
#  define FPU_ASYNC_TRAP_REG  0xA000
#endif
#define FPU_TRAP_FZE_SHIFT    20
#define FPU_TRAP_TCL_SHIFT    1

#define up_tls_info() ((struct tls_info_s *)read_sysreg(a8) - 1)

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct xcptcontext
{
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  /* These are saved copies of the context used during
   * signal processing.
   */

  uintptr_t *saved_regs;
#endif

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uintptr_t *regs;

#ifdef CONFIG_LIB_SYSCALL
  /* The following array holds information needed to return from each nested
   * system call.
   */

  int nsyscalls;
  uintptr_t *syscall_regs[CONFIG_SYS_NNEST];
#ifdef CONFIG_ARCH_HAVE_PPRS
  uint32_t syscall_pprs[CONFIG_SYS_NNEST];
#endif
#endif

#ifdef CONFIG_BUILD_PROTECTED
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;

#endif

#ifdef CONFIG_ARCH_HAVE_PPRS
  uint32_t pprs;
  uint32_t saved_pprs;
#endif
};

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
noinstrument_function
static inline_function int up_cpu_index(void)
{
  /* Register CORE_ID[2:0] holds cpu number. */

  uint8_t id = tricore_mfcr(TRICORE_CPU_CORE_ID) & 0x7;

#ifdef CONFIG_ARCH_CHIP_TC3X
  id = id > 5 ? 5 : id;
#endif

  return id;
}
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Return program counter */

noinstrument_function static inline_function uintptr_t up_getpc(void)
{
  return (uintptr_t)tricore_mfcr(TRICORE_CPU_PC);
}

noinstrument_function static inline_function uintptr_t up_getsp(void)
{
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  return (uintptr_t)__get_sp();
#else
  return (uintptr_t)__builtin_frame_address(0);
#endif
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous interrupt state.
 *
 *   When CONFIG_ARCH_HIPRI_INTERRUPT is enabled, this raises ICR.CCPN
 *   to TRICORE_IRQ_DISABLE_PRIORITY so that high-priority interrupts
 *   (SRPN > TRICORE_IRQ_DISABLE_PRIORITY) remain unmasked. up_irq_save
 *   only raises CCPN, up_irq_restore only lowers it. This prevents the
 *   masking level from being accidentally lowered.
 *
 *   Otherwise, uses the hardware disable instruction (ICR.IE = 0)
 *   for minimal overhead.
 *
 ****************************************************************************/

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t flags;

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  uint32_t icr = tricore_mfcr(TRICORE_CPU_ICR);

  flags = icr & TRICORE_ICR_CCPN_MASK;

  if (TRICORE_IRQ_DISABLE_PRIORITY > flags)
    {
      icr = (icr & ~TRICORE_ICR_CCPN_MASK) |
            TRICORE_IRQ_DISABLE_PRIORITY;
      tricore_mtcr(TRICORE_CPU_ICR, icr);
      UP_ISB();
    }
#else
  __asm__ __volatile__ ("disable %0" : "=d"(flags) :: "memory");
#endif

  return flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the previous interrupt state saved by up_irq_save().
 *
 *   When CONFIG_ARCH_HIPRI_INTERRUPT is enabled, restores ICR.CCPN.
 *   Otherwise, restores ICR.IE via the hardware restore instruction.
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  uint32_t icr = tricore_mfcr(TRICORE_CPU_ICR);

  if (flags < (icr & TRICORE_ICR_CCPN_MASK))
    {
      icr = (icr & ~TRICORE_ICR_CCPN_MASK) | flags;
      tricore_mtcr(TRICORE_CPU_ICR, icr);
      UP_ISB();
    }
#else
  __asm__ __volatile__ ("restore %0" :: "d"(flags) : "memory");
#endif
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt
 *   handler context.
 *
 ****************************************************************************/

noinstrument_function
static inline_function bool up_interrupt_context(void)
{
  return (tricore_mfcr(TRICORE_CPU_PSW) & PSW_IS) != 0;
}

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

static inline_function uintptr_t up_getusrsp(void *regs)
{
  uintptr_t *csaregs = (uintptr_t *)regs;

  if (csaregs == NULL)
    {
      return 0;
    }

  if (csaregs[REG_LPCXI] & PCXI_UL)
    {
      csaregs = tricore_csa2addr(csaregs[REG_LPCXI]);
    }
  else
    {
      csaregs += TC_CONTEXT_REGS;
    }

  return csaregs[REG_SP];
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Name: up_switch_context
 ****************************************************************************/

#define up_switch_context(tcb, rtcb)                              \
  do {                                                            \
    if (!up_interrupt_context())                                  \
      {                                                           \
        sys_call0(SYS_switch_context);                            \
      }                                                           \
    UNUSED(rtcb);                                                 \
  } while (0)

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uint32_t *)((regs) ? (regs) : running_regs()))[REG_UPC])

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_IRQ_H */
