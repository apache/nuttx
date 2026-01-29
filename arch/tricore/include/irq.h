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

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>
#include <arch/arch.h>
#include <IfxCpu_Intrinsics.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Address <--> Context Save Areas */

#define tricore_csa2addr(csa) ((uintptr_t *)((((csa) & 0x000F0000) << 12) \
                                             | (((csa) & 0x0000FFFF) << 6)))
#define tricore_addr2csa(addr) ((uintptr_t)(((((uintptr_t)(addr)) & 0xF0000000) >> 12) \
                                            | (((uintptr_t)(addr) & 0x003FFFC0) >> 6)))

/* Upper CSA */

#define REG_UPCXI        0
#define REG_PSW          1
#define REG_A10          2
#define REG_UA11         3
#define REG_D8           4
#define REG_D9           5
#define REG_D10          6
#define REG_D11          7
#define REG_A12          8
#define REG_A13          9
#define REG_A14          10
#define REG_A15          11
#define REG_D12          12
#define REG_D13          13
#define REG_D14          14
#define REG_D15          15

/* Lower CSA */

#define REG_LPCXI        0
#define REG_LA11         1
#define REG_A2           2
#define REG_A3           3
#define REG_D0           4
#define REG_D1           5
#define REG_D2           6
#define REG_D3           7
#define REG_A4           8
#define REG_A5           9
#define REG_A6           10
#define REG_A7           11
#define REG_D4           12
#define REG_D5           13
#define REG_D6           14
#define REG_D7           15

#define REG_RA           REG_UA11
#define REG_SP           REG_A10
#define REG_UPC          REG_UA11

#define REG_LPC          REG_LA11

#define TC_CONTEXT_REGS  (16)
#define TC_CONTEXT_SIZE  (sizeof(void *) * TC_CONTEXT_REGS)

#define XCPTCONTEXT_REGS (TC_CONTEXT_REGS * 2)
#define XCPTCONTEXT_SIZE (sizeof(void *) * XCPTCONTEXT_REGS)

#define NR_IRQS          (2048)

/* PSW: Program Status Word Register */

#define PSW_CDE         (1 << 7) /* Bits 7: Call Depth Count Enable */
#define PSW_IS          (1 << 9) /* Bits 9: Interrupt Stack Control */
#define PSW_IO          (10)     /* Bits 10-11: Access Privilege Level Control (I/O Privilege) */
#  define PSW_IO_USER0      (0 << PSW_IO)
#  define PSW_IO_USER1      (1 << PSW_IO)
#  define PSW_IO_SUPERVISOR (2 << PSW_IO)

/* PCXI: Previous Context Information and Pointer Register */

#define PCXI_UL         (1 << 20) /* Bits 20: Upper or Lower Context Tag */
#define PCXI_PIE        (1 << 21) /* Bits 21: Previous Interrupt Enable */

/* FCX: Free CSA List Head Pointer Register */

#define FCX_FCXO        (0)       /* Bits 0-15: FCX Offset Address */
#define FCX_FCXS        (16)      /* Bits 16-19: FCX Segment Address */
#define FCX_FCXO_MASK   (0xffff << FCX_FCXO)
#define FCX_FCXS_MASK   (0xf    << FCX_FCXS)
#define FCX_FREE        (FCX_FCXS_MASK | FCX_FCXO_MASK) /* Free CSA manipulation */

#define TRICORE_SRCNUM_PER_GPSR  8
#define TRICORE_SRC2IRQ(src_addr) \
  (((uintptr_t)(src_addr) - (uintptr_t)&MODULE_SRC) / 4)
#define TRICORE_GPSR_IRQNUM(src_cpu, dest_cpu)  \
  TRICORE_SRC2IRQ(&SRC_GPSR00 + src_cpu * 8 + dest_cpu)

/* For use with EABI and floating point, the stack must be aligned to 8-byte
 * addresses.
 */

#define STACKFRAME_ALIGN 8

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
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
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_interrupt_context store irq status */

EXTERN volatile bool g_interrupt_context[CONFIG_SMP_NCPUS];

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
int up_cpu_index(void) noinstrument_function;
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
 *   Disable interrupts and return the previous value of the mstatus register
 *
 ****************************************************************************/

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  return __disable_and_save();
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the value of the mstatus register
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  __restore(flags);
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_set_interrupt_context
 *
 * Description:
 *   Set the interrupt handler context.
 *
 ****************************************************************************/

noinstrument_function
static inline_function void up_set_interrupt_context(bool flag)
{
#ifdef CONFIG_SMP
  g_interrupt_context[up_this_cpu()] = flag;
#else
  g_interrupt_context[0] = flag;
#endif
}

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
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
  bool ret = g_interrupt_context[up_this_cpu()];

  up_irq_restore(flags);

  return ret;
#else
  return g_interrupt_context[0];
#endif
}

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

static inline_function uintptr_t up_getusrsp(void *regs)
{
  uintptr_t *csaregs = (uintptr_t *)regs;

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
      UNUSED(rtcb);                                               \
  } while (0)

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uint32_t *)((regs) ? (regs) : running_regs()))[REG_UPC])

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_IRQ_H */
