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
#  include <arch/syscall.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/arch.h>
#include <arch/chip/irq.h>

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
#  include <arch/tc3x/irq.h>
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#  include <arch/tc4x/irq.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifndef STACKFRAME_ALIGN
#  define STACKFRAME_ALIGN 8
#endif

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

/* MPU CSFR numbers for TRICORE_MFCR/MTCR (TC1.6.2P/TC1.8 core arch manuals).
 * Set enables 4-7 are not contiguous with sets 0-3.
 */

#define CPU_PSW          TRICORE_CPU_PSW
#define CPU_CORECON      0xfe14

#define CPU_DPR0_L       0xc000
#define CPU_DPR0_U       0xc004
#define CPU_DPR1_L       0xc008
#define CPU_DPR1_U       0xc00c
#define CPU_DPR2_L       0xc010
#define CPU_DPR2_U       0xc014
#define CPU_DPR3_L       0xc018
#define CPU_DPR3_U       0xc01c
#define CPU_DPR4_L       0xc020
#define CPU_DPR4_U       0xc024
#define CPU_DPR5_L       0xc028
#define CPU_DPR5_U       0xc02c
#define CPU_DPR6_L       0xc030
#define CPU_DPR6_U       0xc034
#define CPU_DPR7_L       0xc038
#define CPU_DPR7_U       0xc03c
#define CPU_DPR8_L       0xc040
#define CPU_DPR8_U       0xc044
#define CPU_DPR9_L       0xc048
#define CPU_DPR9_U       0xc04c
#define CPU_DPR10_L      0xc050
#define CPU_DPR10_U      0xc054
#define CPU_DPR11_L      0xc058
#define CPU_DPR11_U      0xc05c
#define CPU_DPR12_L      0xc060
#define CPU_DPR12_U      0xc064
#define CPU_DPR13_L      0xc068
#define CPU_DPR13_U      0xc06c
#define CPU_DPR14_L      0xc070
#define CPU_DPR14_U      0xc074
#define CPU_DPR15_L      0xc078
#define CPU_DPR15_U      0xc07c
#define CPU_DPR16_L      0xc080
#define CPU_DPR16_U      0xc084
#define CPU_DPR17_L      0xc088
#define CPU_DPR17_U      0xc08c
#define CPU_DPR18_L      0xc090
#define CPU_DPR18_U      0xc094
#define CPU_DPR19_L      0xc098
#define CPU_DPR19_U      0xc09c
#define CPU_DPR20_L      0xc0a0
#define CPU_DPR20_U      0xc0a4
#define CPU_DPR21_L      0xc0a8
#define CPU_DPR21_U      0xc0ac
#define CPU_DPR22_L      0xc0b0
#define CPU_DPR22_U      0xc0b4
#define CPU_DPR23_L      0xc0b8
#define CPU_DPR23_U      0xc0bc

#define CPU_CPR0_L       0xd000
#define CPU_CPR0_U       0xd004
#define CPU_CPR1_L       0xd008
#define CPU_CPR1_U       0xd00c
#define CPU_CPR2_L       0xd010
#define CPU_CPR2_U       0xd014
#define CPU_CPR3_L       0xd018
#define CPU_CPR3_U       0xd01c
#define CPU_CPR4_L       0xd020
#define CPU_CPR4_U       0xd024
#define CPU_CPR5_L       0xd028
#define CPU_CPR5_U       0xd02c
#define CPU_CPR6_L       0xd030
#define CPU_CPR6_U       0xd034
#define CPU_CPR7_L       0xd038
#define CPU_CPR7_U       0xd03c
#define CPU_CPR8_L       0xd040
#define CPU_CPR8_U       0xd044
#define CPU_CPR9_L       0xd048
#define CPU_CPR9_U       0xd04c
#define CPU_CPR10_L      0xd050
#define CPU_CPR10_U      0xd054
#define CPU_CPR11_L      0xd058
#define CPU_CPR11_U      0xd05c
#define CPU_CPR12_L      0xd060
#define CPU_CPR12_U      0xd064
#define CPU_CPR13_L      0xd068
#define CPU_CPR13_U      0xd06c
#define CPU_CPR14_L      0xd070
#define CPU_CPR14_U      0xd074
#define CPU_CPR15_L      0xd078
#define CPU_CPR15_U      0xd07c

#define CPU_CPXE_0       0xe000
#define CPU_CPXE_1       0xe004
#define CPU_CPXE_2       0xe008
#define CPU_CPXE_3       0xe00c
#define CPU_CPXE_4       0xe040
#define CPU_CPXE_5       0xe044
#define CPU_CPXE_6       0xe048
#define CPU_CPXE_7       0xe04c

#define CPU_DPRE_0       0xe010
#define CPU_DPRE_1       0xe014
#define CPU_DPRE_2       0xe018
#define CPU_DPRE_3       0xe01c
#define CPU_DPRE_4       0xe050
#define CPU_DPRE_5       0xe054
#define CPU_DPRE_6       0xe058
#define CPU_DPRE_7       0xe05c

#define CPU_DPWE_0       0xe020
#define CPU_DPWE_1       0xe024
#define CPU_DPWE_2       0xe028
#define CPU_DPWE_3       0xe02c
#define CPU_DPWE_4       0xe060
#define CPU_DPWE_5       0xe064
#define CPU_DPWE_6       0xe068
#define CPU_DPWE_7       0xe06c

#ifndef __ASSEMBLY__

#ifdef __cplusplus

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * g_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

noinstrument_function static inline_function uintptr_t up_getsp(void)
{
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  return (uintptr_t)__get_sp();
#else
  return __builtin_frame_address(0);
#endif
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous PSW.IE state.
 *
 ****************************************************************************/

#define TRICORE_IRQ_DISABLE_AND_SAVE(x) \
  __asm__ __volatile__("disable %0":"=d"(x))
#define TRICORE_IRQ_RESTORE(x) \
  __asm__ __volatile__("restore %0"::"d"(x))

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t state;
  TRICORE_IRQ_DISABLE_AND_SAVE(state);
  return state;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the previous PSW.IE state from up_irq_save().
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  TRICORE_IRQ_RESTORE(flags);
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function uintptr_t *up_current_regs(void)
{
  return (uintptr_t *)g_current_regs[0];
}

static inline_function void up_set_current_regs(uintptr_t *regs)
{
  g_current_regs[0] = regs;
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
  bool ret = up_current_regs() != NULL;

  return ret;
}

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

static inline_function uintptr_t up_getusrsp(void *regs)
{
  uintptr_t *csa = regs;

  while (((uintptr_t)csa & PCXI_UL) == 0)
    {
      csa = tricore_csa2addr((uintptr_t)csa);
      csa = (uintptr_t *)csa[0];
    }

  csa = tricore_csa2addr((uintptr_t)csa);

  return csa[REG_SP];
}

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uintptr_t *)((regs) ? (regs) : running_regs()))[REG_PC])

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

#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_IRQ_H */
