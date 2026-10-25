/****************************************************************************
 * arch/risc-v/include/ab32vg1/irq.h
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

/* The AB32VG1 core implements RV32IMC but not the standard machine-mode
 * trap architecture (CONFIG_ARCH_RV_CUSTOM_TRAP):
 *
 * - There are no CSRs.  Any CSR access, ecall or ebreak raises an
 *   exception which leaves the core in a state where every following
 *   instruction traps again, so these instructions must never be used.
 * - Interrupt masking is bit 0 of the memory-mapped PICCON register.  On
 *   trap entry the previous value is saved in EPICCON and mret restores it.
 *
 * System calls therefore use a software trap: RISCV_SYSCALL_INSN calls
 * ab32vg1_syscall_trap(), which builds the same register frame as a
 * hardware trap with interrupts masked, dispatches it as RISCV_IRQ_ECALLM
 * and leaves through the common trap exit (EPC/EPICCON + mret), which also
 * works from thread mode.  ra is preserved on the stack so that the
 * instruction sequence does not clobber any register.
 */

#ifndef __ARCH_RISCV_INCLUDE_AB32VG1_IRQ_H
#define __ARCH_RISCV_INCLUDE_AB32VG1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <nuttx/compiler.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The AB32VG1 interrupt controller (PIC) has one enable/pending bit per
 * source.  Several peripherals share a source.  NuttX IRQ numbers follow
 * the RISC-V exception numbers (0..RISCV_MAX_EXCEPTION).
 */

#define AB32VG1_IRQ_FIRST       RISCV_IRQ_ASYNC

#define AB32VG1_IRQ_SW          (AB32VG1_IRQ_FIRST + 2)  /* Software */
#define AB32VG1_IRQ_TMR0        (AB32VG1_IRQ_FIRST + 3)  /* Timer 0 */
#define AB32VG1_IRQ_TMR1        (AB32VG1_IRQ_FIRST + 4)  /* Timer 1 */
#define AB32VG1_IRQ_TMR245      (AB32VG1_IRQ_FIRST + 5)  /* Timer 2, 4, 5 */
#define AB32VG1_IRQ_IRRX        (AB32VG1_IRQ_FIRST + 6)  /* Timer 3, IR rx */
#define AB32VG1_IRQ_USB         (AB32VG1_IRQ_FIRST + 7)  /* USB */
#define AB32VG1_IRQ_SD          (AB32VG1_IRQ_FIRST + 8)  /* SD host */
#define AB32VG1_IRQ_AUBUF       (AB32VG1_IRQ_FIRST + 9)  /* Audio buffer 0/1 */
#define AB32VG1_IRQ_SDADC       (AB32VG1_IRQ_FIRST + 10) /* SD ADC */
#define AB32VG1_IRQ_AUDEC       (AB32VG1_IRQ_FIRST + 11) /* Audio codec */
#define AB32VG1_IRQ_SRC         (AB32VG1_IRQ_FIRST + 12) /* SRC, PLC, CVSD */
#define AB32VG1_IRQ_FM          (AB32VG1_IRQ_FIRST + 13) /* FM TX/RX, SPDIF */
#define AB32VG1_IRQ_UART        (AB32VG1_IRQ_FIRST + 14) /* UART 0, 1, 2 */
#define AB32VG1_IRQ_HSUART      (AB32VG1_IRQ_FIRST + 15) /* High speed UART */
#define AB32VG1_IRQ_RTC         (AB32VG1_IRQ_FIRST + 16) /* RTC, LVD, WDT */
#define AB32VG1_IRQ_I2S         (AB32VG1_IRQ_FIRST + 17) /* I2S */
#define AB32VG1_IRQ_GPIO        (AB32VG1_IRQ_FIRST + 18) /* Port interrupt */

#define AB32VG1_NPIC_IRQS       23

#define NR_IRQS                 (AB32VG1_IRQ_FIRST + AB32VG1_NPIC_IRQS)

/* Interrupt controller registers used by the inline functions below */

#define AB32VG1_PICCONCLR_ADDR  0x00000530
#define AB32VG1_PICCONSET_ADDR  0x00000534
#define AB32VG1_PICCON_ADDR     0x00000540
#define AB32VG1_PICCON_IE       (1 << 0)

/* Software trap used by sys_callN() in place of ecall */

#define RISCV_SYSCALL_INSN \
  "addi sp, sp, -16\n" \
  "sw   ra, 0(sp)\n" \
  "jal  ra, ab32vg1_syscall_trap\n" \
  "lw   ra, 0(sp)\n" \
  "addi sp, sp, 16\n"

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous interrupt enable state.
 *
 ****************************************************************************/

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  /* The SFRs are below 4 KB: address them relative to x0 */

  __asm__ __volatile__
    (
      "lw   %0, %1(zero)\n"
      "sw   %2, %3(zero)\n"
      : "=&r" (flags)
      : "i" (AB32VG1_PICCON_ADDR), "r" (AB32VG1_PICCON_IE),
        "i" (AB32VG1_PICCONCLR_ADDR)
      : "memory"
    );

  return flags & AB32VG1_PICCON_IE;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the interrupt enable state returned by up_irq_save().
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  if (flags & AB32VG1_PICCON_IE)
    {
      __asm__ __volatile__
        (
          "sw   %0, %1(zero)\n"
          :
          : "r" (AB32VG1_PICCON_IE), "i" (AB32VG1_PICCONSET_ADDR)
          : "memory"
        );
    }
  else
    {
      __asm__ __volatile__
        (
          "sw   %0, %1(zero)\n"
          :
          : "r" (AB32VG1_PICCON_IE), "i" (AB32VG1_PICCONCLR_ADDR)
          : "memory"
        );
    }
}

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_AB32VG1_IRQ_H */
