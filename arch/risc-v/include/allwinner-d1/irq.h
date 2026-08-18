/****************************************************************************
 * arch/risc-v/include/allwinner-d1/irq.h
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __ARCH_RISCV_INCLUDE_ALLWINNER_D1_IRQ_H
#define __ARCH_RISCV_INCLUDE_ALLWINNER_D1_IRQ_H

/* The D1 PLIC exposes interrupt source IDs 1 through 175. */

#define ALLWINNER_D1_PLIC_TIMER0 75
#define ALLWINNER_D1_PLIC_TIMER1 76

#define ALLWINNER_D1_IRQ_TIMER0 \
  (RISCV_IRQ_SEXT + ALLWINNER_D1_PLIC_TIMER0)
#define ALLWINNER_D1_IRQ_TIMER1 \
  (RISCV_IRQ_SEXT + ALLWINNER_D1_PLIC_TIMER1)

#define NR_IRQS (RISCV_IRQ_SEXT + 176)

#endif /* __ARCH_RISCV_INCLUDE_ALLWINNER_D1_IRQ_H */
