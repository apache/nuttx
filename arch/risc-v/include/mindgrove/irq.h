#ifndef __ARCH_RISCV_INCLUDE_SECURE_IOT_IRQ_H
#define __ARCH_RISCV_INCLUDE_SECURE_IOT_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define MINDGROVE_PLIC_START   ((RISCV_IRQ_MEXT ))

/* Total number of IRQs */

#define NR_IRQS               (MINDGROVE_PLIC_START + 58)

#endif /* __ARCH_RISCV_INCLUDE_SECURE_IOT_IRQ_H */
