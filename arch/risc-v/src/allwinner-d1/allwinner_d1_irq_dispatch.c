/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_irq_dispatch.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RV_IRQ_MASK 59

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  int irq = (vector >> RV_IRQ_MASK) | (vector & 0xf);
  uint32_t source = 0;

  if (irq == RISCV_IRQ_EXT)
    {
      source = getreg32(ALLWINNER_D1_PLIC_CLAIM);
      if (source != 0)
        {
          irq += source;
        }
    }

  if (irq != RISCV_IRQ_EXT)
    {
      regs = riscv_doirq(irq, regs);
    }

  if (source != 0)
    {
      putreg32(source, ALLWINNER_D1_PLIC_CLAIM);
    }

  return regs;
}
