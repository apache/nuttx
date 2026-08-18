/****************************************************************************
 * arch/risc-v/src/allwinner-d1/allwinner_d1_irq.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uintptr_t claim;
  int id;
  int word;

  up_irq_save();
  riscv_exception_attach();

  for (word = 0; word < (ALLWINNER_D1_PLIC_NDEV + 31) / 32; word++)
    {
      putreg32(0, ALLWINNER_D1_PLIC_ENABLE + 4 * word);
    }

  claim = getreg32(ALLWINNER_D1_PLIC_CLAIM);
  if (claim != 0)
    {
      putreg32(claim, ALLWINNER_D1_PLIC_CLAIM);
    }

  for (id = 1; id <= ALLWINNER_D1_PLIC_NDEV; id++)
    {
      putreg32(1, ALLWINNER_D1_PLIC_PRIORITY + 4 * id);
    }

  putreg32(0, ALLWINNER_D1_PLIC_THRESHOLD);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  riscv_color_intstack();
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 ****************************************************************************/

void up_disable_irq(int irq)
{
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      CLEAR_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      CLEAR_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;
      if (extirq > 0 && extirq <= ALLWINNER_D1_PLIC_NDEV)
        {
          modifyreg32(ALLWINNER_D1_PLIC_ENABLE + 4 * (extirq / 32),
                      1u << (extirq % 32), 0);
        }
    }
}

/****************************************************************************
 * Name: up_enable_irq
 ****************************************************************************/

void up_enable_irq(int irq)
{
  int extirq;

  if (irq == RISCV_IRQ_SOFT)
    {
      SET_CSR(CSR_IE, IE_SIE);
    }
  else if (irq == RISCV_IRQ_TIMER)
    {
      SET_CSR(CSR_IE, IE_TIE);
    }
  else if (irq > RISCV_IRQ_EXT)
    {
      extirq = irq - RISCV_IRQ_EXT;
      if (extirq > 0 && extirq <= ALLWINNER_D1_PLIC_NDEV)
        {
          modifyreg32(ALLWINNER_D1_PLIC_ENABLE + 4 * (extirq / 32),
                      0, 1u << (extirq % 32));
        }
    }
}

/****************************************************************************
 * Name: up_irq_enable
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t oldstat;

  SET_CSR(CSR_IE, IE_EIE);
  oldstat = READ_AND_SET_CSR(CSR_STATUS, STATUS_IE);
  return oldstat;
}
