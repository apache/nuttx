/****************************************************************************
 * arch/arm/src/a1x/a1x_pio.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "a1x_pio.h"
#include "chip/a1x_pio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: a1x_pio_pin
 *
 * Description:
 *   Return the port number
 *
 ****************************************************************************/

static inline int a1x_pio_port(pio_pinset_t cfgset)
{
  return ((cfgset & PIO_PORT_MASK) >> PIO_PORT_SHIFT);
}

/****************************************************************************
 * Name: a1x_pio_pin
 *
 * Description:
 *   Return the pin bit position
 *
 ****************************************************************************/

static inline int a1x_pio_pin(pio_pinset_t cfgset)
{
  return 1 << ((cfgset & PIO_PIN_MASK) >> PIO_PIN_SHIFT);
}

/****************************************************************************
 * Name: a1x_pio_interrupt
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   PIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
static int a1x_pio_interrupt(int irq, void *context)
{
  uint32_t status;
  uint32_t mask;
  uint32_t pending;
  int irq;

  /* Read the set of pending GPIO interrupts */

  status  = getreg32(A1X_PIO_INT_STA);
  mask    = getreg32(A1X_PIO_INT_CTL);
  pending = status & mask;

  /* Re-dispatch all pending GPIO interrupts */

  for (irq = A1X_PIO_EINT0; pending != 0 && irq <= A1X_PIO_EINT31; irq++)
    {
      /* Check for pending interrupts in any of the lower 16-bits */

      if ((pending & 0x0000ffff) == 0)
        {
          irq      += 16;
          pending >>= 16;
        }

      /* Check for pending interrupts in any of the lower 16-bits */

      else if ((pending & 0x000000ff) == 0)
        {
          irq      += 8;
          pending >>= 8;
        }

      /* Check for pending interrupts in any of the lower 4-bits */

      else if ((pending & 0x0000000f) == 0)
        {
          irq      += 4;
          pending >>= 4;
        }

      /* Check for pending interrupts in any of the lower 2-bits */

      else if ((pending & 0x00000003) == 0)
        {
          irq      += 2;
          pending >>= 2;
        }

      /* Check for pending interrupts in any of the last bits */

      else
        {
          if ((pending & 0x00000001) == 0)
            {
              /* Yes.. dispatch the interrupt */

              (void)arm_doirq(irq, regs);
            }

          irq++;
          pending >>= 1;
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a1x_pio_irqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   PIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
void a1x_pio_irqinitialize(void)
{
  int ret;

  /* Disable all external PIO interrupts */

  putreg32(0, A1X_PIO_INT_CTL);

  /* Attach the PIO interrupt handler */

  ret = irq_attach(A1X_IRQ_PIO)
  if (ret < 0)
    {
      return ret;
    }

  /* And enable the PIO interrupt */

  up_enable_irq(A1X_IRQ_PIO);
}
#endif

/****************************************************************************
 * Name: a1x_pio_config
 *
 * Description:
 *   Configure a PIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int a1x_pio_config(pio_pinset_t cfgset)
{
  unsigned int port = a1x_pio_port(cfgset);
  unsigned int pin  = a1x_pio_pin(cfgset);
  unsigned int shift;
  unsigned int value;
  uintptr_t cfgaddr;
  uintptr_t puaddr;
  uintptr_t drvaddr;
  uintptr_t intaddr;
  uintptr_t dataddr;
  uint32_t regval;
  irqstate_t flags;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Set the peripheral ID (0=input, 1=output) and interrupt mode */

  switch (pin >> 3)
    {
      case 0: /* PIO 0-7 */
        cfgaddr = A1X_PIO_CFG0(port);
        intaddr = A1X_PIO_INT_CFG0;
        break;

      case 1: /* PIO 8-15 */
        cfgaddr = A1X_PIO_CFG1(port);
        intaddr = A1X_PIO_INT_CFG1;
        break;

      case 2: /* PIO 16-23 */
        cfgaddr = A1X_PIO_CFG2(port);
        intaddr = A1X_PIO_INT_CFG2;
        break;

      case 3: /* PIO 24-31 */
        cfgaddr = A1X_PIO_CFG3(port);
        intaddr = A1X_PIO_INT_CFG3;
        break;

      default:
        return -EINVAL;
    }

  value = (cfgset & PIO_MODE_MASK) >> PIO_MODE_SHIFT;
  shift = (port & 7) << 4;

  regval = getreg32(cfgaddr);
  regval &= ~(7 << shift);
  regval |= (value << shift);
  putreg32(regval, cfgaddr);

  /* Do not modify the INT MASK unless this pin is configured
   * as an external PIO interrupt.
   */

  if ((cfgset & PIO_EINT_MASK) == PIO_EINT)
    {
      value = (cfgset & PIO_INT_MASK) >> PIO_INT_SHIFT;

      regval = getreg32(intaddr);
      regval &= ~(7 << shift);
      regval |= (value << shift);
      putreg32(regval, intaddr);
    }

  /* Set the pull-up/down and drive strength */

  switch (pin >> 4)
    {
      case 0: /* PIO 0-15 */
        puaddr  = A1X_PIO_PUL0(port);
        drvaddr = A1X_PIO_DRV0(port);
        break;

      case 1: /* PIO 16-31 */
        puaddr  = A1X_PIO_PUL1(port);
        drvaddr = A1X_PIO_DRV1(port);
        break;

      default:
        return -EINVAL;
    }

  value = (cfgset & PIO_PULL_MASK) >> PIO_PULL_SHIFT;
  shift = (port & 15) << 2;

  regval = getreg32(puaddr);
  regval &= ~(3 << shift);
  regval |= (value << shift);
  putreg32(regval, puaddr);

  value = (cfgset & PIO_DRIVE_MASK) >> PIO_DRIVE_SHIFT;

  regval = getreg32(drvaddr);
  regval &= ~(3 << shift);
  regval |= (value << shift);
  putreg32(regval, drvaddr);

  /* Set the output value (will have no effect on inputs) */

  dataddr = A1X_PIO_DAT(port);
  regval = getreg32(dataddr);

  if ((cfgset & PIO_OUTPUT_SET) != 0)
    {
      regval |= PIO_DAT(pin);
    }
  else
    {
      regval &= ~PIO_DAT(pin);
    }

  putreg32(regval, dataddr);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: a1x_piowrite
 *
 * Description:
 *   Write one or zero to the selected PIO pin
 *
 ****************************************************************************/

void a1x_pio_write(pio_pinset_t pinset, bool value)
{
  unsigned int port = a1x_pio_port(pinset);
  unsigned int pin  = a1x_pio_pin(pinset);
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;

  /* Disable interrupts to prohibit re-entrance. */

  flags = enter_critical_section();

  /* Set the output value (will have no effect on inputs */

  regaddr = A1X_PIO_DAT(port);
  regval  = getreg32(regaddr);

  if (value)
    {
      regval |= PIO_DAT(pin);
    }
  else
    {
      regval &= ~PIO_DAT(pin);
    }

  putreg32(regval, regaddr);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: a1x_pio_read
 *
 * Description:
 *   Read one or zero from the selected PIO pin
 *
 ****************************************************************************/

bool a1x_pio_read(pio_pinset_t pinset)
{
  unsigned int port = a1x_pio_port(pinset);
  unsigned int pin  = a1x_pio_pin(pinset);
  uintptr_t regaddr;
  uint32_t regval;

  /* Get the input value */

  regaddr = A1X_PIO_DAT(port);
  regval  = getreg32(regaddr);
  return ((regval & PIO_DAT(pin)) != 0);
}

/************************************************************************************
 * Name: a1x_pio_irqenable
 *
 * Description:
 *   Enable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_A1X_PIO_IRQ
void a1x_pio_irqenable(int irq)
{
  irqstate_t flags;
  uint32_t regval;
  int pin;

  if (irq >= A1X_PIO_EINT0 && irq <= A1X_PIO_EINT31)
    {
      /* Convert the IRQ number to a bit position */

      pin = irq - A1X_PIO_EINT0

      /* Un-mask the interrupt be setting the corresponding bit in the PIO INT CTL
       * register.
       */

      flags   = enter_critical_section();
      regval  = getreg32(A1X_PIO_INT_CTL);
      regval |= PIO_INT_CTL(irq);
      leave_critical_section(flags);
    }
}
#endif

/************************************************************************************
 * Name: a1x_pio_irqdisable
 *
 * Description:
 *   Disable the interrupt for specified PIO IRQ
 *
 ************************************************************************************/
#ifdef CONFIG_A1X_PIO_IRQ

void a1x_pio_irqdisable(int irq)
{
  irqstate_t flags;
  uint32_t regval;
  int pin;

  if (irq >= A1X_PIO_EINT0 && irq <= A1X_PIO_EINT31)
    {
      /* Convert the IRQ number to a bit position */

      pin = irq - A1X_PIO_EINT0

      /* Mask the interrupt be clearning the corresponding bit in the PIO INT CTL
       * register.
       */

      flags   = enter_critical_section();
      regval  = getreg32(A1X_PIO_INT_CTL);
      regval &= ~PIO_INT_CTL(irq);
      leave_critical_section(flags);
    }
}
#endif
