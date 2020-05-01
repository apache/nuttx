/****************************************************************************
 * arch/arm/src/am335x/am335x_irq.c
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchenko@gmail.com>
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

#include <assert.h>

#include <nuttx/arch.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "sctlr.h"

#include "am335x_gpio.h"
#include "am335x_irq.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

#ifdef CONFIG_ARCH_LOWVECTORS
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A8 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *  The second method is used by this logic.
   */

  /* Set the VBAR register to the address of the vector table */

  DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)&_vector_start);
#endif /* CONFIG_ARCH_LOWVECTORS */

  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the INTC.
   */

  /* Reset the ARM interrupt controller */

  putreg32(INTC_SYSCONFIG_SOFTRESET, AM335X_INTC_SYSCONFIG);

  /* Wait for the reset to complete */

  while ((getreg32(AM335X_INTC_SYSSTATUS) & INTC_SYSSTATUS_RESETDONE) !=
         INTC_SYSSTATUS_RESETDONE)
    {
    }

  /* Enable any interrupt generation by setting priority threshold */

  putreg32(INTC_THRESHOLD_DISABLE, AM335X_INTC_THRESHOLD);

  /* Disable, mask, and clear all interrupts */

  for (i = 0; i < AM335X_IRQ_NINT; i += 32)
    {
      putreg32(0xffffffff, AM335X_INTC_MIR_SET(i)); /* 1 masks corresponding interrupt */
      getreg32(AM335X_INTC_PEND_IRQ(i));   /* Reading status clears pending interrupts */
      getreg32(AM335X_INTC_PEND_FIQ(i));   /* Reading status clears pending interrupts */
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  CURRENT_REGS = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
   */

#ifdef CONFIG_AM335X_GPIO_IRQ
  am335x_gpio_irqinitialize();
#endif

  /* And finally, enable interrupts */

  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   This function is called from the IRQ vector handler in arm_vectors.S.
 *   At this point, the interrupt has been taken and the registers have
 *   been saved on the stack.  This function simply needs to determine the
 *   the irq number of the interrupt and then to call arm_doirq to dispatch
 *   the interrupt.
 *
 *  Input parameters:
 *   regs - A pointer to the register save area on the stack.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
#if 1 /* Use PEND registers instead */
  uint32_t regval;

  /* Get active interrupt line */

  regval = getreg32(AM335X_INTC_SIR_IRQ) & INTC_SIR_IRQ_ACTIVE_MASK;

  /* Dispatch the interrupt */

  regs = arm_doirq((int)regval, regs);

  /* Enable new interrupt generation */

  putreg32(INTC_CONTROL_NEWIRQAGR, AM335X_INTC_CONTROL);

  return regs;
#else
  uintptr_t regaddr;
  uint32_t pending;
  int startirq;
  int lastirq;
  int irq;

#if 0
  /* Check each PEND register for pending interrupts.  Since the unused
   * interrupts are disabled, we do not have to be concerned about which
   * are MASKed.
   */

  for (startirq = 0, regaddr = AM335X_INTC_IRQ_PEND0;
       startirq < AM335X_IRQ_NINT;
       startirq += 32, regaddr += 4)
    {
      /* Check this register for pending interrupts */

      pending = getreg32(regaddr);
      if (pending != 0)
        {
          /* The last interrupt in this register */

          lastirq = startirq + 32;
          if (lastirq > AM335X_IRQ_NINT)
            {
              lastirq = AM335X_IRQ_NINT;
            }

          for (irq = startirq; irq < lastirq && pending != 0; )
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
                  if ((pending & 0x00000001) != 0)
                    {
                      /* Yes.. dispatch the interrupt */

                      regs = arm_doirq(irq, regs);
                    }

                  irq++;
                  pending >>= 1;
                }
            }
        }
    }
#endif

  return regs;
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq < AM335X_IRQ_NINT)
    {
      __asm__ __volatile__ ("\tdsb");

      /* Disable interrupt on INTC */

      putreg32(INTC_MIR_SET(irq), AM335X_INTC_MIR_SET(irq));
    }
#ifdef CONFIG_AM335X_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) GPIO IRQ */

      am335x_gpioirq_disable(irq);
    }
#endif
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq < AM335X_IRQ_NINT)
    {
      __asm__ __volatile__ ("\tdsb");

      /* Enable interrupt on INTC */

      putreg32(INTC_MIR_CLEAR(irq), AM335X_INTC_MIR_CLEAR(irq));
    }
#ifdef CONFIG_AM335X_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) GPIO IRQ */

      am335x_gpioirq_enable(irq);
    }
#endif
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;

  DEBUGASSERT(irq < AM335X_IRQ_NINT && (unsigned)priority <= INTC_PRIO_MAX);
  if (irq < AM335X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

#if 0 // TODO
      /* Set the new priority */

      regaddr = A1X_INTC_PRIO_OFFSET(irq);
      regval  = getreg32(regaddr);
      regval &= ~INTC_PRIO_MASK(irq);
      regval |= INTC_PRIO(irq, priority);
      putreg32(regval, regaddr);
#endif

      leave_critical_section(flags);
      return OK;
    }

  return -EINVAL;
}
#endif
