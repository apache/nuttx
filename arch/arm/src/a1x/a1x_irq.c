/****************************************************************************
 * arch/arm/src/a1x/a1x_irq.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "a1x_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: a1x_dumpintc
 *
 * Description:
 *   Dump some interesting INTC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void a1x_dumpintc(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();
  lldbg("INTC (%s, irq=%d):\n", msg, irq);

  /* Select the register set associated with this irq */

  putreg32(irq, A1X_INTC_SSR);

  /* Then dump all of the (readable) register contents */

  lldbg("  VECTOR: %08x BASE: %08x PROTECT: %08x NMICTRL: %08x\n",
        getreg32(A1X_INTC_VECTOR),    getreg32(A1X_INTC_BASEADDR),
        getreg32(A1X_INTC_PROTECT),   getreg32(A1X_INTC_NMICTRL));
  lldbg("  IRQ PEND: %08x %08x %08x\n",
        getreg32(A1X_INTC_IRQ_PEND0), getreg32(A1X_INTC_IRQ_PEND1),
        getreg32(A1X_INTC_IRQ_PEND2));
  lldbg("  FIQ PEND: %08x %08x %08x\n",
        getreg32(A1X_INTC_FIQ_PEND0), getreg32(A1X_INTC_FIQ_PEND1),
        getreg32(A1X_INTC_FIQ_PEND2));
  lldbg("  SEL:      %08x %08x %08x\n",
        getreg32(A1X_INTC_IRQ_SEL0),  getreg32(A1X_INTC_IRQ_SEL1),
        getreg32(A1X_INTC_IRQ_SEL2));
  lldbg("  EN:       %08x %08x %08x\n",
        getreg32(A1X_INTC_EN0),       getreg32(A1X_INTC_EN1),
        getreg32(A1X_INTC_EN2));
  lldbg("  MASK:     %08x %08x %08x\n",
        getreg32(A1X_INTC_MASK0),     getreg32(A1X_INTC_MASK1),
        getreg32(A1X_INTC_MASK2));
  lldbg("  RESP:     %08x %08x %08x\n",
        getreg32(A1X_INTC_RESP0),     getreg32(A1X_INTC_RESP1),
        getreg32(A1X_INTC_RESP2));
  lldbg("  FF:       %08x %08x %08x\n",
        getreg32(A1X_INTC_FF0),       getreg32(A1X_INTC_FF1),
        getreg32(A1X_INTC_FF2));
  lldbg("  PRIO:     %08x %08x %08x %08x %08x\n",
        getreg32(A1X_INTC_PRIO0),     getreg32(A1X_INTC_PRIO1),
        getreg32(A1X_INTC_PRIO2),     getreg32(A1X_INTC_PRIO3),
        getreg32(A1X_INTC_PRIO4));
  irqrestore(flags);
}
#else
#  define a1x_dumpintc(msg, irq)
#endif

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

  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the INTC.
   */

  /* Colorize the interrupt stack for debug purposes */

#if defined(CONFIG_DEBUG_STACK) && CONFIG_ARCH_INTERRUPTSTACK > 3
  {
    size_t intstack_size = (CONFIG_ARCH_INTERRUPTSTACK & ~3);
    up_stack_color((FAR void *)((uintptr_t)&g_intstackbase - intstack_size),
                   intstack_size);
  }
#endif

  /* Set the interrupt base address to zero.  We do not use the vectored
   * interrupts.
   */

  putreg32(0, A1X_INTC_BASEADDR);

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * PIO pins.
   */

#ifdef CONFIG_A1X_GPIO_IRQ
  a1x_gpio_irqinitialize();
#endif

  /* And finally, enable interrupts */

  (void)irqenable();
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
  uint32_t regval;

 /* During initialization, the BASE address register was set to zero.
  * Therefore, when we read the VECTOR address register, we get the IRQ number
  * shifted left by two.
  */

  regval = getreg32(A1X_INTC_VECTOR);

  /* REVISIT:  I am thinking that we need to read the PEND0-2 registers
   * in order to clear the pending interrupt.
   */
#warning Missing logic

  /* Dispatch the interrupt */

  return arm_doirq((int)(regval >> 2), regs);
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
  irqstate_t flags;
  uintptr_t enabreg;
  uintptr_t maskreg;
  uint32_t regval;
  uint32_t bit;

  if (irq < A1X_IRQ_NINT)
    {
      DEBUGASSERT(irg < 96);

      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      if (irq < 32)
        {
          enabreg = A1X_INTC_EN0;
          maskreg = A1X_INTC_MASK0;
          bit     = irq;
        }
      else if (irq < 64)
        {
          enabreg = A1X_INTC_EN1;
          maskreg = A1X_INTC_MASK1;
          bit     = irq - 32;
        }
      else if (irq < 64)
        {
          enabreg = A1X_INTC_EN2;
          maskreg = A1X_INTC_MASK2;
          bit     = irq - 64;
        }
      else
        {
          /* Will not happen */

          return;
        }

      /* Mask the interrupt by setting the bit in the mask register */

      regval = getreg32(maskreg);
      regval |= (1 << bit);
      putreg32(regval, maskreg);

      /* Make sure that the interrupt is enabled.  The interrupt must still
       * be enabled in order for interrupts to pend while masked.
       */

      regval = getreg32(enabreg);
      regval |= (1 << bit);
      putreg32(regval, enabreg);

      a1x_dumpintc("disable", irq);
      irqrestore(flags);
    }
#ifdef CONFIG_A1X_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      a1x_gpio_irqdisable(irq);
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
  irqstate_t flags;
  uintptr_t enabreg;
  uintptr_t maskreg;
  uint32_t regval;
  uint32_t bit;

  if (irq < A1X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      if (irq < 32)
        {
          enabreg = A1X_INTC_EN0;
          maskreg = A1X_INTC_MASK0;
          bit     = irq;
        }
      else if (irq < 64)
        {
          enabreg = A1X_INTC_EN1;
          maskreg = A1X_INTC_MASK1;
          bit     = irq - 32;
        }
      else if (irq < 64)
        {
          enabreg = A1X_INTC_EN2;
          maskreg = A1X_INTC_MASK2;
          bit     = irq - 64;
        }
      else
        {
          /* Will not happen */

          return;
        }

      /* Make sure that the interrupt is enabled. */

      regval  = getreg32(enabreg);
      regval |= (1 << bit);
      putreg32(regval, enabreg);

      /* Un-mask the interrupt by clearing the bit in the mask register */

      regval = getreg32(maskreg);
      regval &= ~(1 << bit);
      putreg32(regval, maskreg);

      a1x_dumpintc("enable", irq);
      irqrestore(flags);
    }
#ifdef CONFIG_A1X_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) PIO IRQ */

      a1x_gpio_irqenable(irq);
    }
#endif
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  /* It is not clear to me how the interrupts are acknowledge.  Perhaps it
   * is simply by reading the IRQ pending register?  If so, where is that
   * done?
   */
#warning Missing logic
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  /* Disable the interrupt */

  up_disable_irq(irq);

  /* Then acknowledge it */

  up_ack_irq(irq);
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
  int offset;

  DEBUGASSERT(irq < A1X_IRQ_NINT && (unsigned)priority <= INTC_PRIO_MAX);
  if (irq < A1X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = irqsave();

      /* Select the register set associated with this irq */

      if (irq < 16)
        {
          regaddr = A1X_INTC_PRIO0;
          offset  = irq;
        }
      else if (irq < 32)
        {
          regaddr = A1X_INTC_PRIO1;
          offset  = irq - 16;
        }
      else if (irq < 48)
        {
          regaddr = A1X_INTC_PRIO2;
          offset  = irq - 32;
        }
      else if (irq < 64)
        {
          regaddr = A1X_INTC_PRIO3;
          offset  = irq - 48;
        }
      else if (irq < 80)
        {
          regaddr = A1X_INTC_PRIO4;
          offset  = irq - 64;
        }
      else
        {
          /* Should not happen */

          return -EINVAL;
        }

      /* Set the new priority */

      regval  = getreg32(regaddr);
      regval &= ~INTC_PRIO_MASK(offset);
      regval |= INTC_PRIO(offset, priority);
      putreg32(regval, regaddr);

      a1x_dumpintc("prioritize", irq);
      irqrestore(flags);
      return OK;
    }

  return -EINVAL;
}
#endif
