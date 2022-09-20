/****************************************************************************
 * arch/arm/src/a1x/a1x_irq.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sctlr.h"

#include "a1x_pio.h"
#include "a1x_irq.h"

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

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void a1x_dumpintc(const char *msg, int irq)
{
  irqstate_t flags;

  /* Dump some relevant ARMv7 register contents */

  flags = enter_critical_section();

  irqinfo("ARMv7 (%s, irq=%d):\n", msg, irq);
  irqinfo("  CPSR: %08x SCTLR: %08x\n", flags, cp15_rdsctlr());

  /* Dump all of the (readable) INTC register contents */

  irqinfo("INTC (%s, irq=%d):\n", msg, irq);
  irqinfo("  VECTOR: %08x BASE: %08x PROTECT: %08x NMICTRL: %08x\n",
          getreg32(A1X_INTC_VECTOR),    getreg32(A1X_INTC_BASEADDR),
          getreg32(A1X_INTC_PROTECT),   getreg32(A1X_INTC_NMICTRL));
  irqinfo("  IRQ PEND: %08x %08x %08x\n",
          getreg32(A1X_INTC_IRQ_PEND0), getreg32(A1X_INTC_IRQ_PEND1),
          getreg32(A1X_INTC_IRQ_PEND2));
  irqinfo("  FIQ PEND: %08x %08x %08x\n",
          getreg32(A1X_INTC_FIQ_PEND0), getreg32(A1X_INTC_FIQ_PEND1),
          getreg32(A1X_INTC_FIQ_PEND2));
  irqinfo("  SEL:      %08x %08x %08x\n",
          getreg32(A1X_INTC_IRQ_SEL0),  getreg32(A1X_INTC_IRQ_SEL1),
          getreg32(A1X_INTC_IRQ_SEL2));
  irqinfo("  EN:       %08x %08x %08x\n",
          getreg32(A1X_INTC_EN0),       getreg32(A1X_INTC_EN1),
          getreg32(A1X_INTC_EN2));
  irqinfo("  MASK:     %08x %08x %08x\n",
          getreg32(A1X_INTC_MASK0),     getreg32(A1X_INTC_MASK1),
          getreg32(A1X_INTC_MASK2));
  irqinfo("  RESP:     %08x %08x %08x\n",
          getreg32(A1X_INTC_RESP0),     getreg32(A1X_INTC_RESP1),
          getreg32(A1X_INTC_RESP2));
  irqinfo("  FF:       %08x %08x %08x\n",
          getreg32(A1X_INTC_FF0),       getreg32(A1X_INTC_FF1),
          getreg32(A1X_INTC_FF2));
  irqinfo("  PRIO:     %08x %08x %08x %08x %08x\n",
          getreg32(A1X_INTC_PRIO0),     getreg32(A1X_INTC_PRIO1),
          getreg32(A1X_INTC_PRIO2),     getreg32(A1X_INTC_PRIO3),
          getreg32(A1X_INTC_PRIO4));

  leave_critical_section(flags);
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
  int i;

  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the INTC.
   */

  /* Disable, mask, and clear all interrupts */

  for (i = 0; i < A1X_IRQ_NINT; i += 32)
    {
      putreg32(0x00000000, A1X_INTC_EN(i));   /* 0 disables corresponding interrupt */
      putreg32(0xffffffff, A1X_INTC_MASK(i)); /* 1 masks corresponding interrupt */
      getreg32(A1X_INTC_IRQ_PEND(i));         /* Reading status clears pending interrupts */
    }

  /* Set the interrupt base address to zero.  We do not use the vectored
   * interrupts.
   */

  putreg32(0, A1X_INTC_BASEADDR);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
#ifdef CONFIG_A1X_PIO_IRQ
  /* Initialize logic to support a second level of interrupt decoding
   * for PIO pins.
   */

  a1x_pio_irqinitialize();
#endif

  /* And finally, enable interrupts */

  up_irq_enable();
#endif

  a1x_dumpintc("initial", 0);
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
  /* REVISIT:  I think that if you want to have prioritized interrupts, you
   * would have to get the highest priority pending interrupt from the VECTOR
   * register.  But, in that case, you would also need to clear the pending
   * interrupt by reading the PEND register.  However, won't that clear up
   * to 32 pending interrupts?
   */

#if 0 /* Use PEND registers instead */
  uint32_t regval;

  /* During initialization, the BASE address register was set to zero.
   * Therefore, when we read the VECTOR address register, we get the IRQ
   * number shifted left by two.
   */

  regval = getreg32(A1X_INTC_VECTOR);

  /* Dispatch the interrupt */

  return arm_doirq((int)(regval >> 2), regs);
#else
  uintptr_t regaddr;
  uint32_t pending;
  int startirq;
  int lastirq;
  int irq;

  /* Check each PEND register for pending interrupts.  Since the unused
   * interrupts are disabled, we do not have to be concerned about which
   * are MASKed.
   */

  for (startirq = 0, regaddr = A1X_INTC_IRQ_PEND0;
       startirq < A1X_IRQ_NINT;
       startirq += 32, regaddr += 4)
    {
      /* Check this register for pending interrupts */

      pending = getreg32(regaddr);
      if (pending != 0)
        {
          /* The last interrupt in this register */

          lastirq = startirq + 32;
          if (lastirq > A1X_IRQ_NINT)
            {
              lastirq = A1X_IRQ_NINT;
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
  irqstate_t flags;
  uintptr_t regaddr;
  uint32_t regval;

  if (irq < A1X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Make sure that the interrupt is disabled. */

      regaddr = A1X_INTC_EN(irq);
      regval  = getreg32(regaddr);
      regval &= ~INTC_EN(irq);
      putreg32(regval, regaddr);

      /* Mask the interrupt by setting the bit in the mask register */

      regaddr = A1X_INTC_MASK(irq);
      regval  = getreg32(regaddr);
      regval |= INTC_MASK(irq);
      putreg32(regval, regaddr);

      a1x_dumpintc("disable", irq);
      leave_critical_section(flags);
    }

#ifdef CONFIG_A1X_PIO_IRQ
  /* Perhaps this is a second level PIO interrupt */

  else
    {
      a1x_pio_irqdisable(irq);
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
  uintptr_t regaddr;
  uint32_t regval;

  if (irq < A1X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Make sure that the interrupt is enabled. */

      regaddr = A1X_INTC_EN(irq);
      regval  = getreg32(regaddr);
      regval |= INTC_EN(irq);
      putreg32(regval, regaddr);

      /* Un-mask the interrupt by clearing the bit in the mask register */

      regaddr = A1X_INTC_MASK(irq);
      regval  = getreg32(regaddr);
      regval &= ~INTC_MASK(irq);
      putreg32(regval, regaddr);

      a1x_dumpintc("enable", irq);
      leave_critical_section(flags);
    }

#ifdef CONFIG_A1X_PIO_IRQ
  /* Perhaps this is a second level PIO interrupt */

  else
    {
      a1x_pio_irqenable(irq);
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

  DEBUGASSERT(irq < A1X_IRQ_NINT && (unsigned)priority <= INTC_PRIO_MAX);
  if (irq < A1X_IRQ_NINT)
    {
      /* These operations must be atomic */

      flags = enter_critical_section();

      /* Set the new priority */

      regaddr = A1X_INTC_PRIO_OFFSET(irq);
      regval  = getreg32(regaddr);
      regval &= ~INTC_PRIO_MASK(irq);
      regval |= INTC_PRIO(irq, priority);
      putreg32(regval, regaddr);

      a1x_dumpintc("prioritize", irq);
      leave_critical_section(flags);
      return OK;
    }

  return -EINVAL;
}
#endif
