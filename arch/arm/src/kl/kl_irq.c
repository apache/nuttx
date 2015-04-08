/****************************************************************************
 * arch/arm/src/kl/kl_irq.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "up_arch.h"
#include "up_internal.h"

#include "kl_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | NVIC_SYSH_PRIORITY_DEFAULT << 16 |\
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | NVIC_SYSH_PRIORITY_DEFAULT)

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
 * Name: kl_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ)
static void kl_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();

  lldbg("NVIC (%s, irq=%d):\n", msg, irq);
  lldbg("  ISER:       %08x ICER:   %08x\n",
        getreg32(ARMV6M_NVIC_ISER), getreg32(ARMV6M_NVIC_ICER));
  lldbg("  ISPR:       %08x ICPR:   %08x\n",
        getreg32(ARMV6M_NVIC_ISPR), getreg32(ARMV6M_NVIC_ICPR));
  lldbg("  IRQ PRIO:   %08x %08x %08x %08x\n",
        getreg32(ARMV6M_NVIC_IPR0), getreg32(ARMV6M_NVIC_IPR1),
        getreg32(ARMV6M_NVIC_IPR2), getreg32(ARMV6M_NVIC_IPR3));
  lldbg("              %08x %08x %08x %08x\n",
        getreg32(ARMV6M_NVIC_IPR4), getreg32(ARMV6M_NVIC_IPR5),
        getreg32(ARMV6M_NVIC_IPR6), getreg32(ARMV6M_NVIC_IPR7));

  lldbg("SYSCON:\n");
  lldbg("  CPUID:      %08x\n",
        getreg32(ARMV6M_SYSCON_CPUID));
  lldbg("  ICSR:       %08x AIRCR:  %08x\n",
        getreg32(ARMV6M_SYSCON_ICSR), getreg32(ARMV6M_SYSCON_AIRCR));
  lldbg("  SCR:        %08x CCR:    %08x\n",
        getreg32(ARMV6M_SYSCON_SCR), getreg32(ARMV6M_SYSCON_CCR));
  lldbg("  SHPR2:      %08x SHPR3:  %08x\n",
        getreg32(ARMV6M_SYSCON_SHPR2), getreg32(ARMV6M_SYSCON_SHPR3));

  irqrestore(flags);
}

#else
#  define kl_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: kl_nmi, kl_busfault, kl_usagefault, kl_pendsv,
 *       kl_dbgmonitor, kl_pendsv, kl_reserved
 *
 * Description:
 *   Handlers for various execptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG
static int kl_nmi(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int kl_pendsv(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int kl_reserved(int irq, FAR void *context)
{
  (void)irqsave();
  dbg("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: kl_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.
 *
 ****************************************************************************/

static inline void kl_clrpend(int irq)
{
  /* This will be called on each interrupt exit whether the interrupt can be
   * enambled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= KL_IRQ_EXTINT && irq < (KL_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - KL_IRQ_EXTINT)), ARMV6M_NVIC_ICPR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int i;

  /* Disable all interrupts */

  putreg32(0xffffffff, ARMV6M_NVIC_ICER);

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR2);
  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR3);

  /* Now set all of the interrupt lines to the default priority */

  for (i = 0; i < 8; i++)
    {
      regaddr = ARMV6M_NVIC_IPR(i);
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(KL_IRQ_SVCALL, up_svcall);
  irq_attach(KL_IRQ_HARDFAULT, up_hardfault);

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(KL_IRQ_NMI, kl_nmi);
  irq_attach(KL_IRQ_PENDSV, kl_pendsv);
  irq_attach(KL_IRQ_RESERVED, kl_reserved);
#endif

  kl_dumpnvic("initial", NR_IRQS);

  /* Initialize logic to support a second level of interrupt decoding for
   * configured pin interrupts.
   */

#ifdef CONFIG_GPIO_IRQ
  kl_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  irqenable();
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
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= KL_IRQ_EXTINT && irq < (KL_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ICER register to disable the
       * interrupt
       */

      putreg32((1 << (irq - KL_IRQ_EXTINT)), ARMV6M_NVIC_ICER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == KL_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, SYSTICK_CSR_ENABLE, 0);
    }

  kl_dumpnvic("disable", irq);
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
  /* This will be called on each interrupt exit whether the interrupt can be
   * enabled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= KL_IRQ_EXTINT && irq < (KL_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - KL_IRQ_EXTINT)), ARMV6M_NVIC_ISER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == KL_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, 0, SYSTICK_CSR_ENABLE);
    }

  kl_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  kl_clrpend(irq);
}
