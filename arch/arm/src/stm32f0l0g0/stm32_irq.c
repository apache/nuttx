/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_irq.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"

/* #include "stm32_irq.h" */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | NVIC_SYSH_PRIORITY_DEFAULT)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void stm32_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  ISER:       %08x ICER:   %08x\n",
          getreg32(ARMV6M_NVIC_ISER), getreg32(ARMV6M_NVIC_ICER));
  irqinfo("  ISPR:       %08x ICPR:   %08x\n",
          getreg32(ARMV6M_NVIC_ISPR), getreg32(ARMV6M_NVIC_ICPR));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(ARMV6M_NVIC_IPR0), getreg32(ARMV6M_NVIC_IPR1),
          getreg32(ARMV6M_NVIC_IPR2), getreg32(ARMV6M_NVIC_IPR3));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(ARMV6M_NVIC_IPR4), getreg32(ARMV6M_NVIC_IPR5),
          getreg32(ARMV6M_NVIC_IPR6), getreg32(ARMV6M_NVIC_IPR7));

  irqinfo("SYSCON:\n");
  irqinfo("  CPUID:      %08x\n",
          getreg32(ARMV6M_SYSCON_CPUID));
  irqinfo("  ICSR:       %08x AIRCR:  %08x\n",
          getreg32(ARMV6M_SYSCON_ICSR), getreg32(ARMV6M_SYSCON_AIRCR));
  irqinfo("  SCR:        %08x CCR:    %08x\n",
          getreg32(ARMV6M_SYSCON_SCR), getreg32(ARMV6M_SYSCON_CCR));
  irqinfo("  SHPR2:      %08x SHPR3:  %08x\n",
          getreg32(ARMV6M_SYSCON_SHPR2), getreg32(ARMV6M_SYSCON_SHPR3));

  leave_critical_section(flags);
}

#else
#  define stm32_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: stm32_nmi, stm32_busfault, stm32_usagefault, stm32_pendsv,
 *       stm32_dbgmonitor, stm32_pendsv, stm32_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int stm32_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int stm32_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int stm32_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.
 *
 ****************************************************************************/

static inline void stm32_clrpend(int irq)
{
  /* This will be called on each interrupt exit whether the interrupt can be
   * enambled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= STM32_IRQ_EXTINT && irq < (STM32_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - STM32_IRQ_EXTINT)), ARMV6M_NVIC_ICPR);
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

  CURRENT_REGS = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(STM32_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(STM32_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(STM32_IRQ_NMI, stm32_nmi, NULL);
  irq_attach(STM32_IRQ_PENDSV, stm32_pendsv, NULL);
  irq_attach(STM32_IRQ_RESERVED, stm32_reserved, NULL);
#endif

  stm32_dumpnvic("initial", NR_IRQS);

  /* Initialize logic to support a second level of interrupt decoding for
   * configured pin interrupts.
   */

#ifdef CONFIG_STM32F0L0G0_GPIOIRQ
  stm32_gpioirqinitialize();
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  up_irq_enable();
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

  if (irq >= STM32_IRQ_EXTINT && irq < (STM32_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ICER register to disable the
       * interrupt
       */

      putreg32((1 << (irq - STM32_IRQ_EXTINT)), ARMV6M_NVIC_ICER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == STM32_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, SYSTICK_CSR_ENABLE, 0);
    }

  stm32_dumpnvic("disable", irq);
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

  if (irq >= STM32_IRQ_EXTINT && irq < (STM32_IRQ_EXTINT + 32))
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      putreg32((1 << (irq - STM32_IRQ_EXTINT)), ARMV6M_NVIC_ISER);
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == STM32_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, 0, SYSTICK_CSR_ENABLE);
    }

  stm32_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  stm32_clrpend(irq);
}
