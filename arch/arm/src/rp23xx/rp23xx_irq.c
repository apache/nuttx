/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_irq.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include "ram_vectors.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | NVIC_SYSH_PRIORITY_DEFAULT)

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
#  define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
extern void rp23xx_send_irqreq(int irqreq);
#endif

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint64_t g_intstack_alloc[INTSTACK_ALLOC >> 3];

/* These definitions provide the "top" of the push-down stacks. */

const uint32_t g_cpu_intstack_top[CONFIG_SMP_NCPUS] =
{
  (uint32_t)g_intstack_alloc + INTSTACK_SIZE,
#if CONFIG_SMP_NCPUS > 1
  (uint32_t)g_intstack_alloc + (2 * INTSTACK_SIZE),
#endif /* CONFIG_SMP_NCPUS > 1 */
};
#endif /* defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void rp23xx_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  ISER0:      %08x ICER0:   %08x\n",
          getreg32(NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ0_31_CLEAR));
  irqinfo("  ISER1:      %08x ICER1:   %08x\n",
          getreg32(NVIC_IRQ32_63_ENABLE), getreg32(NVIC_IRQ32_63_CLEAR));
  irqinfo("  ISPR0:      %08x ICPR0:   %08x\n",
          getreg32(NVIC_IRQ0_31_PEND), getreg32(NVIC_IRQ0_31_CLRPEND));
  irqinfo("  ISPR1:      %08x ICPR1:   %08x\n",
          getreg32(NVIC_IRQ32_63_PEND), getreg32(NVIC_IRQ32_63_CLRPEND));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY),
          getreg32(NVIC_IRQ20_23_PRIORITY),
          getreg32(NVIC_IRQ24_27_PRIORITY),
          getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY),
          getreg32(NVIC_IRQ36_39_PRIORITY),
          getreg32(NVIC_IRQ40_43_PRIORITY),
          getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY),
          getreg32(NVIC_IRQ52_55_PRIORITY),
          getreg32(NVIC_IRQ56_59_PRIORITY),
          getreg32(NVIC_IRQ60_63_PRIORITY));

  irqinfo("SYSCON:\n");
  irqinfo("  CPUID:      %08x\n",
          getreg32(NVIC_CPUID_BASE));
  irqinfo("  ICSR:       %08x AIRCR:  %08x\n",
          getreg32(NVIC_INTCTRL), getreg32(NVIC_AIRCR));
  irqinfo("  SCR:        %08x CCR:    %08x\n",
          getreg32(NVIC_SYSCON), getreg32(NVIC_CFGCON));
  irqinfo("  SHPR1:      %08x SHPR2:  %08x\n",
          getreg32(NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY));
  irqinfo("  SHPR3:      %08x\n",
          getreg32(NVIC_SYSH12_15_PRIORITY));

  leave_critical_section(flags);
}

#else
#  define rp23xx_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: rp23xx_nmi, rp23xx_busfault, rp23xx_usagefault, rp23xx_pendsv,
 *       rp23xx_dbgmonitor, rp23xx_pendsv, rp23xx_reserved
 *
 * Description:
 *   Handlers for various exceptions.  None are handled and all are fatal
 *   error conditions.  The only advantage these provided over the default
 *   unexpected interrupt handler is that they provide a diagnostic output.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int rp23xx_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int rp23xx_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int rp23xx_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Name: rp23xx_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

static inline void rp23xx_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval  = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

/****************************************************************************
 * Name: rp23xx_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.
 *
 ****************************************************************************/

static inline void rp23xx_clrpend(int irq)
{
  /* This will be called on each interrupt exit whether the interrupt can be
   * enabled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

  /* Check for an external interrupt */

  if (irq >= RP23XX_IRQ_EXTINT && irq < RP23XX_IRQ_NIRQS)
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      if (irq - RP23XX_IRQ_EXTINT < 32)
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT)), NVIC_IRQ0_31_CLRPEND);
        }
      else
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT - 32)),
                   NVIC_IRQ32_63_CLRPEND);
        }
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

  putreg32(0xffffffff, NVIC_IRQ0_31_CLEAR);
  putreg32(0xffffffff, NVIC_IRQ32_63_CLEAR);
  putreg32(0, NVIC_SYSTICK_CTRL);

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* Make sure that we are using the correct vector table.  The default
   * vector address is 0x0000:0000 but if we are executing code that is
   * positioned in SRAM or in external FLASH, then we may need to reset
   * the interrupt vector so that it refers to the table in SRAM or in
   * external FLASH.
   */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  arm_ramvec_initialize();
#endif

  /* Now set all of the interrupt lines to the default priority */

  for (i = 0; i < 12; i++)
    {
      regaddr = NVIC_IRQ_PRIORITY(i);
      putreg32(DEFPRIORITY32, regaddr);
    }

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(RP23XX_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(RP23XX_IRQ_HARDFAULT, arm_hardfault, NULL);

  rp23xx_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(RP23XX_IRQ_NMI, rp23xx_nmi, NULL);
  irq_attach(RP23XX_IRQ_PENDSV, rp23xx_pendsv, NULL);
  irq_attach(RP23XX_IRQ_RESERVED, rp23xx_reserved, NULL);
#endif

  rp23xx_dumpnvic("initial", NR_IRQS);

  /* And finally, enable interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
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

#ifdef CONFIG_SMP
  if (irq >= RP23XX_IRQ_EXTINT && irq != RP23XX_SIO_IRQ_FIFO &&
      this_cpu() != 0)
    {
      /* Must be handled by Core 0 */

      rp23xx_send_irqreq(-irq);
      return;
    }
#endif

  /* Check for an external interrupt */

  if (irq - RP23XX_IRQ_EXTINT >= RP23XX_IRQ_EXTINT && irq < RP23XX_IRQ_NIRQS)
    {
      /* Set the appropriate bit in the ICER register to disable the
       * interrupt
       */

      if (irq < 32)
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT)), NVIC_IRQ0_31_CLEAR);
        }
      else
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT - 32)),
                   NVIC_IRQ32_63_CLEAR);
        }
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == RP23XX_IRQ_SYSTICK)
    {
      modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_TICKINT, 0);
    }

  rp23xx_dumpnvic("disable", irq);
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
   * enambled or not.  So this assertion is necessarily lame.
   */

  DEBUGASSERT((unsigned)irq < NR_IRQS);

#ifdef CONFIG_SMP
  if (irq >= RP23XX_IRQ_EXTINT && irq != RP23XX_SIO_IRQ_FIFO &&
      this_cpu() != 0)
    {
      /* Must be handled by Core 0 */

      rp23xx_send_irqreq(irq);
      return;
    }
#endif

  /* Check for external interrupt */

  if (irq >= RP23XX_IRQ_EXTINT && irq < RP23XX_IRQ_NIRQS)
    {
      /* Set the appropriate bit in the ISER register to enable the
       * interrupt
       */

      if (irq - RP23XX_IRQ_EXTINT < 32)
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT)), NVIC_IRQ0_31_ENABLE);
        }
      else
        {
          putreg32((1 << (irq - RP23XX_IRQ_EXTINT - 32)),
                   NVIC_IRQ32_63_ENABLE);
        }
    }

  /* Handle processor exceptions.  Only SysTick can be disabled */

  else if (irq == RP23XX_IRQ_SYSTICK)
    {
      modifyreg32(NVIC_SYSTICK_CTRL, 0, NVIC_SYSTICK_CTRL_TICKINT);
    }

  rp23xx_dumpnvic("enable", irq);
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
  rp23xx_clrpend(irq);
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
  uint32_t regaddr;
  uint32_t regval;
  int shift;

  DEBUGASSERT(irq == RP23XX_IRQ_SVCALL ||
              irq == RP23XX_IRQ_PENDSV ||
              irq == RP23XX_IRQ_SYSTICK ||
             (irq >= RP23XX_IRQ_EXTINT && irq < NR_IRQS));
  DEBUGASSERT(priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);

  /* Check for external interrupt */

  if (irq >= RP23XX_IRQ_EXTINT && irq < RP23XX_IRQ_EXTINT + 32)
    {
      /* ARMV6M_NVIC_IPR() maps register IPR0-IPR7 with four settings per
       * register.
       */

      regaddr = ARMV6M_NVIC_IPR(irq >> 2);
      shift   = (irq & 3) << 3;
    }

  /* Handle processor exceptions.  Only SVCall, PendSV, and SysTick can be
   * reprioritized.  And we will not permit modification of SVCall through
   * this function.
   */

  else if (irq == RP23XX_IRQ_PENDSV)
    {
      regaddr = ARMV6M_SYSCON_SHPR2;
      shift   = SYSCON_SHPR3_PRI_14_SHIFT;
    }
  else if (irq == RP23XX_IRQ_SYSTICK)
    {
      regaddr = ARMV6M_SYSCON_SHPR2;
      shift   = SYSCON_SHPR3_PRI_15_SHIFT;
    }
  else
    {
      return ERROR;
    }

  /* Set the priority */

  regval  = getreg32(regaddr);
  regval &= ~((uint32_t)0xff << shift);
  regval |= ((uint32_t)priority << shift);
  putreg32(regval, regaddr);

  rp23xx_dumpnvic("prioritize", irq);
  return OK;
}
#endif

/****************************************************************************
 * Name: up_get_intstackbase
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t up_get_intstackbase(int cpu)
{
  return g_cpu_intstack_top[cpu] - INTSTACK_SIZE;
}
#endif
