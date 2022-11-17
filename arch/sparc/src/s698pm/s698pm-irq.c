/****************************************************************************
 * arch/sparc/src/s698pm/s698pm-irq.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/irq.h>

#include "sparc_internal.h"
#include "s698pm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7

#if defined(CONFIG_SMP)
#  define INTSTACK_ALLOC (CONFIG_SMP_NCPUS * INTSTACK_SIZE)
#else
#  define INTSTACK_ALLOC (INTSTACK_SIZE)
#endif

#endif

/* IRQ to CPU and CPU interrupts mapping:
 *
 * Encoding: CCCIIIII
 *  C: CPU that enabled the interrupt (0 ~ 3).
 *  I: Associated CPU interrupt (0 ~ 31).
 */

#define IRQ_UNMAPPED      0xff
#define IRQ_GETCPU(m)     (((m) & 0xe0) >> 0x05)
#define IRQ_GETCPUINT(m)  ((m) & 0x1f)
#define IRQ_MKMAP(c, i)   (((c) << 0x05) | (i))

/****************************************************************************
 * Public Data
 ****************************************************************************/

static volatile uint8_t g_irqmap[NR_IRQS];

#if CONFIG_ARCH_INTERRUPTSTACK > 7
/* In the SMP configuration, we will need custom interrupt stacks.
 * These definitions provide the aligned stack allocations.
 */

static uint64_t g_intstack_alloc[INTSTACK_ALLOC >> 3];

/* These definitions provide the "top" of the push-down stacks. */

uintptr_t g_cpu_intstack_top[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_intstack_alloc + INTSTACK_SIZE,
#if defined(CONFIG_SMP)

#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_intstack_alloc + (2 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_intstack_alloc + (3 * INTSTACK_SIZE),
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_intstack_alloc + (4 * INTSTACK_SIZE),
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */

#endif /* defined(CONFIG_SMP) */
};
#endif /* if CONFIG_ARCH_INTERRUPTSTACK > 7 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static int up_prioritize_irq(int irq, int priority);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int irq = 0;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      g_irqmap[irq] = IRQ_UNMAPPED;
    }

  /* Initialize CPU interrupts */

  s698pm_cpuint_initialize();

  /* Set interrupts priority */

  for (irq = S698PM_IRQ_FIRST_INT; irq <= S698PM_IRQ_LAST_INT; irq++)
    {
      /* Set all interrupts to the default (low) priority */

      (void)up_prioritize_irq(irq, 0);
    }

  /* Attach software interrupts */

  irq_attach(S698PM_IRQ_SW_SYSCALL_TA0, sparc_swint0, NULL);
  irq_attach(S698PM_IRQ_SW_SYSCALL_TA8, sparc_swint1, NULL);

  /* And finally, enable cpu interrupts */

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name:  s698pm_cpuint_initialize
 *
 * Description:
 *   Initialize CPU interrupts
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s698pm_cpuint_initialize(void)
{
  uintptr_t regaddr;
  int cpu = 0;

#ifdef CONFIG_SMP
  /* Which CPU are we initializing */

  cpu = up_cpu_index();
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);
#endif

  /* get Interrupt_Mask reg address of cpu */

  regaddr = S698PM_IRQREG_P0_MASK + 4 * cpu;

  /* Disable all CPU interrupts on this CPU */

  putreg32(0, regaddr);

#if defined CONFIG_SMP
  /* Attach IPI interrupts */

  irq_attach(S698PM_IPI_IRQ, s698pm_pause_handler, NULL);

  (void)s698pm_setup_irq(cpu, S698PM_IPI_IRQ, 0);

  /* And enable the IPI interrupt */

  up_enable_irq(S698PM_IPI_IRQ);
#endif

  return OK;
}

/****************************************************************************
 * Name:  s698pm_setup_irq
 *
 * Description:
 *   This function sets up the IRQ. It allocates a CPU interrupt of the given
 *   priority andattaches it to the given irq.
 *
 * Input Parameters:
 *   cpu      - The CPU to receive the interrupt 0~3
 *   irq      - The irq number from irq.h to be assigned to a EXT interrupt.
 *   priority - Interrupt's priority (0~1).
 *
 * Returned Value:
 *   The allocated CPU interrupt on success, a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int s698pm_setup_irq(int cpu, int irq, int priority)
{
  irqstate_t irqstate;
  int cpuint;

  irqstate = enter_critical_section();

  if (irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT)
    {
      cpuint = irq - S698PM_IRQ_FIRST_INT + 1;
    }
  else if (irq > S698PM_IRQ_LAST && irq < NR_IRQS)
    {
      /* Second level interrupt share a IRQ and connnect to a interrupt 14
       * of first level interrupt(that is IRQ 0x1E), We extend Second level
       * interrupt to IRQ 0x100~0x10F. because first level interrupt in the
       * interrupt reister is bit 0~15 and second level interrupt is bit 16
       * ~31, so cpuint of second level interrupt is irq - 256 +
       * S698PM_EXTENDED_START
       */

      cpuint = irq - 256 + S698PM_EXTENDED_START;
    }
  else
    {
      cpuint = -1;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint < S698PM_CPUINT_MAX);
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  g_irqmap[irq] = IRQ_MKMAP(cpu, cpuint);
  (void)up_prioritize_irq(irq, priority);

  leave_critical_section(irqstate);

  return cpuint;
}

/****************************************************************************
 * Name:  s698pm_teardown_irq
 *
 * Description:
 *   This function undoes the operations done by s698pm_setup_irq.
 *   It detaches a ext interrupt from a CPU irq.
 *
 * Input Parameters:
 *   irq      - The irq number from irq.h to be assigned to a EXT interrupt.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void s698pm_teardown_irq(int irq)
{
  irqstate_t irqstate;

  irqstate = enter_critical_section();

  g_irqmap[irq] = IRQ_UNMAPPED;

  leave_critical_section(irqstate);
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
  uintptr_t regaddr;
  int cpu = IRQ_GETCPU(g_irqmap[irq]);
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  if (g_irqmap[irq] == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint < S698PM_CPUINT_MAX);
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if ((irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT) ||
      (irq > S698PM_IRQ_LAST && irq < NR_IRQS))
    {
      /* get Interrupt_Mask reg address of cpu */

      regaddr = S698PM_IRQREG_P0_MASK + 4 * cpu;

      /* Disable the interrupt by clearing the associated bit in the
       * Interrupt_Mask
       */

      modifyreg32(regaddr, 1 << cpuint, 0);
    }
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
  uintptr_t regaddr;
  int cpu = IRQ_GETCPU(g_irqmap[irq]);
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  if (g_irqmap[irq] == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint < S698PM_CPUINT_MAX);
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if ((irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT) ||
      (irq > S698PM_IRQ_LAST && irq < NR_IRQS))
    {
      /* get Interrupt_Mask reg address of cpu */

      regaddr = S698PM_IRQREG_P0_MASK + 4 * cpu;

      /* Disable the interrupt by clearing the associated bit in the
       * Interrupt_Mask
       */

      modifyreg32(regaddr, 1 << cpuint, 1 << cpuint);
    }
}

/****************************************************************************
 * Name: sparc_pending_irq
 *
 * Description:
 *   Return true if the interrupt is pending and unmasked.
 *
 ****************************************************************************/

bool sparc_pending_irq(int irq)
{
  uintptr_t regaddr;
  uint16_t regval;
  uint16_t regval1;
  uint16_t regval2;
  int cpu = IRQ_GETCPU(g_irqmap[irq]);
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  /* Test if the interrupt is pending by reading both the MASK and ACK
   * register. Return true if the bit associated with the irq is both
   * pending the ACK and enabled in the MASK.
   */

  if (g_irqmap[irq] == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return false;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint < S698PM_CPUINT_MAX);
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS);

  if ((irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT) ||
      (irq > S698PM_IRQ_LAST && irq < NR_IRQS))
    {
      /* get interrupt mask register address of cpu */

      regaddr = S698PM_IRQREG_P0_MASK + 4 * cpu;
      regval1 = getreg32(regaddr);

      /* get interrupt pend register address of cpu */

      regaddr = S698PM_IRQREG_IPEND;
      regval2 = getreg32(regaddr);

      /* Get the set of unmasked, pending interrupts.   */

      regval  = regval1 & regval2;

      /* Return true if the interrupt is pending and unmask. */

      return (regval & (1 << cpuint)) != 0;
    }

  return false;
}

/****************************************************************************
 * Name: sparc_clrpend_irq
 *
 * Description:
 *   Clear any pending interrupt
 *
 ****************************************************************************/

void sparc_clrpend_irq(int irq)
{
  int cpuint = IRQ_GETCPUINT(g_irqmap[irq]);

  /* Test if the interrupt is pending by reading both the MASK and ACK
   * register. Return true if the bit associated with the irq is both
   * pending the ACK and enabled in the MASK.
   */

  if (g_irqmap[irq] == IRQ_UNMAPPED)
    {
      /* This interrupt is already disabled. */

      return;
    }

  DEBUGASSERT(cpuint >= 0 && cpuint < S698PM_CPUINT_MAX);

  /* Acknowledge the interrupt by clearing the associated bit in the ITP
   * register.  It is necessary to do this BEFORE lowering the interrupt
   * priority level otherwise recursive interrupts would occur.
   */

  if ((irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT) ||
      (irq > S698PM_IRQ_LAST && irq < NR_IRQS))
    {
      /* written with a ‘1’, in Interrupt Clear Register
       * will clear the corresponding bit(s) in the interrupt pending
       * register
       */

      putreg32(1 << cpuint, S698PM_IRQREG_ICLEAR);
    }

  return;
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 * It is possible to change the priority level of an interrupt using the two
 * priority levels from the interrupt mask and priority register (ITMP).
 * Each interrupt can be assigned to one of two levels as programmed in the
 * Interrupt mask and priority register. Level 1 has higher priority than
 * level 0. Within each level the interrupts are prioritised
 ****************************************************************************/

#ifndef CONFIG_ARCH_IRQPRIO
static
#endif
int up_prioritize_irq(int irq, int priority)
{
  int shift;

  /* Don't allow this function to be used for disabling interrupts. */

  DEBUGASSERT((unsigned)irq < NR_IRQS && (unsigned)(priority) < 2);

  if (irq >= S698PM_IRQ_FIRST_INT && irq <= S698PM_IRQ_LAST_INT)
    {
      shift = irq - S698PM_IRQ_FIRST_INT + 1;

      /* Set the new interrupt priority */

      putreg32(1 << shift, S698PM_IRQREG_ILEVEL);
    }
  else
    {
      /* Value out of range.. just ignore */

      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: sparc_intstack_top
 *
 * Description:
 *   Return a pointer to the top the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t sparc_intstack_top(void)
{
#if defined(CONFIG_SMP)
  return g_cpu_intstack_top[up_cpu_index()];
#else
  return g_cpu_intstack_top[0];
#endif
}
#endif

/****************************************************************************
 * Name: sparc_intstack_alloc
 *
 * Description:
 *   Return a pointer to the "alloc" the correct interrupt stack allocation
 *   for the current CPU.
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 7
uintptr_t sparc_intstack_alloc(void)
{
#if defined(CONFIG_SMP)
  return g_cpu_intstack_top[up_cpu_index()] - INTSTACK_SIZE;
#else
  return g_cpu_intstack_top[0] - INTSTACK_SIZE;
#endif
}
#endif
