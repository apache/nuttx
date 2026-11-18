/****************************************************************************
 * arch/tricore/src/common/tricore_irq.c
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
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/bits.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "tricore_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_color_intstack
 *
 * Description:
 *   Set the interrupt stack to a value so that later we can determine how
 *   much stack space was used by interrupt handling logic
 *
 ****************************************************************************/

#if defined(CONFIG_STACK_COLORATION) && CONFIG_ARCH_INTERRUPTSTACK > 15
static inline void tricore_color_intstack(void)
{
  uint32_t *ptr = (uint32_t *)g_intstackalloc;
  ssize_t size;

  for (size = (CONFIG_ARCH_INTERRUPTSTACK & ~15);
       size > 0;
       size -= sizeof(uint32_t))
    {
      *ptr++ = INTSTACK_COLOR;
    }
}
#else
#  define tricore_color_intstack()
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SMP) && defined(CONFIG_ARCH_CHIP_TC3X)
#  define TRICORE_IRQ_ROUTED_BITS   32
#  define TRICORE_IRQ_ROUTED_WORDS  (NR_IRQS / TRICORE_IRQ_ROUTED_BITS)
#endif

/* GPSR groups, one per sending CPU, set up for each receiving CPU.  With
 * SMP only the CPUs of the kernel send.
 */

#ifdef CONFIG_SMP
#  define TRICORE_GPSR_NSRC         CONFIG_SMP_NCPUS
#else
#  define TRICORE_GPSR_NSRC         6
#endif

#define TRICORE_SRC_TOS_GET(regval) \
  (((regval) & TRICORE_SRC_TOS_MASK) >> TRICORE_SRC_TOS_SHIFT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_irqlock = SP_UNLOCKED;
static int g_irqmap_count = 1;

/* On TC3x an SRC that nobody routed still reads TOS 0, the same as one
 * routed to CPU0, so a bitmap remembers which ones up_affinity_irq()
 * routed.  On TC4x an SRC that nobody routed reads TOS 0xf.
 */

#if defined(CONFIG_SMP) && defined(CONFIG_ARCH_CHIP_TC3X)
static atomic_t g_irq_routed[TRICORE_IRQ_ROUTED_WORDS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_src_init
 *
 * Description:
 *   Route a service request to a service provider (TOS), a core or for
 *   example the DMA, with a priority and clear a pending request.  The
 *   enable bit is left as it is.
 *
 ****************************************************************************/

static void tricore_src_init(volatile uint32_t *src, int tos, int srpn)
{
  uint32_t regval = *src;

  regval &= ~(TRICORE_SRC_SRPN_MASK | TRICORE_SRC_TOS_MASK);
#ifdef TRICORE_SRC_VM_MASK
  regval &= ~TRICORE_SRC_VM_MASK;
#endif

  regval |= (srpn & TRICORE_SRC_SRPN_MASK) |
            ((tos << TRICORE_SRC_TOS_SHIFT) & TRICORE_SRC_TOS_MASK);
  *src = regval | TRICORE_SRC_CLRR;
}

/****************************************************************************
 * Name: tricore_irq_is_routed
 *
 * Description:
 *   Return true if the SRC of the IRQ has a CPU chosen for it, so that
 *   up_enable_irq() keeps its TOS instead of routing it to the caller.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static bool tricore_irq_is_routed(int irq)
{
  if (irq < 0 || irq >= NR_IRQS)
    {
      return false;
    }

#ifdef CONFIG_ARCH_CHIP_TC3X
  return (atomic_read(&g_irq_routed[irq / TRICORE_IRQ_ROUTED_BITS]) &
          BIT(irq % TRICORE_IRQ_ROUTED_BITS)) != 0;
#else
  return (*TRICORE_IRQ2SRC(irq) & TRICORE_SRC_TOS_MASK) !=
         TRICORE_SRC_TOS_MASK;
#endif
}

/****************************************************************************
 * Name: tricore_irq_set_routed
 *
 * Description:
 *   Remember that the SRC of the IRQ has a CPU chosen for it.
 *
 ****************************************************************************/

static void tricore_irq_set_routed(int irq)
{
#ifdef CONFIG_ARCH_CHIP_TC3X
  if (irq >= 0 && irq < NR_IRQS)
    {
      atomic_or(&g_irq_routed[irq / TRICORE_IRQ_ROUTED_BITS],
                BIT(irq % TRICORE_IRQ_ROUTED_BITS));
    }
#else
  UNUSED(irq);
#endif
}
#endif

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
/****************************************************************************
 * Name: tricore_gpsrinitialize
 *
 * Description:
 *   Perform gpsr initialization for the CPU
 *
 ****************************************************************************/

static void tricore_gpsrinit_cpu(int cpu)
{
  int tos = TRICORE_SRC_TOS_CPU(cpu);
  int irq;
  int i;

  /* Cpux gpsr init */

  for (i = 0; i < TRICORE_GPSR_NSRC; i++)
    {
      irq = TRICORE_GPSR_IRQNUM(i, cpu);
      tricore_src_init(TRICORE_IRQ2SRC(irq), tos, IRQ_TO_NDX(irq));
#ifdef CONFIG_SMP
      tricore_irq_set_routed(irq);
#endif
    }

  /* Cpucs gpsr init */

#if !defined(CONFIG_ARCH_CHIP_TC3X) && !defined(CONFIG_SMP)
  tricore_src_init(TRICORE_IRQ2SRC(TRICORE_GPSR_IRQNUM(6, cpu)), tos,
                   IRQ_TO_NDX(TRICORE_GPSR_IRQNUM(6, cpu)));
#endif
}

static void tricore_gpsrinitialize(void)
{
#ifdef CONFIG_SMP
  int cpu;

  for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      tricore_gpsrinit_cpu(cpu);
    }
#else
  tricore_gpsrinit_cpu(up_cpu_index());
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void)
{
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  tricore_mtcr(TRICORE_CPU_ICR,
               tricore_mfcr(TRICORE_CPU_ICR) & ~TRICORE_ICR_CCPN_MASK);
  UP_ISB();
#endif

  __asm__ __volatile__ ("enable" : : : "memory");
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
  tricore_gpsrinitialize();
#endif

  tricore_color_intstack();
  up_irq_enable();
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
  volatile uint32_t *src = TRICORE_IRQ2SRC(irq);

#ifdef CONFIG_SMP
  *src &= ~TRICORE_SRC_SRE;
#else
  /* Disable and clear, keep SRPN */

  *src = (*src & TRICORE_SRC_SRPN_MASK) | TRICORE_SRC_TOS_MASK;
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
  volatile uint32_t *src = TRICORE_IRQ2SRC(irq);
  int tos = TRICORE_SRC_TOS_CPU(up_cpu_index());

#ifdef CONFIG_SMP
  if (tricore_irq_is_routed(irq))
    {
      tos = TRICORE_SRC_TOS_GET(*src);
    }
#endif

  tricore_src_init(src, tos, IRQ_TO_NDX(irq));
  *src |= TRICORE_SRC_SRE;
}

/****************************************************************************
 * Name: up_affinity_irq
 *
 * Description:
 *   Set an IRQ affinity by software.
 *
 ****************************************************************************/

void up_affinity_irq(int irq, cpu_set_t cpuset)
{
  volatile uint32_t *src = TRICORE_IRQ2SRC(irq);
#ifdef CONFIG_SMP
  uint32_t enabled;
  int cpu;

  cpu = ffs(cpuset) - 1;
  if (irq < 0 || irq >= NR_IRQS || cpu < 0 || cpu >= CONFIG_SMP_NCPUS)
    {
      return;
    }

  enabled = *src & TRICORE_SRC_SRE;
  *src &= ~TRICORE_SRC_SRE;
  *src = (*src & ~TRICORE_SRC_TOS_MASK) |
         ((TRICORE_SRC_TOS_CPU(cpu) << TRICORE_SRC_TOS_SHIFT) &
          TRICORE_SRC_TOS_MASK);
  tricore_irq_set_routed(irq);
  *src |= enabled;
#else
  *src &= ~TRICORE_SRC_SRE;

  /* Only support interrupt routing mode 0,
   * so routing to the first cpu in cpuset.
   */

  tricore_src_init(src, TRICORE_SRC_TOS_CPU(ffs(cpuset) - 1),
                   *src & TRICORE_SRC_SRPN_MASK);
  *src |= TRICORE_SRC_SRE;
#endif
}

/****************************************************************************
 * Name: tricore_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void tricore_ack_irq(int irq)
{
  *TRICORE_IRQ2SRC(irq) |= TRICORE_SRC_CLRR;
}

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software. May not be supported by all architectures.
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  *(volatile uint32_t *)(TRICORE_INT_SRB_BASE + up_cpu_index() * 4) =
    cpuset;
}
#endif

/****************************************************************************
 * Name: up_irq_to_ndx
 *
 * Description:
 *   Irq to ndx
 *
 ****************************************************************************/

int up_irq_to_ndx(int irq)
{
  volatile uint32_t *src = TRICORE_IRQ2SRC(irq);
  irqstate_t flags;
  int ndx;

  ndx = *src & TRICORE_SRC_SRPN_MASK;
  if (ndx != 0)
    {
      return ndx;
    }

  flags = spin_lock_irqsave(&g_irqlock);
  ndx = *src & TRICORE_SRC_SRPN_MASK;
  if (ndx == 0)
    {
      ndx = g_irqmap_count++;
      g_irqrevmap[ndx] = irq;
      *src = (*src & ~TRICORE_SRC_SRPN_MASK) | ndx;
    }

  spin_unlock_irqrestore(&g_irqlock, flags);

  return ndx;
}
