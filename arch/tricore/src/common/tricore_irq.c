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
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Service request control register of an IRQ */

#define TRICORE_SRC(irq) \
  ((volatile uint32_t *)(TRICORE_SRC_BASE + (irq) * 4))

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
 * Private Data
 ****************************************************************************/

static spinlock_t g_irqlock = SP_UNLOCKED;
static int g_irqmap_count = 1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_src_init
 *
 * Description:
 *   Route a service request to a core with a priority and clear a pending
 *   request.  The enable bit is left as it is.
 *
 ****************************************************************************/

static void tricore_src_init(volatile uint32_t *src, int cpu, int srpn)
{
  uint32_t regval = *src;

  regval &= ~(TRICORE_SRC_SRPN_MASK | TRICORE_SRC_TOS_MASK);
#ifdef TRICORE_SRC_VM_MASK
  regval &= ~TRICORE_SRC_VM_MASK;
#endif
#ifdef TRICORE_SRC_CS
  if (up_cpu_index() == 6)
    {
      regval |= TRICORE_SRC_CS;
    }
#endif

  regval |= (srpn & TRICORE_SRC_SRPN_MASK) |
            (TRICORE_SRC_TOS_CPU(cpu) << TRICORE_SRC_TOS_SHIFT);
  *src = regval | TRICORE_SRC_CLRR;
}

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
/****************************************************************************
 * Name: tricore_gpsrinitialize
 *
 * Description:
 *   Perform gpsr initialization for the CPU
 *
 ****************************************************************************/

static void tricore_gpsrinitialize(void)
{
  int cpu = up_cpu_index();
  int i;

  /* Cpux gpsr init */

  for (i = 0; i < 6; i++)
    {
      int irq = TRICORE_GPSR_IRQNUM(i, cpu);

      tricore_src_init(TRICORE_SRC(irq), cpu, IRQ_TO_NDX(irq));
    }

  /* Cpucs gpsr init */

#ifndef CONFIG_ARCH_CHIP_TC3XX
  tricore_src_init(TRICORE_SRC(TRICORE_GPSR_IRQNUM(6, cpu)), cpu,
                   IRQ_TO_NDX(TRICORE_GPSR_IRQNUM(6, cpu)));
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
  __asm__ volatile ("enable" : : : "memory");
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
  volatile uint32_t *src = TRICORE_SRC(irq);

  /* Disable and clear, keep SRPN */

  *src = (*src & TRICORE_SRC_SRPN_MASK) | TRICORE_SRC_TOS_MASK;
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
  volatile uint32_t *src = TRICORE_SRC(irq);

  tricore_src_init(src, up_cpu_index(), IRQ_TO_NDX(irq));
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
  volatile uint32_t *src = TRICORE_SRC(irq);

  *src &= ~TRICORE_SRC_SRE;

  /* Only support interrupt routing mode 0,
   * so routing to the first cpu in cpuset.
   */

  tricore_src_init(src, ffs(cpuset) - 1, *src & TRICORE_SRC_SRPN_MASK);
  *src |= TRICORE_SRC_SRE;
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
  *TRICORE_SRC(irq) |= TRICORE_SRC_CLRR;
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
  volatile uint32_t *src = TRICORE_SRC(irq);
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
