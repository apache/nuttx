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

static const uint8_t g_tricore_cpu2tos[] =
{
  TRICORE_TOS_CPU0,
  TRICORE_TOS_CPU1,
  TRICORE_TOS_CPU2,
  TRICORE_TOS_CPU3,
  TRICORE_TOS_CPU4,
  TRICORE_TOS_CPU5,
#ifdef CONFIG_ARCH_CHIP_TC4XX
  TRICORE_TOS_CSRM,
#endif
};

static irq_t g_srpn_to_irq[TRICORE_IRQ_MAX_PRIORITY + 1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  int irq;
  int i;

  /* Cpux gpsr init */

  for (i = 0; i < 6; i++)
    {
      irq = TRICORE_GPSR_IRQNUM(i, cpu);
      tricore_src_init(irq, g_tricore_cpu2tos[cpu],
                       tricore_irq_to_srpn(irq));
    }

  /* Cpucs gpsr init */

#ifdef CONFIG_HAVE_SECURITY_CORE
  irq = TRICORE_GPSR_IRQNUM(6, cpu);
  tricore_src_init(irq, g_tricore_cpu2tos[cpu], tricore_irq_to_srpn(irq));
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_irq_to_srpn
 *
 * Description:
 *   Get the SRPN for an IRQ. If not yet assigned, auto-allocate the lowest
 *   available priority.
 *
 *   Lookup searches the full range [MIN, MAX].
 *   Allocation range depends on CONFIG_ARCH_HIPRI_INTERRUPT:
 *     - Disabled: [MIN, MAX]
 *     - Enabled:  [MIN, DISABLE_PRIORITY], SRPNs above DISABLE_PRIORITY
 *       are reserved for high-priority interrupts via up_prioritize_irq().
 *
 ****************************************************************************/

int tricore_irq_to_srpn(int irq)
{
  int avail_srpn = 0;
  int i;

  /* Scheduling GPSR always gets the lowest priority */

#ifdef CONFIG_ARCH_IRQPRIO
  if (irq == TRICORE_SCHEDIRQ)
    {
      g_srpn_to_irq[TRICORE_IRQ_MIN_PRIORITY] = irq;

      return TRICORE_IRQ_MIN_PRIORITY;
    }
#endif

  /* Lookup: search the full SRPN range [MIN + 1, MAX] */

  for (i = TRICORE_IRQ_MIN_PRIORITY + 1; i <= TRICORE_IRQ_MAX_PRIORITY; i++)
    {
      if (g_srpn_to_irq[i] == irq)
        {
          return i;
        }

      /* Record first free slot within allocation range */

      if (!avail_srpn && !g_srpn_to_irq[i]
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
          && i <= TRICORE_IRQ_DISABLE_PRIORITY
#endif
         )
        {
          avail_srpn = i;
        }
    }

  if (avail_srpn)
    {
      g_srpn_to_irq[avail_srpn] = irq;
      return avail_srpn;
    }

  /* Should never get here: SRPN pool exhausted */

  DEBUGPANIC();
  return 0;
}

/****************************************************************************
 * Name: tricore_srpn_to_irq
 *
 * Description:
 *   Convert SRPN to IRQ number using the mapping table.
 *
 ****************************************************************************/

int tricore_srpn_to_irq(int srpn)
{
  DEBUGASSERT(srpn >= TRICORE_IRQ_MIN_PRIORITY &&
              srpn <= TRICORE_IRQ_MAX_PRIORITY);

  return g_srpn_to_irq[srpn];
}

/****************************************************************************
 * Name: tricore_register_irq_srpn
 *
 * Description:
 *   Register the SRPN->IRQ mapping for an SRC that was configured
 *   externally (e.g., by MCAL IrqDma_Init / IrqAsclin_Init). Reads
 *   SRPN directly from the SRC register and records it in the NuttX
 *   dispatch table.
 *
 * Note:
 *   This function conflicts with `tricore_irq_to_srpn`, therefore only one
 *   of these two functions can be chosen.
 *
 ****************************************************************************/

void tricore_register_irq_srpn(int irq)
{
  uintptr_t src = TRICORE_IRQ2SRC(irq);
  int srpn = (getreg32(src) & SRCR_SRPN_MASK) >> SRCR_SRPN_SHIFT;

  DEBUGASSERT(srpn >= TRICORE_IRQ_MIN_PRIORITY &&
              srpn <= TRICORE_IRQ_MAX_PRIORITY);

  g_srpn_to_irq[srpn] = irq;
}

/****************************************************************************
 * Name: tricore_src_enable
 ****************************************************************************/

static inline_function void tricore_src_enable(int irq)
{
  uintptr_t src = TRICORE_IRQ2SRC(irq);

  putreg32(getreg32(src) | SRCR_SRE, src);
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void)
{
  __asm__ volatile ("enable" ::: "memory");
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

#ifdef CONFIG_ARCH_IRQPRIO
  up_enable_irq(TRICORE_SCHEDIRQ);
#endif

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
  /* Clear SRE and reset TOS to "no service provider" */

  putreg32(SRCR_TOS_MASK, TRICORE_IRQ2SRC(irq));
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
  tricore_src_init(irq, g_tricore_cpu2tos[up_cpu_index()],
                   tricore_irq_to_srpn(irq));
  tricore_src_enable(irq);
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
  uintptr_t src = TRICORE_IRQ2SRC(irq);
  int irq_prio = (getreg32(src) & SRCR_SRPN_MASK) >> SRCR_SRPN_SHIFT;

  putreg32(getreg32(src) & ~SRCR_SRE, src);

  /* Only support interrupt routing mode 0,
   * so routing to the first cpu in cpuset.
   */

  tricore_src_init(irq, g_tricore_cpu2tos[ffs(cpuset) - 1], irq_prio);
  tricore_src_enable(irq);
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
  uintptr_t src = TRICORE_IRQ2SRC(irq);

  putreg32(getreg32(src) | SRCR_CLRR, src);
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
  putreg32(cpuset, TRICORE_INT_SRB0_ADDR +
                   up_cpu_index() * TRICORE_SRC_SPACING);
}
#endif

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority (SRPN) of an IRQ. This is the only way to assign an
 *   SRPN above TRICORE_IRQ_DISABLE_PRIORITY when
 *   CONFIG_ARCH_HIPRI_INTERRUPT is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  int i;

  DEBUGASSERT(priority > TRICORE_IRQ_MIN_PRIORITY &&
              priority <= TRICORE_IRQ_MAX_PRIORITY &&
              irq != TRICORE_SCHEDIRQ);

  /* IRQ already mapped to the target SRPN, nothing to do */

  if (g_srpn_to_irq[priority] == irq)
    {
      return OK;
    }

  /* Check if target SRPN is already occupied by another IRQ */

  DEBUGASSERT(!g_srpn_to_irq[priority]);

  /* Clear old mapping if this IRQ had a different SRPN */

  for (i = TRICORE_IRQ_MIN_PRIORITY + 1; i <= TRICORE_IRQ_MAX_PRIORITY; i++)
    {
      if (g_srpn_to_irq[i] == irq)
        {
          g_srpn_to_irq[i] = 0;
          break;
        }
    }

  g_srpn_to_irq[priority] = irq;

  return OK;
}
#endif

/****************************************************************************
 * Name: tricore_src_init
 *
 * Description:
 *   Route the service request of 'irq' to 'tos' with priority 'srpn'.
 *   The request is left disabled and any pending request is cleared.
 *
 ****************************************************************************/

void tricore_src_init(int irq, uint8_t tos, uint8_t srpn)
{
  uintptr_t src = TRICORE_IRQ2SRC(irq);
  uint32_t regval = getreg32(src);

  /* Clear VM (virtual machine) and SRE (service request enable). */

  regval &= ~(SRCR_VM_MASK | SRCR_SRE);

  /* Update tos (target cpu) and srpn (priority) fields. */

  regval &= ~(SRCR_TOS_MASK | SRCR_SRPN_MASK);
  regval |= (srpn << SRCR_SRPN_SHIFT) | (tos << SRCR_TOS_SHIFT);

  /* Cyber security */

#ifdef CONFIG_TRICORE_CSRM
  regval |= SRCR_CS;
#endif

  /* Clear pending request */

  regval |= SRCR_CLRR;

  putreg32(regval, src);
}
