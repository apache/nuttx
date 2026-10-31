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

#include "IfxSrc.h"
#include "IfxCpu.h"
#include "IfxInt_reg.h"

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

static const IfxSrc_Tos g_tricore_tos[] =
{
  IfxSrc_Tos_cpu0,
  IfxSrc_Tos_cpu1,
  IfxSrc_Tos_cpu2,
  IfxSrc_Tos_cpu3,
  IfxSrc_Tos_cpu4,
  IfxSrc_Tos_cpu5,
#ifdef CONFIG_ARCH_CHIP_TC4XX
  IfxSrc_Tos_csrm,
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
  volatile Ifx_SRC_SRCR *src;
  int cpu_idx;
  int i;

  cpu_idx = up_cpu_index();
  src = &SRC_GPSR00 + cpu_idx;

  /* Cpux gpsr init */

  for (i = 0; i < 6; i++)
    {
#ifdef CONFIG_ARCH_CHIP_TC3XX
      IfxSrc_init(src, g_tricore_tos[cpu_idx],
                  tricore_irq_to_srpn(TRICORE_SRC2IRQ(src)));
#else
      IfxSrc_init(src, g_tricore_tos[cpu_idx],
                  tricore_irq_to_srpn(TRICORE_SRC2IRQ(src)),
                  IfxSrc_VmId_none);
#endif

      src += TRICORE_SRCNUM_PER_GPSR;
    }

  /* Cpucs gpsr init */

#ifndef CONFIG_ARCH_CHIP_TC3XX
  src = &SRC_GPSR6_SR0 + cpu_idx;
  IfxSrc_init(src, g_tricore_tos[cpu_idx],
              tricore_irq_to_srpn(TRICORE_SRC2IRQ(src)),
              IfxSrc_VmId_none);
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

  for (i = TRICORE_IRQ_MIN_PRIORITY; i <= TRICORE_IRQ_MAX_PRIORITY; i++)
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
  volatile Ifx_SRC_SRCR *src = &SRC_CPU_CPU0_SB + irq;
  int srpn = src->B.SRPN;

  DEBUGASSERT(srpn >= TRICORE_IRQ_MIN_PRIORITY &&
              srpn <= TRICORE_IRQ_MAX_PRIORITY);

  g_srpn_to_irq[srpn] = irq;
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
  IfxCpu_enableInterrupts();
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
  volatile Ifx_SRC_SRCR *src = &SRC_CPU_CPU0_SB + irq;

  IfxSrc_disable(src);
  IfxSrc_deinit(src);
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
  volatile Ifx_SRC_SRCR *src = &SRC_CPU_CPU0_SB + irq;

#ifdef CONFIG_ARCH_CHIP_TC3XX
  IfxSrc_init(src, g_tricore_tos[up_cpu_index()], tricore_irq_to_srpn(irq));
#else
  IfxSrc_init(src, g_tricore_tos[up_cpu_index()], tricore_irq_to_srpn(irq),
              IfxSrc_VmId_none);
#endif

  IfxSrc_enable(src);
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
  volatile Ifx_SRC_SRCR *src = &SRC_CPU_CPU0_SB + irq;
  int irq_prio = tricore_irq_to_srpn(irq);

  IfxSrc_disable(src);

  /* Only support interrupt routing mode 0,
   * so routing to the first cpu in cpuset.
   */

#ifdef CONFIG_ARCH_CHIP_TC3XX
  IfxSrc_init(src, g_tricore_tos[ffs(cpuset) - 1], irq_prio);
#else
  IfxSrc_init(src, g_tricore_tos[ffs(cpuset) - 1], irq_prio,
              IfxSrc_VmId_none);
#endif

  IfxSrc_enable(src);
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
  volatile Ifx_SRC_SRCR *src = &SRC_CPU_CPU0_SB + irq;

  IfxSrc_clearRequest(src);
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
  volatile Ifx_INT_SRB *srb = &INT_SRB0 + up_cpu_index();

  srb->U = cpuset;
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

  DEBUGASSERT(priority >= TRICORE_IRQ_MIN_PRIORITY &&
              priority <= TRICORE_IRQ_MAX_PRIORITY);

  /* IRQ already mapped to the target SRPN, nothing to do */

  if (g_srpn_to_irq[priority] == irq)
    {
      return OK;
    }

  /* Check if target SRPN is already occupied by another IRQ */

  DEBUGASSERT(!g_srpn_to_irq[priority]);

  /* Clear old mapping if this IRQ had a different SRPN */

  for (i = TRICORE_IRQ_MIN_PRIORITY; i <= TRICORE_IRQ_MAX_PRIORITY; i++)
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
