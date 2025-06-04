/****************************************************************************
 * arch/tricore/src/common/tricore_csa.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/tls.h>
#include <arch/barriers.h>
#include <arch/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_alloc_csa
 ****************************************************************************/

uintptr_t *tricore_alloc_csa(struct tcb_s *tcb, uintptr_t pc,
                             uintptr_t sp, uintptr_t psw,
                             bool irqsave)
{
  uintptr_t *plcsa;
  uintptr_t *pucsa;

  /* If the task corresponding to tcb is in running state, it must be in
   * interrupt context to ensure the correct management of csa.
   */

  DEBUGASSERT(tcb != nxsched_self() || up_interrupt_context());

  /* Init new uppercsa and lowcsa */

  if (tcb->xcp.regs == NULL)
    {
      pucsa = tcb->stack_base_ptr;
    }
  else
    {
      pucsa = tcb->xcp.regs + TC_CONTEXT_REGS;
    }

  plcsa = pucsa + TC_CONTEXT_REGS;

  memset(pucsa, 0, TC_CONTEXT_SIZE);
  memset(plcsa, 0, TC_CONTEXT_SIZE);

  /* Save the function entry point */

  plcsa[REG_LPC] = pc;
  pucsa[REG_SP]  = sp;
  pucsa[REG_PSW] = psw;

  plcsa[REG_LPCXI] = (PCXI_UL | tricore_addr2csa(pucsa));

  /* Determine whether to enable interrupt */

  if (!irqsave)
    {
      plcsa[REG_LPCXI] |= PCXI_PIE;
    }

  return plcsa;
}

/****************************************************************************
 * Name: tricore_reclaim_csa
 ****************************************************************************/

void tricore_reclaim_csa(uintptr_t pcxi)
{
  uintptr_t *tail;

  while ((pcxi & FCX_FREE) != 0)
    {
      tail = tricore_csa2addr(pcxi);
      pcxi = tail[0];
      tail[0] = tricore_addr2csa(tail +TC_CONTEXT_REGS);
    }
}

/****************************************************************************
 * Name: tricore_region_csainit
 ****************************************************************************/

void tricore_region_csainit(void *regionbase, size_t nbytes)
{
  uintptr_t *cur;
  uintptr_t *next;
  uintptr_t *end;

  cur = (uintptr_t *)ALIGN_UP((uintptr_t)regionbase, TC_CONTEXT_SIZE);
  end = (uintptr_t *)ALIGN_DOWN((uintptr_t)regionbase + nbytes,
                                TC_CONTEXT_SIZE);

  for (next = cur + TC_CONTEXT_REGS; next < end;
       next += TC_CONTEXT_REGS)
    {
      cur[REG_UPCXI] = tricore_addr2csa(next);
      cur = next;
    }

  cur[REG_UPCXI] = 0;
}
