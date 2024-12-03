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
#include <nuttx/tls.h>
#include <arch/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_alloc_csa
 ****************************************************************************/

uintptr_t *tricore_alloc_csa(uintptr_t pc, uintptr_t sp,
                             uintptr_t psw, bool irqsave)
{
  uintptr_t *plcsa;
  uintptr_t *pucsa;

  plcsa = (uintptr_t *)tricore_csa2addr(__mfcr(CPU_FCX));

  /* DSYNC instruction should be executed immediately prior to the MTCR */

  __dsync();

  pucsa = (uintptr_t *)tricore_csa2addr(plcsa[REG_UPCXI]);

  __mtcr(CPU_FCX, pucsa[REG_UPCXI]);

  /* ISYNC instruction executed immediately following MTCR */

  __isync();

  memset(pucsa, 0, XCPTCONTEXT_SIZE);
  memset(plcsa, 0, XCPTCONTEXT_SIZE);

  pucsa[REG_SP]  = sp;
  pucsa[REG_PSW] = psw;

  /* Save the task entry point */

  pucsa[REG_UPC] = pc;
  plcsa[REG_LPC] = pc;

  plcsa[REG_LPCXI] = (PCXI_UL | tricore_addr2csa(pucsa));

  if (!irqsave)
    {
      plcsa[REG_LPCXI] |= PCXI_PIE;
    }

  return (uintptr_t *)tricore_addr2csa(plcsa);
}

/****************************************************************************
 * Name: tricore_reclaim_csa
 ****************************************************************************/

void tricore_reclaim_csa(uintptr_t pcxi)
{
  uintptr_t head, tail, free;
  uintptr_t *next;

  /* A pointer to the first CSA in the list of CSAs consumed by the task is
   * stored in the first element of the tasks TCB structure (where the stack
   * pointer would be on a traditional stack based architecture).
   */

  head = pcxi & FCX_FREE;

  /* Mask off everything in the CSA link field other than the address.  If
   * the address is NULL, then the CSA is not linking anywhere and there is
   * nothing to do.
   */

  tail = head;

  /* Convert the link value to contain just a raw address and store this
   * in a local variable.
   */

  next = tricore_csa2addr(tail);

  /* Iterate over the CSAs that were consumed as part of the task.  The
   * first field in the CSA is the pointer to then next CSA.  Mask off
   * everything in the pointer to the next CSA, other than the link address.
   * If this is NULL, then the CSA currently being pointed to is the last in
   * the chain.
   */

  while ((next[0] & FCX_FREE) != 0)
    {
      /* Clear all bits of the pointer to the next in the chain, other
       * than the address bits themselves.
       */

      next[0] = next[0] & FCX_FREE;

      /* Move the pointer to point to the next CSA in the list. */

      tail = next[0];

      /* Update the local pointer to the CSA. */

      next = tricore_csa2addr(tail);
    }

  /* Look up the current free CSA head. */

  free = __mfcr(CPU_FCX);

  /* Join the current Free onto the Tail of what is being reclaimed. */

  tricore_csa2addr(tail)[0] = free;

  /* Move the head of the reclaimed into the Free. */

  __mtcr(CPU_FCX, head);
}
