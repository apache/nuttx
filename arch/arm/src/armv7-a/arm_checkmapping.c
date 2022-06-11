/****************************************************************************
 * arch/arm/src/armv7-a/arm_checkmapping.c
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

#include <nuttx/sched.h>
#include <nuttx/page.h>

#include "arm_internal.h"

#ifdef CONFIG_PAGING

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_checkmapping()
 *
 * Description:
 *  The function arm_checkmapping() returns an indication if the page fill
 *  still needs to performed or not. In certain conditions, the page fault
 *  may occur on several threads and be queued multiple times. This function
 *  will prevent the same page from be filled multiple times.
 *
 * Input Parameters:
 *   tcb - A reference to the task control block of the task that we believe
 *         needs to have a page fill.  Architecture-specific logic can
 *         retrieve page fault information from the architecture-specific
 *         context information in this TCB and can consult processor
 *         resources (page tables or TLBs or ???) to determine if the fill
 *         still needs to be performed or not.
 *
 * Returned Value:
 *   This function will return true if the mapping is in place and false
 *   if the mapping is still needed.  Errors encountered should be
 *   interpreted as fatal.
 *
 * Assumptions:
 *   - This function is called from the normal tasking context (but with
 *     interrupts disabled).  The implementation must take whatever actions
 *     are necessary to assure that the operation is safe within this
 *     context.
 *
 ****************************************************************************/

bool arm_checkmapping(struct tcb_s *tcb)
{
  uintptr_t vaddr;
  uint32_t *pte;

  /* Since interrupts are disabled, we don't need to anything special. */

  DEBUGASSERT(tcb);

  /* Get the virtual address that caused the fault */

  vaddr = tcb->xcp.far;
  DEBUGASSERT(vaddr >= PG_PAGED_VBASE && vaddr < PG_PAGED_VEND);

  /* Get the PTE associated with this virtual address */

  pte = arm_va2pte(vaddr);

  /* Return true if this virtual address is mapped. */

  return (*pte != 0);
}

#endif /* CONFIG_PAGING */
