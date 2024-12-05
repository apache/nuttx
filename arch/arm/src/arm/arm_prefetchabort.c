/****************************************************************************
 * arch/arm/src/arm/arm_prefetchabort.c
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

#include <inttypes.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#endif

#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_prefetchabort
 *
 * Description:
 *   This is the prefetch abort exception handler. The ARM prefetch abort
 *   exception occurs when a memory fault is detected during an an
 *   instruction fetch.
 *
 ****************************************************************************/

void arm_prefetchabort(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();

#ifdef CONFIG_LEGACY_PAGING
  uint32_t *saveregs;
  bool savestate;

  savestate = up_interrupt_context();
  saveregs = tcb->xcp.regs;
#endif
  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

#ifdef CONFIG_LEGACY_PAGING
  /* Get the (virtual) address of instruction that caused the prefetch
   * abort.  When the exception occurred, this address was provided in the
   * lr register and this value was saved in the context save area as the PC
   * at the REG_R15 index.
   *
   * Check to see if this miss address is within the configured range of
   * virtual addresses.
   */

  pginfo("VADDR: %08" PRIx32 " VBASE: %08x VEND: %08x\n",
         regs[REG_PC], PG_PAGED_VBASE, PG_PAGED_VEND);

  if (regs[REG_R15] >= PG_PAGED_VBASE && regs[REG_R15] < PG_PAGED_VEND)
    {
      /* Save the offending PC as the fault address in the TCB of the
       * currently executing task.  This value is, of course, already known
       * in regs[REG_R15], but saving it in this location will allow common
       * paging logic for both prefetch and data aborts.
       */

      tcb->xcp.far  = regs[REG_R15];

      /* Call pg_miss() to schedule the page fill.  A consequences of this
       * call are:
       *
       * (1) The currently executing task will be blocked and saved on
       *     on the g_waitingforfill task list.
       * (2) An interrupt-level context switch will occur so that when
       *     this function returns, it will return to a different task,
       *     most likely the page fill worker thread.
       * (3) The page fill worker task has been signalled and should
       *     execute immediately when we return from this exception.
       */

      pg_miss();

      /* Restore the previous value of saveregs. */

      up_set_interrupt_context(savestate);
      tcb->xcp.regs = saveregs;
    }
  else
#endif
    {
      _alert("Prefetch abort. PC: %08" PRIx32 "\n", regs[REG_PC]);
      PANIC_WITH_REGS("panic", regs);
    }
}
