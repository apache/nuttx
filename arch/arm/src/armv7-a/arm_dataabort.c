/****************************************************************************
 * arch/arm/src/armv7-a/arm_dataabort.c
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

#include "mmu.h"
#include "sched/sched.h"
#include "arm_internal.h"

#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#  include "arm.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_dataabort
 *
 * Input Parameters:
 *   regs - The standard, ARM register save array.
 *
 * If CONFIG_LEGACY_PAGING is selected in the NuttX configuration file, then
 * these additional input values are expected:
 *
 *   dfar - Fault address register.  On a data abort, the ARM MMU places the
 *     miss virtual address (MVA) into the DFAR register.  This is the
 *     address of the data which, when accessed, caused the fault.
 *   dfsr - Fault status register.  On a data a abort, the ARM MMU places an
 *     encoded four-bit value, the fault status, along with the four-bit
 *     encoded domain number, in the data DFSR
 *
 * Description:
 *   This is the data abort exception handler. The ARM data abort exception
 *   occurs when a memory fault is detected during a data transfer.
 *
 ****************************************************************************/

#ifdef CONFIG_LEGACY_PAGING
uint32_t *arm_dataabort(uint32_t *regs, uint32_t dfar, uint32_t dfsr)
{
  struct tcb_s *tcb = this_task();
  uint32_t *saveregs;
  bool savestate;

  savestate = up_interrupt_context();
  saveregs = tcb->xcp.regs;
  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

  /* In the NuttX on-demand paging implementation, only the read-only, .text
   * section is paged.  However, the ARM compiler generated PC-relative data
   * fetches from within the .text sections.  Also, it is customary to locate
   * read-only data (.rodata) within the same section as .text so that it
   * does not require copying to RAM. Misses in either of these case should
   * cause a data abort.
   *
   * We are only interested in data aborts due to page translations faults.
   * Sections should already be in place and permissions should already be
   * be set correctly (to read-only) so any other data abort reason is a
   * fatal error.
   */

  pginfo("DFSR: %08x DFAR: %08x\n", dfsr, dfar);
  if (FSR_FAULT(dfsr) == FSR_FAULT_DEBUG)
    {
      arm_dbgmonitor(0, (void *)dfar, regs);
    }
  else if((dfsr & FSR_MASK) != FSR_PAGE)
    {
      goto segfault;
    }

  /* Check the (virtual) address of data that caused the data abort. When
   * the exception occurred, this address was provided in the DFAR register.
   * (It has not yet been saved in the register context save area).
   */

  else if (dfar < PG_PAGED_VBASE || dfar >= PG_PAGED_VEND)
    {
      goto segfault;
    }
  else
    {
      pginfo("VBASE: %08x VEND: %08x\n", PG_PAGED_VBASE, PG_PAGED_VEND);

      /* Save the offending data address as the fault address in the TCB of
       * the currently task.  This fault address is also used by the prefetch
       * abort handling; this will allow common paging logic for both
       * prefetch and data aborts.
       */

      tcb->xcp.dfar = regs[REG_R15];

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
    }

  /* Restore the previous value of saveregs. */

  up_set_interrupt_context(savestate);
  tcb->xcp.regs = saveregs;
  return regs;

segfault:
  _alert("Data abort. PC: %08" PRIx32 " DFAR: %08" PRIx32 " DFSR: %08"
         PRIx32 "\n", regs[REG_PC], dfar, dfsr);
  PANIC_WITH_REGS("panic", regs);
  return regs; /* To keep the compiler happy */
}

#else /* CONFIG_LEGACY_PAGING */

uint32_t *arm_dataabort(uint32_t *regs, uint32_t dfar, uint32_t dfsr)
{
  struct tcb_s *tcb = this_task();

  tcb->xcp.regs = regs;
  up_set_interrupt_context(true);

  /* Crash -- possibly showing diagnostic debug information. */

  _alert("Data abort. PC: %08" PRIx32 " DFAR: %08" PRIx32 " DFSR: %08"
         PRIx32 "\n", regs[REG_PC], dfar, dfsr);

  if (FSR_FAULT(dfsr) == FSR_FAULT_DEBUG)
    {
      arm_dbgmonitor(0, (void *)dfar, regs);
    }
  else
    {
      PANIC_WITH_REGS("panic", regs);
    }

  up_set_interrupt_context(false);
  return regs; /* To keep the compiler happy */
}

#endif /* CONFIG_LEGACY_PAGING */
