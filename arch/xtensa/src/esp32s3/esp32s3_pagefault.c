/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pagefault.c
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

#include <stdbool.h>
#include <stdint.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <arch/irq.h>
#include <arch/xtensa/xtensa_corebits.h>

#include "xtensa.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESP32S3_PAGEFAULT_SELFTEST
/* Runtime self-test: proof of the recoverable-fault primitive on silicon.
 * A load of this out-of-cache-region address raises a precise LoadProhibited
 * (EXCCAUSE 28, "cache attribute does not allow load") whose EXCVADDR tracks
 * the address exactly.  The dispatcher returns "serviced" WITHOUT making the
 * address accessible for the first PF_SELFTEST_REPEATS re-executions, so the
 * exception vector's RFE re-runs the identical faulting instruction.
 * Being re-entered that many times for one instruction proves that RFE
 * cleanly restarts a faulted precise access (the write-buffer / prefetch
 * corner case that gates recoverable page faults); it then steps the saved
 * PC past the 2-byte l32i.n so the faulting task resumes.  Trigger it from a
 * user task, e.g. examples/pffault:  nsh> pffault r 0x80000000
 */

#  define PF_SELFTEST_VADDR    0x80000000ul
#  define PF_SELFTEST_REPEATS  3

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile int g_pf_selftest_hits;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_pagefault_dispatch
 *
 * Description:
 *   Offered the precise, restartable exceptions raised by a cache-attribute
 *   permission violation (EXCCAUSE Load/Store/InstrFetch Prohibited).  The
 *   faulting virtual address is in regs[REG_EXCVADDR] and the faulting PC in
 *   regs[REG_PC]; both are populated by the common Xtensa user exception
 *   handler (frame layout: NuttX REG_*).
 *
 *   This unit establishes the recoverable-fault primitive.  The full
 *   servicing (map a page / restore a cache attribute, then RFE-restart) is
 *   built on top in the addrenv / demand-paging units; here the dispatcher
 *   reports the fault (with its tracking EXCVADDR) and declines to service
 *   it, except under the self-test which proves the RFE-restart on silicon.
 *
 * Returned Value:
 *   OK if the faulting instruction may be (re-)executed via RFE; a negated
 *   errno otherwise (the caller then panics / aborts the faulting task).
 *
 ****************************************************************************/

int esp32s3_pagefault_dispatch(int exccause, uint32_t *regs)
{
  uintptr_t vaddr = (uintptr_t)regs[REG_EXCVADDR];
  uintptr_t pc    = (uintptr_t)regs[REG_PC];

#ifdef CONFIG_ESP32S3_PAGEFAULT_SELFTEST
  if (exccause == EXCCAUSE_LOAD_PROHIBITED && vaddr == PF_SELFTEST_VADDR)
    {
      g_pf_selftest_hits++;

      _alert("PAGEFAULT SELFTEST: restart #%d precise LoadProhibited "
             "EXCVADDR=%08x PC=%08x\n",
             g_pf_selftest_hits, (unsigned)vaddr, (unsigned)pc);

      if (g_pf_selftest_hits < PF_SELFTEST_REPEATS)
        {
          /* Return serviced without changing anything: the RFE must
           * re-execute the identical faulting load and land back here.
           */

          return OK;
        }

      /* Proof complete: step past the 2-byte l32i.n so the task resumes. */

      regs[REG_PC]       = pc + 2;
      g_pf_selftest_hits = 0;
      _alert("PAGEFAULT SELFTEST: RFE cleanly restarted the load %d times; "
             "resuming\n", PF_SELFTEST_REPEATS);
      return OK;
    }
#endif

  /* Report the precise fault (with its tracking EXCVADDR) and decline to
   * service it, so the caller falls through to the panic / abort path.
   */

  _alert("cache fault: EXCCAUSE=%d EXCVADDR=%08x PC=%08x task=%s\n",
         exccause, (unsigned)vaddr, (unsigned)pc,
         get_task_name(this_task()));

  return -EFAULT;
}
