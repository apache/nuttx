/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_userfault.c
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

#include <stdbool.h>
#include <stdint.h>
#include <debug.h>
#include <signal.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <arch/irq.h>
#include <arch/xtensa/xtensa_corebits.h>

#include "xtensa.h"
#include "sched/sched.h"
#include "signal/signal.h"

#include "esp32s3_userfault.h"
#ifdef CONFIG_ESP32S3_PAGEFAULT
#include "esp32s3_pagefault.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_userfault_abort
 *
 * Description:
 *   Terminate just the faulting unprivileged task by delivering a fatal
 *   SIGSEGV, instead of panicking the whole system.
 *
 *   This is deliberately independent of what the fault was.  A user task
 *   must not be able to stop the machine, and it has many ways to raise a
 *   synchronous exception that no dispatcher can service:  an unaligned
 *   load, a divide by zero, a privileged instruction, a corrupt opcode.  All
 *   of them arrive here through the same vector with the same frame, so all
 *   of them get the same answer.  Only the caller decides who is eligible --
 *   see xtensa_user(), which gates on the interruptee's PS.UM.
 *
 *   Mirrors the interrupt-dispatch handshake: record the exception frame as
 *   the task context, dispatch the signal -- which redirects the task to the
 *   signal trampoline via up_schedule_sigaction() -- then return the
 *   redirected frame so the vector's RFE resumes the task in the trampoline,
 *   whose SIGSEGV default action (_exit) tears the task down and resumes.
 *
 * Input Parameters:
 *   exccause - The EXCCAUSE of the user exception, for reporting
 *   regs     - The register save area at the time of the exception
 *
 * Returned Value:
 *   The register frame to resume (the signal trampoline for the faulting
 *   task).
 *
 ****************************************************************************/

uint32_t *esp32s3_userfault_abort(int exccause, uint32_t *regs)
{
  struct tcb_s *tcb = this_task();
  siginfo_t     info;

#ifdef CONFIG_ESP32S3_PAGEFAULT
  /* Reaching here means the fault was contained and the system carried on,
   * so the dispatcher's repeat counter has served its purpose.  Clear it, or
   * a probe run three times at one address would trip that guard and halt a
   * perfectly healthy system.  Only *unbroken* recursion -- a report that
   * faults before the abort can happen -- should stop the machine.
   */

  esp32s3_pagefault_clear_repeat();
#endif

  _alert("SIGSEGV task %s: EXCCAUSE=%d EXCVADDR=%08x PC=%08x\n",
         get_task_name(tcb), exccause, (unsigned)regs[REG_EXCVADDR],
         (unsigned)regs[REG_PC]);

  up_set_interrupt_context(true);
  tcb->xcp.regs = regs;

  info.si_signo           = SIGSEGV;
  info.si_code            = SI_USER;
  info.si_errno           = 0;
  info.si_value.sival_ptr = (FAR void *)regs[REG_EXCVADDR];

  nxsig_tcbdispatch(tcb, &info, false);

  regs          = tcb->xcp.regs;
  tcb->xcp.regs = NULL;
  up_set_interrupt_context(false);
  return regs;
}
