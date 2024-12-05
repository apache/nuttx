/****************************************************************************
 * arch/arm/src/arm/arm_syscall.c
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
#include <syscall.h>

#include <nuttx/addrenv.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_syscall
 *
 * Description:
 *   SWI interrupts will vector here with insn=the SWI instruction and
 *   xcp=the interrupt context
 *
 *   The handler may get the SWI number be de-referencing
 *   the return address saved in the xcp and decoding
 *   the SWI instruction
 *
 ****************************************************************************/

uint32_t *arm_syscall(uint32_t *regs)
{
  int cpu = this_cpu();
  struct tcb_s **running_task = &g_running_tasks[cpu];
  struct tcb_s *tcb = this_task();
  uint32_t cmd;

  /* Nested interrupts are not supported */

  DEBUGASSERT(!up_interrupt_context());

  /* Current regs non-zero indicates that we are processing an interrupt;
   * current_regs is also used to manage interrupt level context switches.
   */

  up_set_interrupt_context(true);

  /* The SYSCALL command is in R0 on entry.  Parameters follow in R1..R7 */

  cmd = regs[REG_R0];

  /* if cmd == SYS_restore_context (*running_task)->xcp.regs is valid
   * should not be overwriten
   */

  if (cmd != SYS_restore_context)
    {
      (*running_task)->xcp.regs = regs;
    }

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      case SYS_switch_context:

        /* Update scheduler parameters */

        nxsched_resume_scheduler(tcb);

      case SYS_restore_context:
        nxsched_suspend_scheduler(*running_task);
        *running_task = tcb;

        /* Restore the cpu lock */

        restore_critical_section(tcb, cpu);
#ifdef CONFIG_ARCH_ADDRENV
        addrenv_switch(tcb);
#endif
        break;

      default:
        {
          svcerr("ERROR: Bad SYS call: 0x%" PRIx32 "\n", regs[REG_R0]);
          _alert("Syscall from 0x%" PRIx32 "\n", regs[REG_PC]);
          PANIC();
        }
        break;
    }

  /* Set irq flag */

  up_set_interrupt_context(false);

  /* Return the last value of curent_regs.  This supports context switches
   * on return from the exception.  That capability is only used with the
   * SYS_context_switch system call.
   */

  return tcb->xcp.regs;
}
