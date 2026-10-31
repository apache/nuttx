/****************************************************************************
 * arch/rh850/src/common/rh850_swint.c
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

#include <assert.h>
#include <stdint.h>
#include <syscall.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "sched/sched.h"
#include "rh850_internal.h"
#include "rh850_sysreg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rh850_doswi
 *
 * Description:
 *   TRAP 0 system call: r6 holds the call number.  Returns the frame to
 *   restore.
 *
 ****************************************************************************/

uintptr_t *rh850_doswi(uintptr_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  struct tcb_s *tcb;
  uintptr_t cmd = regs[REG_R6];
  uint32_t eiic;

  eiic = rh850_getsr(RH850_SR_EIIC);
  if (eiic != RH850_EIIC_TRAP0)
    {
      rh850_exception(eiic, RH850_EXC_LEVEL_EI, regs);
    }

  if (cmd != SYS_restore_context)
    {
      (*running_task)->xcp.regs = regs;
    }

  up_set_interrupt_context(true);

  switch (cmd)
    {
      case SYS_restore_context:
      case SYS_switch_context:
        restore_critical_section(this_task(), this_cpu());
        break;

      default:
        PANIC();
        break;
    }

  tcb = this_task();
  if (*running_task != tcb || cmd == SYS_restore_context)
    {
      nxsched_switch_context(*running_task, tcb);
      *running_task = tcb;
    }

  up_set_interrupt_context(false);

  regs = tcb->xcp.regs;
  (*running_task)->xcp.regs = NULL;
  return regs;
}
