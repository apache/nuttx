/****************************************************************************
 * arch/arm/src/armv7-a/arm_userfault.c
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
#include <signal.h>
#include <unistd.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "sched/sched.h"
#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fault_kill
 *
 * Description:
 *   A fault of user code ends the process (SIGSEGV), not the system: the
 *   abort returns to _exit() in SYS mode, on the kernel stack of the task
 *   if it has one.  A fault of the kernel (also of a system call) is not
 *   handled here.
 *
 * Input Parameters:
 *   regs - The register state of the abort
 *
 * Returned Value:
 *   True if the task ends; false if the abort is fatal.
 *
 ****************************************************************************/

bool arm_fault_kill(uint32_t *regs)
{
  struct tcb_s *tcb = this_task();
  struct tcb_s *ptcb;

  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL ||
      (tcb->flags & TCB_FLAG_SYSCALL) != 0 ||
      (regs[REG_CPSR] & PSR_MODE_MASK) != PSR_MODE_USR)
    {
      return false;
    }

  ptcb = nxsched_get_tcb(tcb->group->tg_pid);
  _alert("Segmentation fault in %s (PID %d: %s)\n",
         ptcb != NULL ? get_task_name(ptcb) : "<gone>",
         tcb->pid, get_task_name(tcb));

  tcb->flags |= TCB_FLAG_FORCED_CANCEL;

  regs[REG_PC]   = (uint32_t)_exit;
  regs[REG_R0]   = SIGSEGV;
  regs[REG_CPSR] = PSR_MODE_SYS;
#ifdef CONFIG_ARM_THUMB
  regs[REG_CPSR] |= PSR_T_BIT;
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* The frames on the kernel stack are no longer needed */

  if (tcb->xcp.kstack != NULL)
    {
      regs[REG_SP] = (uint32_t)tcb->xcp.kstack + ARCH_KERNEL_STACKSIZE;
    }
#endif

  return true;
}
