/****************************************************************************
 * arch/tricore/src/common/tricore_svcall.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <sched/sched.h>
#include <nuttx/sched.h>

#include "tricore_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

void tricore_svcall(volatile void *trap)
{
  uintptr_t *regs;
  uint32_t cmd;

  regs = tricore_csa2addr(__mfcr(CPU_PCXI));

  CURRENT_REGS = regs;

  cmd = regs[REG_D8];

  /* Handle the SVCall according to the command in R0 */

  switch (cmd)
    {
      /* R0=SYS_restore_context:  This a restore context command:
       *
       *   void tricore_fullcontextrestore(uint32_t *restoreregs)
       *          noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   R0 = SYS_restore_context
       *   R1 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore
       * register area referenced in the saved R1. context == CURRENT_REGS
       * is the normal exception return.  By setting CURRENT_REGS =
       * context[R1], we force the return to the saved context referenced
       * in R1.
       */

      case SYS_restore_context:
        {
          tricore_reclaim_csa(regs[REG_UPCXI]);
          CURRENT_REGS = (uintptr_t *)regs[REG_D9];
        }
        break;

      case SYS_switch_context:
        {
          *(uintptr_t **)regs[REG_D9] = (uintptr_t *)regs[REG_UPCXI];
          CURRENT_REGS = (uintptr_t *)regs[REG_D10];
        }
        break;

      default:
        {
          svcerr("ERROR: Bad SYS call: %d\n", (int)regs[REG_D0]);
        }
        break;
    }

  if (regs != CURRENT_REGS)
    {
      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      g_running_tasks[this_cpu()] = this_task();

      regs[REG_UPCXI] = (uintptr_t)CURRENT_REGS;

      __isync();
    }

  CURRENT_REGS = NULL;
}
