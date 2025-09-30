/****************************************************************************
 * sched/sched/nxsched_checkstackoverflow.c
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

#include <time.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/sched_note.h>

#include "clock/clock.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_checkstackoverflow
 *
 * Description:
 *   Verify that the given thread has not overflowed its stack.  This check
 *   is enabled when either CONFIG_STACKCHECK_SOFTWARE is selected or a
 *   non-zero CONFIG_STACKCHECK_MARGIN is defined.
 *
 *   - When CONFIG_STACKCHECK_SOFTWARE is enabled, the current stack pointer
 *     is compared against the thread’s allocated stack boundaries.
 *   - When CONFIG_STACKCHECK_MARGIN is non-zero, additional margin checking
 *     is performed to ensure the stack does not exceed the reserved safety
 *     area.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to be checked.
 *
 * Returned Value:
 *   None.  This function will assert if a stack overflow condition is
 *   detected.
 *
 ****************************************************************************/

void nxsched_checkstackoverflow(FAR struct tcb_s *tcb)
{
  if (tcb == NULL)
    {
      return;
    }

#ifdef CONFIG_STACKCHECK_SOFTWARE
  if (tcb->xcp.regs != NULL)
    {
      uintptr_t sp   = up_getusrsp(tcb->xcp.regs);
      uintptr_t top  = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size;
      uintptr_t bot  = (uintptr_t)tcb->stack_base_ptr;

      DEBUGASSERT(sp > bot && sp <= top);
    }
#endif

#if CONFIG_STACKCHECK_MARGIN > 0
  DEBUGASSERT(up_check_tcbstack(tcb, CONFIG_STACKCHECK_MARGIN) == 0);
#endif
}
