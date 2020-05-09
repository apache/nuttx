/****************************************************************************
 * sched/sched/nxsched_foreach.c
 *
 *   Copyright (C) 2007, 2009, 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sched.h>

#include <nuttx/irq.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_foreach
 *
 * Description:
 *   Enumerate over each task and provide the TCB of each task to a user
 *   callback functions.
 *
 *   NOTE:  This function examines the TCB and calls each handler within a
 *   critical section.  However, that critical section is released and
 *   reacquired for each TCB.  When it is released, there may be changes in
 *   tasking.  If the caller requires absolute stability through the
 *   traversal, then the caller should establish the critical section BEFORE
 *   calling this function.
 *
 * Input Parameters:
 *   handler - The function to be called with the TCB of
 *     each task
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_foreach(nxsched_foreach_t handler, FAR void *arg)
{
  irqstate_t flags;
  int ndx;

  /* Visit each active task */

  for (ndx = 0; ndx < CONFIG_MAX_TASKS; ndx++)
    {
      /* This test and the function call must be atomic */

      flags = enter_critical_section();
      if (g_pidhash[ndx].tcb)
        {
          handler(g_pidhash[ndx].tcb, arg);
        }

      leave_critical_section(flags);
    }
}
