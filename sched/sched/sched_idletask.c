/****************************************************************************
 * sched/sched/sched_idletask.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/init.h>
#include <nuttx/sched.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_idletask
 *
 * Description:
 *   Check if the caller is an IDLE thread.  For most implementations of
 *   the SYSLOG output semaphore locking is required for mutual exclusion.
 *   The idle threads are unable to lock semaphores because they cannot
 *   want.  So IDLE thread output is a special case and is treated much as
 *   we treat debug output from an interrupt handler.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the calling task is and IDLE thread.
 *
 ****************************************************************************/

bool sched_idletask(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* If called early in the initialization sequence, the tasks lists may not
   * have been initialized and, in that case, rtcb may be NULL.
   */

  DEBUGASSERT(rtcb != NULL || g_nx_initstate < OSINIT_TASKLISTS);
  if (rtcb != NULL)
    {
      /* The IDLE task TCB is distinguishable by a few things:
       *
       * (1) It always lies at the end of the task list,
       * (2) It always has priority zero, and
       * (3) It should have the TCB_FLAG_CPU_LOCKED flag set.
       *
       * In the non-SMP case, the IDLE task will also have PID=0, but that
       * is not a portable test because there are multiple IDLE tasks with
       * different PIDs in the SMP configuration.
       */

      return (rtcb->flink == NULL);
    }

  /* We must be on the IDLE thread if we are early in initialization */

  return true;
}
