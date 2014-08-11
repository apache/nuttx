/****************************************************************************
 * shced/sched_removereadytorun.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <queue.h>
#include <assert.h>

#include <nuttx/clock.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_removereadytorun
 *
 * Description:
 *   This function removes a TCB from the ready to run list.
 *
 * Inputs:
 *   rtcb - Points to the TCB that is ready-to-run
 *
 * Return Value:
 *   true if the currently active task (the head of the
 *     g_readytorun list) has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use irqsave()).
 * - The caller handles the condition that occurs if the
 *   the head of the g_readytorun list is changed.
 *
 ****************************************************************************/

bool sched_removereadytorun(FAR struct tcb_s *rtcb)
{
  FAR struct tcb_s *ntcb = NULL;
  bool ret = false;

  /* Check if the TCB to be removed is at the head of the ready to run list.
   * In this case, we are removing the currently active task.
   */

  if (!rtcb->blink)
    {
      /* There must always be at least one task in the list (the idle task) */

      ntcb = (FAR struct tcb_s *)rtcb->flink;
      DEBUGASSERT(ntcb != NULL);

      /* Inform the instrumentation layer that we are switching tasks */

      sched_note_switch(rtcb, ntcb);
      ntcb->task_state = TSTATE_TASK_RUNNING;

      /* Remove the TCB from the head of the ready-to-run list */

      (void)dq_remfirst((FAR dq_queue_t *)&g_readytorun);

#if CONFIG_RR_INTERVAL > 0
      /* Reset the round robin timeslice interval of the new head of the
       * ready-to-run list.
       */

      ntcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);

#if 0 /* REVISIT: This can cause crashes in certain cases */
      /* Whenever the task at the head of the ready-to-run changes, we
       * must reassess the interval time that controls time-slicing.
       */

      if ((rtcb->flags & TCB_FLAG_ROUND_ROBIN) != 0 ||
          (ntcb->flags & TCB_FLAG_ROUND_ROBIN) != 0)
        {
          sched_timer_reassess();
        }
#endif
#endif
      /* Indicate that a context switch is occurring */

      ret = true;
    }
  else
    {
      /* Remove the TCB from the ready-to-run list (not from the head) */

      dq_rem((FAR dq_entry_t *)rtcb, (FAR dq_queue_t *)&g_readytorun);
    }

  /* Since the TCB is not in any list, it is now invalid */

  rtcb->task_state = TSTATE_TASK_INVALID;
  return ret;
}
