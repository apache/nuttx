/****************************************************************************
 * sched/sched/sched_setscheduler.c
 *
 *   Copyright (C) 2007, 2009, 2012, 2015 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:sched_setscheduler
 *
 * Description:
 *   sched_setscheduler() sets both the scheduling policy and the priority
 *   for the task identified by pid. If pid equals zero, the scheduler of
 *   the calling task will be set.  The parameter 'param' holds the priority
 *   of the thread under the new policy.
 *
 * Inputs:
 *   pid - the task ID of the task to modify.  If pid is zero, the calling
 *      task is modified.
 *   policy - Scheduling policy requested (either SCHED_FIFO or SCHED_RR)
 *   param - A structure whose member sched_priority is the new priority.
 *      The range of valid priority numbers is from SCHED_PRIORITY_MIN
 *      through SCHED_PRIORITY_MAX.
 *
 * Return Value:
 *   On success, sched_setscheduler() returns OK (zero).  On error, ERROR
 *   (-1) is returned, and errno is set appropriately:
 *
 *   EINVAL The scheduling policy is not one of the recognized policies.
 *   ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setscheduler(pid_t pid, int policy,
                       FAR const struct sched_param *param)
{
  FAR struct tcb_s *tcb;
  irqstate_t saved_state;
  int ret;

  /* Check for supported scheduling policy */

  if (policy != SCHED_FIFO
#if CONFIG_RR_INTERVAL > 0
      && policy != SCHED_RR
#endif
#ifdef CONFIG_SCHED_SPORADIC
      && policy != SCHED_SPORADIC
#endif
    )
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Check if the task to modify the calling task */

  if (pid == 0 )
    {
      pid = getpid();
    }

  /* Verify that the pid corresponds to a real task */

  tcb = sched_gettcb(pid);
  if (!tcb)
    {
      set_errno(ESRCH);
      return ERROR;
    }

  /* Prohibit any context switches while we muck with priority and scheduler
   * settings.
   */

  sched_lock();

  /* Further, disable timer interrupts while we set up scheduling policy. */

  saved_state = irqsave();
  tcb->flags &= TCB_FLAG_POLICY_MASK;
  switch (policy)
    {
      default:
        DEBUGPANIC();
      case SCHED_FIFO:
        {
          tcb->flags       |= TCB_FLAG_SCHED_FIFO;
#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
          tcb->timeslice    = 0;
#endif
#ifdef CONFIG_SCHED_SPORADIC
          tcb->low_priority = 0;
          tcb->max_repl     = 0;
          tcb->repl_period  = 0;
          tcb->budget       = 0;
#endif
        }
        break;

#if CONFIG_RR_INTERVAL > 0
      case SCHED_RR:
        {
          tcb->flags       |= TCB_FLAG_SCHED_RR;
          tcb->timeslice    = MSEC2TICK(CONFIG_RR_INTERVAL);
#ifdef CONFIG_SCHED_SPORADIC
          tcb->low_priority = 0;
          tcb->max_repl     = 0;
          tcb->repl_period  = 0;
          tcb->budget       = 0;
#endif
        }
        break;
#endif

#ifdef CONFIG_SCHED_SPORADIC
      case SCHED_SPORADIC:
        {
          int ticks;

          DEBUGASSERT(param->sched_ss_max_repl <= UINT8_MAX);

          tcb->flags       |= TCB_FLAG_SCHED_SPORADIC;
          tcb->timeslice    = MSEC2TICK(CONFIG_RR_INTERVAL);
          tcb->low_priority = param->sched_ss_low_priority;
          tcb->max_repl     = param->sched_ss_max_repl;

          (void)clock_time2ticks(&param->sched_ss_repl_period, &ticks);
          tcb->repl_period  = ticks;

          (void)clock_time2ticks(&param->sched_ss_init_budget, &ticks);
          tcb->budget       = ticks;
        }
        break;
#endif

#if 0 /* Not supported */
      case SCHED_OTHER:
        tcb->flags    |= TCB_FLAG_SCHED_OTHER;
        break;
#endif
    }

  irqrestore(saved_state);

  /* Set the new priority */

  ret = sched_reprioritize(tcb, param->sched_priority);
  sched_unlock();

  if (ret != OK)
    {
      return ERROR;
    }
  else
    {
      return OK;
    }
}
