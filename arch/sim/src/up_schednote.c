/****************************************************************************
 * arch/sim/src/up_schednote.c
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
#include <syslog.h>
#include <nuttx/sched.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_start, sched_note_stop, sched_note_switch
 *
 * Description:
 *   Create the pthread-specific data key and set the indication of CPU0
 *   the the main thread.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

void sched_note_start(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "CPU%d: Start %s, TCB@%p, state=%d\n",
            tcb->cpu, tcb->name, tcb, tcb->task_state);
#else  
  lowsyslog(LOG_INFO, "CPU%d: Start TCB@%p, state=%d\n"
            tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "Start %s, TCB@%p, state=%d\n",
            tcb->name, tcb, tcb->task_state);
#else  
  lowsyslog(LOG_INFO, "Start TCB@%p, state=%d\n",
            tcb, tcb->task_state);
#endif
#endif
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "CPU%d: Stop %s, TCB@%p, state=%d\n",
            tcb->cpu, tcb->name, tcb, tcb->task_state);
#else  
  lowsyslog(LOG_INFO, "CPU%d: Stop TCB@%p, state=%d\n",
            tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "Stop %s, TCB@%p, state=%d\n",
            tcb->name, tcb, tcb->task_state);
#else  
  lowsyslog(LOG_INFO, "Stop TCB@%p, state=%d\n",
            tcb, tcb->task_state);
#endif
#endif
}

void sched_note_switch(FAR struct tcb_s *from, FAR struct tcb_s *to)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "CPU%d: Suspend %s, TCB@%p, state=%d\n",
            from->cpu, from->name, from, from->task_state);
  lowsyslog(LOG_INFO, "CPU%d: Resume %s, TCB@%p, state=%d\n",
            to->cpu, to->name, to, to->task_state);
#else  
  lowsyslog(LOG_INFO, "CPU%d: Stop TCB@%p, state=%d\n",
            from->cpu, from, from->task_state);
  lowsyslog(LOG_INFO, "CPU%d: Resume TCB@%p, state=%d\n",
            to->cpu, to, to->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  lowsyslog(LOG_INFO, "Stop %s, TCB@%p, state=%d\n",
            from->name, from, from->task_state);
  lowsyslog(LOG_INFO, "Resume %s, TCB@%p\, state=%dn",
            to->name, to, to->task_state);
#else  
  lowsyslog(LOG_INFO, "Stop TCB@%p, state=%d\n",
            from, from->task_state);
  lowsyslog(LOG_INFO, "Resume TCB@%p, state=%d\n",
            to, to->task_state);
#endif
#endif
}

#endif /* CONFIG_SCHED_INSTRUMENTATION */
