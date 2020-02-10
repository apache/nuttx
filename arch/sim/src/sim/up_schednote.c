/****************************************************************************
 * arch/sim/src/sim/up_schednote.c
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
#include <syslog.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_start, sched_note_stop, sched_note_switch,
 *       sched_note_premption
 *
 * Description:
 *   Hooks to scheduler monitor
 *
 * Input Parameters:
 *   Varies
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_note_start(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Start %s, TCB@%p, state=%d\n",
         tcb->cpu, tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "CPU%d: Start TCB@%p, state=%d\n"
         tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Start %s, TCB@%p, state=%d\n",
         tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "Start TCB@%p, state=%d\n",
         tcb, tcb->task_state);
#endif
#endif
}

void sched_note_stop(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Stop %s, TCB@%p, state=%d\n",
         tcb->cpu, tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "CPU%d: Stop TCB@%p, state=%d\n",
         tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Stop %s, TCB@%p, state=%d\n",
         tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "Stop TCB@%p, state=%d\n",
         tcb, tcb->task_state);
#endif
#endif
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Suspend %s, TCB@%p, state=%d\n",
         tcb->cpu, tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "CPU%d: Suspend TCB@%p, state=%d\n",
         tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Suspend %s, TCB@%p, state=%d\n",
         tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "Suspend TCB@%p, state=%d\n",
         tcb, tcb->task_state);
#endif
#endif
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Resume %s, TCB@%p, state=%d\n",
         tcb->cpu, tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "CPU%d: Resume TCB@%p, state=%d\n",
         tcb->cpu, tcb, tcb->task_state);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Resume %s, TCB@%p, state=%d\n",
         tcb->name, tcb, tcb->task_state);
#else
  syslog(LOG_INFO, "Resume TCB@%p, state=%d\n",
         tcb, tcb->task_state);
#endif
#endif
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
/* This does not work well... it interferes with the operation of the
 * simulated /dev/console device which, of course, does disable preemption
 * and does use critical sections.
 */

#warning CONFIG_SCHED_INSTRUMENTATION_PREEMPTION is a bad idea

void sched_note_premption(FAR struct tcb_s *tcb, bool locked)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p preemption %s\n",
         tcb->cpu, tcb->name, tcb, locked ? "LOCKED" : "UNLOCKED");
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p preemption %s\n",
         tcb->cpu, tcb, locked ? "LOCKED" : "UNLOCKED");
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Task %s, TCB@%p preemption %s\n",
         tcb->name, tcb, locked ? "LOCKED" : "UNLOCKED");
#else
  syslog(LOG_INFO, "TCB@%p preemption %s\n",
         tcb, locked ? "LOCKED" : "UNLOCKED");
#endif
#endif
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
/* This does not work well... it interferes with the operation of the
 * simulated /dev/console device which, of course, does disable preemption
 * and does use critical sections.
 */

#warning CONFIG_SCHED_INSTRUMENTATION_CSECTION is a bad idea

void sched_note_csection(FAR struct tcb_s *tcb, bool enter)
{
#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p critical section %s\n",
         tcb->cpu, tcb->name, tcb, enter ? "ENTER" : "LEAVE");
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p critical section %s\n",
         tcb->cpu, tcb, enter ? "ENTER" : "LEAVE");
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Task %s, TCB@%p critical section %s\n",
         tcb->name, tcb, enter ? "ENTER" : "LEAVE");
#else
  syslog(LOG_INFO, "TCB@%p critical section %s\n",
         tcb, enter ? "ENTER" : "LEAVE");
#endif
#endif
}
#endif
