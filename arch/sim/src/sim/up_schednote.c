/****************************************************************************
 * arch/sim/src/sim/up_schednote.c
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
