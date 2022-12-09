/****************************************************************************
 * drivers/note/notelog_driver.c
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
#include <stdarg.h>
#include <stdio.h>
#include <syscall.h>
#include <syslog.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: note_spincommon
 *
 * Description:
 *   Common logic for NOTE_SPINLOCK, NOTE_SPINLOCKED, and NOTE_SPINUNLOCK
 *
 * Input Parameters:
 *   tcb  - The TCB containing the information
 *   note - The common note structure to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static inline void note_spincommon(FAR struct tcb_s *tcb,
                                   FAR volatile void *spinlock,
                                   int type)
{
  FAR static const char * const tmp[] =
    {
      "LOCK",
      "LOCKED",
      "UNLOCK",
      "ABORT"
    };

  FAR const char * msg = tmp[type - NOTE_SPINLOCK_LOCK];

#ifdef CONFIG_SMP
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p spinlock@%p %s\n",
         tcb->cpu, tcb->name, tcb, spinlock, msg);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p spinlock@%p %s\n",
         tcb->cpu, tcb, spinlock, msg);
#endif
#else
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "Task %s TCB@%p spinlock@%p %s\n",
         tcb->name, tcb, spinlock, msg);
#else
  syslog(LOG_INFO, "TCB@%p spinlock@%p %s\n",
         tcb, spinlock, msg);
#endif
#endif
}
#endif

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

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
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
#endif

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d START\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d START\n",
         tcb->cpu, tcb, cpu);
#endif
}

void sched_note_cpu_started(FAR struct tcb_s *tcb)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d STARTED\n",
         tcb->cpu, tcb->name, tcb, tcb->cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d STARTED\n",
         tcb->cpu, tcb, tcb->cpu);
#endif
}

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d PAUSE\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d PAUSE\n",
         tcb->cpu, tcb, cpu);
#endif
}

void sched_note_cpu_paused(FAR struct tcb_s *tcb)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d PAUSED\n",
         tcb->cpu, tcb->name, tcb, tcb->cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d PAUSED\n",
         tcb->cpu, tcb, tcb->cpu);
#endif
}

void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d RESUME\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d RESUME\n",
         tcb->cpu, tcb, cpu);
#endif
}

void sched_note_cpu_resumed(FAR struct tcb_s *tcb)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d RESUMED\n",
         tcb->cpu, tcb->name, tcb, tcb->cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d RESUMED\n",
         tcb->cpu, tcb, tcb->cpu);
#endif
}
#endif
#endif

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

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock(FAR struct tcb_s *tcb,
                         FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCK);
}

void sched_note_spinlocked(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_LOCKED);
}

void sched_note_spinunlock(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_UNLOCK);
}

void sched_note_spinabort(FAR struct tcb_s *tcb,
                          FAR volatile void *spinlock)
{
  note_spincommon(tcb, spinlock, NOTE_SPINLOCK_ABORT);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...)
{
  char buf[128];
  FAR char *p = buf;
  va_list ap;

  va_start(ap, argc);
  while (argc-- > 0)
    {
      if (argc)
        {
          p += sprintf(p, "%#"PRIxPTR", ", va_arg(ap, uintptr_t));
        }
      else
        {
          p += sprintf(p, "%#"PRIxPTR, va_arg(ap, uintptr_t));
        }
    }

  va_end(ap);
  syslog(LOG_INFO, "%s@%d ENTER %s\n", g_funcnames[nr], nr, buf);
}

void sched_note_syscall_leave(int nr, uintptr_t result)
{
  syslog(LOG_INFO, "%s@%d LEAVE %"PRIdPTR"\n", g_funcnames[nr], nr, result);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
  syslog(LOG_INFO, "IRQ%d handler@%p %s\n",
         irq, handler, enter ? "ENTER" : "LEAVE");
}
#endif
