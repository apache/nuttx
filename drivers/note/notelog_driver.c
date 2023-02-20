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

#include <syslog.h>

#include <nuttx/note/note_driver.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void notelog_start(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb);
static void notelog_stop(FAR struct note_driver_s *drv,
                         FAR struct tcb_s *tcb);
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
static void notelog_suspend(FAR struct note_driver_s *drv,
                            FAR struct tcb_s *tcb);
static void notelog_resume(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb);
#  ifdef CONFIG_SMP
static void notelog_cpu_start(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, int cpu);
static void notelog_cpu_started(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb);
static void notelog_cpu_pause(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, int cpu);
static void notelog_cpu_paused(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb);
static void notelog_cpu_resume(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu);
static void notelog_cpu_resumed(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb);
#  endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
static void notelog_premption(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, bool locked);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
static void notelog_csection(FAR struct note_driver_s *drv,
                             FAR struct tcb_s *tcb, bool enter);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
static void note_spinlock(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb,
                          FAR volatile void *spinlock,
                          int type);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static void notelog_irqhandler(FAR struct note_driver_s *drv, int irq,
                               FAR void *handler, bool enter);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct note_driver_ops_s g_notelog_ops =
{
  NULL,                  /* add */
  notelog_start,         /* start */
  notelog_stop,          /* stop */
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  notelog_suspend,       /* suspend */
  notelog_resume,        /* resume */
#ifdef CONFIG_SMP
  notelog_cpu_start,     /* cpu_start */
  notelog_cpu_started,   /* cpu_started */
  notelog_cpu_pause,     /* cpu_pause */
  notelog_cpu_paused,    /* cpu_paused */
  notelog_cpu_resume,    /* cpu_resume */
  notelog_cpu_resumed,   /* cpu_resumed */
#endif
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
  notelog_premption,     /* premption */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
  notelog_csection,      /* csection */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  note_spinlock,         /* spinlock */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  NULL,                  /* syscall_enter */
  NULL,                  /* syscall_leave */
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  notelog_irqhandler,    /* irqhandler */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct note_driver_s g_notelog_driver =
{
  &g_notelog_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notelog_*
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

static void notelog_start(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb)
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

static void notelog_stop(FAR struct note_driver_s *drv,
                         FAR struct tcb_s *tcb)
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
static void notelog_suspend(FAR struct note_driver_s *drv,
                            FAR struct tcb_s *tcb)
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

static void notelog_resume(FAR struct note_driver_s *drv,
                           FAR struct tcb_s *tcb)
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
static void notelog_cpu_start(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d START\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d START\n",
         tcb->cpu, tcb, cpu);
#endif
}

static void notelog_cpu_started(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb)
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
static void notelog_cpu_pause(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d PAUSE\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d PAUSE\n",
         tcb->cpu, tcb, cpu);
#endif
}

static void notelog_cpu_paused(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d PAUSED\n",
         tcb->cpu, tcb->name, tcb, tcb->cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d PAUSED\n",
         tcb->cpu, tcb, tcb->cpu);
#endif
}

static void notelog_cpu_resume(FAR struct note_driver_s *drv,
                               FAR struct tcb_s *tcb, int cpu)
{
#if CONFIG_TASK_NAME_SIZE > 0
  syslog(LOG_INFO, "CPU%d: Task %s TCB@%p CPU%d RESUME\n",
         tcb->cpu, tcb->name, tcb, cpu);
#else
  syslog(LOG_INFO, "CPU%d: TCB@%p CPU%d RESUME\n",
         tcb->cpu, tcb, cpu);
#endif
}

static void notelog_cpu_resumed(FAR struct note_driver_s *drv,
                                FAR struct tcb_s *tcb)
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
static void notelog_premption(FAR struct note_driver_s *drv,
                              FAR struct tcb_s *tcb, bool locked)
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
static void notelog_csection(FAR struct note_driver_s *drv,
                             FAR struct tcb_s *tcb, bool enter)
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
static void note_spinlock(FAR struct note_driver_s *drv,
                          FAR struct tcb_s *tcb,
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

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
static void notelog_irqhandler(FAR struct note_driver_s *drv, int irq,
                               FAR void *handler, bool enter)
{
  syslog(LOG_INFO, "IRQ%d handler@%p %s\n",
         irq, handler, enter ? "ENTER" : "LEAVE");
}
#endif
