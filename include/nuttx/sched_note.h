/****************************************************************************
 * include/nuttx/sched_note.h
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

#ifndef __INCLUDE_NUTTX_SCHED_NOTE_H
#define __INCLUDE_NUTTX_SCHED_NOTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include <nuttx/sched.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Provide defaults for some configuration settings (could be undefined with
 * old configuration files)
 */

#ifndef CONFIG_SCHED_INSTRUMENTATION_CPUSET
#  define CONFIG_SCHED_INSTRUMENTATION_CPUSET 0xffff
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This type identifies a note structure */

enum note_type_e
{
  NOTE_START           = 0,
  NOTE_STOP            = 1,
  NOTE_SUSPEND         = 2,
  NOTE_RESUME          = 3
#ifdef CONFIG_SMP
  ,
  NOTE_CPU_START       = 4,
  NOTE_CPU_STARTED     = 5,
  NOTE_CPU_PAUSE       = 6,
  NOTE_CPU_PAUSED      = 7,
  NOTE_CPU_RESUME      = 8,
  NOTE_CPU_RESUMED     = 9
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
  ,
  NOTE_PREEMPT_LOCK    = 10,
  NOTE_PREEMPT_UNLOCK  = 11
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
  ,
  NOTE_CSECTION_ENTER  = 12,
  NOTE_CSECTION_LEAVE  = 13
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  ,
  NOTE_SPINLOCK_LOCK   = 14,
  NOTE_SPINLOCK_LOCKED = 15,
  NOTE_SPINLOCK_UNLOCK = 16,
  NOTE_SPINLOCK_ABORT  = 17
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
  ,
  NOTE_SYSCALL_ENTER   = 18,
  NOTE_SYSCALL_LEAVE   = 19
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  ,
  NOTE_IRQ_ENTER       = 20,
  NOTE_IRQ_LEAVE       = 21
#endif
};

/* This structure provides the common header of each note */

struct note_common_s
{
  uint8_t nc_length;           /* Length of the note */
  uint8_t nc_type;             /* See enum note_type_e */
  uint8_t nc_priority;         /* Thread/task priority */
#ifdef CONFIG_SMP
  uint8_t nc_cpu;              /* CPU thread/task running on */
#endif
  uint8_t nc_pid[2];           /* ID of the thread/task */
  uint8_t nc_systime[4];       /* Time when note was buffered */
};

/* This is the specific form of the NOTE_START note */

struct note_start_s
{
  struct note_common_s nst_cmn; /* Common note parameters */
#if CONFIG_TASK_NAME_SIZE > 0
  char    nst_name[1];          /* Start of the name of the thread/task */
#endif
};

/* This is the specific form of the NOTE_STOP note */

struct note_stop_s
{
  struct note_common_s nsp_cmn; /* Common note parameters */
};

/* This is the specific form of the NOTE_SUSPEND note */

struct note_suspend_s
{
  struct note_common_s nsu_cmn; /* Common note parameters */
  uint8_t nsu_state;            /* Task state */
};

/* This is the specific form of the NOTE_RESUME note */

struct note_resume_s
{
  struct note_common_s nre_cmn; /* Common note parameters */
};

#ifdef CONFIG_SMP

/* This is the specific form of the NOTE_CPU_START note */

struct note_cpu_start_s
{
  struct note_common_s ncs_cmn; /* Common note parameters */
  uint8_t ncs_target;           /* CPU being started */
};

/* This is the specific form of the NOTE_CPU_STARTED note */

struct note_cpu_started_s
{
  struct note_common_s ncs_cmn; /* Common note parameters */
};

/* This is the specific form of the NOTE_CPU_PAUSE note */

struct note_cpu_pause_s
{
  struct note_common_s ncp_cmn; /* Common note parameters */
  uint8_t ncp_target;           /* CPU being paused */
};

/* This is the specific form of the NOTE_CPU_PAUSED note */

struct note_cpu_paused_s
{
  struct note_common_s ncp_cmn; /* Common note parameters */
};

/* This is the specific form of the NOTE_CPU_RESUME note */

struct note_cpu_resume_s
{
  struct note_common_s ncr_cmn; /* Common note parameters */
  uint8_t ncr_target;           /* CPU being resumed */
};

/* This is the specific form of the NOTE_CPU_RESUMED note */

struct note_cpu_resumed_s
{
  struct note_common_s ncr_cmn; /* Common note parameters */
};
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
/* This is the specific form of the NOTE_PREEMPT_LOCK/UNLOCK note */

struct note_preempt_s
{
  struct note_common_s npr_cmn; /* Common note parameters */
  uint8_t npr_count[2];         /* Count of nested locks */
};
#endif /* CONFIG_SCHED_INSTRUMENTATION_PREEMPTION */

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
/* This is the specific form of the NOTE_CSECTION_ENTER/LEAVE note */

struct note_csection_s
{
  struct note_common_s ncs_cmn; /* Common note parameters */
#ifdef CONFIG_SMP
  uint8_t ncs_count[2];         /* Count of nested csections */
#endif
};
#endif /* CONFIG_SCHED_INSTRUMENTATION_CSECTION */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
/* This is the specific form of the NOTE_SPINLOCK_LOCK/LOCKED/UNLOCK/ABORT
 * note.
 */

struct note_spinlock_s
{
  struct note_common_s nsp_cmn; /* Common note parameters */
  FAR void *nsp_spinlock;       /* Address of spinlock */
  uint8_t nsp_value;            /* Value of spinlock */
};
#endif /* CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
/* This is the specific form of the NOTE_SYSCALL_ENTER/LEAVE notes */

struct note_syscall_enter_s
{
  struct note_common_s nsc_cmn; /* Common note parameters */
  uint8_t nsc_nr;               /* System call number */
};

struct note_syscall_leave_s
{
  struct note_common_s nsc_cmn; /* Common note parameters */
  uintptr_t nsc_result;         /* Result of the system call */
  uint8_t nsc_nr;               /* System call number */
};
#endif /* CONFIG_SCHED_INSTRUMENTATION_SYSCALL */

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
/* This is the specific form of the NOTE_IRQ_ENTER/LEAVE notes */

struct note_irqhandler_s
{
  struct note_common_s nih_cmn; /* Common note parameters */
  uint8_t nih_irq;              /* IRQ number */
};
#endif /* CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sched_note_*
 *
 * Description:
 *   If instrumentation of the scheduler is enabled, then some outboard
 *   logic must provide the following interfaces.  These interfaces are not
 *   available to application code.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_note_start(FAR struct tcb_s *tcb);
void sched_note_stop(FAR struct tcb_s *tcb);
void sched_note_suspend(FAR struct tcb_s *tcb);
void sched_note_resume(FAR struct tcb_s *tcb);

#ifdef CONFIG_SMP
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu);
void sched_note_cpu_started(FAR struct tcb_s *tcb);
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu);
void sched_note_cpu_paused(FAR struct tcb_s *tcb);
void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu);
void sched_note_cpu_resumed(FAR struct tcb_s *tcb);
#else
#  define sched_note_cpu_start(t,c)
#  define sched_note_cpu_started(t)
#  define sched_note_cpu_pause(t,c)
#  define sched_note_cpu_paused(t)
#  define sched_note_cpu_resume(t,c)
#  define sched_note_cpu_resumed(t)
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
void sched_note_premption(FAR struct tcb_s *tcb, bool locked);
#else
#  define sched_note_premption(t,l)
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
void sched_note_csection(FAR struct tcb_s *tcb, bool enter);
#else
#  define sched_note_csection(t,e)
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock(FAR struct tcb_s *tcb,
                         FAR volatile void *spinlock);
void sched_note_spinlocked(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock);
void sched_note_spinunlock(FAR struct tcb_s *tcb,
                           FAR volatile void *spinlock);
void sched_note_spinabort(FAR struct tcb_s *tcb,
                          FAR volatile void *spinlock);
#else
#  define sched_note_spinlock(t,s)
#  define sched_note_spinlocked(t,s)
#  define sched_note_spinunlock(t,s)
#  define sched_note_spinabort(t,s)
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr, int argc, ...);
void sched_note_syscall_leave(int nr, uintptr_t result);
#else
#  define sched_note_syscall_enter(n,a...)
#  define sched_note_syscall_leave(n,r)
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter);
#else
#  define sched_note_irqhandler(i,h,e)
#endif

/****************************************************************************
 * Name: note_add
 *
 * Description:
 *   Add the variable length note to the transport layer
 *
 * Input Parameters:
 *   note    - The note buffer
 *   notelen - The buffer length
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   We are within a critical section.
 *
 ****************************************************************************/

void note_add(FAR const uint8_t *note, uint8_t notelen);

#else /* CONFIG_SCHED_INSTRUMENTATION */

#  define sched_note_start(t)
#  define sched_note_stop(t)
#  define sched_note_suspend(t)
#  define sched_note_resume(t)
#  define sched_note_cpu_start(t,c)
#  define sched_note_cpu_started(t)
#  define sched_note_cpu_pause(t,c)
#  define sched_note_cpu_paused(t)
#  define sched_note_cpu_resume(t,c)
#  define sched_note_cpu_resumed(t)
#  define sched_note_premption(t,l)
#  define sched_note_csection(t,e)
#  define sched_note_spinlock(t,s)
#  define sched_note_spinlocked(t,s)
#  define sched_note_spinunlock(t,s)
#  define sched_note_spinabort(t,s)
#  define sched_note_syscall_enter(n,a...)
#  define sched_note_syscall_leave(n,r)
#  define sched_note_irqhandler(i,h,e)

#endif /* CONFIG_SCHED_INSTRUMENTATION */
#endif /* __INCLUDE_NUTTX_SCHED_NOTE_H */
