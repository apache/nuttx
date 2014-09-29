/****************************************************************************
 * sched/sched/sched.h
 *
 *   Copyright (C) 2007-2014 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_SCHED_SCHED_H
#define __SCHED_SCHED_SCHED_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <queue.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Although task IDs can take the (positive, non-zero)
 * range of pid_t, the number of tasks that will be supported
 * at any one time is (artificially) limited by the CONFIG_MAX_TASKS
 * configuration setting. Limiting the number of tasks speeds certain
 * OS functions (this is the only limitation in the number of
 * tasks built into the design).
 */

#define MAX_TASKS_MASK      (CONFIG_MAX_TASKS-1)
#define PIDHASH(pid)        ((pid) & MAX_TASKS_MASK)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This structure defines the format of the hash table that is used to (1)
 * determine if a task ID is unique, and (2) to map a process ID to its
 * corresponding TCB.
 *
 * NOTE also that CPU load measurement data is retained in his table vs. in
 * the TCB which would seem to be the more logic place.  It is place in the
 * hash table, instead, to facilitate CPU load adjustments on all threads
 * during timer interrupt handling. sched_foreach() could do this too, but
 * this would require a little more overhead.
 */

struct pidhash_s
{
  FAR struct tcb_s *tcb;       /* TCB assigned to this PID */
  pid_t pid;                   /* The full PID value */
#ifdef CONFIG_SCHED_CPULOAD
  uint32_t ticks;              /* Number of ticks on this thread */
#endif
};

/* This structure defines an element of the g_tasklisttable[].
 * This table is used to map a task_state enumeration to the
 * corresponding task list.
 */

struct tasklist_s
{
  DSEG volatile dq_queue_t *list; /* Pointer to the task list */
  bool prioritized;               /* true if the list is prioritized */
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* Declared in os_start.c ***************************************************/

/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists need
 * to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  The head of this
 * list is the currently active task; the tail of this list is always the
 * IDLE task.
 */

extern volatile dq_queue_t g_readytorun;

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

extern volatile dq_queue_t g_pendingtasks;

/* This is the list of all tasks that are blocked waiting for a semaphore */

extern volatile dq_queue_t g_waitingforsemaphore;

/* This is the list of all tasks that are blocked waiting for a signal */

#ifndef CONFIG_DISABLE_SIGNALS
extern volatile dq_queue_t g_waitingforsignal;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-empty.
 */

#ifndef CONFIG_DISABLE_MQUEUE
extern volatile dq_queue_t g_waitingformqnotempty;
#endif

/* This is the list of all tasks that are blocked waiting for a message
 * queue to become non-full.
 */

#ifndef CONFIG_DISABLE_MQUEUE
extern volatile dq_queue_t g_waitingformqnotfull;
#endif

/* This is the list of all tasks that are blocking waiting for a page fill */

#ifdef CONFIG_PAGING
extern volatile dq_queue_t g_waitingforfill;
#endif

/* This the list of all tasks that have been initialized, but not yet
 * activated. NOTE:  This is the only list that is not prioritized.
 */

extern volatile dq_queue_t g_inactivetasks;

/* These are lists of dayed memory deallocations that need to be handled
 * within the IDLE loop or worker thread.  These deallocations get queued
 * by sched_kufree and sched_kfree() if the OS needs to deallocate memory
 * while it is within an interrupt handler.
 */

extern volatile sq_queue_t g_delayed_kufree;

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
extern volatile sq_queue_t g_delayed_kfree;
#endif

/* This is the value of the last process ID assigned to a task */

extern volatile pid_t g_lastpid;

/* The following hash table is used for two things:
 *
 * 1. This hash table greatly speeds the determination of a new unique
 *    process ID for a task, and
 * 2. Is used to quickly map a process ID into a TCB.
 *
 * It has the side effects of using more memory and limiting the number
 * of tasks to CONFIG_MAX_TASKS.
 */

extern struct pidhash_s g_pidhash[CONFIG_MAX_TASKS];

/* This is a table of task lists.  This table is indexed by the task state
 * enumeration type (tstate_t) and provides a pointer to the associated
 * static task list (if there is one) as well as a boolean indication as to
 * if the list is an ordered list or not.
 */

extern const struct tasklist_s g_tasklisttable[NUM_TASK_STATES];

#ifdef CONFIG_SCHED_CPULOAD
/* This is the total number of clock tick counts.  Essentially the
 * 'denominator' for all CPU load calculations.
 */

extern volatile uint32_t g_cpuload_total;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

bool sched_addreadytorun(FAR struct tcb_s *rtrtcb);
bool sched_removereadytorun(FAR struct tcb_s *rtrtcb);
bool sched_addprioritized(FAR struct tcb_s *newTcb, DSEG dq_queue_t *list);
bool sched_mergepending(void);
void sched_addblocked(FAR struct tcb_s *btcb, tstate_t task_state);
void sched_removeblocked(FAR struct tcb_s *btcb);
int  sched_setpriority(FAR struct tcb_s *tcb, int sched_priority);

#ifdef CONFIG_PRIORITY_INHERITANCE
int  sched_reprioritize(FAR struct tcb_s *tcb, int sched_priority);
#else
#  define sched_reprioritize(tcb,sched_priority) \
     sched_setpriority(tcb,sched_priority)
#endif


#ifdef CONFIG_SCHED_TICKLESS
unsigned int sched_timer_cancel(void);
void sched_timer_resume(void);
void sched_timer_reassess(void);
#else
#  define sched_timer_cancel() (0)
#  define sched_timer_resume()
#  define sched_timer_reassess()
#endif

#if defined(CONFIG_SCHED_CPULOAD) && !defined(CONFIG_SCHED_CPULOAD_EXTCLK)
void weak_function sched_process_cpuload(void);
#endif

bool sched_verifytcb(FAR struct tcb_s *tcb);
int  sched_releasetcb(FAR struct tcb_s *tcb, uint8_t ttype);

#endif /* __SCHED_SCHED_SCHED_H */
