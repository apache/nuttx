/****************************************************************************
 * sched/sched/sched.h
 *
 *   Copyright (C) 2007-2014, 2016, 2018 Gregory Nutt. All rights reserved.
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

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>

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

#if CONFIG_MAX_TASKS & (CONFIG_MAX_TASKS - 1)
#  error CONFIG_MAX_TASKS must be power of 2
#endif

#define MAX_TASKS_MASK           (CONFIG_MAX_TASKS-1)
#define PIDHASH(pid)             ((pid) & MAX_TASKS_MASK)

/* These are macros to access the current CPU and the current task on a CPU.
 * These macros are intended to support a future SMP implementation.
 * NOTE: this_task() for SMP is implemented in sched_thistask.c if the CPU
 * supports disabling of inter-processor interrupts or if it supports the
 * atomic fetch add operation.
 */

#ifdef CONFIG_SMP
#  define current_task(cpu)      ((FAR struct tcb_s *)g_assignedtasks[cpu].head)
#  define this_cpu()             up_cpu_index()
#  if !defined(CONFIG_ARCH_GLOBAL_IRQDISABLE) && !defined(CONFIG_ARCH_HAVE_FETCHADD)
#    define this_task()          (current_task(this_cpu()))
#  endif
#else
#  define current_task(cpu)      ((FAR struct tcb_s *)g_readytorun.head)
#  define this_cpu()             (0)
#  define this_task()            (current_task(this_cpu()))
#endif

/* This macro returns the running task which may different from this_task()
 * during interrupt level context switches.
 */

#define running_task() \
  (up_interrupt_context() ? g_running_tasks[this_cpu()] : this_task())

/* List attribute flags */

#define TLIST_ATTR_PRIORITIZED   (1 << 0) /* Bit 0: List is prioritized */
#define TLIST_ATTR_INDEXED       (1 << 1) /* Bit 1: List is indexed by CPU */
#define TLIST_ATTR_RUNNABLE      (1 << 2) /* Bit 2: List includes running tasks */

#define __TLIST_ATTR(s)          g_tasklisttable[s].attr
#define TLIST_ISPRIORITIZED(s)   ((__TLIST_ATTR(s) & TLIST_ATTR_PRIORITIZED) != 0)
#define TLIST_ISINDEXED(s)       ((__TLIST_ATTR(s) & TLIST_ATTR_INDEXED) != 0)
#define TLIST_ISRUNNABLE(s)      ((__TLIST_ATTR(s) & TLIST_ATTR_RUNNABLE) != 0)

#define __TLIST_HEAD(s)          (FAR dq_queue_t *)g_tasklisttable[s].list
#define __TLIST_HEADINDEXED(s,c) (&(__TLIST_HEAD(s))[c])

#ifdef CONFIG_SMP
#  define TLIST_HEAD(s,c) \
  ((TLIST_ISINDEXED(s)) ? __TLIST_HEADINDEXED(s,c) : __TLIST_HEAD(s))
#  define TLIST_BLOCKED(s)       __TLIST_HEAD(s)
#else
#  define TLIST_HEAD(s)          __TLIST_HEAD(s)
#  define TLIST_BLOCKED(s)       __TLIST_HEAD(s)
#endif

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

/* This structure defines an element of the g_tasklisttable[].  This table
 * is used to map a task_state enumeration to the corresponding task list.
 */

struct tasklist_s
{
  DSEG volatile dq_queue_t *list; /* Pointer to the task list */
  uint8_t attr;                   /* List attribute flags */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Declared in nx_start.c ***************************************************/

/* The state of a task is indicated both by the task_state field of the TCB
 * and by a series of task lists.  All of these tasks lists are declared
 * below. Although it is not always necessary, most of these lists are
 * prioritized so that common list handling logic can be used (only the
 * g_readytorun, the g_pendingtasks, and the g_waitingforsemaphore lists
 * need to be prioritized).
 */

/* This is the list of all tasks that are ready to run.  This is a
 * prioritized list with head of the list holding the highest priority
 * (unassigned) task.  In the non-SMP case, the head of this list is the
 * currently active task and the tail of this list, the lowest priority
 * task, is always the IDLE task.
 */

extern volatile dq_queue_t g_readytorun;

#ifdef CONFIG_SMP
/* In order to support SMP, the function of the g_readytorun list changes,
 * The g_readytorun is still used but in the SMP case it will contain only:
 *
 *  - Only tasks/threads that are eligible to run, but not currently running,
 *    and
 *  - Tasks/threads that have not been assigned to a CPU.
 *
 * Otherwise, the TCB will be retained in an assigned task list,
 * g_assignedtasks.  As its name suggests, on 'g_assignedtasks queue for CPU
 * 'n' would contain only tasks/threads that are assigned to CPU 'n'.  Tasks/
 * threads would be assigned a particular CPU by one of two mechanisms:
 *
 *  - (Semi-)permanently through an RTOS interfaces such as
 *    pthread_attr_setaffinity(), or
 *  - Temporarily through scheduling logic when a previously unassigned task
 *    is made to run.
 *
 * Tasks/threads that are assigned to a CPU via an interface like
 * pthread_attr_setaffinity() would never go into the g_readytorun list, but
 * would only go into the g_assignedtasks[n] list for the CPU 'n' to which
 * the thread has been assigned.  Hence, the g_readytorun list would hold
 * only unassigned tasks/threads.
 *
 * Like the g_readytorun list in in non-SMP case, each g_assignedtask[] list
 * is prioritized:  The head of the list is the currently active task on this
 * CPU.  Tasks after the active task are ready-to-run and assigned to this
 * CPU. The tail of this assigned task list, the lowest priority task, is
 * always the CPU's IDLE task.
 */

extern volatile dq_queue_t g_assignedtasks[CONFIG_SMP_NCPUS];

/* g_running_tasks[] holds a references to the running task for each cpu.
 * It is valid only when up_interrupt_context() returns true.
 */

extern FAR struct tcb_s *g_running_tasks[CONFIG_SMP_NCPUS];

#else

extern FAR struct tcb_s *g_running_tasks[1];

#endif

/* This is the list of all tasks that are ready-to-run, but cannot be placed
 * in the g_readytorun list because:  (1) They are higher priority than the
 * currently active task at the head of the g_readytorun list, and (2) the
 * currently active task has disabled pre-emption.
 */

extern volatile dq_queue_t g_pendingtasks;

/* This is the list of all tasks that are blocked waiting for a semaphore */

extern volatile dq_queue_t g_waitingforsemaphore;

/* This is the list of all tasks that are blocked waiting for a signal */

extern volatile dq_queue_t g_waitingforsignal;

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

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
extern volatile sq_queue_t g_delayed_kfree;
#endif

#ifndef CONFIG_BUILD_KERNEL
/* REVISIT:  It is not safe to defer user allocation in the kernel mode
 * build.  Why?  Because the correct user context will not be in place
 * when these deferred de-allocations are performed.  In order to make
 * this work, we would need to do something like:  (1) move g_delayed_kufree
 * into the group structure, then traverse the groups to collect garbage on
 * a group-by-group basis.
 */

extern volatile sq_queue_t g_delayed_kufree;
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

/* This is a table of task lists.  This table is indexed by the task stat
 * enumeration type (tstate_t) and provides a pointer to the associated
 * static task list (if there is one) as well as a a set of attribute flags
 * indicating properties of the list, for example, if the list is an
 * ordered list or not.
 */

extern const struct tasklist_s g_tasklisttable[NUM_TASK_STATES];

#ifdef CONFIG_SCHED_CPULOAD
/* This is the total number of clock tick counts.  Essentially the
 * 'denominator' for all CPU load calculations.
 */

extern volatile uint32_t g_cpuload_total;
#endif

/* Declared in sched_lock.c *************************************************/

/* Pre-emption is disabled via the interface sched_lock(). sched_lock()
 * works by preventing context switches from the currently executing tasks.
 * This prevents other tasks from running (without disabling interrupts) and
 * gives the currently executing task exclusive access to the (single) CPU
 * resources. Thus, sched_lock() and its companion, sched_unlock(), are
 * used to implement some critical sections.
 *
 * In the single CPU case, Pre-emption is disabled using a simple lockcount
 * in the TCB. When the scheduling is locked, the lockcount is incremented;
 * when the scheduler is unlocked, the lockcount is decremented. If the
 * lockcount for the task at the head of the g_readytorun list has a
 * lockcount > 0, then pre-emption is disabled.
 *
 * No special protection is required since only the executing task can
 * modify its lockcount.
 */

#ifdef CONFIG_SMP
/* In the multiple CPU, SMP case, disabling context switches will not give a
 * task exclusive access to the (multiple) CPU resources (at least without
 * stopping the other CPUs): Even though pre-emption is disabled, other
 * threads will still be executing on the other CPUS.
 *
 * There are additional rules for this multi-CPU case:
 *
 * 1. There is a global lock count 'g_cpu_lockset' that includes a bit for
 *    each CPU: If the bit is '1', then the corresponding CPU has the
 *    scheduler locked; if '0', then the CPU does not have the scheduler
 *    locked.
 * 2. Scheduling logic would set the bit associated with the cpu in
 *    'g_cpu_lockset' when the TCB at the head of the g_assignedtasks[cpu]
 *    list transitions has 'lockcount' > 0. This might happen when sched_lock()
 *    is called, or after a context switch that changes the TCB at the
 *    head of the g_assignedtasks[cpu] list.
 * 3. Similarly, the cpu bit in the global 'g_cpu_lockset' would be cleared
 *    when the TCB at the head of the g_assignedtasks[cpu] list has
 *    'lockcount' == 0. This might happen when sched_unlock() is called, or
 *    after a context switch that changes the TCB at the head of the
 *    g_assignedtasks[cpu] list.
 * 4. Modification of the global 'g_cpu_lockset' must be protected by a
 *    spinlock, 'g_cpu_schedlock'. That spinlock would be taken when
 *    sched_lock() is called, and released when sched_unlock() is called.
 *    This assures that the scheduler does enforce the critical section.
 *    NOTE: Because of this spinlock, there should never be more than one
 *    bit set in 'g_cpu_lockset'; attempts to set additional bits should
 *    be cause the CPU to block on the spinlock.  However, additional bits
 *    could get set in 'g_cpu_lockset' due to the context switches on the
 *    various CPUs.
 * 5. Each the time the head of a g_assignedtasks[] list changes and the
 *    scheduler modifies 'g_cpu_lockset', it must also set 'g_cpu_schedlock'
 *    depending on the new state of 'g_cpu_lockset'.
 * 5. Logic that currently uses the currently running tasks lockcount
 *    instead uses the global 'g_cpu_schedlock'. A value of SP_UNLOCKED
 *    means that no CPU has pre-emption disabled; SP_LOCKED means that at
 *    least one CPU has pre-emption disabled.
 */

extern volatile spinlock_t g_cpu_schedlock SP_SECTION;

/* Used to keep track of which CPU(s) hold the IRQ lock. */

extern volatile spinlock_t g_cpu_locksetlock SP_SECTION;
extern volatile cpu_set_t g_cpu_lockset SP_SECTION;

/* Used to lock tasklist to prevent from concurrent access */

extern volatile spinlock_t g_cpu_tasklistlock SP_SECTION;

#if defined(CONFIG_ARCH_HAVE_FETCHADD) && !defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
/* This is part of the sched_lock() logic to handle atomic operations when
 * locking the scheduler.
 */

extern volatile int16_t g_global_lockcount;
#endif

#endif /* CONFIG_SMP */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Task list manipulation functions */

bool sched_addreadytorun(FAR struct tcb_s *rtrtcb);
bool sched_removereadytorun(FAR struct tcb_s *rtrtcb);
bool sched_addprioritized(FAR struct tcb_s *tcb, DSEG dq_queue_t *list);
void sched_mergeprioritized(FAR dq_queue_t *list1, FAR dq_queue_t *list2,
                            uint8_t task_state);
bool sched_mergepending(void);
void sched_addblocked(FAR struct tcb_s *btcb, tstate_t task_state);
void sched_removeblocked(FAR struct tcb_s *btcb);
int  nxsched_setpriority(FAR struct tcb_s *tcb, int sched_priority);

/* Priority inheritance support */

#ifdef CONFIG_PRIORITY_INHERITANCE
int  nxsched_reprioritize(FAR struct tcb_s *tcb, int sched_priority);
#else
#  define nxsched_reprioritize(tcb,sched_priority) \
     nxsched_setpriority(tcb,sched_priority)
#endif

/* Support for tickless operation */

#ifdef CONFIG_SCHED_TICKLESS
unsigned int sched_timer_cancel(void);
void sched_timer_resume(void);
void sched_timer_reassess(void);
#else
#  define sched_timer_cancel() (0)
#  define sched_timer_resume()
#  define sched_timer_reassess()
#endif

/* Scheduler policy support */

#if CONFIG_RR_INTERVAL > 0
uint32_t sched_roundrobin_process(FAR struct tcb_s *tcb, uint32_t ticks,
                                  bool noswitches);
#endif

#ifdef CONFIG_SCHED_SPORADIC
int  sched_sporadic_initialize(FAR struct tcb_s *tcb);
int  sched_sporadic_start(FAR struct tcb_s *tcb);
int  sched_sporadic_stop(FAR struct tcb_s *tcb);
int  sched_sporadic_reset(FAR struct tcb_s *tcb);
int  sched_sporadic_resume(FAR struct tcb_s *tcb);
int  sched_sporadic_suspend(FAR struct tcb_s *tcb);
uint32_t sched_sporadic_process(FAR struct tcb_s *tcb, uint32_t ticks,
                                bool noswitches);
void sched_sporadic_lowpriority(FAR struct tcb_s *tcb);
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION
void sched_suspend(FAR struct tcb_s *tcb);
void sched_continue(FAR struct tcb_s *tcb);
#endif

#ifdef CONFIG_SMP
#if defined(CONFIG_ARCH_GLOBAL_IRQDISABLE) || defined(CONFIG_ARCH_HAVE_FETCHADD)
FAR struct tcb_s *this_task(void);
#endif

int  sched_cpu_select(cpu_set_t affinity);
int  sched_cpu_pause(FAR struct tcb_s *tcb);

irqstate_t sched_tasklist_lock(void);
void sched_tasklist_unlock(irqstate_t lock);

#if defined(CONFIG_ARCH_HAVE_FETCHADD) && !defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
#  define sched_islocked_global() \
     (spin_islocked(&g_cpu_schedlock) || g_global_lockcount > 0)
#else
#  define sched_islocked_global() \
     spin_islocked(&g_cpu_schedlock)
#endif

#  define sched_islocked_tcb(tcb) sched_islocked_global()

#else
#  define sched_cpu_select(a)     (0)
#  define sched_cpu_pause(t)      (-38)  /* -ENOSYS */
#  define sched_islocked_tcb(tcb) ((tcb)->lockcount > 0)
#endif

#if defined(CONFIG_SCHED_CPULOAD) && !defined(CONFIG_SCHED_CPULOAD_EXTCLK)
/* CPU load measurement support */

void weak_function nxsched_process_cpuload(void);
#endif

/* Critical section monitor */

#ifdef CONFIG_SCHED_CRITMONITOR
void sched_critmon_preemption(FAR struct tcb_s *tcb, bool state);
void sched_critmon_csection(FAR struct tcb_s *tcb, bool state);
void sched_critmon_resume(FAR struct tcb_s *tcb);
void sched_critmon_suspend(FAR struct tcb_s *tcb);
#endif

/* TCB operations */

bool sched_verifytcb(FAR struct tcb_s *tcb);
int  sched_releasetcb(FAR struct tcb_s *tcb, uint8_t ttype);

#endif /* __SCHED_SCHED_SCHED_H */
