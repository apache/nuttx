/****************************************************************************
 * include/nuttx/wqueue.h
 *
 *   Copyright (C) 2009, 2011-2013 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_WQUEUE_H
#define __INCLUDE_NUTTX_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <signal.h>
#include <queue.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_SCHED_WORKQUEUE.  Create a dedicated "worker" thread to
 *   handle delayed processing from interrupt handlers.  This feature
 *   is required for some drivers but, if there are not complaints,
 *   can be safely disabled.  The worker thread also performs
 *   garbage collection -- completing any delayed memory deallocations
 *   from interrupt handlers.  If the worker thread is disabled,
 *   then that clean will be performed by the IDLE thread instead
 *   (which runs at the lowest of priority and may not be appropriate
 *   if memory reclamation is of high priority).  If CONFIG_SCHED_WORKQUEUE
 *   is enabled, then the following options can also be used:
 * CONFIG_SCHED_HPWORK - Build the high priority work queue. To preserve
 *   legacy behavior, CONFIG_SCHED_HPWORK is assumed to be true in a flat
 *   build (CONFIG_SCHED_KERNEL=n) but must be defined in kernel mode
 *   in order to build the high priority work queue.
 * CONFIG_SCHED_WORKPRIORITY - The execution priority of the worker
 *   thread.  Default: 192
 * CONFIG_SCHED_WORKPERIOD - How often the worker thread checks for
 *   work in units of microseconds.  Default: 50*1000 (50 MS).
 * CONFIG_SCHED_WORKSTACKSIZE - The stack size allocated for the worker
 *   thread.  Default: CONFIG_IDLETHREAD_STACKSIZE.
 * CONFIG_SIG_SIGWORK - The signal number that will be used to wake-up
 *   the worker thread.  Default: 17
 *
 * CONFIG_SCHED_LPWORK. If CONFIG_SCHED_WORKQUEUE is defined, then a single
 *   work queue is created by default.  If CONFIG_SCHED_LPWORK is also defined
 *   then an additional, lower-priority work queue will also be created.  This
 *   lower priority work queue is better suited for more extended processing
 *   (such as file system clean-up operations)
 * CONFIG_SCHED_LPWORKPRIORITY - The execution priority of the lower priority
 *   worker thread.  Default: 50
 * CONFIG_SCHED_LPWORKPERIOD - How often the lower priority worker thread
 *  checks for work in units of microseconds.  Default: 50*1000 (50 MS).
 * CONFIG_SCHED_LPWORKSTACKSIZE - The stack size allocated for the lower
 *   priority worker thread.  Default: CONFIG_IDLETHREAD_STACKSIZE.
 */

/* Is this a kernel build (CONFIG_NUTTX_KERNEL=y) */

#ifdef CONFIG_NUTTX_KERNEL

  /* Yes.. kernel worker threads are not built in a kernel build when we are
   * building the user-space libraries.
   */

#  ifndef __KERNEL__
#    undef CONFIG_SCHED_HPWORK
#    undef CONFIG_SCHED_LPWORK

  /* User-space worker threads are not built in a kernel build when we are
   * building the kernel-space libraries.
   */

#  else
#    undef CONFIG_SCHED_USRWORK
#  endif

  /* User-space worker threads are not built in a flat build
   * (CONFIG_NUTTX_KERNEL=n)
   */

#else

  /* To preserve legacy behavior, CONFIG_SCHED_HPWORK is assumed to be true
   * in a flat build (CONFIG_SCHED_KERNEL=n) but must be defined in kernel
   * mode in order to build the high priority work queue.
   *
   * In the kernel build, it is possible that no kernel work queues will be
   * built.  But in the flat build, the high priority work queue will always
   * be built.
   */

#  undef CONFIG_SCHED_HPWORK
#  undef CONFIG_SCHED_USRWORK
#  define CONFIG_SCHED_HPWORK 1
#endif

/* We never build the low priority work queue without building the high
 * priority work queue.
 */

#if defined(CONFIG_SCHED_LPWORK) && !defined(CONFIG_SCHED_HPWORK)
#  error "CONFIG_SCHED_LPWORK defined, but CONFIG_SCHED_HPWORK not defined"
#  undef CONFIG_SCHED_LPWORK
#endif

/* We might not be building any work queue support in this context */

#if !defined(CONFIG_SCHED_HPWORK) && !defined(CONFIG_SCHED_LPWORK) && !defined(CONFIG_SCHED_USRWORK)
#  undef CONFIG_SCHED_WORKQUEUE
#endif

#ifdef CONFIG_SCHED_WORKQUEUE

/* We are building work queues... Work queues need signal support */

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_DISABLE_SIGNALS)
#  warning "Worker thread support requires signals"
#endif

/* High priority, kernel work queue configuration ***************************/

#ifdef CONFIG_SCHED_HPWORK

#  ifndef CONFIG_SCHED_WORKPRIORITY
#    define CONFIG_SCHED_WORKPRIORITY 192
#  endif

#  ifndef CONFIG_SCHED_WORKPERIOD
#    define CONFIG_SCHED_WORKPERIOD (50*1000) /* 50 milliseconds */
#  endif

#  ifndef CONFIG_SCHED_WORKSTACKSIZE
#    define CONFIG_SCHED_WORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

/* Low priority kernel work queue configuration *****************************/

#ifdef CONFIG_SCHED_LPWORK

#  ifndef CONFIG_SCHED_LPWORKPRIORITY
#    define CONFIG_SCHED_LPWORKPRIORITY 50
#  endif

#  ifndef CONFIG_SCHED_LPWORKPERIOD
#    define CONFIG_SCHED_LPWORKPERIOD (50*1000) /* 50 milliseconds */
#  endif

#  ifndef CONFIG_SCHED_LPWORKSTACKSIZE
#    define CONFIG_SCHED_LPWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

/* The high priority worker thread should be higher priority than the low
 * priority worker thread.
 */

#if CONFIG_SCHED_LPWORKPRIORITY > CONFIG_SCHED_WORKPRIORITY
#  warning "The Lower priority worker thread has the higher priority"
#endif

#endif /* CONFIG_SCHED_LPWORK */
#endif /* CONFIG_SCHED_HPWORK */

/* User space work queue configuration **************************************/

#ifdef CONFIG_SCHED_USRWORK

#  ifndef CONFIG_SCHED_USRWORKPRIORITY
#    define CONFIG_SCHED_USRWORKPRIORITY 50
#  endif

#  ifndef CONFIG_SCHED_USRWORKPERIOD
#    define CONFIG_SCHED_USRWORKPERIOD (50*1000) /* 50 milliseconds */
#  endif

#  ifndef CONFIG_SCHED_USRWORKSTACKSIZE
#    define CONFIG_SCHED_USRWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

#endif /* CONFIG_SCHED_USRWORK */

/* How many worker threads are there?  In the user-space phase of a kernel
 * build, there will be no more than one.
 *
 * Work queue IDs (indices):
 *
 *   Kernel Work Queues:  There are none and any attempts to use them
 *     should generate errors.
 *
 *   User Work Queue:  Will be available if CONFIG_SCHED_USRWORK is defined
 */

#if defined(CONFIG_NUTTX_KERNEL) && !defined(__KERNEL__)
#  ifdef CONFIG_SCHED_USRWORK
#    define NWORKERS 1
#    define USRWORK 0
#  endif
#else

  /* In a flat build (CONFIG_NUTTX_KERNEL=n) or during the kernel phase of
   * the kernel build, there may be 0, 1, or 2 work queues.
   *
   * Work queue IDs (indices):
   *
   * Kernel Work Queues:
   *   HPWORK: This ID of the high priority work queue that should only be
   *     used for hi-priority, time-critical, driver bottom-half functions.
   *
   *   LPWORK: This is the ID of the low priority work queue that can be
   *     used for any purpose.  if CONFIG_SCHED_LPWORK is not defined, then
   *     there is only one kernel work queue and LPWORK == HPWORK.
   *
   * User Work Queue:
   *   USRWORK:  In the kernel phase a a kernel build, there should be no
   *     references to user-space work queues.  That would be an error. 
   *     Otherwise, in a flat build, user applications will use the lower
   *     priority work queue (if there is one).
   */

#  define HPWORK 0
#  ifdef CONFIG_SCHED_LPWORK
#    define LPWORK (HPWORK+1)
#    define NWORKERS 2
#  else
#    define LPWORK HPWORK
#    define NWORKERS 1
#  endif

#  ifndef CONFIG_NUTTX_KERNEL
#    define USRWORK LPWORK
#  endif

#endif /* CONFIG_NUTTX_KERNEL && !__KERNEL__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* This structure defines the state on one work queue.  This structure is
 * used internally by the OS and worker queue logic and should not be
 * accessed by application logic.
 */

struct wqueue_s
{
  pid_t             pid; /* The task ID of the worker thread */
  struct dq_queue_s q;   /* The queue of pending work */
};

/* Defines the work callback */

typedef void (*worker_t)(FAR void *arg);

/* Defines one entry in the work queue.  The user only needs this structure
 * in order to declare instances of the work structure.  Handling of all
 * fields is performed by the work APIs
 */

struct work_s
{
  struct dq_entry_s dq;  /* Implements a doubly linked list */
  worker_t  worker;      /* Work callback */
  FAR void *arg;         /* Callback argument */
  uint32_t  qtime;       /* Time work queued */
  uint32_t  delay;       /* Delay until work performed */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The state of each work queue.  This data structure is used internally by
 * the OS and worker queue logic and should not be accessed by application
 * logic.
 */

#ifdef CONFIG_NUTTX_KERNEL

  /* Play some games in the kernel mode build to assure that different
   * naming is used for the global work queue data structures.  This may
   * not be necessary but it safer.
   */

#  ifdef __KERNEL__
EXTERN struct wqueue_s g_kernelwork[NWORKERS];
#    define g_work g_kernelwork
#  else
EXTERN struct wqueue_s g_usrwork[NWORKERS];
#    define g_work g_usrwork
#  endif

#else /* CONFIG_NUTTX_KERNEL */

EXTERN struct wqueue_s g_work[NWORKERS];

#endif /* CONFIG_NUTTX_KERNEL */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_hpthread, work_lpthread, and work_usrthread
 *
 * Description:
 *   These are the worker threads that performs actions placed on the work
 *   lists.
 *
 *   work_hpthread and work_lpthread:  These are the kernel mode work queues
 *     (also build in the flat build).  One of these threads also performs
 *     periodic garbage collection (that is otherwise performed by the idle
 *     thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *
 *     These worker threads are started by the OS during normal bringup.
 *
 *   work_usrthread:  This is a user mode work queue.  It must be started
 *     by application code by calling work_usrstart().
 *
 *   All of these entrypoints are referenced by OS internally and should not
 *   not be accessed by application logic.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
int work_hpthread(int argc, char *argv[]);
#endif

#ifdef CONFIG_SCHED_LPWORK
int work_lpthread(int argc, char *argv[]);
#endif

#ifdef CONFIG_SCHED_USRWORK
int work_usrthread(int argc, char *argv[]);
#endif

/****************************************************************************
 * Name: work_usrstart
 *
 * Description:
 *   Start the user mode work queue.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_USRWORK
int work_usrstart(void);
#endif

/****************************************************************************
 * Name: work_queue
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of of execution (not the caller's).
 *
 *   The work structure is allocated by caller, but completely managed by
 *   the work queue logic.  The caller should never modify the contents of
 *   the work queue structure; the caller should not call work_queue()
 *   again until either (1) the previous work has been performed and removed
 *   from the queue, or (2) work_cancel() has been called to cancel the work
 *   and remove it from the work queue.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will invoked
 *            on the worker thread of execution.
 *   arg    - The argument that will be passed to the workder callback when
 *            int is invoked.
 *   delay  - Delay (in clock ticks) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, uint32_t delay);

/****************************************************************************
 * Name: work_cancel
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been canceled, it may be re-queue by calling work_queue()
 *   again.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *   work   - The previously queue work structure to cancel
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_cancel(int qid, FAR struct work_s *work);

/****************************************************************************
 * Name: work_signal
 *
 * Description:
 *   Signal the worker thread to process the work queue now.  This function
 *   is used internally by the work logic but could also be used by the
 *   user to force an immediate re-assessment of pending work.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_signal(int qid);

/****************************************************************************
 * Name: work_available
 *
 * Description:
 *   Check if the work structure is available.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   true if available; false if busy (i.e., there is still pending work).
 *
 ****************************************************************************/

#define work_available(work) ((work)->worker == NULL)

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SCHED_WORKQUEUE */
#endif /* __INCLUDE_NUTTX_WQUEUE_H */
