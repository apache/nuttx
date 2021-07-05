/****************************************************************************
 * include/nuttx/wqueue.h
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

#ifndef __INCLUDE_NUTTX_WQUEUE_H
#define __INCLUDE_NUTTX_WQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <queue.h>

#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_SCHED_WORKQUEUE.  Not selectable.  Set by the configuration system
 *   if either CONFIG_SCHED_HPWORK or CONFIG_SCHED_LPWORK are selected.
 * CONFIG_SCHED_HPWORK.  Create a dedicated "worker" thread to
 *   handle delayed processing from interrupt handlers.  This feature
 *   is required for some drivers but, if there are not complaints,
 *   can be safely disabled.  The worker thread also performs
 *   from interrupt handlers.  If the worker thread is disabled,
 *   then that clean will be performed by the IDLE thread instead
 *   (which runs at the lowest of priority and may not be appropriate
 *   if memory reclamation is of high priority).  If CONFIG_SCHED_HPWORK
 *   is enabled, then the following options can also be used:
 * CONFIG_SCHED_HPNTHREADS - The number of thread in the high-priority
 *   queue's thread pool.  Default: 1
 * CONFIG_SCHED_HPWORKPRIORITY - The execution priority of the high-
 *   priority worker thread.  Default: 224
 * CONFIG_SCHED_HPWORKSTACKSIZE - The stack size allocated for the worker
 *   thread.  Default: 2048.
 * CONFIG_SIG_SIGWORK - The signal number that will be used to wake-up
 *   the worker thread.  Default: 17
 *
 * CONFIG_SCHED_LPWORK. If CONFIG_SCHED_LPWORK is selected then a lower-
 *   priority work queue will be created.  This lower priority work queue
 *   is better suited for more extended processing (such as file system
 *   clean-up operations)
 * CONFIG_SCHED_LPNTHREADS - The number of thread in the low-priority queue's
 *   thread pool.  Default: 1
 * CONFIG_SCHED_LPWORKPRIORITY - The minimum execution priority of the lower
 *   priority worker thread.  Default: 50
 * CONFIG_SCHED_LPWORKPRIOMAX - The maximum execution priority of the lower
 *   priority worker thread.  Default: 176
 * CONFIG_SCHED_LPWORKSTACKSIZE - The stack size allocated for the lower
 *   priority worker thread.  Default: 2048.
 *
 * The user-mode work queue is only available in the protected or kernel
 * builds.  This those configurations, the user-mode work queue provides the
 * same (non-standard) facility for use by applications.
 *
 * CONFIG_LIB_USRWORK. If CONFIG_LIB_USRWORK is also defined then the
 *   user-mode work queue will be created.
 * CONFIG_LIB_USRWORKPRIORITY - The minimum execution priority of the lower
 *   priority worker thread.  Default: 100
 * CONFIG_LIB_USRWORKSTACKSIZE - The stack size allocated for the lower
 *   priority worker thread.  Default: 2048.
 */

/* Is this a flat build (CONFIG_BUILD_FLAT=y) */

#if defined(CONFIG_BUILD_FLAT)

  /* Yes.. user-space worker threads are not built in a flat build */

#  undef CONFIG_LIB_USRWORK

#elif !defined(__KERNEL__)

  /* Kernel worker threads are not built in a kernel build when we are
   * building the user-space libraries.
   */

#  undef CONFIG_SCHED_HPWORK
#  undef CONFIG_SCHED_LPWORK
#  undef CONFIG_SCHED_WORKQUEUE

  /* User-space worker threads are not built in a kernel build when we are
   * building the kernel-space libraries (but we still need to know that it
   * is configured).
   */

#endif

/* High priority, kernel work queue configuration ***************************/

#ifdef CONFIG_SCHED_HPWORK

#  ifndef CONFIG_SCHED_HPNTHREADS
#    define CONFIG_SCHED_HPNTHREADS 1
#  endif

#  ifndef CONFIG_SCHED_HPWORKPRIORITY
#    define CONFIG_SCHED_HPWORKPRIORITY 224
#  endif

#  ifndef CONFIG_SCHED_HPWORKSTACKSIZE
#    define CONFIG_SCHED_HPWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

#endif /* CONFIG_SCHED_HPWORK */

/* Low priority kernel work queue configuration *****************************/

#ifdef CONFIG_SCHED_LPWORK

#  ifndef CONFIG_SCHED_LPNTHREADS
#    define CONFIG_SCHED_LPNTHREADS 1
#endif

#  ifndef CONFIG_SCHED_LPWORKPRIORITY
#    define CONFIG_SCHED_LPWORKPRIORITY 50
#  endif

#  ifndef CONFIG_SCHED_LPWORKPRIOMAX
#    ifdef CONFIG_SCHED_HPWORK
#      define CONFIG_SCHED_LPWORKPRIOMAX (CONFIG_SCHED_HPWORKPRIORITY-16)
#    else
#      define CONFIG_SCHED_LPWORKPRIOMAX 176
#    endif
#  endif

#  ifdef CONFIG_SCHED_HPWORK
#    if CONFIG_SCHED_LPWORKPRIORITY >= CONFIG_SCHED_HPWORKPRIORITY
#      error CONFIG_SCHED_LPWORKPRIORITY >= CONFIG_SCHED_HPWORKPRIORITY
#    endif
#    if CONFIG_SCHED_LPWORKPRIOMAX >= CONFIG_SCHED_HPWORKPRIORITY
#      error CONFIG_SCHED_LPWORKPRIOMAX >= CONFIG_SCHED_HPWORKPRIORITY
#    endif
#  endif

#  if CONFIG_SCHED_LPWORKPRIORITY > CONFIG_SCHED_LPWORKPRIOMAX
#    error CONFIG_SCHED_LPWORKPRIORITY > CONFIG_SCHED_LPWORKPRIOMAX
#  endif

#  ifndef CONFIG_SCHED_LPWORKSTACKSIZE
#    define CONFIG_SCHED_LPWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

#  ifdef CONFIG_WORK_HPWORK
  /* The high priority worker thread should be higher priority than the low
   * priority worker thread.
   */

#  if CONFIG_SCHED_LPWORKPRIORITY > CONFIG_SCHED_HPWORKPRIORITY
#    warning "The Lower priority worker thread has the higher priority"
#  endif
#endif

#endif /* CONFIG_SCHED_LPWORK */

/* User space work queue configuration **************************************/

#ifdef CONFIG_LIB_USRWORK

#  ifndef CONFIG_LIB_USRWORKPRIORITY
#    define CONFIG_LIB_USRWORKPRIORITY 100
#  endif

#  ifndef CONFIG_LIB_USRWORKSTACKSIZE
#    define CONFIG_LIB_USRWORKSTACKSIZE CONFIG_IDLETHREAD_STACKSIZE
#  endif

#endif /* CONFIG_LIB_USRWORK */

/* Work queue IDs:
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

#if defined(CONFIG_LIB_USRWORK) && !defined(__KERNEL__)
/* User mode */

#  define USRWORK  2          /* User mode work queue */
#  define HPWORK   USRWORK    /* Redirect kernel-mode references */
#  define LPWORK   USRWORK

#else
/* Kernel mode */

#  define HPWORK   0          /* High priority, kernel-mode work queue */
#  if defined(CONFIG_SCHED_LPWORK) && defined(CONFIG_SCHED_HPWORK)
#    define LPWORK (HPWORK+1) /* Low priority, kernel-mode work queue */
#  else
#    define LPWORK HPWORK     /* Redirect low-priority references */
#  endif
#  define USRWORK  LPWORK     /* Redirect user-mode references */

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Defines the work callback */

typedef CODE void (*worker_t)(FAR void *arg);

/* Defines one entry in the work queue.  The user only needs this structure
 * in order to declare instances of the work structure.  Handling of all
 * fields is performed by the work APIs
 */

struct work_s
{
  struct dq_entry_s dq;  /* Implements a doubly linked list */
  worker_t  worker;      /* Work callback */
  FAR void *arg;         /* Callback argument */
  clock_t qtime;         /* Time work queued */
  clock_t delay;         /* Delay until work performed */
};

/* This is an enumeration of the various events that may be
 * notified via work_notifier_signal().
 */

enum work_evtype_e
{
  WORK_IOB_AVAIL  = 1,     /* Notify availability of an IOB */
  WORK_NET_DOWN,           /* Notify that the network is down */
  WORK_TCP_READAHEAD,      /* Notify that TCP read-ahead data is available */
  WORK_TCP_WRITEBUFFER,    /* Notify that TCP write buffer is empty */
  WORK_TCP_DISCONNECT,     /* Notify loss of TCP connection */
  WORK_UDP_READAHEAD,      /* Notify that UDP read-ahead data is available */
  WORK_UDP_WRITEBUFFER,    /* Notify that UDP write buffer is empty */
  WORK_NETLINK_RESPONSE,   /* Notify that Netlink response is available */
  WORK_CAN_READAHEAD,      /* Notify that CAN read-ahead data is available */
  WORK_USB_MSC_CONNECT,    /* Notify that an USB MSC connect occurred */
  WORK_USB_MSC_DISCONNECT  /* Notify that an USB MSC connect occurred */
};

/* This structure describes one notification and is provided as input to
 * to work_notifier_setup().  This input is copied by work_notifier_setup()
 * into an allocated instance of struct work_notifier_entry_s and need not
 * persist on the caller's side.
 */

struct work_notifier_s
{
  uint8_t evtype;      /* See enum work_evtype_e */
  uint8_t qid;         /* The work queue to use: HPWORK or LPWORK */
  FAR void *qualifier; /* Event qualifier value */
  FAR void *arg;       /* User-defined worker function argument */
  worker_t worker;     /* The worker function to schedule */
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: work_usrstart
 *
 * Description:
 *   Start the user mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The task ID of the worker thread is returned on success.  A negated
 *   errno value is returned on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_USRWORK) && !defined(__KERNEL__)
int work_usrstart(void);
#endif

/****************************************************************************
 * Name: work_queue
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of execution (not the caller's).
 *
 *   The work structure is allocated and must be initialized to all zero by
 *   the caller.  Otherwise, the work structure is completely managed by the
 *   work queue logic.  The caller should never modify the contents of the
 *   work queue structure directly.  If work_queue() is called before the
 *   previous work has been performed and removed from the queue, then any
 *   pending work will be canceled and lost.
 *
 * Input Parameters:
 *   qid    - The work queue ID
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will be
 *            invoked on the worker thread of execution.
 *   arg    - The argument that will be passed to the worker callback when
 *            it is invoked.
 *   delay  - Delay (in clock ticks) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, clock_t delay);

/****************************************************************************
 * Name: work_cancel
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been cancelled, it may be requeued by calling
 *   work_queue() again.
 *
 * Input Parameters:
 *   qid    - The work queue ID
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 *   -ENOENT - There is no such work queued.
 *   -EINVAL - An invalid work queue was specified
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
 * Input Parameters:
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
 * Input Parameters:
 *   work - The work queue structure to check.
 *   None
 *
 * Returned Value:
 *   true if available; false if busy (i.e., there is still pending work).
 *
 ****************************************************************************/

#define work_available(work) ((work)->worker == NULL)

/****************************************************************************
 * Name: lpwork_boostpriority
 *
 * Description:
 *   Called by the work queue client to assure that the priority of the low-
 *   priority worker thread is at least at the requested level, reqprio. This
 *   function would normally be called just before calling work_queue().
 *
 * Input Parameters:
 *   reqprio - Requested minimum worker thread priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_LPWORK) && defined(CONFIG_PRIORITY_INHERITANCE)
void lpwork_boostpriority(uint8_t reqprio);
#endif

/****************************************************************************
 * Name: lpwork_restorepriority
 *
 * Description:
 *   This function is called to restore the priority after it was previously
 *   boosted.  This is often done by client logic on the worker thread when
 *   the scheduled work completes.  It will check if we need to drop the
 *   priority of the worker thread.
 *
 * Input Parameters:
 *   reqprio - Previously requested minimum worker thread priority to be
 *     "unboosted"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_LPWORK) && defined(CONFIG_PRIORITY_INHERITANCE)
void lpwork_restorepriority(uint8_t reqprio);
#endif

/****************************************************************************
 * Name: work_notifier_setup
 *
 * Description:
 *   Set up to provide a notification when event occurs.
 *
 * Input Parameters:
 *   info - Describes the work notification.
 *
 * Returned Value:
 *   > 0   - The key which may be used later in a call to
 *           work_notifier_teardown().
 *   == 0  - Not used (reserved for wrapper functions).
 *   < 0   - An unexpected error occurred and no notification will be sent.
 *           The returned value is a negated errno value that indicates the
 *           nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WQUEUE_NOTIFIER
int work_notifier_setup(FAR struct work_notifier_s *info);
#endif

/****************************************************************************
 * Name: work_notifier_teardown
 *
 * Description:
 *   Eliminate a notification previously setup by work_notifier_setup().
 *   This function should only be called if the notification should be
 *   aborted prior to the notification.  The notification will automatically
 *   be torn down after the notification is executed.
 *
 * Input Parameters:
 *   key - The key value returned from a previous call to
 *         work_notifier_setup().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_WQUEUE_NOTIFIER
int work_notifier_teardown(int key);
#endif

/****************************************************************************
 * Name: work_notifier_signal
 *
 * Description:
 *   An event has just occurred.  Notify all threads waiting for that event.
 *
 *   When an event of interest occurs, *all* of the workers waiting for this
 *   event will be executed.  If there are multiple workers for a resource
 *   then only the first to execute will get the resource.  Others will
 *   need to call work_notifier_setup() once again.
 *
 * Input Parameters:
 *   evtype    - The type of the event that just occurred.
 *   qualifier - Event qualifier to distinguish different cases of the
 *               generic event type.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_WQUEUE_NOTIFIER
void work_notifier_signal(enum work_evtype_e evtype,
                           FAR void *qualifier);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_WQUEUE_H */
