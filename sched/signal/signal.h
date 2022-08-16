/****************************************************************************
 * sched/signal/signal.h
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

#ifndef __SCHED_SIGNAL_SIGNAL_H
#define __SCHED_SIGNAL_SIGNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <queue.h>
#include <sched.h>

#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following definition determines the number of signal structures to
 * allocate in a block
 */

#define NUM_SIGNAL_ACTIONS       4
#define NUM_PENDING_ACTIONS      4
#define NUM_SIGNALS_PENDING      4

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This enumeration identifies the type of signal data allocation */

enum sigalloc_e
{
  SIG_ALLOC_FIXED = 0,  /* pre-allocated; never freed */
  SIG_ALLOC_DYN,        /* dynamically allocated; free when unused */
  SIG_ALLOC_IRQ         /* Preallocated, reserved for interrupt handling */
};

/* The following defines the sigaction queue entry */

struct sigactq
{
  FAR struct sigactq *flink;     /* Forward link */
  struct sigaction act;          /* Sigaction data */
  uint8_t   signo;               /* Signal associated with action */
};
typedef struct sigactq  sigactq_t;

/* The following defines the queue structure within each TCB to hold pending
 * signals received by the task.  These are signals that cannot be processed
 * because:  (1) the task is not waiting for them, or (2) the task has no
 * action associated with the signal.
 */

struct sigpendq
{
  FAR struct sigpendq *flink;    /* Forward link */
  siginfo_t info;                /* Signal information */
  uint8_t   type;                /* (Used to manage allocations) */
};
typedef struct sigpendq sigpendq_t;

/* The following defines the queue structure within each TCB to hold queued
 * signal actions that need action by the task
 */

struct sigq_s
{
  FAR struct sigq_s *flink;      /* Forward link */
  union
  {
    void (*sighandler)(int signo, siginfo_t *info, void *context);
  } action;                      /* Signal action */
  sigset_t  mask;                /* Additional signals to mask while the
                                  * the signal-catching function executes */
  siginfo_t info;                /* Signal information */
  uint8_t   type;                /* (Used to manage allocations) */
};
typedef struct sigq_s sigq_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_sigfreeaction data structure is a list of available signal action
 * structures.
 */

extern sq_queue_t  g_sigfreeaction;

/* The g_sigpendingaction data structure is a list of available pending
 * signal action structures.
 */

extern sq_queue_t  g_sigpendingaction;

/* The g_sigpendingirqaction is a list of available pending signal actions
 * that are reserved for use by interrupt handlers.
 */

extern sq_queue_t  g_sigpendingirqaction;

/* The g_sigpendingsignal data structure is a list of available pending
 * signal structures.
 */

extern sq_queue_t  g_sigpendingsignal;

/* The g_sigpendingirqsignal data structure is a list of available pending
 * signal structures that are reserved for use by interrupt handlers.
 */

extern sq_queue_t  g_sigpendingirqsignal;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Internal signal-related interfaces ***************************************/

/* Forward references */

struct task_group_s;

/* sig_initializee.c */

void               nxsig_initialize(void);

/* sig_action.c */

void               nxsig_release_action(FAR sigactq_t *sigact);

/* sig_default.c */

#ifdef CONFIG_SIG_DEFAULT
bool               nxsig_isdefault(FAR struct tcb_s *tcb, int signo);
bool               nxsig_iscatchable(int signo);
_sa_handler_t      nxsig_default(FAR struct tcb_s *tcb, int signo,
                                 bool defaction);
int                nxsig_default_initialize(FAR struct tcb_s *tcb);
#endif

/* sig_pending.c */

sigset_t           nxsig_pendingset(FAR struct tcb_s *stcb);

/* sig_dispatch.c */

int                nxsig_tcbdispatch(FAR struct tcb_s *stcb,
                                     FAR siginfo_t *info);
int                nxsig_dispatch(pid_t pid, FAR siginfo_t *info);

/* sig_cleanup.c */

void               nxsig_cleanup(FAR struct tcb_s *stcb);
void               nxsig_release(FAR struct task_group_s *group);

/* sig_timedwait.c */

#ifdef CONFIG_CANCELLATION_POINTS
void nxsig_wait_irq(FAR struct tcb_s *wtcb, int errcode);
#endif

/* In files of the same name */

FAR sigq_t        *nxsig_alloc_pendingsigaction(void);
void               nxsig_deliver(FAR struct tcb_s *stcb);
FAR sigactq_t     *nxsig_find_action(FAR struct task_group_s *group,
                                     int signo);
int                nxsig_lowest(FAR sigset_t *set);
void               nxsig_release_pendingsigaction(FAR sigq_t *sigq);
void               nxsig_release_pendingsignal(FAR sigpendq_t *sigpend);
FAR sigpendq_t    *nxsig_remove_pendingsignal(FAR struct tcb_s *stcb,
                                              int signo);
bool               nxsig_unmask_pendingsignal(void);

#endif /* __SCHED_SIGNAL_SIGNAL_H */
