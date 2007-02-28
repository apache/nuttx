/************************************************************
 * sched.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __SCHED_H
#define __SCHED_H

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/config.h>
#include <queue.h>
#include <signal.h>
#include <semaphore.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <nuttx/irq.h>

/************************************************************
 * Definitions
 ************************************************************/

/* Task Management Definitins *******************************/

/* This is the number of arguments that are passed to tasks
 * on start-up. This number was selected because this is the
 * number of parameters that can be passed to a MIPS function
 * in registers
 . */

#define NUM_TASK_ARGS       4

/* This is the maximum number of times that a lock can be set */

#define MAX_LOCK_COUNT    127

/* Values for the _TCB flags flag bits */

#define TCB_FLAG_PTHREAD        0x0001 /* Thread is a pthread */
#define TCB_FLAG_NONCANCELABLE  0x0002 /* Pthread is non-cancelable */
#define TCB_FLAG_CANCEL_PENDING 0x0004 /* Pthread cancel is pending */
#define TCB_FLAG_ROUND_ROBIN    0x0008 /* Round robin sched enabled */

/* Pthread definitions **************************************/

#define PTHREAD_KEYS_MAX CONFIG_NPTHREAD_KEYS

/************************************************************
 * Global Type Definitions
 ************************************************************/

#ifndef __ASSEMBLY__

/* General Task Management Types ****************************/

/* This is the type of the task_state field of the TCB.
 * NOTE: the order and content of this enumeration is
 * critical since there are some OS tables indexed by these
 * values.
 */

typedef enum tstate_e
{
  TSTATE_TASK_INVALID    = 0, /* INVALID - TCB has not yet been initialized */

  TSTATE_TASK_PENDING    = 1, /* READY_TO_RUN - Pending preemption unlock */
  TSTATE_TASK_READYTORUN = 2, /* READY-TO-RUN - But not running */
  TSTATE_TASK_RUNNING    = 3, /* READY_TO_RUN - Aand running */

  TSTATE_TASK_INACTIVE   = 4, /* BLOCKED - Initialized but not yet activated */
  TSTATE_WAIT_SEM        = 5, /* BLOCKED - Waiting for a semaphore */
#ifndef CONFIG_DISABLE_MQUEUE
  TSTATE_WAIT_SIG        = 6, /* BLOCKED - Waiting for a signal */
#endif
#ifndef CONFIG_DISABLE_MQUEUE
  TSTATE_WAIT_MQNOTEMPTY,     /* BLOCKED - Waiting for a MQ to become not empty. */
  TSTATE_WAIT_MQNOTFULL       /* BLOCKED - Waiting for a MQ to become not full. */
#endif
};
typedef enum tstate_e tstate_t;

/* The following definitions are determined by tstate_t */

#define FIRST_READY_TO_RUN_STATE TSTATE_TASK_READYTORUN
#define LAST_READY_TO_RUN_STATE  TSTATE_TASK_RUNNING
#define FIRST_BLOCKED_STATE      TSTATE_TASK_INACTIVE
#ifndef CONFIG_DISABLE_MQUEUE
# define LAST_BLOCKED_STATE      TSTATE_WAIT_MQNOTFULL
# ifndef CONFIG_DISABLE_SIGNALS
#  define NUM_TASK_STATES        9
# else
#  define NUM_TASK_STATES        8
# endif
#else
# ifndef CONFIG_DISABLE_SIGNALS
#  define LAST_BLOCKED_STATE     TSTATE_WAIT_SIG
#  define NUM_TASK_STATES        7
# else
#  define LAST_BLOCKED_STATE     TSTATE_WAIT_SEM
#  define NUM_TASK_STATES        6
# endif
#endif

/* The following is the form of a thread start-up function */

typedef void (*start_t)(void);

/* This is the entry point into the main thread of the task
 * or into a created pthread within the task.
 */

union entry_u
{
  pthread_startroutine_t pthread;
  main_t main;
};
typedef union entry_u entry_t;

/* This is the type of the function that is executed with
 * exit() is called (if registered via atexit()).
 */

typedef void (*exitfunc_t)(void);

/* POSIX Message queue */

typedef struct msgq_s msgq_t;

/* This is the task control block (TCB) */

struct _TCB
{
  /* Fields used to support list management ***************************/

  FAR struct _TCB *flink;         /* link in DQ of TCBs               */
  FAR struct _TCB *blink;

  /* Task Management Fields *******************************************/

  pid_t    pid;                   /* This is the ID of the thread     */
  start_t  start;                 /* Thread start function            */
  entry_t  entry;                 /* Entry Point into the thread      */
  exitfunc_t exitfunc;            /* Called if exit is called.        */
  ubyte    sched_priority;        /* Current priority of the thread   */
  tstate_t task_state;            /* Current state of the thread      */
  uint16   flags;                 /* Misc. general status flags       */
  sint16   lockcount;             /* 0=preemptable (not-locked)       */
  FAR void *joininfo;             /* Detach-able info to support join */
#if CONFIG_RR_INTERVAL > 0
  int      timeslice;             /* RR timeslice interval remaining  */
#endif

  /* Values needed to restart a task **********************************/

  ubyte    init_priority;         /* Initial priority of the task     */
  FAR char *argv[NUM_TASK_ARGS+1]; /* Name + start-up parameters      */

  /* Stack-Related Fields *********************************************/

  size_t  adj_stack_size;         /* Stack size after adjustment      */
                                  /* for hardware, processor, etc.    */
                                  /* (for debug purposes only)        */
  FAR void *stack_alloc_ptr;      /* Pointer to allocated stack       */
                                  /* Need to deallocate stack         */
  FAR void *adj_stack_ptr;        /* Adjusted StatckAllocPtr for HW   */
                                  /* The initial stack pointer value  */

  /* POSIX thread Specific Data ***************************************/

#if CONFIG_NPTHREAD_KEYS > 0
  FAR void *pthread_data[CONFIG_NPTHREAD_KEYS];
#endif

  /* POSIX Semaphore Control Fields ***********************************/

  sem_t *waitsem;                 /* Semaphore ID waiting on          */

  /* POSIX Signal Control Fields **************************************/

#ifndef CONFIG_DISABLE_SIGNALS
  sigset_t   sigprocmask;         /* Signals that are blocked         */
  sigset_t   sigwaitmask;         /* Waiting for pending signals      */
  sq_queue_t sigactionq;          /* List of actions for signals      */
  sq_queue_t sigpendingq;         /* List of Pending Signals          */
  sq_queue_t sigpendactionq;      /* List of pending signal actions   */
  sq_queue_t sigpostedq;          /* List of posted signals           */
  siginfo_t  sigunbinfo;          /* Signal info when task unblocked  */
#endif

  /* POSIX Named Message Queue Fields *********************************/

#ifndef CONFIG_DISABLE_MQUEUE
  sq_queue_t msgdesq;             /* List of opened message queues    */
  FAR msgq_t *msgwaitq;           /* Waiting for this message queue   */
#endif

  /* Library related fields *******************************************/

  int        errno;               /* Current per-thread errno         */

  /* File system support **********************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
  FAR struct filelist *filelist;  /* Maps file descriptor to file     */
#endif

#if CONFIG_NFILE_STREAMS > 0
  FAR struct streamlist *streams; /* Holds C buffered I/O info        */
#endif

  /* State save areas *************************************************/
  /* The form and content of these fields are processor-specific.     */

  struct xcptcontext xcp;        /* Interrupt register save area      */

#if CONFIG_TASK_NAME_SIZE > 0
  char name[CONFIG_TASK_NAME_SIZE]; /* Task name                      */
#endif

};
typedef struct _TCB _TCB;
#endif /* __ASSEMBLY__ */

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Task Control Interfaces (non-standard) */

EXTERN STATUS  task_init(FAR _TCB *tcb, const char *name, int priority,
                         FAR uint32 *stack, uint32 stack_size, main_t entry,
                         FAR char *arg1, FAR char *arg2,
                         FAR char *arg3, FAR char *arg4);
EXTERN STATUS  task_activate(FAR _TCB *tcb);
EXTERN int     task_create(const char *name, int priority, int stack_size, main_t main,
                           FAR char *arg1, FAR char *arg2,
                           FAR char *arg3, FAR char *arg4);
EXTERN STATUS  task_delete(pid_t pid);
EXTERN STATUS  task_restart(pid_t pid);

/* Task Scheduling Interfaces (based on POSIX APIs) */

EXTERN int     sched_setparam(pid_t pid,
                              const struct sched_param *param);
EXTERN int     sched_getparam(pid_t pid,
                              struct sched_param *param);
EXTERN int     sched_setscheduler(pid_t pid, int policy,
                                  const struct sched_param *param);
EXTERN int     sched_getscheduler(pid_t pid);
EXTERN int     sched_yield(void);
EXTERN int     sched_get_priority_max(int policy);
EXTERN int     sched_get_priority_min(int policy);
EXTERN int     sched_rr_get_interval(pid_t pid,
                                     struct timespec *interval);

/* Task Switching Interfaces (non-standard) */

EXTERN STATUS  sched_lock(void);
EXTERN STATUS  sched_unlock(void);
EXTERN sint32  sched_lockcount(void);

/* If instrumentation of the scheduler is enabled, then some
 * outboard logic must provide the following interfaces.
 */

#ifdef CONFIG_SCHED_INSTRUMENTATION

EXTERN void sched_note_start(FAR _TCB *tcb );
EXTERN void sched_note_stop(FAR _TCB *tcb );
EXTERN void sched_note_switch(FAR _TCB *pFromTcb, FAR _TCB *pToTcb);

#else
# define sched_note_start(t)
# define sched_note_stop(t)
# define sched_note_switch(t1, t2)
#endif /* CONFIG_SCHED_INSTRUMENTATION */

/* File system helpers */

#if CONFIG_NFILE_DESCRIPTORS > 0
EXTERN FAR struct filelist *sched_getfiles(void);
#if CONFIG_NFILE_STREAMS > 0
EXTERN FAR struct streamlist *sched_getstreams(void);
#endif /* CONFIG_NFILE_STREAMS */
#endif /* CONFIG_NFILE_DESCRIPTORS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __SCHED_H */
