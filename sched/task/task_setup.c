/****************************************************************************
 * sched/task/task_setup.c
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "pthread/pthread.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_STACK_ARGS 256

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the name for un-named tasks */

static const char g_noname[] = "<noname>";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_assign_pid
 *
 * Description:
 *   This function assigns the next unique task ID to a task.
 *
 * Input Parameters:
 *   tcb - TCB of task
 *
 * Returned Value:
 *   OK on success; ERROR on failure (errno is not set)
 *
 ****************************************************************************/

static int nxtask_assign_pid(FAR struct tcb_s *tcb)
{
  pid_t next_pid;
  int   hash_ndx;
  int   tries;

  /* Disable pre-emption.  This should provide sufficient protection
   * for the following operation.
   */

  sched_lock();

  /* We'll try every allowable pid */

  for (tries = 0; tries < CONFIG_MAX_TASKS; tries++)
    {
      /* Get the next process ID candidate */

      next_pid = ++g_lastpid;

      /* Verify that the next_pid is in the valid range */

      if (next_pid <= 0)
        {
          g_lastpid = 1;
          next_pid  = 1;
        }

      /* Get the hash_ndx associated with the next_pid */

      hash_ndx = PIDHASH(next_pid);

      /* Check if there is a (potential) duplicate of this pid */

      if (!g_pidhash[hash_ndx].tcb)
        {
          /* Assign this PID to the task */

          g_pidhash[hash_ndx].tcb   = tcb;
          g_pidhash[hash_ndx].pid   = next_pid;
#ifdef CONFIG_SCHED_CPULOAD
          g_pidhash[hash_ndx].ticks = 0;
#endif
          tcb->pid = next_pid;

          sched_unlock();
          return OK;
        }
    }

  /* If we get here, then the g_pidhash[] table is completely full.
   * We cannot allow another task to be started.
   */

  sched_unlock();
  return ERROR;
}

/****************************************************************************
 * Name: nxtask_inherit_affinity
 *
 * Description:
 *   exec(), task_create(), and vfork() all inherit the affinity mask from
 *   the parent thread.  This is the default for pthread_create() as well
 *   but the affinity mask can be specified in the pthread attributes as
 *   well.  pthread_setup() will have to fix up the affinity mask in this
 *   case.
 *
 * Input Parameters:
 *   tcb - The TCB of the new task.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The parent of the new task is the task at the head of the assigned task
 *   list for the current CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline void nxtask_inherit_affinity(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *rtcb = this_task();
  tcb->affinity = rtcb->affinity;
}
#else
#  define nxtask_inherit_affinity(tcb)
#endif

/****************************************************************************
 * Name: nxtask_save_parent
 *
 * Description:
 *   Save the task ID of the parent task in the child task's group and
 *   allocate a child status structure to catch the child task's exit
 *   status.
 *
 * Input Parameters:
 *   tcb   - The TCB of the new, child task.
 *   ttype - Type of the new thread: task, pthread, or kernel thread
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The parent of the new task is the task at the head of the ready-to-run
 *   list.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_HAVE_PARENT
static inline void nxtask_save_parent(FAR struct tcb_s *tcb, uint8_t ttype)
{
  DEBUGASSERT(tcb != NULL && tcb->group != NULL);

  /* Only newly created tasks (and kernel threads) have parents.  None of
   * this logic applies to pthreads with reside in the same group as the
   * parent and share that same child/parent relationships.
   */

#ifndef CONFIG_DISABLE_PTHREAD
  if ((tcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
#endif
    {
      /* Get the TCB of the parent task.  In this case, the calling task. */

      FAR struct tcb_s *rtcb = this_task();

      DEBUGASSERT(rtcb != NULL && rtcb->group != NULL);

#ifdef HAVE_GROUP_MEMBERS
      /* Save the ID of the parent tasks' task group in the child's task
       * group.  Copy the ID from the parent's task group structure to
       * child's task group.
       */

      tcb->group->tg_pgrpid = rtcb->group->tg_grpid;

#else
      /* Save the parent task's ID in the child task's group. */

      tcb->group->tg_ppid = rtcb->pid;
#endif

#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Tasks can also suppress retention of their child status by applying
       * the SA_NOCLDWAIT flag with sigaction().
       */

      if ((rtcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0)
        {
          FAR struct child_status_s *child;

          /* Make sure that there is not already a structure for this PID in
           * the parent TCB.  There should not be.
           */

          child = group_find_child(rtcb->group, tcb->pid);
          DEBUGASSERT(child == NULL);
          if (child == NULL)
            {
              /* Allocate a new status structure  */

              child = group_alloc_child();
            }

          /* Did we successfully find/allocate the child status structure? */

          DEBUGASSERT(child != NULL);
          if (child != NULL)
            {
              /* Yes.. Initialize the structure */

              child->ch_flags  = ttype;
              child->ch_pid    = tcb->pid;
              child->ch_status = 0;

              /* Add the entry into the group's list of children */

              group_add_child(rtcb->group, child);
            }
        }

#else /* CONFIG_SCHED_CHILD_STATUS */
      /* Child status is not retained.  Simply keep track of the number
       * child tasks created.
       */

      DEBUGASSERT(rtcb->group->tg_nchildren < UINT16_MAX);
      rtcb->group->tg_nchildren++;

#endif /* CONFIG_SCHED_CHILD_STATUS */
    }
}
#else
#  define nxtask_save_parent(tcb,ttype)
#endif

/****************************************************************************
 * Name: nxtask_dup_dspace
 *
 * Description:
 *   When a new task or thread is created from a PIC module, then that
 *   module (probably) intends the task or thread to execute in the same
 *   D-Space.  This function will duplicate the D-Space for that purpose.
 *
 * Input Parameters:
 *   tcb - The TCB of the new task.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The parent of the new task is the task at the head of the ready-to-run
 *   list.
 *
 ****************************************************************************/

#ifdef CONFIG_PIC
static inline void nxtask_dup_dspace(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *rtcb = this_task();
  if (rtcb->dspace != NULL)
    {
      /* Copy the D-Space structure reference and increment the reference
       * count on the memory.  The D-Space memory will persist until the
       * last thread exits (see nxsched_release_tcb()).
       */

      tcb->dspace = rtcb->dspace;
      tcb->dspace->crefs++;
    }
}
#else
#  define nxtask_dup_dspace(tcb)
#endif

/****************************************************************************
 * Name: nxthread_setup_scheduler
 *
 * Description:
 *   This functions initializes the common portions of the Task Control Block
 *   (TCB) in preparation for starting a new thread.
 *
 *   nxthread_setup_scheduler() is called from nxtask_setup_scheduler() and
 *   pthread_setup_scheduler().
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   priority   - Priority of the new task
 *   start      - Thread startup routine
 *   entry      - Thread user entry point
 *   ttype      - Type of the new thread: task, pthread, or kernel thread
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 *   This function can only failure is it is unable to assign a new, unique
 *   task ID to the TCB (errno is not set).
 *
 ****************************************************************************/

static int nxthread_setup_scheduler(FAR struct tcb_s *tcb, int priority,
                                    start_t start, CODE void *entry,
                                    uint8_t ttype)
{
  int ret;

  /* Assign a unique task ID to the task. */

  ret = nxtask_assign_pid(tcb);
  if (ret == OK)
    {
      /* Save task priority and entry point in the TCB */

      tcb->sched_priority = (uint8_t)priority;
      tcb->init_priority  = (uint8_t)priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
      tcb->base_priority  = (uint8_t)priority;
#endif
      tcb->start          = start;
      tcb->entry.main     = (main_t)entry;

      /* Save the thread type.  This setting will be needed in
       * up_initial_state() is called.
       */

      ttype              &= TCB_FLAG_TTYPE_MASK;
      tcb->flags         &= ~TCB_FLAG_TTYPE_MASK;
      tcb->flags         |= ttype;

#ifdef CONFIG_CANCELLATION_POINTS
      /* Set the deferred cancellation type */

      tcb->flags         |= TCB_FLAG_CANCEL_DEFERRED;
#endif

      /* Save the task ID of the parent task in the TCB and allocate
       * a child status structure.
       */

      nxtask_save_parent(tcb, ttype);

#ifdef CONFIG_SMP
      /* exec(), task_create(), and vfork() all inherit the affinity mask
       * from the parent thread.  This is the default for pthread_create()
       * as well but the affinity mask can be specified in the pthread
       * attributes as well.  pthread_create() will have to fix up the
       * affinity mask in this case.
       */

      nxtask_inherit_affinity(tcb);
#endif

      /* exec(), pthread_create(), task_create(), and vfork() all
       * inherit the signal mask of the parent thread.
       */

      nxsig_procmask(SIG_SETMASK, NULL, &tcb->sigprocmask);

      /* Initialize the task state.  It does not get a valid state
       * until it is activated.
       */

      tcb->task_state = TSTATE_TASK_INVALID;

      /* Clone the parent tasks D-Space (if it was running PIC).  This
       * must be done before calling up_initial_state() so that the
       * state setup will take the PIC address base into account.
       */

      nxtask_dup_dspace(tcb);

      /* Initialize the processor-specific portion of the TCB */

      up_initial_state(tcb);

      /* Add the task to the inactive task list */

      sched_lock();
      dq_addfirst((FAR dq_entry_t *)tcb, (FAR dq_queue_t *)&g_inactivetasks);
      tcb->task_state = TSTATE_TASK_INACTIVE;
      sched_unlock();
    }

  return ret;
}

/****************************************************************************
 * Name: nxtask_setup_name
 *
 * Description:
 *   Assign the task name.
 *
 * Input Parameters:
 *   tcb  - Address of the new task's TCB
 *   name - Name of the new task
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE > 0
static void nxtask_setup_name(FAR struct task_tcb_s *tcb,
                              FAR const char *name)
{
  /* Give a name to the unnamed tasks */

  if (!name)
    {
      name = (FAR char *)g_noname;
    }

  /* Copy the name into the TCB */

  strncpy(tcb->cmn.name, name, CONFIG_TASK_NAME_SIZE);
  tcb->cmn.name[CONFIG_TASK_NAME_SIZE] = '\0';
}
#else
#  define nxtask_setup_name(t,n)
#endif /* CONFIG_TASK_NAME_SIZE */

/****************************************************************************
 * Name: nxtask_setup_stackargs
 *
 * Description:
 *   This functions is called only from nxtask_setup_arguments()  It will
 *   allocate space on the new task's stack and will copy the argv[] array
 *   and all strings to the task's stack where it is readily accessible to
 *   the task.  Data on the stack, on the other hand, is guaranteed to be
 *   accessible no matter what privilege mode the task runs in.
 *
 * Input Parameters:
 *   tcb  - Address of the new task's TCB
 *   argv - A pointer to an array of input parameters. The arrau should be
 *          terminated with a NULL argv[] value. If no parameters are
 *          required, argv may be NULL.
 *
 * Returned Value:
 *  Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static inline int nxtask_setup_stackargs(FAR struct task_tcb_s *tcb,
                                         FAR char * const argv[])
{
  FAR char **stackargv;
  FAR const char *name;
  FAR char *str;
  size_t strtablen;
  size_t argvlen;
  int nbytes;
  int argc;
  int i;

  /* Get the name string that we will use as the first argument */

#if CONFIG_TASK_NAME_SIZE > 0
  name = tcb->cmn.name;
#else
  name = (FAR const char *)g_noname;
#endif /* CONFIG_TASK_NAME_SIZE */

  /* Get the size of the task name (including the NUL terminator) */

  strtablen = (strlen(name) + 1);

  /* Count the number of arguments and get the accumulated size of the
   * argument strings (including the null terminators).  The argument count
   * does not include the task name in that will be in argv[0].
   */

  argc = 0;
  if (argv != NULL)
    {
      /* A NULL argument terminates the list */

      while (argv[argc])
        {
          /* Add the size of this argument (with NUL terminator).
           * Check each time if the accumulated size exceeds the
           * size of the allocated stack.
           */

          strtablen += (strlen(argv[argc]) + 1);
          if (strtablen >= tcb->cmn.adj_stack_size)
            {
              return -ENAMETOOLONG;
            }

          /* Increment the number of args.  Here is a sanity check to
           * prevent running away with an unterminated argv[] list.
           * MAX_STACK_ARGS should be sufficiently large that this never
           * happens in normal usage.
           */

          if (++argc > MAX_STACK_ARGS)
            {
              return -E2BIG;
            }
        }
    }

  /* Allocate a stack frame to hold argv[] array and the strings.  NOTE
   * that argc + 2 entries are needed:  The number of arguments plus the
   * task name plus a NULL argv[] entry to terminate the list.
   */

  argvlen   = (argc + 2) * sizeof(FAR char *);
  stackargv = (FAR char **)up_stack_frame(&tcb->cmn, argvlen + strtablen);

  DEBUGASSERT(stackargv != NULL);
  if (stackargv == NULL)
    {
      return -ENOMEM;
    }

  /* Get the address of the string table that will lie immediately after
   * the argv[] array and mark it as a null string.
   */

  str = (FAR char *)stackargv + argvlen;

  /* Copy the task name.  Increment str to skip over the task name and its
   * NUL terminator in the string buffer.
   */

  stackargv[0] = str;
  nbytes       = strlen(name) + 1;
  strcpy(str, name);
  str         += nbytes;

  /* Copy each argument */

  for (i = 0; i < argc; i++)
    {
      /* Save the pointer to the location in the string buffer and copy
       * the argument into the buffer.  Increment str to skip over the
       * argument and its NUL terminator in the string buffer.
       */

      stackargv[i + 1] = str;
      nbytes           = strlen(argv[i]) + 1;
      strcpy(str, argv[i]);
      str             += nbytes;
    }

  /* Put a terminator entry at the end of the argv[] array.  Then save the
   * argv[] array pointer in the TCB where it will be recovered later by
   * nxtask_start().
   */

  stackargv[argc + 1] = NULL;
  tcb->argv = stackargv;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_setup_scheduler
 *
 * Description:
 *   This functions initializes a Task Control Block (TCB) in preparation
 *   for starting a new task.
 *
 *   nxtask_setup_scheduler() is called from nxtask_init() and
 *   nxtask_start().
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   priority   - Priority of the new task
 *   start      - Start-up function (probably nxtask_start())
 *   main       - Application start point of the new task
 *   ttype      - Type of the new thread: task or kernel thread
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 *   This function can only failure is it is unable to assign a new, unique
 *   task ID to the TCB (errno is not set).
 *
 ****************************************************************************/

int nxtask_setup_scheduler(FAR struct task_tcb_s *tcb, int priority,
                           start_t start, main_t main, uint8_t ttype)
{
  /* Perform common thread setup */

  return nxthread_setup_scheduler((FAR struct tcb_s *)tcb, priority,
                                  start, (CODE void *)main, ttype);
}

/****************************************************************************
 * Name: pthread_setup_scheduler
 *
 * Description:
 *   This functions initializes a Task Control Block (TCB) in preparation
 *   for starting a new pthread.
 *
 *   pthread_setup_scheduler() is called from pthread_create(),
 *
 * Input Parameters:
 *   tcb      - Address of the new task's TCB
 *   priority - Priority of the new task
 *   start    - Start-up function (probably pthread_start())
 *   entry    - Entry point of the new pthread
 *   ttype    - Type of the new thread: task, pthread, or kernel thread
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 *   This function can only failure is it is unable to assign a new, unique
 *   task ID to the TCB (errno is not set).
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PTHREAD
int pthread_setup_scheduler(FAR struct pthread_tcb_s *tcb, int priority,
                            start_t start, pthread_startroutine_t entry)
{
  /* Perform common thread setup */

  return nxthread_setup_scheduler((FAR struct tcb_s *)tcb, priority,
                                  start, (CODE void *)entry,
                                  TCB_FLAG_TTYPE_PTHREAD);
}
#endif

/****************************************************************************
 * Name: nxtask_setup_arguments
 *
 * Description:
 *   This functions sets up parameters in the Task Control Block (TCB) in
 *   preparation for starting a new thread.
 *
 *   nxtask_setup_arguments() is called only from nxtask_init() and
 *   nxtask_start() to create a new task.  In the "normal" case, the argv[]
 *   array is a structure in the TCB, the arguments are cloned via strdup.
 *
 *   In the kernel build case, the argv[] array and all strings are copied
 *   to the task's stack.  This is done because the TCB (and kernel allocated
 *   strings) are only accessible in kernel-mode.  Data on the stack, on the
 *   other hand, is guaranteed to be accessible no matter what mode the
 *   task runs in.
 *
 * Input Parameters:
 *   tcb  - Address of the new task's TCB
 *   name - Name of the new task (not used)
 *   argv - A pointer to an array of input parameters.  The array should be
 *          terminated with a NULL argv[] value.  If no parameters are
 *          required, argv may be NULL.
 *
 * Returned Value:
 *  OK
 *
 ****************************************************************************/

int nxtask_setup_arguments(FAR struct task_tcb_s *tcb, FAR const char *name,
                           FAR char * const argv[])
{
  /* Setup the task name */

  nxtask_setup_name(tcb, name);

  /* Copy the argv[] array and all strings are to the task's stack.  Data on
   * the stack is guaranteed to be accessible by the ask no matter what
   * privilege mode the task runs in.
   */

  return nxtask_setup_stackargs(tcb, argv);
}
