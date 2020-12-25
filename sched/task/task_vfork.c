/****************************************************************************
 * sched/task/task_vfork
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

#include <sys/wait.h>
#include <stdint.h>
#include <sched.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/task.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* vfork() requires architecture-specific support as well as waipid(). */

#if defined(CONFIG_ARCH_HAVE_VFORK) && defined(CONFIG_SCHED_WAITPID)

/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_VFORK_ARGS 256

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxvfork_setup_name
 *
 * Description:
 *   Copy the task name.
 *
 * Input Parameters:
 *   tcb        - Address of the new task's TCB
 *   name       - Name of the new task
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE > 0
static inline void nxvfork_setup_name(FAR struct tcb_s *parent,
                                      FAR struct task_tcb_s *child)
{
  /* Copy the name from the parent into the child TCB */

  strncpy(child->cmn.name, parent->name, CONFIG_TASK_NAME_SIZE);
}
#else
#  define nxvfork_setup_name(p,c)
#endif /* CONFIG_TASK_NAME_SIZE */

/****************************************************************************
 * Name: nxvfork_setup_stackargs
 *
 * Description:
 *   Clone the task arguments in the same relative positions on the child's
 *   stack.
 *
 * Input Parameters:
 *   parent - Address of the parent task's TCB
 *   child  - Address of the child task's TCB
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static inline int nxvfork_setup_stackargs(FAR struct tcb_s *parent,
                                          FAR struct task_tcb_s *child)
{
  /* Is the parent a task? or a pthread?  Only tasks (and kernel threads)
   * have command line arguments.
   */

  child->argv = NULL;
  if ((parent->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct task_tcb_s *ptcb = (FAR struct task_tcb_s *)parent;
      uintptr_t offset;
      int argc;

      /* Get the address correction */

      offset = (uintptr_t)child->cmn.adj_stack_ptr -
               (uintptr_t)parent->adj_stack_ptr;

      /* Change the child argv[] to point into its stack (instead of its
       * parent's stack).
       */

      child->argv = (FAR char **)((uintptr_t)ptcb->argv + offset);

      /* Copy the adjusted address for each argument */

      argc = 0;
      while (ptcb->argv[argc])
        {
          uintptr_t newaddr = (uintptr_t)ptcb->argv[argc] + offset;
          child->argv[argc] = (FAR char *)newaddr;

          /* Increment the number of args.  Here is a sanity check to
           * prevent running away with an unterminated argv[] list.
           * MAX_VFORK_ARGS should be sufficiently large that this never
           * happens in normal usage.
           */

          if (++argc > MAX_VFORK_ARGS)
            {
              return -E2BIG;
            }
        }

      /* Put a terminator entry at the end of the child argv[] array. */

      child->argv[argc] = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: nxvfork_setup_arguments
 *
 * Description:
 *   Clone the argument list from the parent to the child.
 *
 * Input Parameters:
 *   parent - Address of the parent task's TCB
 *   child  - Address of the child task's TCB
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

static inline int nxvfork_setup_arguments(FAR struct tcb_s *parent,
                                          FAR struct task_tcb_s *child)
{
  /* Clone the task name */

  nxvfork_setup_name(parent, child);

  /* Adjust and copy the argv[] array. */

  return nxvfork_setup_stackargs(parent, child);
}

/****************************************************************************
 * Name: nxvfork_sizeof_arguments
 *
 * Description:
 *   Get the parent's argument size.
 *
 * Input Parameters:
 *   parent - Address of the parent task's TCB
 *
 * Return Value:
 *   The parent's argument size.
 *
 ****************************************************************************/

static inline size_t nxvfork_sizeof_arguments(FAR struct tcb_s *parent)
{
  if ((parent->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_PTHREAD)
    {
      FAR struct task_tcb_s *ptcb = (FAR struct task_tcb_s *)parent;
      size_t strtablen = 0;
      int argc = 0;

      while (ptcb->argv[argc])
        {
          /* Add the size of this argument (with NUL terminator) */

          strtablen += strlen(ptcb->argv[argc++]) + 1;
        }

      /* Return the size to hold argv[] array and the strings. */

      return (argc + 1) * sizeof(FAR char *) + strtablen;
    }
  else
    {
      return 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_setup_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall vfork() sequence:  It
 *   Allocates and initializes the child task's TCB.  The overall sequence
 *   is:
 *
 *   1) User code calls vfork().  vfork() is provided in
 *      architecture-specific code.
 *   2) vfork()and calls nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) up_vfork() provides any additional operating context. up_vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) up_vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   retaddr - Return address
 *   argsize - Location to return the argument size
 *
 * Returned Value:
 *   Upon successful completion, nxtask_setup_vfork() returns a pointer to
 *   newly allocated and initialized child task's TCB.  NULL is returned
 *   on any failure and the errno is set appropriately.
 *
 ****************************************************************************/

FAR struct task_tcb_s *nxtask_setup_vfork(start_t retaddr, size_t *argsize)
{
  FAR struct tcb_s *parent = this_task();
  FAR struct task_tcb_s *child;
  uint8_t ttype;
  int priority;
  int ret;

  DEBUGASSERT(retaddr != NULL && argsize != NULL);

  /* Get the type of the fork'ed task (kernel or user) */

  if ((parent->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      /* Fork'ed from a kernel thread */

      ttype = TCB_FLAG_TTYPE_KERNEL;
    }
  else
    {
      /* Fork'ed from a user task or pthread */

      ttype = TCB_FLAG_TTYPE_TASK;
    }

  /* Allocate a TCB for the child task. */

  child = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (!child)
    {
      serr("ERROR: Failed to allocate TCB\n");
      set_errno(ENOMEM);
      return NULL;
    }

  /* Allocate a new task group with the same privileges as the parent */

  ret = group_allocate(child, parent->flags);
  if (ret < 0)
    {
      goto errout_with_tcb;
    }

  /* Associate file descriptors with the new task */

  ret = group_setuptaskfiles(child);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Get the priority of the parent task */

#ifdef CONFIG_PRIORITY_INHERITANCE
  priority = parent->base_priority;   /* "Normal," unboosted priority */
#else
  priority = parent->sched_priority;  /* Current priority */
#endif

  /* Initialize the task control block.  This calls up_initial_state() */

  sinfo("Child priority=%d start=%p\n", priority, retaddr);
  ret = nxtask_setup_scheduler(child, priority, retaddr, parent->entry.main,
                          ttype);
  if (ret < OK)
    {
      goto errout_with_tcb;
    }

  /* Return the argument size */

  *argsize = nxvfork_sizeof_arguments(parent);

  sinfo("parent=%p, returning child=%p\n", parent, child);
  return child;

errout_with_tcb:
  nxsched_release_tcb((FAR struct tcb_s *)child, ttype);
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: nxtask_start_vfork
 *
 * Description:
 *   The vfork() function has the same effect as fork(), except that the
 *   behavior is undefined if the process created by vfork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from vfork(), or returns from the function in which vfork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   This function provides one step in the overall vfork() sequence:  It
 *   starts execution of the previously initialized TCB.  The overall
 *   sequence is:
 *
 *   1) User code calls vfork()
 *   2) Architecture-specific code provides vfork()and calls
 *      nxtask_setup_vfork().
 *   3) nxtask_setup_vfork() allocates and configures the child task's TCB.
 *      This consists of:
 *      - Allocation of the child task's TCB.
 *      - Initialization of file descriptors and streams
 *      - Configuration of environment variables
 *      - Setup the input parameters for the task.
 *      - Initialization of the TCB (including call to up_initial_state()
 *   4) vfork() provides any additional operating context. vfork must:
 *      - Allocate and initialize the stack
 *      - Initialize special values in any CPU registers that were not
 *        already configured by up_initial_state()
 *   5) vfork() then calls nxtask_start_vfork()
 *   6) nxtask_start_vfork() then executes the child thread.
 *
 * Input Parameters:
 *   retaddr - The return address from vfork() where the child task
 *     will be started.
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t nxtask_start_vfork(FAR struct task_tcb_s *child)
{
  FAR struct tcb_s *parent = this_task();
  pid_t pid;
  int rc;
  int ret;

  sinfo("Starting Child TCB=%p, parent=%p\n", child, this_task());
  DEBUGASSERT(child);

  /* Duplicate the original argument list in the forked child TCB */

  ret = nxvfork_setup_arguments(parent, child);
  if (ret < 0)
    {
      nxtask_abort_vfork(child, -ret);
      return ERROR;
    }

  /* Now we have enough in place that we can join the group */

  ret = group_initialize(child);
  if (ret < 0)
    {
      nxtask_abort_vfork(child, -ret);
      return ERROR;
    }

  /* Get the assigned pid before we start the task */

  pid = (int)child->cmn.pid;

  /* Eliminate a race condition by disabling pre-emption.  The child task
   * can be instantiated, but cannot run until we call waitpid().  This
   * assures us that we cannot miss the death-of-child signal (only
   * needed in the SMP case).
   */

  sched_lock();

  /* Activate the task */

  nxtask_activate((FAR struct tcb_s *)child);

  /* The child task has not yet ran because pre-emption is disabled.
   * The child task has the same priority as the parent task, so that
   * would typically be the case anyway.  However, in the SMP
   * configuration, the child thread might have already ran on
   * another CPU if pre-emption were not disabled.
   *
   * It is a requirement that the parent environment be stable while
   * vfork runs; the child thread is still dependent on things in the
   * parent thread... like the pointers into parent thread's stack
   * which will still appear in the child's registers and environment.
   *
   * We assure that by waiting for the child thread to exit before
   * returning to the parent thread.  NOTE that pre-emption will be
   * re-enabled while we are waiting, giving the child thread the
   * opportunity to run.
   */

  rc = 0;

#ifdef CONFIG_DEBUG_FEATURES
  ret = waitpid(pid, &rc, 0);
  if (ret < 0)
    {
      serr("ERROR: waitpid failed: %d\n", errno);
    }
#else
  waitpid(pid, &rc, 0);
#endif

  sched_unlock();
  return pid;
}

/****************************************************************************
 * Name: nxtask_abort_vfork
 *
 * Description:
 *   Recover from any errors after nxtask_setup_vfork() was called.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_abort_vfork(FAR struct task_tcb_s *child, int errcode)
{
  /* The TCB was added to the active task list by nxtask_setup_scheduler() */

  dq_rem((FAR dq_entry_t *)child, (FAR dq_queue_t *)&g_inactivetasks);

  /* Release the TCB */

  nxsched_release_tcb((FAR struct tcb_s *)child,
                   child->cmn.flags & TCB_FLAG_TTYPE_MASK);
  set_errno(errcode);
}

#endif /* CONFIG_ARCH_HAVE_VFORK && CONFIG_SCHED_WAITPID */
