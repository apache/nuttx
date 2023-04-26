/****************************************************************************
 * sched/task/task_spawn.c
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
#include <sched.h>
#include <spawn.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/kthread.h>
#include <nuttx/spawn.h>

#include "sched/sched.h"
#include "group/group.h"
#include "task/spawn.h"
#include "task/task.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_spawn_create
 *
 * Description:
 *   This function creates and activates a new thread of the specified type
 *   with a specified priority and returns its system-assigned ID.  It is the
 *   internal, common implementation of task_create() and kthread_create().
 *   See comments with task_create() for further information.
 *
 * Input Parameters:
 *   name       - Name of the new task
 *   ttype      - Type of the new task
 *   priority   - Priority of the new task
 *   stack_addr - Address of the stack needed
 *   stack_size - Size (in bytes) of the stack needed
 *   entry      - Entry point of a new task
 *   arg        - A pointer to an array of input parameters.  The array
 *                should be terminated with a NULL argv[] value. If no
 *                parameters are required, argv may be NULL.
 *   envp       - A pointer to an array of environment strings. Terminated
 *                with a NULL entry.
 *   actions    - The spawn file actions
 *
 * Returned Value:
 *   Returns the positive, non-zero process ID of the new task or a negated
 *   errno value to indicate the nature of any failure.  If memory is
 *   insufficient or the task cannot be created -ENOMEM will be returned.
 *
 ****************************************************************************/

static int nxtask_spawn_create(FAR const char *name, int priority,
                              FAR void *stack_addr, int stack_size,
                              main_t entry, FAR char * const argv[],
                              FAR char * const envp[],
                              FAR const posix_spawn_file_actions_t *actions)
{
  FAR struct task_tcb_s *tcb;
  pid_t pid;
  int ret;

  /* Allocate a TCB for the new task. */

  tcb = (FAR struct task_tcb_s *)kmm_zalloc(sizeof(struct task_tcb_s));
  if (tcb == NULL)
    {
      serr("ERROR: Failed to allocate TCB\n");
      return -ENOMEM;
    }

  /* Setup the task type */

  tcb->cmn.flags = TCB_FLAG_TTYPE_TASK;

  /* Initialize the task */

  ret = nxtask_init(tcb, name, priority, stack_addr, stack_size,
                    entry, argv, envp);
  if (ret < OK)
    {
      kmm_free(tcb);
      return ret;
    }

  /* Perform file actions */

  if (actions != NULL)
    {
      ret = spawn_file_actions(&tcb->cmn, actions);
      if (ret < 0)
        {
          nxtask_uninit(tcb);
          return ret;
        }
    }

  /* Get the assigned pid before we start the task */

  pid = tcb->cmn.pid;

  /* Activate the task */

  nxtask_activate(&tcb->cmn);

  return (int)pid;
}

/****************************************************************************
 * Name: nxtask_spawn_exec
 *
 * Description:
 *   Execute the task from the file system.
 *
 * Input Parameters:
 *
 *   pidp - Upon successful completion, this will return the task ID of the
 *     child task in the variable pointed to by a non-NULL 'pid' argument.|
 *
 *   name - The name to assign to the child task.
 *
 *   entry - The child task's entry point (an address in memory)
 *
 *   actions - The spawn file actions
 *
 *   attr - If the value of the 'attr' parameter is NULL, the all default
 *     values for the POSIX spawn attributes will be used.  Otherwise, the
 *     attributes will be set according to the spawn flags.  The
 *     following spawn flags are supported:
 *
 *     - POSIX_SPAWN_SETSCHEDPARAM: Set new tasks priority to the sched_param
 *       value.
 *     - POSIX_SPAWN_SETSCHEDULER: Set the new tasks scheduler priority to
 *       the sched_policy value.
 *
 *     NOTE: POSIX_SPAWN_SETSIGMASK is handled in nxtask_spawn_proxy().
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 *   envp - A pointer to an array of environment strings. Terminated with
 *     a NULL entry.
 *
 * Returned Value:
 *   This function will return zero on success. Otherwise, an error number
 *   will be returned as the function return value to indicate the error.
 *   This errno value may be that set by execv(), sched_setpolicy(), or
 *   sched_setparam().
 *
 ****************************************************************************/

static int nxtask_spawn_exec(FAR pid_t *pidp, FAR const char *name,
                             main_t entry,
                             FAR const posix_spawn_file_actions_t *actions,
                             FAR const posix_spawnattr_t *attr,
                             FAR char * const *argv, FAR char * const envp[])
{
  FAR void *stackaddr = NULL;
  size_t stacksize;
  int priority;
  int pid;
  int ret = OK;

  /* Disable pre-emption so that we can modify the task parameters after
   * we start the new task; the new task will not actually begin execution
   * until we re-enable pre-emption.
   */

  sched_lock();

  /* Use the default priority and stack size if no attributes are provided */

  if (attr)
    {
      priority  = attr->priority;
      stacksize = attr->stacksize;
      stackaddr = attr->stackaddr;
    }
  else
    {
      struct sched_param param;

      /* Set the default priority to the same priority as this task */

      ret = nxsched_get_param(0, &param);
      if (ret < 0)
        {
          goto errout;
        }

      priority  = param.sched_priority;
      stacksize = CONFIG_POSIX_SPAWN_DEFAULT_STACKSIZE;
    }

  /* Start the task */

  pid = nxtask_spawn_create(name, priority, stackaddr,
                            stacksize, entry, argv,
                            envp ? envp : environ, actions);
  if (pid < 0)
    {
      ret = pid;
      serr("ERROR: nxtask_spawn_create failed: %d\n", ret);
      goto errout;
    }

  /* Return the task ID to the caller */

  if (pid)
    {
      *pidp = pid;
    }

  /* Now set the attributes.  Note that we ignore all of the return values
   * here because we have already successfully started the task.  If we
   * return an error value, then we would also have to stop the task.
   */

  if (attr)
    {
      spawn_execattrs(pid, attr);
    }

  /* Re-enable pre-emption and return */

errout:
  sched_unlock();
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_spawn
 *
 * Description:
 *   The task_spawn() function will create a new, child task, where the
 *   entry point to the task is an address in memory.
 *
 * Input Parameters:
 *
 *   name - The name to assign to the child task.
 *
 *   entry - The child task's entry point (an address in memory)
 *
 *   file_actions - If 'file_actions' is a null pointer, then file
 *     descriptors open in the calling process will remain open in the
 *     child process (unless CONFIG_FDCLONE_STDIO is defined). If
 *     'file_actions' is not NULL, then the file descriptors open in the
 *     child process will be those open in the calling process as modified
 *     by the spawn file actions object pointed to by file_actions.
 *
 *   attr - If the value of the 'attr' parameter is NULL, the all default
 *     values for the POSIX spawn attributes will be used.  Otherwise, the
 *     attributes will be set according to the spawn flags.  The
 *     posix_spawnattr_t spawn attributes object type is defined in spawn.h.
 *     It will contains these attributes, not all of which are supported by
 *     NuttX:
 *
 *     - POSIX_SPAWN_SETPGROUP:  Setting of the new task's process group is
 *       not supported.  NuttX does not support process groups.
 *     - POSIX_SPAWN_SETSCHEDPARAM: Set new tasks priority to the sched_param
 *       value.
 *     - POSIX_SPAWN_SETSCHEDULER: Set the new task's scheduler policy to
 *       the sched_policy value.
 *     - POSIX_SPAWN_RESETIDS: Resetting of the effective user ID of the
 *       child process is not supported.  NuttX does not support effective
 *       user IDs.
 *     - POSIX_SPAWN_SETSIGMASK: Set the new task's signal mask.
 *     - POSIX_SPAWN_SETSIGDEF:  Resetting signal default actions is not
 *       supported.  NuttX does not support default signal actions.
 *
 *     And the non-standard:
 *
 *     - TASK_SPAWN_SETSTACKSIZE:  Set the stack size for the new task.
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 *   envp - envp[] is an array of character pointers to null-terminated
 *     strings that provide the environment for the new process image.
 *
 * Returned Value:
 *   task_spawn() will return process ID of new task on success.
 *   Otherwise, a negative number will be returned as the function return
 *   value to indicate the error:
 *
 *   - EINVAL: The value specified by 'file_actions' or 'attr' is invalid.
 *   - Any errors that might have been return if vfork() and excec[l|v]()
 *     had been called.
 *
 ****************************************************************************/

int task_spawn(FAR const char *name, main_t entry,
               FAR const posix_spawn_file_actions_t *file_actions,
               FAR const posix_spawnattr_t *attr,
               FAR char * const argv[], FAR char * const envp[])
{
  pid_t pid = INVALID_PROCESS_ID;
  int ret;

  sinfo("name=%s entry=%p file_actions=%p attr=%p argv=%p\n",
        name, entry, file_actions, attr, argv);

  if (attr != NULL)
    {
      spawn_proxyattrs(attr);
    }

  ret = nxtask_spawn_exec(&pid, name, entry,
                          file_actions != NULL ? *file_actions : NULL,
                          attr, argv, envp);

  return ret >= 0 ? (int)pid : ret;
}

#endif /* CONFIG_BUILD_KERNEL */
