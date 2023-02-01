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
                             main_t entry, FAR const posix_spawnattr_t *attr,
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

  pid = nxtask_create(name, priority, stackaddr,
                      stacksize, entry, argv, envp);
  if (pid < 0)
    {
      ret = pid;
      serr("ERROR: nxtask_create failed: %d\n", ret);
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
 * Name: nxtask_spawn_proxy
 *
 * Description:
 *   Perform file_actions, then execute the task from the file system.
 *
 *   Do we really need a proxy task in this case?  Isn't that wasteful?
 *
 *   Q: Why can we do what we need to do here and the just call the
 *      new task's entry point.
 *   A: This would require setting up the name, priority, and stacksize from
 *      the task_spawn, but it do-able.  The only issue I can think of is
 *      that NuttX supports task_restart(), and you would never be able to
 *      restart a task from this point.
 *
 *   Q: Why not use a starthook so that there is callout from nxtask_start()
 *      to perform these operations?
 *   A: Good idea, except that existing nxtask_starthook() implementation
 *      cannot be used here unless we get rid of task_create and, instead,
 *      use nxtask_init() and nxtask_activate().  start_taskhook() could then
 *      be called between nxtask_init() and nxtask_activate().
 *      task_restart() would still be an issue.
 *
 * Input Parameters:
 *   argc, argv - Ignored. The task's start-up parameters are passed via the
 *     semaphore-protected global structure g_spawn_parms.
 *
 * Returned Value:
 *   Standard task return value.
 *
 ****************************************************************************/

static int nxtask_spawn_proxy(int argc, FAR char *argv[])
{
  int ret;

  /* Perform file actions and/or set a custom signal mask.  We get here only
   * if the file_actions parameter to task_spawn[p] was non-NULL and/or the
   * option to change the signal mask was selected.
   */

  UNUSED(argc);
  UNUSED(argv);

  DEBUGASSERT(g_spawn_parms.file_actions ||
              (g_spawn_parms.attr &&
               (g_spawn_parms.attr->flags & POSIX_SPAWN_SETSIGMASK) != 0));

  /* Set the attributes and perform the file actions as appropriate */

  ret = spawn_proxyattrs(g_spawn_parms.attr, g_spawn_parms.file_actions);
  if (ret == OK)
    {
      /* Start the task */

      ret = nxtask_spawn_exec(g_spawn_parms.pid, g_spawn_parms.u.task.name,
                              g_spawn_parms.u.task.entry, g_spawn_parms.attr,
                              g_spawn_parms.argv, g_spawn_parms.envp);

#ifdef CONFIG_SCHED_HAVE_PARENT
      if (ret == OK)
        {
          /* Change of the parent of the task we just spawned to our parent.
           * What should we do in the event of a failure?
           */

          int tmp = task_reparent(0, *g_spawn_parms.pid);
          if (tmp < 0)
            {
              serr("ERROR: task_reparent() failed: %d\n", tmp);
            }
        }
#endif
    }

  /* Post the semaphore to inform the parent task that we have completed
   * what we need to do.
   */

  g_spawn_parms.result = ret;
#ifndef CONFIG_SCHED_WAITPID
  nxsem_post(&g_spawn_execsem);
#endif
  return OK;
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
  struct sched_param param;
  pid_t proxy;
  pid_t pid = INVALID_PROCESS_ID;
#ifdef CONFIG_SCHED_WAITPID
  int status;
#endif
  int ret;

  sinfo("name=%s entry=%p file_actions=%p attr=%p argv=%p\n",
        name, entry, file_actions, attr, argv);

  /* If there are no file actions to be performed and there is no change to
   * the signal mask, then start the new child task directly from the parent
   * task.
   */

  if ((file_actions == NULL || *file_actions == NULL) &&
      (attr == NULL || (attr->flags & POSIX_SPAWN_SETSIGMASK) == 0))
    {
      ret = nxtask_spawn_exec(&pid, name, entry, attr, argv, envp);
      if (ret < 0)
        {
          return ret;
        }

      return pid;
    }

  /* Otherwise, we will have to go through an intermediary/proxy task in
   * order to perform the I/O redirection.  This would be a natural place to
   * fork(). However, true fork() behavior requires an MMU and most
   * implementations of vfork() are not capable of these operations.
   *
   * Even without fork(), we can still do the job, but parameter passing is
   * messier.  Unfortunately, there is no (clean) way to pass binary values
   * as a task parameter, so we will use a semaphore-protected global
   * structure.
   */

  /* Get exclusive access to the global parameter structure */

  ret = nxmutex_lock(&g_spawn_parmlock);
  if (ret < 0)
    {
      serr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Populate the parameter structure */

  g_spawn_parms.result       = ENOSYS;
  g_spawn_parms.pid          = &pid;
  g_spawn_parms.file_actions = file_actions ? *file_actions : NULL;
  g_spawn_parms.attr         = attr;
  g_spawn_parms.argv         = argv;
  g_spawn_parms.envp         = envp;
  g_spawn_parms.u.task.name  = name;
  g_spawn_parms.u.task.entry = entry;

  /* Get the priority of this (parent) task */

  ret = nxsched_get_param(0, &param);
  if (ret < 0)
    {
      serr("ERROR: nxsched_get_param failed: %d\n", ret);
      g_spawn_parms.pid = NULL;
      nxmutex_unlock(&g_spawn_parmlock);
      return ret;
    }

#ifdef CONFIG_SCHED_WAITPID
  /* Disable pre-emption so that the proxy does not run until waitpid
   * is called.  This is probably unnecessary since the nxtask_spawn_proxy
   * has the same priority as this thread; it should be schedule behind
   * this task in the ready-to-run list.
   *
   * REVISIT:  This will may not have the desired effect in SMP mode.
   */

  sched_lock();
#endif

  /* Start the intermediary/proxy task at the same priority as the parent
   * task.
   */

  proxy = nxtask_create("nxtask_spawn_proxy", param.sched_priority,
                        NULL, CONFIG_POSIX_SPAWN_PROXY_STACKSIZE,
                        nxtask_spawn_proxy, NULL, NULL);
  if (proxy < 0)
    {
      ret = proxy;
      serr("ERROR: Failed to start nxtask_spawn_proxy: %d\n", ret);
      goto errout_with_lock;
    }

  /* Wait for the proxy to complete its job */

#ifdef CONFIG_SCHED_WAITPID
  /* REVISIT: This should not call waitpid() directly.  waitpid is a
   * cancellation point and modifies the errno value.  It is inappropriate
   * for use within the OS.
   */

  ret = nxsched_waitpid(proxy, &status, 0);
  if (ret < 0)
    {
      serr("ERROR: waitpid() failed: %d\n", ret);
      goto errout_with_lock;
    }
#else
  ret = nxsem_wait_uninterruptible(&g_spawn_execsem);
  if (ret < 0)
    {
      serr("ERROR: g_spawn_execsem() failed: %d\n", ret);
      goto errout_with_lock;
    }
#endif

  /* Get the result and relinquish our access to the parameter structure */

  ret = -g_spawn_parms.result;
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  ret = (int)pid;

errout_with_lock:
#ifdef CONFIG_SCHED_WAITPID
  sched_unlock();
#endif
  g_spawn_parms.pid = NULL;
  nxmutex_unlock(&g_spawn_parmlock);
  return ret;
}

#endif /* CONFIG_BUILD_KERNEL */
