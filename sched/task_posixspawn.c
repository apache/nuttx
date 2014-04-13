/****************************************************************************
 * sched/task_posixspawn.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/wait.h>
#include <spawn.h>
#include <debug.h>

#include <nuttx/binfmt/binfmt.h>

#include "os_internal.h"
#include "group_internal.h"
#include "spawn_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_exec
 *
 * Description:
 *   Execute the task from the file system.
 *
 * Input Parameters:
 *
 *   pidp - Upon successful completion, this will return the task ID of the
 *     child task in the variable pointed to by a non-NULL 'pid' argument.|
 *
 *   path - The 'path' argument identifies the file to execute.  If
 *     CONFIG_BINFMT_EXEPATH is defined, this may be either a relative or
 *     or an absolute path.  Otherwise, it must be an absolute path.
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
 *     NOTE: POSIX_SPAWN_SETSIGMASK is handled in ps_proxy().
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 * Returned Value:
 *   This function will return zero on success. Otherwise, an error number
 *   will be returned as the function return value to indicate the error.
 *   This errno value may be that set by execv(), sched_setpolicy(), or
 *   sched_setparam().
 *
 ****************************************************************************/

static int posix_spawn_exec(FAR pid_t *pidp, FAR const char *path,
                            FAR const posix_spawnattr_t *attr,
                            FAR char * const argv[])
{
  FAR const struct symtab_s *symtab;
  int nsymbols;
  int pid;
  int ret = OK;

  DEBUGASSERT(path);

  /* Get the current symbol table selection */

  exec_getsymtab(&symtab, &nsymbols);

  /* Disable pre-emption so that we can modify the task parameters after
   * we start the new task; the new task will not actually begin execution
   * until we re-enable pre-emption.
   */

  sched_lock();

  /* Start the task */

  pid = exec(path, (FAR char * const *)argv, symtab, nsymbols);
  if (pid < 0)
    {
      ret = errno;
      sdbg("ERROR: exec failed: %d\n", ret);
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
      (void)spawn_execattrs(pid, attr);
    }

  /* Re-enable pre-emption and return */

errout:
  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: posix_spawn_proxy
 *
 * Description:
 *   Perform file_actions, then execute the task from the file system.
 *
 *   Do we really need this proxy task?  Isn't that wasteful?
 *
 *   Q: Why not use a starthook so that there is callout from task_start()
 *      to perform these operations after the file is loaded from
 *      the file system?
 *   A: That existing task_starthook() implementation cannot be used in
 *      this context; any of task_starthook() will also conflict with
 *      binfmt's use of the start hook to call C++ static initializers.
 *      task_restart() would also be an issue.
 *
 * Input Parameters:
 *   Standard task start-up parameters
 *
 * Returned Value:
 *   Standard task return value.
 *
 ****************************************************************************/

static int posix_spawn_proxy(int argc, FAR char *argv[])
{
  int ret;

  /* Perform file actions and/or set a custom signal mask.  We get here only
   * if the file_actions parameter to posix_spawn[p] was non-NULL and/or the
   * option to change the signal mask was selected.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  DEBUGASSERT(g_spawn_parms.file_actions ||
              (g_spawn_parms.attr &&
              (g_spawn_parms.attr->flags & POSIX_SPAWN_SETSIGMASK) != 0));
#else
  DEBUGASSERT(g_spawn_parms.file_actions);
#endif

  /* Set the attributes and perform the file actions as appropriate */

  ret = spawn_proxyattrs(g_spawn_parms.attr, g_spawn_parms.file_actions);
  if (ret == OK)
    {
      /* Start the task */

      ret = posix_spawn_exec(g_spawn_parms.pid, g_spawn_parms.u.posix.path,
                             g_spawn_parms.attr, g_spawn_parms.argv);

#ifdef CONFIG_SCHED_HAVE_PARENT
      if (ret == OK)
        {
          /* Change of the parent of the task we just spawned to our parent.
           * What should we do in the event of a failure?
           */

          int tmp = task_reparent(0, *g_spawn_parms.pid);
          if (tmp < 0)
            {
              sdbg("ERROR: task_reparent() failed: %d\n", tmp);
            }
        }
#endif
    }

  /* Post the semaphore to inform the parent task that we have completed
   * what we need to do.
   */

  g_spawn_parms.result = ret;
#ifndef CONFIG_SCHED_WAITPID
  spawn_semgive(&g_spawn_execsem);
#endif
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn
 *
 * Description:
 *   The posix_spawn() and posix_spawnp() functions will create a new,
 *   child task, constructed from a regular executable file.
 *
 * Input Parameters:
 *
 *   pid - Upon successful completion, posix_spawn() and posix_spawnp() will
 *     return the task ID of the child task to the parent task, in the
 *     variable pointed to by a non-NULL 'pid' argument.  If the 'pid'
 *     argument is a null pointer, the process ID of the child is not
 *     returned to the caller.
 *
 *   path - The 'path' argument to posix_spawn() is the absolute path that
 *     identifies the file to execute.  The 'path' argument to posix_spawnp()
 *     may also be a relative path and will be used to construct a pathname
 *     that identifies the file to execute.  In the case of a relative path,
 *     the path prefix for the file will be obtained by a search of the
 *     directories passed as the environment variable PATH.
 *
 *     NOTE: NuttX provides only one implementation:  If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
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
 *     - POSIX_SPAWN_RESETIDS: Resetting of the effective user ID of the child
 *       process is not supported.  NuttX does not support effective user
 *       IDs.
 *     - POSIX_SPAWN_SETSIGMASK: Set the new task's signal mask.
 *     - POSIX_SPAWN_SETSIGDEF:  Resetting signal default actions is not
 *       supported.  NuttX does not support default signal actions.
 *
 *   argv - argv[] is the argument list for the new task.  argv[] is an
 *     array of pointers to null-terminated strings. The list is terminated
 *     with a null pointer.
 *
 *   envp - The envp[] argument is not used by NuttX and may be NULL.  In
 *     standard implementations, envp[] is an array of character pointers to
 *     null-terminated strings that provide the environment for the new
 *     process image. The environment array is terminated by a null pointer.
 *     In NuttX, the envp[] argument is ignored and the new task will simply
 *     inherit the environment of the parent task.
 *
 * Returned Value:
 *   posix_spawn() and posix_spawnp() will return zero on success.
 *   Otherwise, an error number will be returned as the function return
 *   value to indicate the error:
 *
 *   - EINVAL: The value specified by 'file_actions' or 'attr' is invalid.
 *   - Any errors that might have been return if vfork() and excec[l|v]()
 *     had been called.
 *
 * Assumptions/Limitations:
 *   - NuttX provides only posix_spawn() or posix_spawnp() behavior
 *     depending upon the setting of CONFIG_BINFMT_EXEPATH: If
 *     CONFIG_BINFMT_EXEPATH is defined, then only posix_spawnp() behavior
 *     is supported; otherwise, only posix_spawn behavior is supported.
 *   - The 'envp' argument is not used and the 'environ' variable is not
 *     altered (NuttX does not support the 'environ' variable).
 *   - Process groups are not supported (POSIX_SPAWN_SETPGROUP).
 *   - Effective user IDs are not supported (POSIX_SPAWN_RESETIDS).
 *   - Signal default actions cannot be modified in the newly task executed
 *     because NuttX does not support default signal actions
 *     (POSIX_SPAWN_SETSIGDEF).
 *
 * POSIX Compatibility
 *   - The value of the argv[0] received by the child task is assigned by
 *     NuttX.  For the caller of posix_spawn(), the provided argv[0] will
 *     correspond to argv[1] received by the new task.
 *
 ****************************************************************************/

#ifdef CONFIG_BINFMT_EXEPATH
int posix_spawnp(FAR pid_t *pid, FAR const char *path,
                 FAR const posix_spawn_file_actions_t *file_actions,
                 FAR const posix_spawnattr_t *attr,
                 FAR char *const argv[], FAR char *const envp[])
#else
int posix_spawn(FAR pid_t *pid, FAR const char *path,
                FAR const posix_spawn_file_actions_t *file_actions,
                FAR const posix_spawnattr_t *attr,
                FAR char *const argv[], FAR char *const envp[])
#endif
{
  struct sched_param param;
  pid_t proxy;
#ifdef CONFIG_SCHED_WAITPID
  int status;
#endif
  int ret;

  DEBUGASSERT(path);

  svdbg("pid=%p path=%s file_actions=%p attr=%p argv=%p\n",
        pid, path, file_actions, attr, argv);

  /* If there are no file actions to be performed and there is no change to
   * the signal mask, then start the new child task directly from the parent task.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  if ((file_actions == NULL || *file_actions == NULL) &&
      (attr == NULL || (attr->flags & POSIX_SPAWN_SETSIGMASK) == 0))
#else
  if (file_actions ==  NULL || *file_actions == NULL)
#endif
    {
      return posix_spawn_exec(pid, path, attr, argv);
    }

  /* Otherwise, we will have to go through an intermediary/proxy task in order
   * to perform the I/O redirection.  This would be a natural place to fork().
   * However, true fork() behavior requires an MMU and most implementations
   * of vfork() are not capable of these operations.
   *
   * Even without fork(), we can still do the job, but parameter passing is
   * messier.  Unfortunately, there is no (clean) way to pass binary values
   * as a task parameter, so we will use a semaphore-protected global
   * structure.
   */

  /* Get exclusive access to the global parameter structure */

  spawn_semtake(&g_spawn_parmsem);

  /* Populate the parameter structure */

  g_spawn_parms.result       = ENOSYS;
  g_spawn_parms.pid          = pid;
  g_spawn_parms.file_actions = file_actions ? *file_actions : NULL;
  g_spawn_parms.attr         = attr;
  g_spawn_parms.argv         = argv;
  g_spawn_parms.u.posix.path = path;

  /* Get the priority of this (parent) task */

  ret = sched_getparam(0, &param);
  if (ret < 0)
    {
      int errcode = errno;

      sdbg("ERROR: sched_getparam failed: %d\n", errcode);
      spawn_semgive(&g_spawn_parmsem);
      return errcode;
    }

  /* Disable pre-emption so that the proxy does not run until waitpid
   * is called.  This is probably unnecessary since the posix_spawn_proxy has
   * the same priority as this thread; it should be schedule behind this
   * task in the ready-to-run list.
   */

#ifdef CONFIG_SCHED_WAITPID
  sched_lock();
#endif

  /* Start the intermediary/proxy task at the same priority as the parent
   * task.
   */

  proxy = TASK_CREATE("posix_spawn_proxy", param.sched_priority,
                      CONFIG_POSIX_SPAWN_PROXY_STACKSIZE,
                      (main_t)posix_spawn_proxy,
                      (FAR char * const *)NULL);
  if (proxy < 0)
    {
      ret = get_errno();
      sdbg("ERROR: Failed to start posix_spawn_proxy: %d\n", ret);

      goto errout_with_lock;
    }

   /* Wait for the proxy to complete its job */

#ifdef CONFIG_SCHED_WAITPID
   ret = waitpid(proxy, &status, 0);
   if (ret < 0)
     {
       sdbg("ERROR: waitpid() failed: %d\n", errno);
       goto errout_with_lock;
     }
#else
   spawn_semtake(&g_spawn_execsem);
#endif

   /* Get the result and relinquish our access to the parameter structure */

   ret = g_spawn_parms.result;

errout_with_lock:
#ifdef CONFIG_SCHED_WAITPID
  sched_unlock();
#endif
  spawn_semgive(&g_spawn_parmsem);
  return ret;
}
