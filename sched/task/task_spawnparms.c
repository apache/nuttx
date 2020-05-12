/****************************************************************************
 * sched/task/task_spawnparms.c
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

#include <fcntl.h>
#include <spawn.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spawn.h>
#include <nuttx/fs/fs.h>

#include "task/spawn.h"
#include "task/task.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

sem_t g_spawn_parmsem = SEM_INITIALIZER(1);
#ifndef CONFIG_SCHED_WAITPID
sem_t g_spawn_execsem = SEM_INITIALIZER(0);
#endif
struct spawn_parms_s g_spawn_parms;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxspawn_close, nxspawn_dup2, and nxspawn_open
 *
 * Description:
 *   Implement individual file actions
 *
 * Input Parameters:
 *   action - describes the action to be performed
 *
 * Returned Value:
 *   posix_spawn() and posix_spawnp() will return zero on success.
 *   Otherwise, an error number will be returned as the function return
 *   value to indicate the error.
 *
 ****************************************************************************/

static inline int nxspawn_close(FAR struct spawn_close_file_action_s *action)
{
  /* The return value from close() is ignored */

  sinfo("Closing fd=%d\n", action->fd);

  close(action->fd);
  return OK;
}

static inline int nxspawn_dup2(FAR struct spawn_dup2_file_action_s *action)
{
  int ret;

  /* Perform the dup */

  sinfo("Dup'ing %d->%d\n", action->fd1, action->fd2);

  ret = dup2(action->fd1, action->fd2);
  if (ret < 0)
    {
      int errcode = get_errno();

      serr("ERROR: dup2 failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

static inline int nxspawn_open(FAR struct spawn_open_file_action_s *action)
{
  int fd;
  int ret = OK;

  /* Open the file */

  sinfo("Open'ing path=%s oflags=%04x mode=%04x\n",
        action->path, action->oflags, action->mode);

  fd = nx_open(action->path, action->oflags, action->mode);
  if (fd < 0)
    {
      ret = fd;
      serr("ERROR: open failed: %d\n", ret);
    }

  /* Does the return file descriptor happen to match the required file
   * descriptor number?
   */

  else if (fd != action->fd)
    {
      /* No.. dup2 to get the correct file number */

      sinfo("Dup'ing %d->%d\n", fd, action->fd);

      ret = dup2(fd, action->fd);
      if (ret < 0)
        {
          ret = get_errno();
          serr("ERROR: dup2 failed: %d\n", ret);
        }

      sinfo("Closing fd=%d\n", fd);
      close(fd);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spawn_semtake and spawn_semgive
 *
 * Description:
 *   Give and take semaphores
 *
 * Input Parameters:
 *
 *   sem - The semaphore to act on.
 *
 * Returned Value:
 *   See nxsem_wait_uninterruptible
 *
 ****************************************************************************/

int spawn_semtake(FAR sem_t *sem)
{
  return nxsem_wait_uninterruptible(sem);
}

/****************************************************************************
 * Name: spawn_execattrs
 *
 * Description:
 *   Set attributes of the new child task after it has been spawned.
 *
 * Input Parameters:
 *
 *   pid - The pid of the new task.
 *   attr - The attributes to use
 *
 * Returned Value:
 *   Errors are not reported by this function.  This is not because errors
 *   cannot occur, but rather that the new task has already been started
 *   so there is no graceful way to handle errors detected in this context
 *   (unless we delete the new task and recover).
 *
 * Assumptions:
 *   That task has been started but has not yet executed because pre-
 *   emption is disabled.
 *
 ****************************************************************************/

int spawn_execattrs(pid_t pid, FAR const posix_spawnattr_t *attr)
{
  struct sched_param param;
  int ret;

  DEBUGASSERT(attr);

  /* Now set the attributes.  Note that we ignore all of the return values
   * here because we have already successfully started the task.  If we
   * return an error value, then we would also have to stop the task.
   */

  /* If we are only setting the priority, then call sched_setparm()
   * to set the priority of the of the new task.
   */

  if ((attr->flags & POSIX_SPAWN_SETSCHEDPARAM) != 0)
    {
#ifdef CONFIG_SCHED_SPORADIC
      /* Get the current sporadic scheduling parameters.  Those will not be
       * modified.
       */

      ret = nxsched_get_param(pid, &param);
      if (ret < 0)
        {
          return ret;
        }
#endif

      /* Get the priority from the attributes */

      param.sched_priority = attr->priority;

      /* If we are setting *both* the priority and the scheduler,
       * then we will call nxsched_set_scheduler() below.
       */

      if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) == 0)
        {
          sinfo("Setting priority=%d for pid=%d\n",
                param.sched_priority, pid);

          ret = nxsched_set_param(pid, &param);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  /* If we are only changing the scheduling policy, then reset
   * the priority to the default value (the same as this thread) in
   * preparation for the nxsched_set_scheduler() call below.
   */

  else if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
    {
      ret = nxsched_get_param(0, &param);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Are we setting the scheduling policy?  If so, use the priority
   * setting determined above.
   */

  if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
    {
      sinfo("Setting policy=%d priority=%d for pid=%d\n",
            attr->policy, param.sched_priority, pid);

#ifdef CONFIG_SCHED_SPORADIC
      /* But take the sporadic scheduler parameters from the attributes */

      param.sched_ss_low_priority        = (int)attr->low_priority;
      param.sched_ss_max_repl            = (int)attr->max_repl;
      param.sched_ss_repl_period.tv_sec  = attr->repl_period.tv_sec;
      param.sched_ss_repl_period.tv_nsec = attr->repl_period.tv_nsec;
      param.sched_ss_init_budget.tv_sec  = attr->budget.tv_sec;
      param.sched_ss_init_budget.tv_nsec = attr->budget.tv_nsec;
#endif
      nxsched_set_scheduler(pid, attr->policy, &param);
    }

  return OK;
}

/****************************************************************************
 * Name: spawn_proxyattrs
 *
 * Description:
 *   Set attributes of the proxy task before it has started the new child
 *   task.
 *
 * Input Parameters:
 *
 *   pid - The pid of the new task.
 *   attr - The attributes to use
 *   file_actions - The attributes to use
 *
 * Returned Value:
 *   0 (OK) on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int spawn_proxyattrs(FAR const posix_spawnattr_t *attr,
                     FAR const posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *entry;
  int ret = OK;

  /* Check if we need to change the signal mask */

  if (attr != NULL && (attr->flags & POSIX_SPAWN_SETSIGMASK) != 0)
    {
      nxsig_procmask(SIG_SETMASK, &attr->sigmask, NULL);
    }

  /* Were we also requested to perform file actions? */

  if (file_actions != NULL)
    {
      /* Yes.. Execute each file action */

      for (entry = (FAR struct spawn_general_file_action_s *)file_actions;
           entry && ret == OK;
           entry = entry->flink)
        {
          switch (entry->action)
            {
              case SPAWN_FILE_ACTION_CLOSE:
                ret = nxspawn_close((FAR struct spawn_close_file_action_s *)
                                    entry);
                break;

              case SPAWN_FILE_ACTION_DUP2:
                ret = nxspawn_dup2((FAR struct spawn_dup2_file_action_s *)
                                   entry);
                break;

              case SPAWN_FILE_ACTION_OPEN:
                ret = nxspawn_open((FAR struct spawn_open_file_action_s *)
                                   entry);
                break;

              case SPAWN_FILE_ACTION_NONE:
              default:
                serr("ERROR: Unknown action: %d\n", entry->action);
                ret = EINVAL;
                break;
            }
        }
    }

  return ret;
}
