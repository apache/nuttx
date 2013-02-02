/****************************************************************************
 * sched/task_spawnparms.c
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

#include <semaphore.h>
#include <fcntl.h>
#include <spawn.h>
#include <debug.h>

#include <nuttx/spawn.h>

#include "spawn_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

sem_t g_spawn_parmsem = SEM_INITIALIZER(1);
#ifndef CONFIG_SCHED_WAITPID
sem_t g_spawn_execsem = SEM_INITIALIZER(0);
#endif
struct spawn_parms_s g_spawn_parms;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spawn_close, spawn_dup2, and spawn_open
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

static inline int spawn_close(FAR struct spawn_close_file_action_s *action)
{
  /* The return value from close() is ignored */

  svdbg("Closing fd=%d\n", action->fd);

  (void)close(action->fd);
  return OK;
}

static inline int spawn_dup2(FAR struct spawn_dup2_file_action_s *action)
{
  int ret;

  /* Perform the dup */

  svdbg("Dup'ing %d->%d\n", action->fd1, action->fd2);

  ret = dup2(action->fd1, action->fd2);
  if (ret < 0)
    {
      int errcode = errno;

      sdbg("ERROR: dup2 failed: %d\n", errcode);
      return errcode;
    }

  return OK;
}

static inline int spawn_open(FAR struct spawn_open_file_action_s *action)
{
  int fd;
  int ret = OK;

  /* Open the file */

  svdbg("Open'ing path=%s oflags=%04x mode=%04x\n",
        action->path, action->oflags, action->mode);

  fd = open(action->path, action->oflags, action->mode);
  if (fd < 0)
    {
      ret = errno;
      sdbg("ERROR: open failed: %d\n", ret);
    }

  /* Does the return file descriptor happen to match the required file
   * desciptor number?
   */

  else if (fd != action->fd)
    {
      /* No.. dup2 to get the correct file number */

      svdbg("Dup'ing %d->%d\n", fd, action->fd);

      ret = dup2(fd, action->fd);
      if (ret < 0)
        {
          ret = errno;
          sdbg("ERROR: dup2 failed: %d\n", ret);
        }

      svdbg("Closing fd=%d\n", fd);
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
 *   None
 *
 ****************************************************************************/

void spawn_semtake(FAR sem_t *sem)
{
  int ret;

  do
    {
      ret = sem_wait(sem);
      ASSERT(ret == 0 || errno == EINTR);
    }
  while (ret != 0);
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
 *   Errors are not reported by this function.  This is because errors
 *   cannot occur, but ratther that the new task has already been started
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
      /* Get the priority from the attrributes */

      param.sched_priority = attr->priority;

      /* If we are setting *both* the priority and the scheduler,
       * then we will call sched_setscheduler() below.
       */

      if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) == 0)
        {
          svdbg("Setting priority=%d for pid=%d\n",
                param.sched_priority, pid);

          (void)sched_setparam(pid, &param);
        }
    }

  /* If we are only changing the scheduling policy, then reset
   * the priority to the default value (the same as this thread) in
   * preparation for the sched_setscheduler() call below.
   */

  else if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
    {
      (void)sched_getparam(0, &param);
    }

  /* Are we setting the scheduling policy?  If so, use the priority
   * setting determined above.
   */

  if ((attr->flags & POSIX_SPAWN_SETSCHEDULER) != 0)
    {
      svdbg("Setting policy=%d priority=%d for pid=%d\n",
            attr->policy, param.sched_priority, pid);

      (void)sched_setscheduler(pid, attr->policy, &param);
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
 *   0 (OK) on successed; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int spawn_proxyattrs(FAR const posix_spawnattr_t *attr,
                     FAR const posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *entry;
  int ret = OK;

  /* Check if we need to change the signal mask */

#ifndef CONFIG_DISABLE_SIGNALS
  if (attr && (attr->flags & POSIX_SPAWN_SETSIGMASK) != 0)
    {
      (void)sigprocmask(SIG_SETMASK, &attr->sigmask, NULL);
    }

  /* Were we also requested to perform file actions? */

  if (file_actions)
#endif
    {
      /* Yes.. Execute each file action */

      for (entry = (FAR struct spawn_general_file_action_s *)file_actions;
           entry && ret == OK;
           entry = entry->flink)
        {
          switch (entry->action)
            {
            case SPAWN_FILE_ACTION_CLOSE:
              ret = spawn_close((FAR struct spawn_close_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_DUP2:
              ret = spawn_dup2((FAR struct spawn_dup2_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_OPEN:
              ret = spawn_open((FAR struct spawn_open_file_action_s *)entry);
              break;

            case SPAWN_FILE_ACTION_NONE:
            default:
              sdbg("ERROR: Unknown action: %d\n", entry->action);
              ret = EINVAL;
              break;
            }
        }
    }

  return ret;
}
