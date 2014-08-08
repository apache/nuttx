/****************************************************************************
 * sched/task/spawn.h
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

#ifndef __SCHED_TASK_SPAWN_H
#define __SCHED_TASK_SPAWN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <spawn.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_POSIX_SPAWN_PROXY_STACKSIZE
#  define CONFIG_POSIX_SPAWN_PROXY_STACKSIZE 1024
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct spawn_parms_s
{
  /* Common parameters */

  int result;
  FAR pid_t *pid;
  FAR const posix_spawn_file_actions_t *file_actions;
  FAR const posix_spawnattr_t *attr;
  FAR char * const *argv;

  /* Parameters that differ for posix_spawn[p] and task_spawn */

  union
  {
    struct
    {
      FAR const char *path;
    } posix;
    struct
    {
      FAR const char *name;
      main_t entry;
    } task;
  } u;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern sem_t g_spawn_parmsem;
#ifndef CONFIG_SCHED_WAITPID
extern sem_t g_spawn_execsem;
#endif
extern struct spawn_parms_s g_spawn_parms;

/****************************************************************************
 * Public Function Prototypes
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

void spawn_semtake(FAR sem_t *sem);
#define spawn_semgive(sem) sem_post(sem)

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

int spawn_execattrs(pid_t pid, FAR const posix_spawnattr_t *attr);

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
                     FAR const posix_spawn_file_actions_t *file_actions);

#endif /* __SCHED_TASK_SPAWN_H */
