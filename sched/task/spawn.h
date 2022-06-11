/****************************************************************************
 * sched/task/spawn.h
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
  FAR char * const *envp;

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

int spawn_semtake(FAR sem_t *sem);
#define spawn_semgive(sem) nxsem_post(sem)

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
 *   0 (OK) on success; A negated errno value is returned on failure.
 *
 ****************************************************************************/

int spawn_proxyattrs(FAR const posix_spawnattr_t *attr,
                     FAR const posix_spawn_file_actions_t *file_actions);

#endif /* __SCHED_TASK_SPAWN_H */
