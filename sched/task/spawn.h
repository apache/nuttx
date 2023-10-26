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
#include <nuttx/mutex.h>
#include <spawn.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#endif /* __SCHED_TASK_SPAWN_H */
