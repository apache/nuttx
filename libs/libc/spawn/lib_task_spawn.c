/****************************************************************************
 * libs/libc/spawn/lib_task_spawn.c
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

#include <spawn.h>
#include <nuttx/spawn.h>

#if defined(CONFIG_LIB_SYSCALL) && !defined(CONFIG_BUILD_KERNEL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_spawn
 *
 * Description:
 *   This function marshals task_spwan() parameters and invokes the
 *   nx_task_spawn() system call.
 *
 *   task_spawn() and posix_spawn() are NuttX OS interfaces.  In PROTECTED
 *   and KERNEL build modes, then can be reached from applications only via
 *   a system call.  Currently, the number of parameters in a system call
 *   is limited to six; these spawn function have seven parameters.  Rather
 *   than extend the maximum number of parameters across all architectures,
 *   I opted instead to marshal the seven parameters into a structure.
 *
 *
 * Input Parameters:
 *   file_actions - The address of the posix_spawn_file_actions_t to be
 *   initialized.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int task_spawn(FAR pid_t *pid, FAR const char *name, main_t entry,
               FAR const posix_spawn_file_actions_t *file_actions,
               FAR const posix_spawnattr_t *attr,
               FAR char * const argv[], FAR char * const envp[])
{
  struct spawn_syscall_parms_s parms;

  parms.pid          = pid;
  parms.name         = name;
  parms.entry        = entry;
  parms.file_actions = file_actions;
  parms.attr         = attr;
  parms.argv         = (FAR char * const *)argv;
  parms.envp         = (FAR char * const *)envp;

  return nx_task_spawn(&parms);
}

#endif /* CONFIG_LIB_SYSCALL && !CONFIG_BUILD_KERNEL */
