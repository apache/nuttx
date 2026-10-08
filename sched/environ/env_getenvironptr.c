/****************************************************************************
 * sched/environ/env_getenvironptr.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef CONFIG_DISABLE_ENVIRON

#include <sched.h>
#include <stdlib.h>
#include "sched/sched.h"
#include "environ/environ.h"

#undef get_environ_ptr

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_environ_ptr
 *
 * Description:
 *   Return a pointer to the thread specific environ variable.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to the per-thread environ variable.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR char **get_environ_ptr(void)
{
  FAR struct task_group_s *group = this_task()->group;
#if defined(CONFIG_BUILD_KERNEL) || defined(CONFIG_LIBC_ENVIRON_ASSIGNABLE)
  FAR char **envp;

  /* The program may have assigned its own array to environ */

  nxrmutex_lock(&group->tg_mutex);
  env_sync_in(group);
  envp = group->tg_envp;
  nxrmutex_unlock(&group->tg_mutex);

  return envp;
#else
  return group->tg_envp;
#endif
}

#endif /* CONFIG_DISABLE_ENVIRON */
