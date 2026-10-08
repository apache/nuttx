/****************************************************************************
 * sched/environ/env_sync.c
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

#include <nuttx/addrenv.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "environ/environ.h"

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_user_environ
 *
 * Description:
 *   Return the address of the program's environ variable, or NULL if the
 *   caller cannot reach it: a kernel thread, a task of another group, or a
 *   program that did not register the variable.
 *
 ****************************************************************************/

static FAR char ***env_user_environ(FAR struct task_group_s *group)
{
  FAR struct tcb_s *rtcb = this_task();

  if (rtcb->group != group ||
      (rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL ||
      rtcb->addrenv_curr != rtcb->addrenv_own)
    {
      return NULL;
    }

  return ARCH_DATA_RESERVE->ar_environ;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: env_sync_in
 ****************************************************************************/

void env_sync_in(FAR struct task_group_s *group)
{
  FAR char ***uenvp = env_user_environ(group);
  FAR char **oldenvp;
  size_t oldenvc;
  size_t oldenvpc;
  int i;

  if (uenvp == NULL || *uenvp == group->tg_envp)
    {
      return;
    }

  /* The program assigned its own array.  Copy it before the old
   * environment is freed: its strings often point into the old one.
   */

  oldenvp  = group->tg_envp;
  oldenvc  = group->tg_envc;
  oldenvpc = group->tg_envpc;

  group->tg_envp  = NULL;
  group->tg_envc  = 0;
  group->tg_envpc = 0;

  if (*uenvp != NULL && env_dup(group, *uenvp) < 0)
    {
      /* Out of memory: keep the old environment */

      group->tg_envp  = oldenvp;
      group->tg_envc  = oldenvc;
      group->tg_envpc = oldenvpc;
      *uenvp = group->tg_envp;
      return;
    }

  if (oldenvp != NULL)
    {
      for (i = 0; oldenvp[i] != NULL; i++)
        {
          group_free(group, oldenvp[i]);
        }

      group_free(group, oldenvp);
    }

  *uenvp = group->tg_envp;
}

/****************************************************************************
 * Name: env_sync_out
 ****************************************************************************/

void env_sync_out(FAR struct task_group_s *group)
{
  FAR char ***uenvp = env_user_environ(group);

  if (uenvp != NULL)
    {
      *uenvp = group->tg_envp;
    }
}

#endif /* CONFIG_BUILD_KERNEL */
