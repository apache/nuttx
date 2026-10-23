/****************************************************************************
 * sched/group/group_rlimit.c
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

#include <sys/resource.h>
#include <errno.h>
#include <limits.h>
#include <sched.h>
#include <stdint.h>

#include <nuttx/sched.h>

#include "group/group.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_rlimit
 *
 * Description:
 *   Return the limit of a task group for a resource that NuttX enforces,
 *   or NULL for any other resource.
 *
 ****************************************************************************/

static inline FAR struct rlimit *group_rlimit(FAR struct task_group_s *group,
                                              int resource)
{
  switch (resource)
    {
      case RLIMIT_STACK:
        return &group->tg_rlimit_stack;

      case RLIMIT_NOFILE:
        return &group->tg_rlimit_nofile;

      default:
        return NULL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_init_rlimit
 *
 * Description:
 *   Give a new task group the resource limits of its parent, or the
 *   defaults if it has none: no stack limit and OPEN_MAX open files.  At
 *   boot, the idle task is its own parent: it gets the defaults.
 *
 ****************************************************************************/

void group_init_rlimit(FAR struct task_group_s *group,
                       FAR struct task_group_s *parent)
{
  if (parent != NULL && parent != group)
    {
      group->tg_rlimit_stack  = parent->tg_rlimit_stack;
      group->tg_rlimit_nofile = parent->tg_rlimit_nofile;
    }
  else
    {
      group->tg_rlimit_stack.rlim_cur  = RLIM_INFINITY;
      group->tg_rlimit_stack.rlim_max  = RLIM_INFINITY;
      group->tg_rlimit_nofile.rlim_cur = OPEN_MAX;
      group->tg_rlimit_nofile.rlim_max = OPEN_MAX;
    }
}

/****************************************************************************
 * Name: getrlimit
 *
 * Description:
 *   Return the soft and hard limit of a resource of the calling process.
 *   RLIMIT_STACK and RLIMIT_NOFILE are limits of the process.  The
 *   resources that NuttX does not limit are reported as RLIM_INFINITY.
 *
 * Input Parameters:
 *   resource - The resource, RLIMIT_*.
 *   rlp      - The location that receives the limits.
 *
 * Returned Value:
 *   Zero (OK) on success.  On failure, -1 (ERROR) is returned and errno is
 *   set to EINVAL: rlp is NULL or resource is not valid.
 *
 ****************************************************************************/

int getrlimit(int resource, FAR struct rlimit *rlp)
{
  FAR struct rlimit *limit;

  if (rlp == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  limit = group_rlimit(nxsched_self()->group, resource);
  if (limit != NULL)
    {
      *rlp = *limit;
      return OK;
    }

  switch (resource)
    {
      case RLIMIT_CORE:
      case RLIMIT_CPU:
      case RLIMIT_DATA:
      case RLIMIT_FSIZE:
      case RLIMIT_AS:
      case RLIMIT_MEMLOCK:
        rlp->rlim_cur = RLIM_INFINITY;
        rlp->rlim_max = RLIM_INFINITY;
        break;

      case RLIMIT_NICE:
        rlp->rlim_cur = 0;
        rlp->rlim_max = 0;
        break;

      case RLIMIT_RTPRIO:
        rlp->rlim_cur = SCHED_PRIORITY_DEFAULT;
        rlp->rlim_max = SCHED_PRIORITY_MAX;
        break;

      case RLIMIT_RTTIME:
        rlp->rlim_cur = UINT32_MAX;
        rlp->rlim_max = UINT32_MAX;
        break;

      default:
        set_errno(EINVAL);
        return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: setrlimit
 *
 * Description:
 *   Set the soft and hard limit of a resource of the calling process.  The
 *   process gives its limits to the processes that it creates.  NuttX
 *   enforces RLIMIT_STACK (the default stack size of new threads and
 *   programs) and RLIMIT_NOFILE (the number of open files).
 *
 * Input Parameters:
 *   resource - RLIMIT_STACK or RLIMIT_NOFILE.
 *   rlp      - The new limits.
 *
 * Returned Value:
 *   Zero (OK) on success.  On failure, -1 (ERROR) is returned and errno is
 *   set:
 *
 *     EINVAL - rlp is NULL, the soft limit is above the hard limit, or
 *              NuttX does not enforce the resource.
 *     EPERM  - The hard limit is raised without privileges, or the limit
 *              of open files is above OPEN_MAX.
 *
 ****************************************************************************/

int setrlimit(int resource, FAR const struct rlimit *rlp)
{
  FAR struct task_group_s *group = nxsched_self()->group;
  FAR struct rlimit *limit;

  limit = group_rlimit(group, resource);
  if (rlp == NULL || limit == NULL || rlp->rlim_cur > rlp->rlim_max)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (resource == RLIMIT_NOFILE && rlp->rlim_max > OPEN_MAX)
    {
      set_errno(EPERM);
      return ERROR;
    }

#ifdef CONFIG_SCHED_USER_IDENTITY
  if (rlp->rlim_max > limit->rlim_max && group->tg_euid != 0)
    {
      set_errno(EPERM);
      return ERROR;
    }
#endif

  *limit = *rlp;
  return OK;
}
