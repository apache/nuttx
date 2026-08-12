/****************************************************************************
 * sched/group/group_setgroups.c
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

#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <sched/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setgroups
 *
 * Description:
 *   setgroups() sets the supplementary group IDs for the calling process.
 *   Only a process with an effective user ID of 0 may change the list.
 *
 * Input Parameters:
 *   size - Number of group IDs in list (0 to clear).
 *   list - Array of supplementary group IDs, or NULL when size is 0.
 *
 * Returned Value:
 *   Zero on success; -1 on failure with errno set.
 *
 ****************************************************************************/

int setgroups(int size, FAR const gid_t *list)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;

  if (size < 0 || size > CONFIG_SCHED_NGROUPS)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (size > 0 && list == NULL)
    {
      set_errno(EFAULT);
      return ERROR;
    }

  rtcb   = this_task();
  rgroup = rtcb->group;
  DEBUGASSERT(rgroup != NULL);

  /* Only root (effective UID 0) may install a new supplementary set. */

  if (rgroup->tg_euid != 0)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (size > 0)
    {
      memcpy(rgroup->tg_groups, list, size * sizeof(gid_t));
    }

  rgroup->tg_ngroups = size;
  return OK;
}
