/****************************************************************************
 * sched/group/group_getgroups.c
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
 * Name: getgroups
 *
 * Description:
 *   getgroups() returns the supplementary group IDs of the calling
 *   process.  The returned list is exactly the set installed by
 *   setgroups()/initgroups() (may be empty).  The effective group ID is
 *   not synthesized into an empty list; callers that need it should use
 *   getegid().
 *
 * Input Parameters:
 *   gidsetsize - Number of slots in grouplist, or 0 to query the count.
 *   grouplist  - Buffer for group IDs (unused when gidsetsize is 0).
 *
 * Returned Value:
 *   Number of group IDs on success; -1 on failure with errno set.
 *
 ****************************************************************************/

int getgroups(int gidsetsize, FAR gid_t grouplist[])
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
  int count;

  if (gidsetsize < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  rtcb   = this_task();
  rgroup = rtcb->group;
  DEBUGASSERT(rgroup != NULL);

  count = rgroup->tg_ngroups;

  if (gidsetsize == 0)
    {
      return count;
    }

  if (grouplist == NULL)
    {
      set_errno(EFAULT);
      return ERROR;
    }

  if (gidsetsize < count)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (count > 0)
    {
      memcpy(grouplist, rgroup->tg_groups, count * sizeof(gid_t));
    }

  return count;
}
