/****************************************************************************
 * libs/libc/grp/lib_initgroups.c
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

#include <grp.h>
#include <limits.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: initgroups
 *
 * Description:
 *   The group database is read to determine all groups of which user is a
 *   member.  The additional group 'group' is also included.  The resulting
 *   set is installed as the calling process's supplementary group IDs via
 *   setgroups().
 *
 * Input Parameters:
 *   user  - Name of the user to query the group database for.
 *   group - Additional gid to add to the list of group IDs.
 *
 * Returned Value:
 *   Zero if successful, and -1 on failure with errno set.
 *
 ****************************************************************************/

int initgroups(FAR const char *user, gid_t group)
{
#if defined(CONFIG_SCHED_NGROUPS) && CONFIG_SCHED_NGROUPS > 0
  gid_t groups[NGROUPS_MAX];
  int ngroups = NGROUPS_MAX;
  int ret;

  if (user == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  ret = getgrouplist(user, group, groups, &ngroups);
  if (ret < 0)
    {
      /* Buffer too small or lookup failure — errno already set by
       * getgrouplist when applicable.
       */

      if (ngroups > NGROUPS_MAX)
        {
          swarn("initgroups: user '%s' has %d groups, NGROUPS_MAX=%d\n",
                user, ngroups, NGROUPS_MAX);
          set_errno(EINVAL);
        }

      return ERROR;
    }

  return setgroups(ret, groups);
#else
  /* Without supplementary group storage, succeed silently. */

  UNUSED(user);
  UNUSED(group);
  return 0;
#endif
}
