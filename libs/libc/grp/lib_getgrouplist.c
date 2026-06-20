/****************************************************************************
 * libs/libc/grp/lib_getgrouplist.c
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

#include "grp/lib_grp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgrouplist
 *
 * Description:
 *   The getgrouplist() function scans the group database to obtain the list
 *   of groups that 'user' belongs to.  The group 'group' (typically the
 *   primary group ID from the password database) is always included in the
 *   returned list and is stored as the first element.
 *
 * Input Parameters:
 *   user    - The user name whose group membership is queried.
 *   group   - An additional group ID (the primary group) to include.
 *   groups  - Location to return the list of group IDs.
 *   ngroups - On input, the number of slots available in groups.  On output,
 *             the actual number of group IDs found.
 *
 * Returned Value:
 *   On success, the number of group IDs found is returned.  If the number of
 *   groups exceeds the available space, -1 is returned and ngroups is set to
 *   the number of group IDs that would have been returned.
 *
 ****************************************************************************/

int getgrouplist(FAR const char *user, gid_t group, FAR gid_t *groups,
                 FAR int *ngroups)
{
  int maxgroups = *ngroups;
  int count;

  /* The primary group is always reported first. */

  if (maxgroups > 0)
    {
      groups[0] = group;
    }

  count = 1;

#ifdef CONFIG_LIBC_GROUP_FILE
  /* Scan the group file for any supplementary groups that 'user' belongs to
   * and append their group IDs to the list.
   */

  grp_findby_user(user, group, groups, maxgroups, &count);
#else
  /* Without a group file there is no membership information, so only the
   * primary group is reported.
   */

  UNUSED(user);
#endif

  /* If the caller's buffer was too small, report the required size and
   * return failure.  Otherwise, return the number of group IDs stored.
   */

  *ngroups = count;
  return count > maxgroups ? -1 : count;
}
