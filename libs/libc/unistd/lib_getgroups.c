/****************************************************************************
 * libs/libc/unistd/lib_getgroups.c
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
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getgroups
 *
 * Description:
 *   The getgroups() function returns the supplementary group IDs of the
 *   calling process in the array grouplist.  NuttX does not support
 *   supplementary group IDs, so the calling process is treated as belonging
 *   to a single group: its effective group ID.
 *
 * Input Parameters:
 *   gidsetsize - The number of elements available in grouplist.
 *   grouplist  - The buffer used to return the group IDs.
 *
 * Returned Value:
 *   On success, the number of group IDs is returned.  If gidsetsize is zero,
 *   the total number of group IDs is returned without modifying grouplist.
 *   On failure, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int getgroups(int gidsetsize, gid_t grouplist[])
{
  if (gidsetsize < 0)
    {
      set_errno(EINVAL);
      return -1;
    }

  /* If gidsetsize is zero, return the number of group IDs without touching
   * grouplist.
   */

  if (gidsetsize == 0)
    {
      return 1;
    }

  if (grouplist == NULL)
    {
      set_errno(EFAULT);
      return -1;
    }

  /* NuttX has no notion of supplementary group IDs.  Report the single
   * effective group ID of the calling process.
   */

  grouplist[0] = getegid();
  return 1;
}
