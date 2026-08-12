/****************************************************************************
 * libs/libc/unistd/lib_setresgid.c
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
 * Name: setresgid
 *
 * Description:
 *   The setresgid() function sets the real, effective, and saved
 *   set-group-ID of the calling process.  Stub when
 *   CONFIG_SCHED_USER_IDENTITY is disabled: only root (0) or unchanged
 *   ((gid_t)-1) values are accepted.
 *
 * Input Parameters:
 *   rgid - Real group ID, or (gid_t)-1 to leave unchanged.
 *   egid - Effective group ID, or (gid_t)-1 to leave unchanged.
 *   sgid - Saved set-group-ID, or (gid_t)-1 to leave unchanged.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setresgid(gid_t rgid, gid_t egid, gid_t sgid)
{
  /* NuttX only supports the group identity 'root' with a gid value of 0. */

  if ((rgid == (gid_t)-1 || rgid == 0) &&
      (egid == (gid_t)-1 || egid == 0) &&
      (sgid == (gid_t)-1 || sgid == 0))
    {
      return 0;
    }

  set_errno(EINVAL);
  return ERROR;
}
