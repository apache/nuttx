/****************************************************************************
 * libs/libc/unistd/lib_setresuid.c
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
 * Name: setresuid
 *
 * Description:
 *   The setresuid() function sets the real, effective, and saved set-user-ID
 *   of the calling process.  Stub when CONFIG_SCHED_USER_IDENTITY is
 *   disabled: only root (0) or unchanged ((uid_t)-1) values are accepted.
 *
 * Input Parameters:
 *   ruid - Real user ID, or (uid_t)-1 to leave unchanged.
 *   euid - Effective user ID, or (uid_t)-1 to leave unchanged.
 *   suid - Saved set-user-ID, or (uid_t)-1 to leave unchanged.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setresuid(uid_t ruid, uid_t euid, uid_t suid)
{
  /* NuttX only supports the user identity 'root' with a uid value of 0. */

  if ((ruid == (uid_t)-1 || ruid == 0) &&
      (euid == (uid_t)-1 || euid == 0) &&
      (suid == (uid_t)-1 || suid == 0))
    {
      return 0;
    }

  set_errno(EINVAL);
  return ERROR;
}
