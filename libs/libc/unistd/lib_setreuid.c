/****************************************************************************
 * libs/libc/unistd/lib_setreuid.c
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
 * Name: setreuid
 *
 * Description:
 *   The setreuid() function sets the real user ID and/or the effective user
 *   ID of the calling task group to ruid and/or euid.
 *
 * Input Parameters:
 *   ruid - Real user identity to set.  The special value (uid_t)-1
 *          indicates that the real user ID should not be changed.
 *   ruid - Effective user identity to set.  The special value (uid_t)-1
 *          indicates that the effective user ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setreuid(uid_t ruid, uid_t euid)
{
  int ret = OK;

  if (ruid != (uid_t)-1)
    {
      /* Set the real user ID.  CAREFUL:  This exploits non-standard behavior
       * of setuid():  setuid() should set the real, effective, and saved
       * user ID.  Here we depend on it setting only the real user ID.
       */

      ret = setuid(ruid);
    }

  if (ret >= 0 && euid != (uid_t)-1)
    {
      /* Set the effective user ID */

      ret = seteuid(euid);
    }

  return ret;
}
