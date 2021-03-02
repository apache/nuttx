/****************************************************************************
 * libs/libc/unistd/lib_setregid.c
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
 * Name: setregid
 *
 * Description:
 *   The setregid() function sets the real group ID and/or the effective
 *   group ID of the calling task group to rgid and/or egid.
 *
 * Input Parameters:
 *   rgid - Real group identity to set.  The special value (gid_t)-1
 *          indicates that the real group ID should not be changed.
 *   rgid - Effective group identity to set.  The special value (gid_t)-1
 *          indicates that the effective group ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setregid(gid_t rgid, gid_t egid)
{
  int ret = OK;

  if (rgid != (gid_t)-1)
    {
      /* Set the real group ID.  CAREFUL:  This exploits non-standard
       * behavior of setgid():  setgid() should set the real, effective, and
       * saved group ID.  Here we depend on it setting only the real group
       * ID.
       */

      ret = setgid(rgid);
    }

  if (ret >= 0 && egid != (gid_t)-1)
    {
      /* Set the effective group ID */

      ret = setegid(egid);
    }

  return ret;
}
