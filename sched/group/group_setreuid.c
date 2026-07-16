/****************************************************************************
 * sched/group/group_setreuid.c
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

#include <sys/types.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <sched/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setreuid
 *
 * Description:
 *   The setreuid() function sets the real user ID and/or the effective user
 *   ID of the calling process.
 *
 * Input Parameters:
 *   ruid - Real user identity to set.  The special value (uid_t)-1
 *          indicates that the real user ID should not be changed.
 *   euid - Effective user identity to set.  The special value (uid_t)-1
 *          indicates that the effective user ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setreuid(uid_t ruid, uid_t euid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
  uid_t old_ruid;
  uid_t old_euid;
  uid_t old_suid;

  if (ruid != (uid_t)-1 && (uint16_t)ruid > INT16_MAX)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (euid != (uid_t)-1 && (uint16_t)euid > INT16_MAX)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (ruid == (uid_t)-1 && euid == (uid_t)-1)
    {
      return OK;
    }

  rtcb   = this_task();
  rgroup = rtcb->group;

  DEBUGASSERT(rgroup != NULL);

  old_ruid = rgroup->tg_uid;
  old_euid = rgroup->tg_euid;
  old_suid = rgroup->tg_suid;

  if (old_euid == 0)
    {
      /* Super-user: may set any combination of real and effective IDs. */

      if (ruid != (uid_t)-1)
        {
          rgroup->tg_uid = ruid;

          if (euid == (uid_t)-1)
            {
              rgroup->tg_euid = ruid;
              rgroup->tg_suid = ruid;
            }
        }

      if (euid != (uid_t)-1)
        {
          rgroup->tg_euid = euid;
          rgroup->tg_suid = euid;
        }

      return OK;
    }

  /* Non-super-user */

  if (ruid != (uid_t)-1 &&
      ruid != old_euid && ruid != old_suid)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (euid != (uid_t)-1 &&
      euid != old_euid && euid != old_suid && euid != old_ruid)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (ruid != (uid_t)-1)
    {
      rgroup->tg_uid = ruid;
    }

  if (euid != (uid_t)-1)
    {
      rgroup->tg_euid = euid;
    }

  /* If the real user ID is being set, or the effective user ID is being
   * changed to a value not equal to the real user ID, update the saved
   * set-user-ID to the new effective user ID.
   */

  if ((ruid != (uid_t)-1 && rgroup->tg_uid != old_ruid) ||
      (euid != (uid_t)-1 && rgroup->tg_euid != old_ruid))
    {
      rgroup->tg_suid = rgroup->tg_euid;
    }

  return OK;
}
