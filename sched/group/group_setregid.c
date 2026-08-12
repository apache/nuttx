/****************************************************************************
 * sched/group/group_setregid.c
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
#include <assert.h>
#include <errno.h>

#include <sched/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setregid
 *
 * Description:
 *   The setregid() function sets the real group ID and/or the effective
 *   group ID of the calling process.
 *
 * Input Parameters:
 *   rgid - Real group identity to set.  The special value (gid_t)-1
 *          indicates that the real group ID should not be changed.
 *   egid - Effective group identity to set.  The special value (gid_t)-1
 *          indicates that the effective group ID should not be changed.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int setregid(gid_t rgid, gid_t egid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
  gid_t old_rgid;
  gid_t old_egid;
  gid_t old_sgid;

  if (rgid == (gid_t)-1 && egid == (gid_t)-1)
    {
      return OK;
    }

  rtcb   = this_task();
  rgroup = rtcb->group;

  DEBUGASSERT(rgroup != NULL);

  old_rgid = rgroup->tg_gid;
  old_egid = rgroup->tg_egid;
  old_sgid = rgroup->tg_sgid;

  if (old_egid == 0)
    {
      /* Super-user: may set any combination of real and effective IDs. */

      if (rgid != (gid_t)-1)
        {
          rgroup->tg_gid = rgid;

          if (egid == (gid_t)-1)
            {
              rgroup->tg_egid = rgid;
              rgroup->tg_sgid = rgid;
            }
        }

      if (egid != (gid_t)-1)
        {
          rgroup->tg_egid = egid;
          rgroup->tg_sgid = egid;
        }

      return OK;
    }

  /* Non-super-user */

  if (rgid != (gid_t)-1 &&
      rgid != old_egid && rgid != old_sgid)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (egid != (gid_t)-1 &&
      egid != old_egid && egid != old_sgid && egid != old_rgid)
    {
      set_errno(EPERM);
      return ERROR;
    }

  if (rgid != (gid_t)-1)
    {
      rgroup->tg_gid = rgid;
    }

  if (egid != (gid_t)-1)
    {
      rgroup->tg_egid = egid;
    }

  /* If the real group ID is being set, or the effective group ID is being
   * changed to a value not equal to the real group ID, update the saved
   * set-group-ID to the new effective group ID.
   */

  if ((rgid != (gid_t)-1 && rgroup->tg_gid != old_rgid) ||
      (egid != (gid_t)-1 && rgroup->tg_egid != old_rgid))
    {
      rgroup->tg_sgid = rgroup->tg_egid;
    }

  return OK;
}
