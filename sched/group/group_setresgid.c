/****************************************************************************
 * sched/group/group_setresgid.c
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
 * Name: setresgid
 *
 * Description:
 *   setresgid() sets the real, effective, and saved set-group-IDs of the
 *   calling process.  The value (gid_t)-1
 *   for any argument leaves that ID unchanged.
 *
 ****************************************************************************/

int setresgid(gid_t rgid, gid_t egid, gid_t sgid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
  gid_t old_rgid;
  gid_t old_egid;
  gid_t old_sgid;
  gid_t new_rgid;
  gid_t new_egid;
  gid_t new_sgid;

  rtcb   = this_task();
  rgroup = rtcb->group;
  DEBUGASSERT(rgroup != NULL);

  old_rgid = rgroup->tg_gid;
  old_egid = rgroup->tg_egid;
  old_sgid = rgroup->tg_sgid;

  new_rgid = (rgid == (gid_t)-1) ? old_rgid : rgid;
  new_egid = (egid == (gid_t)-1) ? old_egid : egid;
  new_sgid = (sgid == (gid_t)-1) ? old_sgid : sgid;

  /* Non-root euid may only select among current real/effective/saved. */

  if (rgroup->tg_euid != 0)
    {
      if ((new_rgid != old_rgid && new_rgid != old_egid &&
           new_rgid != old_sgid) ||
          (new_egid != old_rgid && new_egid != old_egid &&
           new_egid != old_sgid) ||
          (new_sgid != old_rgid && new_sgid != old_egid &&
           new_sgid != old_sgid))
        {
          set_errno(EPERM);
          return ERROR;
        }
    }

  rgroup->tg_gid  = new_rgid;
  rgroup->tg_egid = new_egid;
  rgroup->tg_sgid = new_sgid;
  return OK;
}
