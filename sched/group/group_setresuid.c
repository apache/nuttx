/****************************************************************************
 * sched/group/group_setresuid.c
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
 * Name: setresuid
 *
 * Description:
 *   setresuid() sets the real, effective, and saved set-user-IDs of the
 *   calling process.  The value (uid_t)-1 for any argument leaves that
 *   ID unchanged.
 *
 ****************************************************************************/

int setresuid(uid_t ruid, uid_t euid, uid_t suid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;
  uid_t old_ruid;
  uid_t old_euid;
  uid_t old_suid;
  uid_t new_ruid;
  uid_t new_euid;
  uid_t new_suid;

  rtcb   = this_task();
  rgroup = rtcb->group;
  DEBUGASSERT(rgroup != NULL);

  old_ruid = rgroup->tg_uid;
  old_euid = rgroup->tg_euid;
  old_suid = rgroup->tg_suid;

  new_ruid = (ruid == (uid_t)-1) ? old_ruid : ruid;
  new_euid = (euid == (uid_t)-1) ? old_euid : euid;
  new_suid = (suid == (uid_t)-1) ? old_suid : suid;

  if (old_euid != 0)
    {
      /* Unprivileged: each new ID must be one of the current r/e/s UIDs. */

      if ((new_ruid != old_ruid && new_ruid != old_euid &&
           new_ruid != old_suid) ||
          (new_euid != old_ruid && new_euid != old_euid &&
           new_euid != old_suid) ||
          (new_suid != old_ruid && new_suid != old_euid &&
           new_suid != old_suid))
        {
          set_errno(EPERM);
          return ERROR;
        }
    }

  rgroup->tg_uid  = new_ruid;
  rgroup->tg_euid = new_euid;
  rgroup->tg_suid = new_suid;
  return OK;
}
