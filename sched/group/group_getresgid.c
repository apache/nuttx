/****************************************************************************
 * sched/group/group_getresgid.c
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

#include <sched/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getresgid
 *
 * Description:
 *   The getresgid() function gets the real, effective, and saved set-group
 *   IDs of the calling process.
 *
 * Input Parameters:
 *   rgid - Location to return the real group ID, or NULL.
 *   egid - Location to return the effective group ID, or NULL.
 *   sgid - Location to return the saved set-group ID, or NULL.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   appropriately.
 *
 ****************************************************************************/

int getresgid(FAR gid_t *rgid, FAR gid_t *egid, FAR gid_t *sgid)
{
  FAR struct tcb_s *rtcb          = this_task();
  FAR struct task_group_s *rgroup = rtcb->group;

  DEBUGASSERT(rgroup != NULL);

  if (rgid != NULL)
    {
      *rgid = rgroup->tg_gid;
    }

  if (egid != NULL)
    {
      *egid = rgroup->tg_egid;
    }

  if (sgid != NULL)
    {
      *sgid = rgroup->tg_sgid;
    }

  return OK;
}
