/****************************************************************************
 * sched/group/group_setegid.c
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
 * Name: setegid
 *
 * Description:
 *   The setegid() function sets the effective group ID of the calling
 *   process to gid, given appropriate privileges.
 *
 * Input Parameters:
 *   gid - Identity to set the various process's group ID attributes to.
 *
 * Returned Value:
 *   Zero if successful and -1 in case of failure, in which case errno is set
 *   to one of he following values:
 *
 *   EINVAL - The value of the uid argument is invalid and not supported by
 *            the implementation.
 *   EPERM  - The process does not have appropriate privileges and uid does
 *            not match the effective group ID or the saved set-group-ID.
 *
 ****************************************************************************/

int setegid(gid_t gid)
{
  FAR struct tcb_s *rtcb;
  FAR struct task_group_s *rgroup;

  /* Verify that the GID is in the valid range of 0 through INT16_MAX.
   * OpenGroup.org does not specify a GID_MAX or GID_MIN.  Instead we use a
   * priori knowledge that gid_t is type int16_t.
   */

  if ((uint16_t)gid > INT16_MAX)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Get the currently executing thread's task group. */

  rtcb   = this_task();
  rgroup = rtcb->group;

  /* Set the task group's group identity. */

  DEBUGASSERT(rgroup != NULL);
  rgroup->tg_egid = gid;
  return OK;
}
