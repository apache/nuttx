/****************************************************************************
 * sched/group/group_getegid.c
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
 * Name: getegid
 *
 * Description:
 *   The getegid() function will return the effective group ID of the calling
 *   task group.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The effective group ID of the calling task group.
 *
 ****************************************************************************/

gid_t getegid(void)
{
  FAR struct tcb_s *rtcb          = this_task();
  FAR struct task_group_s *rgroup = rtcb->group;

  /* Set the task group's group identity. */

  DEBUGASSERT(rgroup != NULL);
  return rgroup->tg_egid;
}
