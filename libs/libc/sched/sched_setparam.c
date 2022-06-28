/****************************************************************************
 * libs/libc/sched/sched_setparam.c
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

#include <errno.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_setparam
 *
 * Description:
 *   This function sets the priority of a specified task.  This function is
 *   a simply wrapper around nxsched_set_param() that sets the errno value in
 *   the event of an error.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to  after all other tasks
 *   with the same priority.
 *
 * Input Parameters:
 *   pid - the task ID of the task to reprioritize.  If pid is zero, the
 *      priority of the calling task is changed.
 *   param - A structure whose member sched_priority is the integer priority.
 *      The range of valid priority numbers is from SCHED_PRIORITY_MIN
 *      through SCHED_PRIORITY_MAX.
 *
 * Returned Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1 (ERROR) is
 *   returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make sense for the
 *         current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setparam(pid_t pid, FAR const struct sched_param *param)
{
  int ret = nxsched_set_param(pid, param);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
