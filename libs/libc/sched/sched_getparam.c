/****************************************************************************
 * libs/libc/sched/sched_getparam.c
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
 * Name: sched_getparam
 *
 * Description:
 *   This function gets the scheduling priority of the task specified by
 *   pid.  This function is a simply wrapper around nxsched_get_param() that
 *   sets the errno value in the event of an error.
 *
 * Input Parameters:
 *   pid - the task ID of the task.  If pid is zero, the priority
 *     of the calling task is returned.
 *   param - A structure whose member sched_priority is the integer
 *     priority.  The task's priority is copied to the sched_priority
 *     element of this structure.
 *
 * Returned Value:
 *   0 (OK) if successful, otherwise -1 (ERROR) with the errno value set
 *   to indicate the nature of the problem.
 *
 *   This function can fail if param is null (EINVAL) or if pid does
 *   not correspond to any task (ESRCH).
 *
 ****************************************************************************/

int sched_getparam(pid_t pid, FAR struct sched_param *param)
{
  int ret = nxsched_get_param(pid, param);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
