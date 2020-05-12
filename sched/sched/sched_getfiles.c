/****************************************************************************
 * sched/sched/sched_getfiles.c
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
#include <sched.h>
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_files
 *
 * Description:
 *   Return a pointer to the file list for this thread
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A pointer to the errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

FAR struct filelist *nxsched_get_files(void)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;

  /* The group may be NULL under certain conditions.  For example, if
   * debug output is attempted from the IDLE thead before the group has
   * been allocated.  I have only seen this case when memory management
   * debug is enabled.
   */

  if (group)
    {
      return &group->tg_filelist;
    }

  /* Higher level logic must handle the NULL gracefully */

  return NULL;
}
