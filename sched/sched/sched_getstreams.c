/****************************************************************************
 * sched/sched/sched_getstreams.c
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

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_streams
 *
 * Description:
 *   Return a pointer to the streams list for this thread
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

FAR struct streamlist *nxsched_get_streams(void)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group = rtcb->group;

  DEBUGASSERT(group);

#ifdef CONFIG_MM_KERNEL_HEAP
  return group->tg_streamlist;
#else
  return &group->tg_streamlist;
#endif
}

#endif /* CONFIG_FILE_STREAM */
