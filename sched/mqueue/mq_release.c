/****************************************************************************
 *  sched/mqueue/mq_release.c
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

#include <string.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_release
 *
 * Description:
 *   This function is called when the final member of a task group exits.
 *   This function closes all of the message queues opened by members of
 *   the task group.
 *
 * Input Parameters:
 *   group - The task group that is terminating.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_release(FAR struct task_group_s *group)
{
  while (group->tg_msgdesq.head)
    {
      nxmq_close_group((mqd_t)group->tg_msgdesq.head, group);
    }
}
