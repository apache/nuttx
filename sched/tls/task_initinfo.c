/****************************************************************************
 * sched/tls/task_initinfo.c
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

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/lib/lib.h>

#include "tls.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init_info
 *
 * Description:
 *   Allocate and initialize task_info_s structure.
 *
 * Input Parameters:
 *   - group: The group of new task
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int task_init_info(FAR struct task_group_s *group)
{
  FAR struct task_info_s *info;

  /* Allocate task info for group */

  info = group_zalloc(group, sizeof(struct task_info_s));
  if (info == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize user space mutex */

  nxmutex_init(&info->ta_lock);
  group->tg_info = info;

#ifdef CONFIG_FILE_STREAM
  /* Initialize file streams for the task group */

  lib_stream_initialize(group);
#endif

  return OK;
}
