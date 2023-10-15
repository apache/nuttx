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

#include "tls.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_init_stream
 *
 * Description:
 *   This function is called when a new task is allocated.  It initializes
 *   the streamlist instance that is stored in the task group.
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
static void task_init_stream(FAR struct streamlist *list)
{
  FAR struct file_struct *stream = list->sl_std;

  /* Initialize the list access mutex */

  nxmutex_init(&list->sl_lock);
  list->sl_head = NULL;
  list->sl_tail = NULL;

  /* Initialize stdin, stdout and stderr stream */

  stream[0].fs_fd = -1;
  nxrmutex_init(&stream[0].fs_lock);
  stream[1].fs_fd = -1;
  nxrmutex_init(&stream[1].fs_lock);
  stream[2].fs_fd = -1;
  nxrmutex_init(&stream[2].fs_lock);
}
#endif

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

#ifdef CONFIG_PTHREAD_ATFORK
  list_initialize(&info->ta_atfork);
#endif

#ifdef CONFIG_FILE_STREAM
  /* Initialize file streams for the task group */

  task_init_stream(&info->ta_streamlist);
#endif

  return OK;
}
