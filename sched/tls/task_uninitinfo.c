/****************************************************************************
 * sched/tls/task_uninitinfo.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/lib/lib.h>

#include "tls.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_uninit_info
 *
 * Description:
 *   Uninitilize and free task_info_s structure.
 *
 * Input Parameters:
 *   - group: The group of new task
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void task_uninit_info(FAR struct task_group_s *group)
{
  FAR struct task_info_s *info = group->tg_info;

#ifdef CONFIG_FILE_STREAM
  /* Free resource held by the stream list */

  lib_stream_release(group);
#endif /* CONFIG_FILE_STREAM */

  nxmutex_destroy(&info->ta_lock);
  group_free(group, info);
}
