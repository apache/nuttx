/****************************************************************************
 * sched/group/group_umask.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <sys/stat.h>

#include <nuttx/sched.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: umask
 *
 * Description:
 *   Set the file mode creation mask of the calling process.  Only the file
 *   permission bits of the mask are used.
 *
 * Input Parameters:
 *   mask - The new file mode creation mask.
 *
 * Returned Value:
 *   The previous value of the mask.
 *
 ****************************************************************************/

mode_t umask(mode_t mask)
{
  FAR struct task_group_s *group = nxsched_self()->group;
  mode_t prev;

  prev = group->tg_umask;
  group->tg_umask = mask & (S_IRWXU | S_IRWXG | S_IRWXO);
  return prev;
}

/****************************************************************************
 * Name: getumask
 *
 * Description:
 *   Return the file mode creation mask of the calling process.
 *
 ****************************************************************************/

mode_t getumask(void)
{
  FAR struct task_group_s *group = nxsched_self()->group;

  return group != NULL ? group->tg_umask : 0;
}
