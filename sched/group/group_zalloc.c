/****************************************************************************
 * sched/group/group_zalloc.c
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

#include <sys/types.h>

#include <string.h>

#include "group/group.h"

#ifdef CONFIG_MM_KERNEL_HEAP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_zalloc
 *
 * Description:
 *   Allocate memory and clear appropriate for the group type.  If the
 *   memory is part of a privileged, then it should be allocated so
 *   that it is only accessed by privileged code;  Otherwise, it must be
 *   allocated so that it accessible by unprivileged code.
 *
 ****************************************************************************/

FAR void *group_zalloc(FAR struct task_group_s *group, size_t nbytes)
{
  FAR void *mem = group_malloc(group, nbytes);
  if (mem)
    {
      memset(mem, 0, nbytes);
    }

  return mem;
}

#endif /* CONFIG_MM_KERNEL_HEAP */
