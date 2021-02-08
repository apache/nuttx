/****************************************************************************
 * sched/group/group_free.c
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

#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef CONFIG_MM_KERNEL_HEAP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_free
 *
 * Description:
 *   Free memory appropriate previously allocated via group_malloc() using
 *   the appropriate memory manager.
 *
 ****************************************************************************/

void group_free(FAR struct task_group_s *group, FAR void *mem)
{
  /* A NULL group pointer means the current group */

  if (!group)
    {
      FAR struct tcb_s *tcb = this_task();
      DEBUGASSERT(tcb && tcb->group);
      group = tcb->group;
    }

  /* Check the group is privileged */

  if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
    {
      /* It is a privileged group... use the kernel mode memory allocator */

      kmm_free(mem);
    }
  else
    {
      /* This is an unprivileged group... use the user mode memory
       * allocator.
       */

      kumm_free(mem);
    }
}

#endif /* CONFIG_MM_KERNEL_HEAP */
