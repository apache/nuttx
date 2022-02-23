/****************************************************************************
 * sched/group/group_realloc.c
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
 * Name: group_realloc
 *
 * Description:
 *   Re-allocate memory appropriate for the group type.  If the memory is
 *   part of a privileged group, then it should be allocated so that it
 *   is only accessible by privileged code;  Otherwise, it is a user mode
 *   group and must be allocated so that it accessible by unprivileged
 *   code.
 *
 ****************************************************************************/

FAR void *group_realloc(FAR struct task_group_s *group, FAR void *oldmem,
                        size_t newsize)
{
  /* A NULL group pointer means the current group */

  if (group == NULL)
    {
      FAR struct tcb_s *tcb = this_task();
      DEBUGASSERT(tcb && tcb->group);
      group = tcb->group;
    }

  /* Check the group type */

  if ((group->tg_flags & GROUP_FLAG_PRIVILEGED) != 0)
    {
      /* It is a privileged group... use the kernel mode memory allocator */

      return kmm_realloc(oldmem, newsize);
    }
  else
    {
      /* This is an unprivileged group... use the user mode memory
       * allocator.
       */

      return kumm_realloc(oldmem, newsize);
    }
}

#endif /* CONFIG_MM_KERNEL_HEAP */
