/****************************************************************************
 * sched/group/group_free.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)

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

#endif  /* CONFIG_BUILD_PROTECTED || CONFIG_BUILD_KERNEL) && CONFIG_MM_KERNEL_HEAP */
