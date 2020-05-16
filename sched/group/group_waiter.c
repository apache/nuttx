/****************************************************************************
 * sched/group/group_waiter.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "nuttx/sched.h"
#include "group/group.h"

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  group_add_waiter
 *
 * Description:
 *   Increment the number of instances of waitpid that are waiting for this
 *   group to exit.
 *
 * Assumptions:
 *   Caller has assured mutually exclusive access to the group.
 *
 ****************************************************************************/

void group_add_waiter(FAR struct task_group_s *group)
{
  group->tg_nwaiters++;
  DEBUGASSERT(group->tg_nwaiters > 0);
}

/****************************************************************************
 * Name:  group_add_waiter
 *
 * Description:
 *   Decrement the number of instances of waitpid that are waiting for this
 *   group to exit.  If the count goes to zero and deletion is pending, the
 *   call group_free to release the dangling resources.
 *
 * Assumptions:
 *   Caller has assured mutually exclusive access to the group.
 *
 ****************************************************************************/

void group_del_waiter(FAR struct task_group_s *group)
{
  DEBUGASSERT(group->tg_nwaiters > 0);
  group->tg_nwaiters--;
  if (group->tg_nwaiters == 0 && (group->tg_flags & GROUP_FLAG_DELETED) != 0)
    {
      /* Release the group container (all other resources have already been
       * freed).
       */

      kmm_free(group);
    }
}

#endif /* CONFIG_SCHED_WAITPID && !CONFIG_SCHED_HAVE_PARENT */
