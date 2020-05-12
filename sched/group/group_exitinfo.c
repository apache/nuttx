/****************************************************************************
 * sched/group/group_exitinfo.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/binfmt/binfmt.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef CONFIG_BINFMT_LOADABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_exitinfo
 *
 * Description:
 *   This function may be called to when a task is loaded into memory.  It
 *   will setup the to automatically unload the module when the task exits.
 *
 * Input Parameters:
 *   pid     - The task ID of the newly loaded task
 *   bininfo - This structure allocated with kmm_malloc().  This memory
 *             persists until the task exits and will be used unloads
 *             the module from memory.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int group_exitinfo(pid_t pid, FAR struct binary_s *bininfo)
{
  FAR struct tcb_s *tcb;
  FAR struct task_group_s *group;
  irqstate_t flags;

  DEBUGASSERT(bininfo != NULL);
  flags = spin_lock_irqsave();

  /* Get the TCB associated with the PID */

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL)
    {
      spin_unlock_irqrestore(flags);
      return -ESRCH;
    }

  /* Get the group that this task belongs to */

  group = tcb->group;
  DEBUGASSERT(group != NULL && group->tg_bininfo == NULL);

  /* Save the binary info for use when the task exits */

  group->tg_bininfo = bininfo;

  spin_unlock_irqrestore(flags);
  return OK;
}

#endif /* CONFIG_BINFMT_LOADABLE */
