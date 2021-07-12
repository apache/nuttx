/****************************************************************************
 * sched/group/group_exitinfo.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
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
  flags = spin_lock_irqsave(NULL);

  /* Get the TCB associated with the PID */

  tcb = nxsched_get_tcb(pid);
  if (tcb == NULL)
    {
      spin_unlock_irqrestore(NULL, flags);
      return -ESRCH;
    }

  /* Get the group that this task belongs to */

  group = tcb->group;
  DEBUGASSERT(group != NULL && group->tg_bininfo == NULL);

  /* Save the binary info for use when the task exits */

  group->tg_bininfo = bininfo;

  spin_unlock_irqrestore(NULL, flags);
  return OK;
}

#endif /* CONFIG_BINFMT_LOADABLE */
