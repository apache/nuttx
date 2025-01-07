/****************************************************************************
 * sched/group/group_join.c
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

#include <nuttx/config.h>

#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include "sched/sched.h"
#include "group/group.h"
#include "environ/environ.h"

#ifndef CONFIG_DISABLE_PTHREAD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_bind
 *
 * Description:
 *   A thread joins the group when it is created.
 *
 * Input Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the head of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

void group_bind(FAR struct pthread_tcb_s *tcb)
{
  FAR struct tcb_s *ptcb = this_task();

  DEBUGASSERT(ptcb && tcb && ptcb->group && !tcb->cmn.group);

  /* Copy the group reference from the parent to the child */

  tcb->cmn.group = ptcb->group;
}

/****************************************************************************
 * Name: group_join
 *
 * Description:
 *   A thread joins the group when it is created.
 *
 * Input Parameters:
 *   tcb - The TCB of the new "child" task that need to join the group.
 *
 * Assumptions:
 * - The parent task from which the group will be inherited is the task at
 *   the head of the ready to run list.
 * - Called during thread creation in a safe context.  No special precautions
 *   are required here.
 *
 ****************************************************************************/

void group_join(FAR struct pthread_tcb_s *tcb)
{
  FAR struct task_group_s *group;
  irqstate_t flags;

  DEBUGASSERT(tcb && tcb->cmn.group);

  /* Get the group from the TCB */

  group = tcb->cmn.group;

  /* Add the member to the group */

  flags = spin_lock_irqsave(&group->tg_lock);
  sq_addfirst(&tcb->cmn.member, &group->tg_members);
  spin_unlock_irqrestore(&group->tg_lock, flags);
}

#endif /* !CONFIG_DISABLE_PTHREAD */
