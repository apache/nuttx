/****************************************************************************
 *  sched/mqueue/mq_descreate.c
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

#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <mqueue.h>
#include <sched.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/mqueue.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_alloc_des
 *
 * Description:
 *   Allocate a message queue descriptor.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Reference to the allocated mq descriptor.
 *
 ****************************************************************************/

static mqd_t nxmq_alloc_des(void)
{
  mqd_t mqdes;

  /* Try to get the message descriptorfrom the free list */

  mqdes = (mqd_t)sq_remfirst(&g_desfree);

  /* Check if we got one. */

  if (!mqdes)
    {
      /* Add another block of message descriptors to the list */

      nxmq_alloc_desblock();

      /* And try again */

      mqdes = (mqd_t)sq_remfirst(&g_desfree);
    }

  return mqdes;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_create_des
 *
 * Description:
 *   Create a message queue descriptor for the specified TCB
 *
 * Input Parameters:
 *   mtcb   - task that needs the descriptor.
 *   msgq   - Named message queue containing the message
 *   oflags - access rights for the descriptor
 *
 * Returned Value:
 *   On success, the message queue descriptor is returned.  NULL is returned
 *   on a failure to allocate.
 *
 ****************************************************************************/

mqd_t nxmq_create_des(FAR struct tcb_s *mtcb,
                      FAR struct mqueue_inode_s *msgq, int oflags)
{
  FAR struct task_group_s *group;
  mqd_t mqdes;

  /* A NULL TCB pointer means to use the TCB of the currently executing
   * task/thread.
   */

  if (!mtcb)
    {
      mtcb = nxsched_self();
    }

  group = mtcb->group;
  DEBUGASSERT(group);

  /* Create a message queue descriptor for the TCB */

  mqdes = nxmq_alloc_des();
  if (mqdes)
    {
      /* Initialize the message queue descriptor */

      memset(mqdes, 0, sizeof(struct mq_des));
      mqdes->msgq   = msgq;
      mqdes->oflags = oflags;

      /* And add it to the specified task's TCB */

      sq_addlast((FAR sq_entry_t *)mqdes, &group->tg_msgdesq);
    }

  return mqdes;
}
