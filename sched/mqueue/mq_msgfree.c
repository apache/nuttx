/****************************************************************************
 * sched/mqueue/mq_msgfree.c
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

#include <assert.h>
#include <queue.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_free_msg
 *
 * Description:
 *   The nxmq_free_msg function will return a message to the free pool of
 *   messages if it was a pre-allocated message. If the message was
 *   allocated dynamically it will be deallocated.
 *
 * Input Parameters:
 *   mqmsg - message to free
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_free_msg(FAR struct mqueue_msg_s *mqmsg)
{
  /* If this is a generally available pre-allocated message,
   * then just put it back in the free list.
   */

  if (mqmsg->type == MQ_ALLOC_FIXED)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      sq_addlast((FAR sq_entry_t *)mqmsg, &g_msgfree);
    }

  /* If this is a message pre-allocated for interrupts,
   * then put it back in the correct  free list.
   */

  else if (mqmsg->type == MQ_ALLOC_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      sq_addlast((FAR sq_entry_t *)mqmsg, &g_msgfreeirq);
    }

  /* Otherwise, deallocate it.  Note:  interrupt handlers
   * will never deallocate messages because they will not
   * received them.
   */

  else if (mqmsg->type == MQ_ALLOC_DYN)
    {
      kmm_free(mqmsg);
    }
  else
    {
      DEBUGPANIC();
    }
}
