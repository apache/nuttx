/****************************************************************************
 * sched/mqueue/mq_initialize.c
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

#include <stdint.h>
#include <nuttx/kmalloc.h>
#include <nuttx/trace.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_msgfree is a list of messages that are available for general
 * use.  The number of messages in this list is a system configuration
 * item.
 */

struct list_node g_msgfree = LIST_INITIAL_VALUE(g_msgfree);

/* The g_msgfreeInt is a list of messages that are reserved for use by
 * interrupt handlers.
 */

struct list_node g_msgfreeirq = LIST_INITIAL_VALUE(g_msgfreeirq);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_msgblockalloc
 *
 * Description:
 *   Allocate a block of messages and place them on the free list.
 *
 * Input Parameters:
 *  queue
 *
 ****************************************************************************/

static void
mq_msgblockalloc(FAR struct list_node *list, uint16_t nmsgs,
                 uint8_t alloc_type)
{
  FAR struct mqueue_msg_s *mqmsgblock;

  /* The list must be loaded at initialization time to hold the
   * configured number of messages.
   */

  mqmsgblock = (FAR struct mqueue_msg_s *)
    kmm_malloc(sizeof(struct mqueue_msg_s) * nmsgs);

  if (mqmsgblock)
    {
      int i;
      for (i = 0; i < nmsgs; i++)
        {
          mqmsgblock->type = alloc_type;
          list_add_tail(list, &mqmsgblock->node);
          mqmsgblock++;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_initialize
 *
 * Description:
 *   This function initializes the message system.  This function must
 *   be called early in the initialization sequence before any of the
 *   other message interfaces execute.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_initialize(void)
{
  sched_trace_begin();

  /* Allocate a block of messages for general use */

  mq_msgblockalloc(&g_msgfree, CONFIG_PREALLOC_MQ_MSGS,
                   MQ_ALLOC_FIXED);

  /* Allocate a block of messages for use exclusively by
   * interrupt handlers
   */

  mq_msgblockalloc(&g_msgfreeirq, CONFIG_PREALLOC_MQ_IRQ_MSGS,
                   MQ_ALLOC_IRQ);

  sched_trace_end();
}
