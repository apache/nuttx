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
#include <queue.h>
#include <nuttx/kmalloc.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The g_msgfree is a list of messages that are available for general
 * use.  The number of messages in this list is a system configuration
 * item.
 */

sq_queue_t  g_msgfree;

/* The g_msgfreeInt is a list of messages that are reserved for use by
 * interrupt handlers.
 */

sq_queue_t  g_msgfreeirq;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* g_msgalloc is a pointer to the start of the allocated block of
 * messages.
 */

static struct mqueue_msg_s  *g_msgalloc;

/* g_msgfreeirqalloc is a pointer to the start of the allocated block of
 * messages.
 */

static struct mqueue_msg_s  *g_msgfreeirqalloc;

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

static struct mqueue_msg_s *
mq_msgblockalloc(FAR sq_queue_t *queue, uint16_t nmsgs,
                 uint8_t alloc_type)
{
  struct mqueue_msg_s *mqmsgblock;

  /* The g_msgfree must be loaded at initialization time to hold the
   * configured number of messages.
   */

  mqmsgblock = (FAR struct mqueue_msg_s *)
    kmm_malloc(sizeof(struct mqueue_msg_s) * nmsgs);

  if (mqmsgblock)
    {
      struct mqueue_msg_s *mqmsg = mqmsgblock;
      int      i;

      for (i = 0; i < nmsgs; i++)
        {
          mqmsg->type = alloc_type;
          sq_addlast((FAR sq_entry_t *)mqmsg++, queue);
        }
    }

  return mqmsgblock;
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
  /* Initialize the message free lists */

  sq_init(&g_msgfree);
  sq_init(&g_msgfreeirq);

  /* Allocate a block of messages for general use */

  g_msgalloc =
    mq_msgblockalloc(&g_msgfree, CONFIG_PREALLOC_MQ_MSGS,
                     MQ_ALLOC_FIXED);

  /* Allocate a block of messages for use exclusively by
   * interrupt handlers
   */

  g_msgfreeirqalloc =
    mq_msgblockalloc(&g_msgfreeirq, CONFIG_PREALLOC_MQ_IRQ_MSGS,
                     MQ_ALLOC_IRQ);
}
