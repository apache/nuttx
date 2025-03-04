/****************************************************************************
 * sched/mqueue/mq_initialize.c
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

#include <stdint.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/trace.h>

#include "mqueue/mqueue.h"
#include "mqueue/msg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MQUEUE
#  define MQ_BLOCK_SIZE \
    ALIGN_UP(MQ_MSG_SIZE(MQ_MAX_BYTES), sizeof(void *))
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct msgpool_s
{
#ifndef CONFIG_DISABLE_MQUEUE
  uint8_t mqueue[MQ_BLOCK_SIZE *
                 (CONFIG_PREALLOC_MQ_MSGS +
                  CONFIG_PREALLOC_MQ_IRQ_MSGS)];
#endif
#ifndef CONFIG_DISABLE_MQUEUE_SYSV
  struct msgbuf_s msgbuf[CONFIG_PREALLOC_MQ_MSGS];
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MQUEUE

/* The g_msgfree is a list of messages that are available for general
 * use.  The number of messages in this list is a system configuration
 * item.
 */

struct list_node g_msgfree;

/* The g_msgfreeInt is a list of messages that are reserved for use by
 * interrupt handlers.
 */

struct list_node g_msgfreeirq;

spinlock_t g_msgfreelock = SP_UNLOCKED;

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is a pool of pre-allocated message queue buffers */

static struct msgpool_s g_msgpool;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_msgblockinit
 *
 * Description:
 *   Initialize a block of messages and place them on the free list.
 *
 * Input Parameters:
 *  queue
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MQUEUE
static FAR void * mq_msgblockinit(FAR struct list_node *list,
                                  FAR uint8_t *block,
                                  uint16_t nmsgs, uint8_t alloc_type)
{
  FAR struct mqueue_msg_s *mqmsgblock;
  int i;

  for (i = 0; i < nmsgs; i++)
    {
      mqmsgblock = (FAR struct mqueue_msg_s *)block;

      mqmsgblock->type = alloc_type;
      list_add_tail(list, &mqmsgblock->node);
      block += MQ_BLOCK_SIZE;
    }

  return block;
}
#endif

/****************************************************************************
 * Name: sysv_msgblockinit
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MQUEUE_SYSV
static FAR void *sysv_msgblockinit(FAR struct list_node *list,
                                   FAR struct msgbuf_s *msg, uint16_t nmsgs)
{
  int i;
  for (i = 0; i < nmsgs; i++)
    {
      list_add_tail(list, &msg->node);
      msg++;
    }

  return msg;
}
#endif

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
  FAR void *msg = &g_msgpool;

  sched_trace_begin();

  /* Initialize a block of messages for general use */

#ifndef CONFIG_DISABLE_MQUEUE
  list_initialize(&g_msgfree);

  msg = mq_msgblockinit(&g_msgfree, msg, CONFIG_PREALLOC_MQ_MSGS,
                         MQ_ALLOC_FIXED);

  /* Initialize a block of messages for use exclusively by
   * interrupt handlers
   */

  list_initialize(&g_msgfreeirq);

  msg = mq_msgblockinit(&g_msgfreeirq, msg, CONFIG_PREALLOC_MQ_IRQ_MSGS,
                         MQ_ALLOC_IRQ);
#endif

#ifndef CONFIG_DISABLE_MQUEUE_SYSV
  list_initialize(&g_msgfreelist);

  msg = sysv_msgblockinit(&g_msgfreelist, msg, CONFIG_PREALLOC_MQ_MSGS);
#endif

  sched_trace_end();
}
