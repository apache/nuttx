/****************************************************************************
 *  sched/mqueue/mq_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <queue.h>
#include <nuttx/kmalloc.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/* This is a container for a list of message queue descriptors. */

struct mq_des_block_s
{
  sq_entry_t    queue;
  struct mq_des mqdes[NUM_MSG_DESCRIPTORS];
};

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

/* The g_desfree data structure is a list of message descriptors available
 * to the operating system for general use. The number of messages in the
 * pool is a constant.
 */

sq_queue_t  g_desfree;

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

/* g_desalloc is a list of allocated block of message queue descriptors. */

static sq_queue_t g_desalloc;

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
 *   This function initializes the messasge system.  This function must
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
  sq_init(&g_desalloc);

  /* Allocate a block of messages for general use */

  g_msgalloc =
    mq_msgblockalloc(&g_msgfree, CONFIG_PREALLOC_MQ_MSGS,
                     MQ_ALLOC_FIXED);

  /* Allocate a block of messages for use exclusively by
   * interrupt handlers
   */

  g_msgfreeirqalloc =
    mq_msgblockalloc(&g_msgfreeirq, NUM_INTERRUPT_MSGS,
                     MQ_ALLOC_IRQ);

  /* Allocate a block of message queue descriptors */

  nxmq_alloc_desblock();
}

/****************************************************************************
 * Name: nxmq_alloc_desblock
 *
 * Description:
 *   Allocate a block of message descriptors and place them on the free
 *   list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxmq_alloc_desblock(void)
{
  FAR struct mq_des_block_s *mqdesblock;

  /* Allocate a block of message descriptors */

  mqdesblock = (FAR struct mq_des_block_s *)kmm_malloc(sizeof(struct mq_des_block_s));
  if (mqdesblock)
    {
      int i;

      /* Add the block to the list of allocated blocks (in case
       * we ever need to reclaim the memory.
       */

      sq_addlast((FAR sq_entry_t *)&mqdesblock->queue, &g_desalloc);

      /* Then add each message queue descriptor to the free list */

      for (i = 0; i < NUM_MSG_DESCRIPTORS; i++)
        {
          sq_addlast((FAR sq_entry_t *)&mqdesblock->mqdes[i], &g_desfree);
        }
    }
}

