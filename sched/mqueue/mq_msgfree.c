/****************************************************************************
 *  sched/mqueue/mq_msgfree.c
 *
 *   Copyright (C) 2007, 2013, 2016 Gregory Nutt. All rights reserved.
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
  irqstate_t flags;

  /* If this is a generally available pre-allocated message,
   * then just put it back in the free list.
   */

  if (mqmsg->type == MQ_ALLOC_FIXED)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)mqmsg, &g_msgfree);
      leave_critical_section(flags);
    }

  /* If this is a message pre-allocated for interrupts,
   * then put it back in the correct  free list.
   */

  else if (mqmsg->type == MQ_ALLOC_IRQ)
    {
      /* Make sure we avoid concurrent access to the free
       * list from interrupt handlers.
       */

      flags = enter_critical_section();
      sq_addlast((FAR sq_entry_t *)mqmsg, &g_msgfreeirq);
      leave_critical_section(flags);
    }

  /* Otherwise, deallocate it.  Note:  interrupt handlers
   * will never deallocate messages because they will not
   * received them.
   */

  else if (mqmsg->type == MQ_ALLOC_DYN)
    {
      sched_kfree(mqmsg);
    }
  else
    {
      DEBUGPANIC();
    }
}
