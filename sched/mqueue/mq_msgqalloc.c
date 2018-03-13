/****************************************************************************
 *  sched/mqueue/mq_msgqalloc.c
 *
 *   Copyright (C) 2014, 2017 Gregory Nutt. All rights reserved.
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

#include <mqueue.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/mqueue.h>

#include "sched/sched.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmq_alloc_msgq
 *
 * Description:
 *   This function implements a part of the POSIX message queue open logic.
 *   It allocates and initializes a struct mqueue_inode_s structure.
 *
 * Input Parameters:
 *   mode   - mode_t value is ignored
 *   attr   - The mq_maxmsg attribute is used at the time that the message
 *            queue is created to determine the maximum number of
 *            messages that may be placed in the message queue.
 *
 * Returned Value:
 *   The allocated and initialized message queue structure or NULL in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct mqueue_inode_s *nxmq_alloc_msgq(mode_t mode,
                                           FAR struct mq_attr *attr)
{
  FAR struct mqueue_inode_s *msgq;

  /* Check if the caller is attempting to allocate a message for messages
   * larger than the configured maximum message size.
   */

  DEBUGASSERT(!attr || attr->mq_msgsize <= MQ_MAX_BYTES);
  if (attr && attr->mq_msgsize > MQ_MAX_BYTES)
    {
      return NULL;
    }

  /* Allocate memory for the new message queue. */

  msgq = (FAR struct mqueue_inode_s *)
    kmm_zalloc(sizeof(struct mqueue_inode_s));

  if (msgq)
    {
      /* Initialize the new named message queue */

      sq_init(&msgq->msglist);
      if (attr)
        {
          msgq->maxmsgs    = (int16_t)attr->mq_maxmsg;
          msgq->maxmsgsize = (int16_t)attr->mq_msgsize;
        }
      else
        {
          msgq->maxmsgs    = MQ_MAX_MSGS;
          msgq->maxmsgsize = MQ_MAX_BYTES;
        }

#ifndef CONFIG_DISABLE_SIGNALS
      msgq->ntpid = INVALID_PROCESS_ID;
#endif
    }

  return msgq;
}
