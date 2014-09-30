/****************************************************************************
 *  sched/mqueue/mq_desclose.c
 *
 *   Copyright (C) 2007, 2009, 2013-2014 Gregory Nutt. All rights reserved.
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
#include <sched.h>
#include <assert.h>
#include <queue.h>

#include <nuttx/mqueue.h>

#include "mqueue/mqueue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_desfree
 *
 * Description:
 *   Deallocate a message queue descriptor but returning it to the free list
 *
 * Inputs:
 *   mqdes - message queue descriptor to free
 *
 ****************************************************************************/

#define mq_desfree(mqdes) sq_addlast((FAR sq_entry_t*)mqdes, &g_desfree)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mq_close
 *
 * Description:
 *   This function performs the portion of the mq_close operation related
 *   to freeing resource used by the message queue descriptor itself.
 *
 * Parameters:
 *   mqdes - Message queue descriptor.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 * - Called only from mq_close() with the scheduler locked.
 *
 ****************************************************************************/

void mq_desclose(mqd_t mqdes)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s*)sched_self();
  FAR struct task_group_s *group = rtcb->group;
  FAR struct mqueue_inode_s *msgq;

  DEBUGASSERT(mqdes && group);

  /* Remove the message descriptor from the current task's list of message
   * descriptors.
   */

  sq_rem((FAR sq_entry_t*)mqdes, &group->tg_msgdesq);

  /* Find the message queue associated with the message descriptor */

  msgq = mqdes->msgq;

  /* Check if the calling task has a notification attached to the message
   * queue via this mqdes.
   */

#ifndef CONFIG_DISABLE_SIGNALS
  if (msgq->ntmqdes == mqdes)
    {
      msgq->ntpid   = INVALID_PROCESS_ID;
      msgq->ntsigno = 0;
      msgq->ntvalue.sival_int = 0;
      msgq->ntmqdes = NULL;
    }
#endif

   /* Deallocate the message descriptor */

   mq_desfree(mqdes);
}

