/****************************************************************************
 * include/nuttx/mqueue.h
 *
 *   Copyright (C) 2007, 2009, 2011, 2014-2016 Gregory Nutt. All rights reserved.
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

#ifndef ___INCLUDE_NUTTX_MQUEUE_H
#define ___INCLUDE_NUTTX_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>
#include <queue.h>
#include <signal.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* This structure defines a message queue */

struct mq_des; /* forward reference */

struct mqueue_inode_s
{
  FAR struct inode *inode;    /* Containing inode */
  sq_queue_t msglist;         /* Prioritized message list */
  int16_t maxmsgs;            /* Maximum number of messages in the queue */
  int16_t nmsgs;              /* Number of message in the queue */
  int16_t nwaitnotfull;       /* Number tasks waiting for not full */
  int16_t nwaitnotempty;      /* Number tasks waiting for not empty */
#if CONFIG_MQ_MAXMSGSIZE < 256
  uint8_t maxmsgsize;         /* Max size of message in message queue */
#else
  uint16_t maxmsgsize;        /* Max size of message in message queue */
#endif
#ifndef CONFIG_DISABLE_SIGNALS
  FAR struct mq_des *ntmqdes; /* Notification: Owning mqdes (NULL if none) */
  pid_t ntpid;                /* Notification: Receiving Task's PID */
  struct sigevent ntevent;    /* Notification description */
#endif
};

/* This describes the message queue descriptor that is held in the
 * task's TCB
 */

struct mq_des
{
  FAR struct mq_des *flink;        /* Forward link to next message descriptor */
  FAR struct mqueue_inode_s *msgq; /* Pointer to associated message queue */
  int oflags;                      /* Flags set when message queue was opened */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct mq_attr;       /* Forward reference */
struct tcb_s;         /* Forward reference */
struct task_group_s;  /* Forward reference */

/****************************************************************************
 * Name: mq_msgqfree
 *
 * Description:
 *   This function deallocates an initialized message queue structure.
 *   First, it deallocates all of the queued messages in the message
 *   queue.  It is assumed that this message is fully unlinked and
 *   closed so that no thread will attempt access it while it is being
 *   deleted.
 *
 * Inputs:
 *   msgq - Named essage queue to be freed
 *
 * Return Value:
 *   None
 *
 ****************************************************************************/

void mq_msgqfree(FAR struct mqueue_inode_s *msgq);

/****************************************************************************
 * Name: mq_msgqalloc
 *
 * Description:
 *   This function implements a part of the POSIX message queue open logic.
 *   It allocates and initializes a structu mqueue_inode_s structure.
 *
 * Parameters:
 *   mode   - mode_t value is ignored
 *   attr   - The mq_maxmsg attribute is used at the time that the message
 *            queue is created to determine the maximum number of
 *             messages that may be placed in the message queue.
 *
 * Return Value:
 *   The allocated and initalized message queue structure or NULL in the
 *   event of a failure.
 *
 ****************************************************************************/

FAR struct mqueue_inode_s *mq_msgqalloc(mode_t mode,
                                        FAR struct mq_attr *attr);

/****************************************************************************
 * Name: mq_descreate
 *
 * Description:
 *   Create a message queue descriptor for the specified TCB
 *
 * Inputs:
 *   TCB - task that needs the descriptor.
 *   msgq - Named message queue containing the message
 *   oflags - access rights for the descriptor
 *
 * Return Value:
 *   On success, the message queue descriptor is returned.  NULL is returned
 *   on a failure to allocate.
 *
 ****************************************************************************/

mqd_t mq_descreate(FAR struct tcb_s *mtcb, FAR struct mqueue_inode_s *msgq,
                   int oflags);

/****************************************************************************
 * Name: mq_close_group
 *
 * Description:
 *   This function is used to indicate that all threads in the group are
 *   finished with the specified message queue mqdes.  The mq_close_group()
 *   deallocates any system resources allocated by the system for use by
 *   this task for its message queue.
 *
 * Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Return Value:
 *   0 (OK) if the message queue is closed successfully,
 *   otherwise, -1 (ERROR).
 *
 ****************************************************************************/

int mq_close_group(mqd_t mqdes, FAR struct task_group_s *group);

/****************************************************************************
 * Name: mq_desclose_group
 *
 * Description:
 *   This function performs the portion of the mq_close operation related
 *   to freeing resource used by the message queue descriptor itself.
 *
 * Parameters:
 *   mqdes - Message queue descriptor.
 *   group - Group that has the open descriptor.
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 * - Called only from mq_close() with the scheduler locked.
 *
 ****************************************************************************/

void mq_desclose_group(mqd_t mqdes, FAR struct task_group_s *group);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* ___INCLUDE_NUTTX_MQUEUE_H */

