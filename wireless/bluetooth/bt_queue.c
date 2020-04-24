/****************************************************************************
 * wireless/bluetooth/bt_queue.c
 * Inter-thread buffer queue management
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/wireless/bluetooth/bt_buf.h>

#include "bt_queue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Common essage queue attributes */

#define BT_MSGSIZE   sizeof(struct bt_bufmsg_s)
#define BT_MSGFLAGS  0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* A message is just a buffer structure */

struct bt_bufmsg_s
{
  FAR struct bt_buf_s *buf;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bt_queue_open
 *
 * Description:
 *   Open a message queue for read or write access.
 *
 * Input Parameters:
 *   name   - The name of the message queue to open
 *   oflags - Open flags with access mode
 *   nmsgs  - Max number of messages in queue before bt_queue_send() blocks.
 *   mqd    - The location in which to return the message queue descriptor
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure;
 *
 ****************************************************************************/

int bt_queue_open(FAR const char *name, int oflags, int nmsgs,
                  FAR mqd_t *mqd)
{
  struct mq_attr attr;
  mqd_t newmqd;
  int ret = OK;

  /* Initialize the message queue attributes */

  attr.mq_maxmsg  = nmsgs;
  attr.mq_msgsize = BT_MSGSIZE;
  attr.mq_flags   = BT_MSGFLAGS;

  newmqd = mq_open(name, oflags, 0666, &attr);
  if (newmqd == (mqd_t)-1)
    {
      /* REVISIT: mq_open() modifies the errno value */

      ret = -get_errno();
      gerr("ERROR: mq_open(%s) failed: %d\n", name, ret);
      newmqd = NULL;
    }

  *mqd = newmqd;
  return ret;
}

/****************************************************************************
 * Name: bt_queue_receive
 *
 * Description:
 *   Block until the next buffer is received on the queue.
 *
 * Input Parameters:
 *   mqd - The message queue descriptor previously returned by
 *         bt_open_*queue.
 *   buf - The location in which to return the received buffer.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure;
 *
 ****************************************************************************/

int bt_queue_receive(mqd_t mqd, FAR struct bt_buf_s **buf)
{
  union
  {
    struct bt_bufmsg_s msg;
    char msgbuf[BT_MSGSIZE];
  } u;

  ssize_t msgsize;
  unsigned int priority;

  DEBUGASSERT(mqd != NULL && buf != NULL);

  /* Wait for the next message */

  u.msg.buf = NULL;
  msgsize = nxmq_receive(mqd, u.msgbuf, BT_MSGSIZE, &priority);
  if (msgsize < 0)
    {
      wlerr("ERROR: nxmq_receive() failed: %ld\n", (long)msgsize);
      return (int)msgsize;
    }

  /* Only buffers are expected as messages and all messages should have an
   * attached IOB frame.
   */

  DEBUGASSERT(msgsize == sizeof(struct bt_bufmsg_s));
  DEBUGASSERT(u.msg.buf != NULL && u.msg.buf->frame != NULL);

  /* Return the buffer */

  *buf = u.msg.buf;
  return OK;
}

/****************************************************************************
 * Name: bt_queue_send
 *
 * Description:
 *   Send the buffer to the specified message queue
 *
 * Input Parameters:
 *   mqd      - The message queue descriptor previously returned by
 *              bt_open_*queue.
 *   buf      - A reference to the buffer to be sent
 *   priority - Either BT_NORMAL_PRIO or BT_NORMAL_HIGH.  NOTE:
 *              BT_NORMAL_HIGHis only for use within the stack.  Drivers
 *              should always use BT_NORMAL_PRIO.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure;
 *
 ****************************************************************************/

int bt_queue_send(mqd_t mqd, FAR struct bt_buf_s *buf, unsigned int priority)
{
  struct bt_bufmsg_s msg;
  int ret;

  DEBUGASSERT(mqd != NULL && buf != NULL && buf->frame != NULL);

  /* Format and send the buffer message */

  msg.buf = buf;
  ret = nxmq_send(mqd, (FAR const char *)&msg, sizeof(struct bt_bufmsg_s),
                  priority);
  if (ret < 0)
    {
      wlerr("ERROR: mq_send() failed: %d\n", ret);
    }

  return ret;
}
