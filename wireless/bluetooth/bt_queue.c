/****************************************************************************
 * wireless/bluetooth/bt_queue.c
 * Inter-thread buffer queue management
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

/* Common message queue attributes */

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
                  FAR struct file *mqd)
{
  struct mq_attr attr;
  int ret = OK;

  /* Initialize the message queue attributes */

  attr.mq_maxmsg  = nmsgs;
  attr.mq_msgsize = BT_MSGSIZE;
  attr.mq_flags   = BT_MSGFLAGS;

  ret = file_mq_open(mqd, name, oflags, 0666, &attr);
  if (ret < 0)
    {
      gerr("ERROR: file_mq_open(%s) failed: %d\n", name, ret);
    }

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

int bt_queue_receive(struct file *mqd, FAR struct bt_buf_s **buf)
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
  msgsize = file_mq_receive(mqd, u.msgbuf, BT_MSGSIZE, &priority);
  if (msgsize < 0)
    {
      wlerr("ERROR: file_mq_receive() failed: %ld\n", (long)msgsize);
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

int bt_queue_send(struct file *mqd,
                  FAR struct bt_buf_s *buf,
                  unsigned int priority)
{
  struct bt_bufmsg_s msg;
  int ret;

  DEBUGASSERT(mqd != NULL && buf != NULL && buf->frame != NULL);

  /* Format and send the buffer message */

  msg.buf = buf;
  ret = file_mq_send(mqd, (FAR const char *)&msg,
                     sizeof(struct bt_bufmsg_s), priority);
  if (ret < 0)
    {
      wlerr("ERROR: file_mq_send() failed: %d\n", ret);
    }

  return ret;
}
