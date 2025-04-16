/****************************************************************************
 * sched/mqueue/msgq.c
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/circbuf.h>
#include <nuttx/spinlock.h>

#include <nuttx/msgq.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function irqstate_t nxmsgq_lock(FAR nxmsgq_t *msgq)
{
#ifdef CONFIG_SMP
  return spin_lock_irqsave(&msgq->lock);
#else
  UNUSED(msgq);
  return enter_critical_section();
#endif
}

static inline_function void nxmsgq_unlock(FAR nxmsgq_t *msgq,
                                          irqstate_t flags)
{
#ifdef CONFIG_SMP
  spin_unlock_irqrestore(&msgq->lock, flags);
#else
  UNUSED(msgq);
  leave_critical_section(flags);
#endif
}

static inline_function int nxmsgq_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      return nxsem_post(sem);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxmsgq_init
 *
 * Description:
 *   Initialize a message queue with static ring buffer
 *
 *   This routine initializes a message queue object, prior to its first use.
 *
 *   The message queue's ring buffer must contain space for max_msgs
 *   messages, each of which is msg_size bytes long. Alignment of the
 *   message queue's ring buffer is not necessary.
 *
 * Input Parameters:
 *   msgq     - Address of the message queue.
 *   buffer   - Pointer to ring buffer that holds queued messages.
 *   msg_size - Message size (in bytes).
 *   max_msgs - Maximum number of messages that can be queued.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxmsgq_init(FAR nxmsgq_t *msgq, FAR void *buffer, size_t msg_size,
                 uint32_t max_msgs)
{
  circbuf_init(&msgq->cbuf, buffer, msg_size * max_msgs);

  nxsem_init(&msgq->txsem, 0, 0);
  nxsem_init(&msgq->rxsem, 0, 0);

#ifdef CONFIG_SMP
  spin_lock_init(&msgq->lock);
#endif

  msgq->msg_size = msg_size;
}

/****************************************************************************
 * Name: nxmsgq_create
 *
 * Description:
 *   Create a message queue.
 *
 *   This routine initializes a message queue object, prior to its first use,
 *   allocating its internal ring buffer from the calling thread's resource
 *   pool.
 *
 *   Memory allocated for the ring buffer can be released by calling
 *   nxmsgq_destroy()
 *
 * Input Parameters:
 *   msg_size - Message size (in bytes).
 *   max_msgs - Maximum number of messages that can be queued.
 *
 * Returned Value:
 *   NULL if there was insufficient memory to create nxmsgq.
 *
 ****************************************************************************/

FAR nxmsgq_t *nxmsgq_create(size_t msg_size, uint32_t max_msgs)
{
  FAR nxmsgq_t *msgq;

  msgq = kmm_zalloc(sizeof(*msgq) + msg_size * max_msgs);
  if (msgq == NULL)
    {
      return NULL;
    }

  msgq->alloc = true;

  nxmsgq_init(msgq, msgq + 1, msg_size, max_msgs);

  return msgq;
}

/****************************************************************************
 * Name: nxmsgq_destroy
 *
 * Description:
 *   Destroy Message Queue
 *
 * Input Parameters:
 *   msgq - message queue to cleanup
 *
 ****************************************************************************/

void nxmsgq_destroy(FAR nxmsgq_t *msgq)
{
  irqstate_t flags;

  flags = nxmsgq_lock(msgq);

  nxsem_destroy(&msgq->txsem);
  nxsem_destroy(&msgq->rxsem);

  circbuf_uninit(&msgq->cbuf);

  if (msgq->alloc)
    {
      kmm_free(msgq);
    }

  nxmsgq_unlock(msgq, flags);
}

/****************************************************************************
 * Name: nxmsgq_used
 *
 * Description:
 *   Get the number of messages in a message queue.
 *
 *   This routine returns the number of messages in a message queue's
 *   ring buffer.
 *
 * Returned Value:
 *   return Number of ring buffer entries.
 *
 ****************************************************************************/

int nxmsgq_used(FAR nxmsgq_t *msgq)
{
  irqstate_t flags;
  int num;

  flags = nxmsgq_lock(msgq);

  num = circbuf_used(&msgq->cbuf) / msgq->msg_size;

  nxmsgq_unlock(msgq, flags);

  return num;
}

/****************************************************************************
 * Name: nxmsgq_used
 *
 * Description:
 *   Get the amount of free space in a message queue.
 *
 *   This routine returns the number of unused entries in a message queue's
 *   ring buffer.
 *
 * Returned Value:
 *   return Number of unused ring buffer entries.
 *
 ****************************************************************************/

int nxmsgq_space(FAR nxmsgq_t *msgq)
{
  irqstate_t flags;
  int num;

  flags = nxmsgq_lock(msgq);

  num = circbuf_space(&msgq->cbuf) / msgq->msg_size;

  nxmsgq_unlock(msgq, flags);

  return num;
}

/****************************************************************************
 * Name: nxmsgq_is_empty
 *
 * Description:
 *   Return true if the ring buffer is empty.
 *
 ****************************************************************************/

bool nxmsgq_is_empty(FAR nxmsgq_t *msgq)
{
  irqstate_t flags;
  bool empty;

  flags = nxmsgq_lock(msgq);

  empty = circbuf_is_empty(&msgq->cbuf);

  nxmsgq_unlock(msgq, flags);

  return empty;
}

/****************************************************************************
 * Name: nxmsgq_is_full
 *
 * Description:
 *   Return true if the ring buffer is full.
 *
 ****************************************************************************/

bool nxmsgq_is_full(FAR nxmsgq_t *msgq)
{
  irqstate_t flags;
  bool full;

  flags = nxmsgq_lock(msgq);

  full = circbuf_is_full(&msgq->cbuf);

  nxmsgq_unlock(msgq, flags);

  return full;
}

/****************************************************************************
 * Name: nxmsgq_purge
 *
 * Description:
 *   Purge a message queue.
 *
 *   This routine discards all unreceived messages in a message queue's ring
 *   buffer.
 *
 ****************************************************************************/

void nxmsgq_purge(nxmsgq_t *msgq)
{
  irqstate_t flags;

  flags = nxmsgq_lock(msgq);

  circbuf_reset(&msgq->cbuf);

  nxmsgq_unlock(msgq, flags);
}

/****************************************************************************
 * Name: nxmsgq_ticksend / nxmsgq_send / nxmsgq_trysend
 *
 * Description:
 *   Send a message to a message queue
 *
 *   This routine sends a message to message queue.
 *
 *   The message content is copied from buffer into msgq and the buffer
 *   pointer is not retained, so the message content will not be modified
 *   by this function.
 *
 * Input Parameters:
 *   msgq     - Address of the message queue.
 *   data     - Pointer to the message.
 *   timeout  - Waiting period to add the message
 *
 * Returned Value:
 *   0        - Message sent.
 *   -EAGAIN  - Waiting period timed out.
 *
 ****************************************************************************/

int nxmsgq_ticksend(FAR nxmsgq_t *msgq, FAR const void *buffer,
                    uint32_t delay)
{
  irqstate_t flags;
  int ret = 0;

  flags = nxmsgq_lock(msgq);

  while (circbuf_space(&msgq->cbuf) < msgq->msg_size)
    {
      if (delay == 0)
        {
          ret = -EAGAIN;
          goto bail;
        }

      nxmsgq_unlock(msgq, flags);

      ret = nxsem_tickwait_uninterruptible(&msgq->txsem, delay);

      flags = nxmsgq_lock(msgq);
      if (ret < 0)
        {
          goto bail;
        }
    }

  circbuf_write(&msgq->cbuf, buffer, msgq->msg_size);

  nxmsgq_post(&msgq->rxsem);

bail:
  nxmsgq_unlock(msgq, flags);

  return ret;
}

int nxmsgq_send(FAR nxmsgq_t *msgq, FAR const void *buffer)
{
  return nxmsgq_ticksend(msgq, buffer, UINT32_MAX);
}

int nxmsgq_trysend(FAR nxmsgq_t *msgq, FAR const void *buffer)
{
  return nxmsgq_ticksend(msgq, buffer, 0);
}

/****************************************************************************
 * Name: nxmsgq_tickrecv / nxmsgq_tryrecv / nxmsgq_recv
 *
 * Description:
 *   Receive a message from a message queue
 *
 *   This routine receives a message from message queue in a "first in,
 *   first out" manner.
 *
 * Input Parameters:
 *   msgq    - Address of the message queue.
 *   buffer  - Address of area to hold the received message.
 *   timeout - Waiting period to receive the message,
 *
 * Returned Value:
 *   0       - Message received.
 *   -EAGAIN - Waiting period timed out.
 *
 ****************************************************************************/

static int nxmsgq_recv_internal(FAR nxmsgq_t *msgq, FAR void *buffer,
                                uint32_t delay, bool peek)
{
  irqstate_t flags;
  int ret;

again:
  flags = nxmsgq_lock(msgq);

  if (circbuf_used(&msgq->cbuf) >= msgq->msg_size)
    {
      if (!peek)
        {
          circbuf_read(&msgq->cbuf, buffer, msgq->msg_size);
        }
      else
        {
          circbuf_peek(&msgq->cbuf, buffer, msgq->msg_size);
        }

      nxmsgq_post(&msgq->txsem);

      nxmsgq_unlock(msgq, flags);

      return 0;
    }

  nxmsgq_unlock(msgq, flags);

  if (delay == 0)
    {
      return -ETIMEDOUT;
    }
  else if (delay == UINT32_MAX)
    {
      ret = nxsem_wait_uninterruptible(&msgq->rxsem);
    }
  else
    {
      ret = nxsem_tickwait_uninterruptible(&msgq->rxsem, delay);
    }

  if (ret == 0)
    {
      goto again;
    }

  return ret;
}

int nxmsgq_tickrecv(FAR nxmsgq_t *msgq, FAR void *buffer, uint32_t delay)
{
  return nxmsgq_recv_internal(msgq, buffer, delay, false);
}

int nxmsgq_tryrecv(FAR nxmsgq_t *msgq, FAR void *buffer)
{
  return nxmsgq_recv_internal(msgq, buffer, 0, false);
}

int nxmsgq_recv(FAR nxmsgq_t *msgq, FAR void *buffer)
{
  return nxmsgq_recv_internal(msgq, buffer, UINT32_MAX, false);
}

/****************************************************************************
 * Name: nxmsgq_tickpeek / nxmsgq_trypeek / nxmsgq_peek
 *
 * Description:
 *   Peek/read a message from a message queue.
 *
 *   This routine reads a message from message queue in a "first in,
 *   first out" manner and leaves the message in the queue.
 *
 * Input Parameters:
 *   msgq    - Address of the message queue.
 *   buffer  - Address of area to hold the received message.
 *   timeout - Waiting period to receive the message,
 *
 * Returned Value:
 *   0       - Message read.
 *   -EAGAIN - Waiting period timed out.
 *
 ****************************************************************************/

int nxmsgq_tickpeek(FAR nxmsgq_t *msgq, FAR void *buffer, uint32_t delay)
{
  return nxmsgq_recv_internal(msgq, buffer, delay, true);
}

int nxmsgq_trypeek(FAR nxmsgq_t *msgq, FAR void *buffer)
{
  return nxmsgq_recv_internal(msgq, buffer, 0, true);
}

int nxmsgq_peek(FAR nxmsgq_t *msgq, FAR void *buffer)
{
  return nxmsgq_recv_internal(msgq, buffer, UINT32_MAX, true);
}
