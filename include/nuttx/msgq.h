/****************************************************************************
 * include/nuttx/msgq.h
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

#ifndef ___INCLUDE_NUTTX_MSGQ_H
#define ___INCLUDE_NUTTX_MSGQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/semaphore.h>
#include <nuttx/circbuf.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: NXMSGQ_INITIALIZER
 *
 * Description:
 *   Statically define and initialize a message queue.
 *
 *   The message queue's ring buffer contains space for max_msgs messages,
 *   each of which is msg_size bytes long.
 *
 * Input Parameters:
 *   buffer   - Ring buffer of Message Queue
 *   msg_size - Message size (in bytes).
 *   max_msgs - Maximum number of messages that can be queued.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
#  define NXMSGQ_INITIALIZER(buffer, msg_size, max_msgs) \
    { \
      CIRCBUF_INITIALIZER(buffer, msg_size * max_msgs), \
      msg_size, \
      NXSEM_INITIALIZER(0, 0), \
      NXSEM_INITIALIZER(0, 0), \
      false, \
      SP_UNLOCKED, \
    }
#else
#  define NXMSGQ_INITIALIZER(buffer, msg_size, max_msgs) \
    { \
      CIRCBUF_INITIALIZER(buffer, msg_size * max_msgs), \
      msg_size, \
      NXSEM_INITIALIZER(0, 0), \
      NXSEM_INITIALIZER(0, 0), \
      false, \
    }
#endif

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* This structure defines a kernel message queue */

typedef struct nxmsgq
{
  struct circbuf_s cbuf;
  int              msg_size;
  sem_t            txsem;
  sem_t            rxsem;
  bool             alloc;
#ifdef CONFIG_SMP
  spinlock_t       lock;
#endif
} nxmsgq_t;

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

void nxmsgq_init(FAR nxmsgq_t *msgq, FAR void *buffer,
                 size_t msg_size, uint32_t max_msgs);

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

FAR nxmsgq_t *nxmsgq_create(size_t msg_size, uint32_t max_msgs);

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

void nxmsgq_destroy(FAR nxmsgq_t *msgq);

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

int nxmsgq_used(FAR nxmsgq_t *msgq);

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

int nxmsgq_space(FAR nxmsgq_t *msgq);

/****************************************************************************
 * Name: nxmsgq_is_empty
 *
 * Description:
 *   Return true if the ring buffer is empty.
 *
 ****************************************************************************/

bool nxmsgq_is_empty(FAR nxmsgq_t *msgq);

/****************************************************************************
 * Name: nxmsgq_is_full
 *
 * Description:
 *   Return true if the ring buffer is full.
 *
 ****************************************************************************/

bool nxmsgq_is_full(FAR nxmsgq_t *msgq);

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

void nxmsgq_purge(FAR nxmsgq_t *msgq);

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

int nxmsgq_ticksend(FAR nxmsgq_t *msgq,
                    FAR const void *buffer, uint32_t delay);
int nxmsgq_send(FAR nxmsgq_t *msgq, FAR const void *buffer);
int nxmsgq_trysend(FAR nxmsgq_t *msgq, FAR const void *buffer);

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

int nxmsgq_tickrecv(FAR nxmsgq_t *msgq, FAR void *buffer, uint32_t delay);
int nxmsgq_tryrecv(FAR nxmsgq_t *msgq, FAR void *buffer);
int nxmsgq_recv(FAR nxmsgq_t *msgq, FAR void *buffer);

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

int nxmsgq_tickpeek(FAR nxmsgq_t *msgq, FAR void *buffer, uint32_t delay);
int nxmsgq_trypeek(FAR nxmsgq_t *msgq, FAR void *buffer);
int nxmsgq_peek(FAR nxmsgq_t *msgq, FAR void *buffer);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* ___INCLUDE_NUTTX_MSGQ_H */
