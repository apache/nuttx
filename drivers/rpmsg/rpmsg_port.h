/****************************************************************************
 * drivers/rpmsg/rpmsg_port.h
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

#ifndef __DRIVERS_RPMSG_RPMSG_PORT_H
#define __DRIVERS_RPMSG_RPMSG_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/atomic.h>
#include <nuttx/list.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/rpmsg/rpmsg_port.h>

#include "rpmsg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define rpmsgvbs                rpmsgerr

#define RPMSG_PORT_DROP_TXQ     0x01
#define RPMSG_PORT_DROP_RXQ     0x02
#define RPMSG_PORT_DROP_ALL     0x03

#define RPMSG_PORT_BUF_TO_NODE(q, b) \
  ((q)->node + ((FAR void *)(b) - (q)->buf) / (q)->len)
#define RPMSG_PORT_NODE_TO_BUF(q, n) \
  ((q)->buf + (((n) - (q)->node)) * (q)->len)
#define RPMSG_PORT_SEQ_LT(seq, ack)  ((int16_t)((seq) - (ack)) < 0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This header is for physical layer's use */

begin_packed_struct struct rpmsg_port_header_s
{
  uint16_t crc;                   /* CRC of current port data frame */
  uint16_t cmd;                   /* Reserved for uart/spi port driver */
  uint16_t avail;                 /* Available rx buffer of peer side */
  uint16_t len;                   /* Data frame length */
#ifdef CONFIG_RPMSG_PORT_RETX
  uint16_t seqno;
  uint16_t ackno;
  uint16_t min_seq;
  uint16_t reserved;
#endif
  uint8_t  buf[0];                /* Payload buffer */
} end_packed_struct;

struct rpmsg_port_list_s
{
  uint16_t         num;           /* Number of buffers */
  sem_t            sem;           /* Used to wait for buffer */
  spinlock_t       lock;          /* List lock */
  struct list_node head;          /* List head */
};

struct rpmsg_port_node_s
{
  struct list_node node;          /* List node for free/ready/retx queue */
#ifdef CONFIG_RPMSG_PORT_RETX
  uint16_t         retry_count;   /* Retry counter for retransmission */
#endif
};

struct rpmsg_port_s;

struct rpmsg_port_queue_s
{
  /* Pointer to the rpmsg port handler */

  FAR struct rpmsg_port_s      *port;

  /* Indicate buffers current queue managed is dynamic alloced */

  bool                         alloced;

  /* Buffer array's base address of current queue managed */

  FAR void                     *buf;

  /* Node related to buffer for buffer management */

  FAR struct rpmsg_port_node_s *node;

  /* Length of buffers current queue managed */

  uint16_t                     len;

  /* Free list of buffers which have not been occupied data yet */

  struct rpmsg_port_list_s     free;

  /* Ready list of buffers which have been occupied data already */

  struct rpmsg_port_list_s     ready;
};

typedef void (*rpmsg_port_rx_cb_t)(FAR struct rpmsg_port_s *port,
                                   FAR struct rpmsg_port_header_s *hdr);

struct rpmsg_port_ops_s
{
  /* Notify driver there is buffer to be sent of the tx queue */

  CODE void (*notify_tx_ready)(FAR struct rpmsg_port_s *port);

  /* Notify driver there is a buffer in rx queue is freed */

  CODE void (*notify_rx_free)(FAR struct rpmsg_port_s *port);

  /* Notify driver there is no available buffer */

  CODE int (*notify_queue_noavail)(FAR struct rpmsg_port_s *port,
                                   FAR struct rpmsg_port_queue_s *queue);

  /* Register callback function which should be invoked when there is
   * date received to the rx queue by driver
   */

  CODE void (*register_callback)(FAR struct rpmsg_port_s *port,
                                 rpmsg_port_rx_cb_t callback);

  /* Dump the transport debug information */

  CODE void (*dump)(FAR struct rpmsg_port_s *port, bool verbose);
};

#ifdef CONFIG_RPMSG_PORT_RETX
struct rpmsg_port_retx_s
{
  struct list_node                  queue;
  spinlock_t                        lock;
  atomic_t                          ackdup_count;
  uint16_t                          tx_next;
  uint16_t                          rx_next;
  uint16_t                          last_ackno;
  uint16_t                          retx_max_retry;
};
#endif

struct rpmsg_port_s
{
  struct rpmsg_s                    rpmsg;
  struct rpmsg_device               rdev;   /* Rpmsg device object */
  struct rpmsg_port_queue_s         txq;    /* Port tx queue */
  struct rpmsg_port_queue_s         rxq;    /* Port rx queue */

  /* Ops need implemented by drivers under port layer */

  const FAR struct rpmsg_port_ops_s *ops;

#ifdef CONFIG_RPMSG_PORT_RETX
  /* Retransmission mechanism for DATA frames */

  struct rpmsg_port_retx_s          retx;
#endif
};

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_queue_get_available_buffer
 *
 * Description:
 *   Get buffer from free list of the queue.
 *
 * Input Parameters:
 *   queue - The queue to be getten from.
 *   wait  - If wait or not when there is no available buffer of the queue.
 *
 * Returned Value:
 *   A struct rpmsg_port_header_s's pointer on success or NULL on failure.
 *   The len of struct rpmsg_port_header_s indicates the sum of struct
 *   rpmsg_port_header_s's size and the data size can be used.
 *
 ****************************************************************************/

FAR struct rpmsg_port_header_s *
rpmsg_port_queue_get_available_buffer(FAR struct rpmsg_port_queue_s *queue,
                                      bool wait);

/****************************************************************************
 * Name: rpmsg_port_queue_return_buffer
 *
 * Description:
 *   Return buffer to free list of the queue.
 *
 * Input Parameters:
 *   queue - The queue to be returned to.
 *   hdr   - Pointer to the struct rpmsg_port_header_s to be returned.
 *
 * Returned Value:
 *   No return value.
 *
 ****************************************************************************/

void rpmsg_port_queue_return_buffer(FAR struct rpmsg_port_queue_s *queue,
                                    FAR struct rpmsg_port_header_s *hdr);

/****************************************************************************
 * Name: rpmsg_port_queue_get_buffer
 *
 * Description:
 *   Get buffer from ready list of the queue.
 *
 * Input Parameters:
 *   queue - The queue to be getten from.
 *   wait  - If wait or not when there is no used buffer of the queue.
 *
 * Returned Value:
 *   A struct rpmsg_port_header_s's pointer on success or NULL on failure.
 *   The len of struct rpmsg_port_header_s indicates the sum of struct
 *   rpmsg_port_header_s's size and the data size has be used.
 *
 ****************************************************************************/

FAR struct rpmsg_port_header_s *
rpmsg_port_queue_get_buffer(FAR struct rpmsg_port_queue_s *queue,
                            bool wait);

/****************************************************************************
 * Name: rpmsg_port_queue_add_buffer
 *
 * Description:
 *   Add buffer to ready list of the queue.
 *
 * Input Parameters:
 *   queue - The queue to be added to.
 *   hdr   - Pointer to the struct rpmsg_port_header_s to be added. The
 *           length of it must be set before this function invoked.
 *
 * Returned Value:
 *   No return value.
 *
 ****************************************************************************/

void rpmsg_port_queue_add_buffer(FAR struct rpmsg_port_queue_s *queue,
                                 FAR struct rpmsg_port_header_s *hdr);

/****************************************************************************
 * Name: rpmsg_port_queue_navail
 *
 * Description:
 *   Get available buffer number of free list of the queue.
 *
 * Input Parameters:
 *   queue - The queue is to be calculated.
 *
 * Returned Value:
 *   Number of available buffers.
 *
 ****************************************************************************/

static inline_function
uint16_t rpmsg_port_queue_navail(FAR struct rpmsg_port_queue_s *queue)
{
  irqstate_t flags;
  uint16_t num;

  flags = spin_lock_irqsave(&queue->free.lock);
  num = queue->free.num;
  spin_unlock_irqrestore(&queue->free.lock, flags);
  return num;
}

/****************************************************************************
 * Name: rpmsg_port_queue_nused
 *
 * Description:
 *   Get used buffer number of ready list of the queue.
 *
 * Input Parameters:
 *   queue - The queue is to be calculated.
 *
 * Returned Value:
 *   Number of used buffers.
 *
 ****************************************************************************/

static inline_function
uint16_t rpmsg_port_queue_nused(FAR struct rpmsg_port_queue_s *queue)
{
  irqstate_t flags;
  uint16_t num;

  flags = spin_lock_irqsave(&queue->ready.lock);
  num = queue->ready.num;
  spin_unlock_irqrestore(&queue->ready.lock, flags);
  return num;
}

/****************************************************************************
 * Name: rpmsg_port_update_timestamp
 ****************************************************************************/

void rpmsg_port_update_timestamp(FAR struct rpmsg_port_queue_s *queue,
                                 FAR struct rpmsg_port_header_s *hdr,
                                 bool tx);

/****************************************************************************
 * Name: rpmsg_port_drop_packets
 ****************************************************************************/

void rpmsg_port_drop_packets(FAR struct rpmsg_port_s *rport, uint8_t type);

#ifdef CONFIG_RPMSG_PORT_RETX

/****************************************************************************
 * Name: rpmsg_port_retx_del
 *
 * Description:
 *   Remove a node from the retransmission queue and return the buffer
 *   to txq. Caller must hold port->retx.lock.
 *
 ****************************************************************************/

static inline_function
void rpmsg_port_retx_del(FAR struct rpmsg_port_s *port,
                         FAR struct rpmsg_port_node_s *node)
{
  list_delete(&node->node);
  rpmsg_port_queue_return_buffer(&port->txq,
                                 RPMSG_PORT_NODE_TO_BUF(&port->txq, node));
}

/****************************************************************************
 * Name: rpmsg_port_retx_add
 *
 * Description:
 *   Add a DATA frame to the retransmission queue.
 *
 ****************************************************************************/

static inline_function
void rpmsg_port_retx_add(FAR struct rpmsg_port_s *port,
                         FAR struct rpmsg_port_header_s *txhdr)
{
  FAR struct rpmsg_port_node_s *node;
  irqstate_t flags;

  node = RPMSG_PORT_BUF_TO_NODE(&port->txq, txhdr);
  node->retry_count = 0;

  flags = spin_lock_irqsave(&port->retx.lock);
  txhdr->seqno = port->retx.tx_next++;
  list_add_tail(&port->retx.queue, &node->node);
  spin_unlock_irqrestore(&port->retx.lock, flags);
}

/****************************************************************************
 * Name: rpmsg_port_retx_process_ack
 *
 * Description:
 *   Process an ACK number, removing all acknowledged frames from the
 *   retransmission queue. Trigger retransmission on duplicate ACK.
 *   Also advance rx_next based on peer's min_seq to skip abandoned frames.
 *
 ****************************************************************************/

void rpmsg_port_retx_process_ack(FAR struct rpmsg_port_s *port,
                                  uint16_t ackno, uint16_t min_seq);

/****************************************************************************
 * Name: rpmsg_port_retx_rx_valid
 *
 * Description:
 *   Validate a received DATA frame's seqno and process its ackno.
 *   Called from SPI complete_handler / slave_notify (interrupt context).
 *
 * Returned Value:
 *   true  - frame is valid, caller should enqueue to rxq.ready
 *   false - frame is invalid (duplicate/out-of-order), caller should drop
 *
 ****************************************************************************/

bool rpmsg_port_retx_rx_valid(FAR struct rpmsg_port_s *port,
                              FAR struct rpmsg_port_header_s *hdr);

/****************************************************************************
 * Name: rpmsg_port_retx_reset
 *
 * Description:
 *   Clear the retransmission queue.
 *
 ****************************************************************************/

void rpmsg_port_retx_reset(FAR struct rpmsg_port_s *port);

/****************************************************************************
 * Name: rpmsg_port_retx_get
 *
 * Description:
 *   Get the head node of the retransmission queue.
 *
 * Returned Value:
 *   Pointer to the head node, or NULL if the queue is empty.
 *
 ****************************************************************************/

static inline_function FAR struct rpmsg_port_node_s *
rpmsg_port_retx_get(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_node_s *node = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave(&port->retx.lock);
  if (!list_is_empty(&port->retx.queue))
    {
      node = list_peek_head_type(&port->retx.queue,
                                 struct rpmsg_port_node_s, node);
    }

  spin_unlock_irqrestore(&port->retx.lock, flags);
  return node;
}

/****************************************************************************
 * Name: rpmsg_port_retx_is_pending
 *
 * Description:
 *   Check if retransmission is pending (dup-ack count reached threshold).
 *
 ****************************************************************************/

static inline_function
bool rpmsg_port_retx_is_pending(FAR struct rpmsg_port_s *port)
{
  return atomic_read(&port->retx.ackdup_count) >= 2;
}

/****************************************************************************
 * Name: rpmsg_port_retx_tx_fill
 *
 * Description:
 *   Fill retransmission fields (ackno, min_seq, reserved) in the TX frame
 *   header before sending.
 *
 ****************************************************************************/

static inline_function
void rpmsg_port_retx_tx_fill(FAR struct rpmsg_port_s *port,
                             FAR struct rpmsg_port_header_s *txhdr)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&port->retx.lock);
  txhdr->ackno = port->retx.rx_next;
  txhdr->reserved = 0;

  if (!list_is_empty(&port->retx.queue))
    {
      FAR struct rpmsg_port_node_s *head =
        list_peek_head_type(&port->retx.queue,
                            struct rpmsg_port_node_s, node);
      FAR struct rpmsg_port_header_s *hdr =
        RPMSG_PORT_NODE_TO_BUF(&port->txq, head);
      txhdr->min_seq = hdr->seqno;
    }
  else
    {
      txhdr->min_seq = port->retx.tx_next;
    }

  spin_unlock_irqrestore(&port->retx.lock, flags);
}

#else /* !CONFIG_RPMSG_PORT_RETX */

#  define rpmsg_port_retx_add(port, txhdr)
#  define rpmsg_port_retx_del(port, node)
#  define rpmsg_port_retx_get(port)                         NULL
#  define rpmsg_port_retx_is_pending(port)                  false
#  define rpmsg_port_retx_tx_fill(port, txhdr)
#  define rpmsg_port_retx_process_ack(port, ackno, min_seq)
#  define rpmsg_port_retx_rx_valid(port, hdr)               true
#  define rpmsg_port_retx_reset(port)

#endif /* CONFIG_RPMSG_PORT_RETX */

/****************************************************************************
 * Name: rpmsg_port_initialize
 *
 * Description:
 *   Init port layer by port's configuration. rpmsg port layer creates and
 *   manages queues used for communication of two cpus.
 *
 * Input Parameters:
 *   port - The port to be inited.
 *   cfg  - Port configuration.
 *   ops  - Operation implemented by drivers under port layer.
 *
 * Returned Value:
 *   Zero on success or an negative value on failure.
 *
 ****************************************************************************/

int rpmsg_port_initialize(FAR struct rpmsg_port_s *port,
                          FAR const struct rpmsg_port_config_s *cfg,
                          FAR const struct rpmsg_port_ops_s *ops);

/****************************************************************************
 * Name: rpmsg_port_uninitialize
 *
 * Description:
 *   uninit rpmsg port.
 *
 * Input Parameters:
 *   port - The port to be uninited.
 *
 * Returned Value:
 *   No return value.
 *
 ****************************************************************************/

void rpmsg_port_uninitialize(FAR struct rpmsg_port_s *port);

/****************************************************************************
 * Name: rpmsg_port_register
 *
 * Description:
 *   Invoked to create a rpmsg character device when a connection between
 *   two cpus has established.
 *
 * Input Parameters:
 *   port          - The port has established a connection.
 *   local_cpuname - The local cpuname
 *
 * Returned Value:
 *   Zero on success or an negative value on failure.
 *
 ****************************************************************************/

int rpmsg_port_register(FAR struct rpmsg_port_s *port,
                        FAR const char *local_cpuname);

/****************************************************************************
 * Name: rpmsg_port_unregister
 *
 * Description:
 *   Invoked to unregister the rpmsg character device.
 *
 * Input Parameters:
 *   port - The port has established a connection.
 *
 * Returned Value:
 *   No return value.
 *
 ****************************************************************************/

void rpmsg_port_unregister(FAR struct rpmsg_port_s *port);

#ifdef __cplusplus
}
#endif

#endif
#endif /* __DRIVERS_RPMSG_RPMSG_PORT_H */
