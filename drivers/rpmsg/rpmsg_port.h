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
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/rpmsg/rpmsg_port.h>

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
  uint8_t  buf[0];                /* Payload buffer */
} end_packed_struct;

struct rpmsg_port_list_s
{
  uint16_t         num;           /* Number of buffers */
  sem_t            sem;           /* Used to wait for buffer */
  spinlock_t       lock;          /* List lock */
  struct list_node head;          /* List head */
};

struct rpmsg_port_queue_s
{
  /* Indicate buffers current queue managed is dynamic alloced */

  bool                     alloced;

  /* Buffer array's base address of current queue managed */

  FAR void                 *buf;

  /* Node related to buffer for buffer management */

  FAR struct list_node     *node;

  /* Length of buffers current queue managed */

  uint16_t                 len;

  /* Free list of buffers which have not been occupied data yet */

  struct rpmsg_port_list_s free;

  /* Ready list of buffers which have been occupied data already */

  struct rpmsg_port_list_s ready;
};

struct rpmsg_port_s;

typedef void (*rpmsg_port_rx_cb_t)(FAR struct rpmsg_port_s *port,
                                   FAR struct rpmsg_port_header_s *hdr);

struct rpmsg_port_ops_s
{
  /* Notify driver there is buffer to be sent of the tx queue */

  CODE void (*notify_tx_ready)(FAR struct rpmsg_port_s *port);

  /* Notify driver there is a buffer in rx queue is freed */

  CODE void (*notify_rx_free)(FAR struct rpmsg_port_s *port);

  /* Register callback function which should be invoked when there is
   * date received to the rx queue by driver
   */

  CODE void (*register_callback)(FAR struct rpmsg_port_s *port,
                                 rpmsg_port_rx_cb_t callback);
};

struct rpmsg_port_s
{
  struct rpmsg_s                    rpmsg;
  struct rpmsg_device               rdev;   /* Rpmsg device object */
  struct rpmsg_port_queue_s         txq;    /* Port tx queue */
  struct rpmsg_port_queue_s         rxq;    /* Port rx queue */

  char                              local_cpuname[RPMSG_NAME_SIZE];

  /* Remote cpu name of this port connected to */

  char                              cpuname[RPMSG_NAME_SIZE];

  /* Ops need implemented by drivers under port layer */

  const FAR struct rpmsg_port_ops_s *ops;
};

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
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
  return atomic_read(&queue->free.num);
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
  return atomic_read(&queue->ready.num);
}

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif
#endif /* __DRIVERS_RPMSG_RPMSG_PORT_H */
