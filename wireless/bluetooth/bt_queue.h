/****************************************************************************
 * wireless/bluetooth/bt_queue.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_QUEUE_H
#define __WIRELESS_BLUETOOTH_BT_QUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/mqueue.h>

#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Names of all POSIX message queues. */

#define BT_CONN_TX       "btconntx"
#define BT_HCI_TX        "bthcitx"

/* All messages are sent FIFO at the mid-message priorities except for high-
 * priority messages received from the Bluetooth driver.
 */

#define BT_PRIO_MAX      MQ_PRIO_MAX
#define BT_PRIO_MIN      0

#define BT_NORMAL_PRIO   (BT_PRIO_MIN + (BT_PRIO_MAX - BT_PRIO_MIN) / 2)
#define BT_HIGH_PRIO     (BT_PRIO_MIN + 3 * (BT_PRIO_MAX - BT_PRIO_MIN) / 4)

/* Verify that enough messages have been allocated */

#define BT_NMSGS         (CONFIG_BLUETOOTH_TXCMD_NMSGS + \
                          CONFIG_BLUETOOTH_TXCONN_NMSGS)

#if BT_NMSGS > CONFIG_PREALLOC_MQ_MSGS
#  warning WARNING: not enough pre-allocated messages
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct bt_buf_s; /* Forward Reference */

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
                  FAR struct file *mqd);

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

int bt_queue_receive(struct file *mqd, FAR struct bt_buf_s **buf);

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
                  unsigned int priority);

#endif /* __WIRELESS_BLUETOOTH_BT_QUEUE_H */
