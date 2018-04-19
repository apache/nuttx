/****************************************************************************
 * wireless/bluetooth/bt_queue.h
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

#ifndef __WIRELESS_BLUETOOTH_BT_QUEUE_H
#define __WIRELESS_BLUETOOTH_BT_QUEUE_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <limits.h>
#include <mqueue.h>

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
                  FAR mqd_t *mqd);

/****************************************************************************
 * Name: bt_queue_receive
 *
 * Description:
 *   Block until the next buffer is received on the queue.
 *
 * Input Parameters:
 *   mqd - The message queue descriptor previously returned by bt_open_*queue.
 *   buf - The location in which to return the received buffer.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on any
 *   failure;
 *
 ****************************************************************************/

int bt_queue_receive(mqd_t mqd, FAR struct bt_buf_s **buf);

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

int bt_queue_send(mqd_t mqd, FAR struct bt_buf_s *buf, int priority);

#endif /* __WIRELESS_BLUETOOTH_BT_QUEUE_H */
