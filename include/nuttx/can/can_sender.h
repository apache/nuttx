/****************************************************************************
 * include/nuttx/can/can_sender.h
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

#ifndef __INCLUDE_NUTTX_CAN_SENDER_H
#define __INCLUDE_NUTTX_CAN_SENDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/list.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/can/can.h>

#ifdef CONFIG_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_CAN_TXPRIORITY

/* There are three linked lists to manage TX buffer:
 * tx_free     - can_write function get a tx_free node, write message, then
 *               move this node to tx_pending list.
 * tx_pending  - cache message send from application but not send to H/W.
 *               Sorted in ascending order based on can_id.
 * tx_sending  - Message in H/W is sending to bus, but not confirmation.
 *               Sorted in ascending order based on can_id.
 */

#  define TX_EMPTY(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (list_length(&__sender->tx_free) == CONFIG_CAN_TXFIFOSIZE); \
    }) \

#  define TX_FULL(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      list_is_empty(&__sender->tx_free); \
    }) \

#  define TX_PENDING(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      !list_is_empty(&__sender->tx_pending); \
    }) \

#  define PENDING_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (list_length(&__sender->tx_pending)); \
    }) \

#  define SENDING_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (list_length(&__sender->tx_sending)); \
    }) \

#  define FREE_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (list_length(&__sender->tx_free)); \
    }) \

#else
/* There are three fields used to manage TX FIFO buffer:
 * tx_tail:  Incremented in can_write each time a message is queued in the
 *           FIFO
 * tx_head:  Incremented in can_txdone each time a message completes
 * tx_queue: Incremented each time that a message is sent to the hardware.
 *
 * Logically (ignoring buffer wrap-around): tx_head <= tx_queue <= tx_tail
 * tx_head == tx_queue == tx_tail means that the FIFO is empty
 * tx_head < tx_queue == tx_tail means that all data has been queued, but
 * we are still waiting for transmissions to complete.
 */
#  define TX_EMPTY(sender) \
      ({ \
        FAR struct can_txcache_s *__sender = (sender); \
        (__sender->tx_head == __sender->tx_tail); \
      }) \

#  define TX_FULL(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (__sender->tx_head == \
          (__sender->tx_tail + 1) % CONFIG_CAN_TXFIFOSIZE); \
    }) \

#  define TX_PENDING(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (__sender->tx_queue != __sender->tx_tail); \
    }) \

#  define PENDING_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (CONFIG_CAN_TXFIFOSIZE + __sender->tx_tail - __sender->tx_queue) \
                                              % CONFIG_CAN_TXFIFOSIZE; \
    }) \

#  define SENDING_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (CONFIG_CAN_TXFIFOSIZE + __sender->tx_queue - __sender->tx_head) \
                                              % CONFIG_CAN_TXFIFOSIZE; \
    }) \

#  define FREE_COUNT(sender) \
    ({ \
      FAR struct can_txcache_s *__sender = (sender); \
      (CONFIG_CAN_TXFIFOSIZE - 1 - __sender->tx_tail + __sender->tx_head) \
                                                % CONFIG_CAN_TXFIFOSIZE; \
    }) \

#endif  /* CONFIG_CAN_TXPRIORITY */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: can_sender_init
 *
 * Description:
 *   Initial dev sender.
 *
 * Input Parameters:
 *   cd_sender  - The CAN device sender.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void can_sender_init(FAR struct can_txcache_s *cd_sender);

/****************************************************************************
 * Name: can_add_sendnode
 *
 * Description:
 *   Add message to sender..
 *
 * Input Parameters:
 *   cd_sender  - The CAN device sender.
 *
 * Returned Value:
 *   CAN message which to be send.
 *
 ****************************************************************************/

void can_add_sendnode(FAR struct can_txcache_s *cd_sender,
                      FAR struct can_msg_s *msg, int msglen);

/****************************************************************************
 * Name: can_get_msg
 *
 * Description:
 *   Get message from sender, and change sender recoder.
 *
 * Input Parameters:
 *   cd_sender  - The CAN device sender.
 *
 * Returned Value:
 *   CAN message which to be send.
 *
 ****************************************************************************/

struct can_msg_s *can_get_msg(FAR struct can_txcache_s *cd_sender);

/****************************************************************************
 * Name: can_revert_msg
 *
 * Description:
 *   Revert sender recoder when send failed.
 *
 * Input Parameters:
 *   cd_sender  - The CAN device sender.
 *   msg        - message which to be revert.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void can_revert_msg(FAR struct can_txcache_s *cd_sender,
                    FAR struct can_msg_s *msg);

/****************************************************************************
 * Name: can_send_done
 *
 * Description:
 *   Release the sender resources, after the tragic message is successfully
 *   sent to the bus.
 *
 * Input Parameters:
 *   cd_sender  - The CAN device sender.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void can_send_done(FAR struct can_txcache_s *cd_sender);

#endif /* CONFIG_CAN */
#endif /* __INCLUDE_NUTTX_CAN_SENDER_H */
