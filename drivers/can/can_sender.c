/****************************************************************************
 * drivers/can/can_sender.c
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

#include <nuttx/config.h>
#include <nuttx/can/can_sender.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_xmit_init
 *
 * Description:
 *   Initial dev sender.
 *
 ****************************************************************************/

void can_sender_init(FAR struct can_txcache_s *cd_sender)
{
#if defined(CONFIG_CAN_TXPRIORITY) && CONFIG_CAN_TXFIFOSIZE <= 0
#  error "CONFIG_CAN_TXFIFOSIZE should be positive non-zero value"
#endif

#ifdef CONFIG_CAN_TXPRIORITY
  int i;

  list_initialize(&cd_sender->tx_free);
  list_initialize(&cd_sender->tx_pending);
  list_initialize(&cd_sender->tx_sending);

  for (i = 0; i < CONFIG_CAN_TXFIFOSIZE; i++)
    {
      list_add_tail(&cd_sender->tx_free, &cd_sender->tx_buffer[i].list);
    }

#else
  cd_sender->tx_head  = 0;
  cd_sender->tx_queue = 0;
  cd_sender->tx_tail  = 0;
#endif
}

/****************************************************************************
 * Name: can_add_sendnode
 *
 * Description:
 *   Add message to sender.
 *
 ****************************************************************************/

void can_add_sendnode(FAR struct can_txcache_s *cd_sender,
                      FAR struct can_msg_s *msg, int msglen)
{
#ifdef CONFIG_CAN_TXPRIORITY
  FAR struct list_node *node;
  FAR struct can_msg_node_s *msg_node;
  FAR struct can_msg_node_s *tmp_node;

  node = list_remove_head(&cd_sender->tx_free);
  msg_node = container_of(node, struct can_msg_node_s, list);
  memcpy(&msg_node->msg, msg, msglen);

  list_for_every_entry(&cd_sender->tx_pending, tmp_node,
                       struct can_msg_node_s, list)
    {
      if (tmp_node->msg.cm_hdr.ch_id > msg->cm_hdr.ch_id)
        {
          /* Prioritize tx frame based on canid */

          break;
        }
    }

  if (&tmp_node->list == &cd_sender->tx_pending)
    {
      /* Inserted at the end of the linked list */

      list_add_tail(&cd_sender->tx_pending, &msg_node->list);
    }
  else
    {
      list_add_before(&tmp_node->list, &msg_node->list);
    }
#else
  memcpy(&cd_sender->tx_buffer[cd_sender->tx_tail], msg, msglen);

  /* Increment the tail of the circular buffer */

  cd_sender->tx_tail++;
  if (cd_sender->tx_tail >= CONFIG_CAN_TXFIFOSIZE)
    {
      cd_sender->tx_tail = 0;
    }
#endif
}

/****************************************************************************
 * Name: can_get_msg
 *
 * Description:
 *   Get send message from sender.
 *
 ****************************************************************************/

FAR struct can_msg_s *can_get_msg(FAR struct can_txcache_s *cd_sender)
{
  FAR struct can_msg_s *msg = NULL;

#ifdef CONFIG_CAN_TXPRIORITY
  FAR struct can_msg_node_s *msg_node;
  FAR struct can_msg_node_s *tmp_node;

  if (list_is_empty(&cd_sender->tx_pending))
    {
      return NULL;
    }

  msg_node = list_first_entry(&cd_sender->tx_pending,
                              struct can_msg_node_s, list);
  msg = &msg_node->msg;

  /* Sort unconfirmed messages in ascending order */

  list_for_every_entry(&cd_sender->tx_sending, tmp_node,
                       struct can_msg_node_s, list)
    {
      if (tmp_node->msg.cm_hdr.ch_id == msg->cm_hdr.ch_id)
        {
          /* In order to prevent messages with the same ID from being
           * sent out of order, as long as there is a message with the
           * same ID that has not been sent in H/W, no data will be
           * written to H/W
           */

          return NULL;
        }

      if (tmp_node->msg.cm_hdr.ch_id > msg->cm_hdr.ch_id)
        {
          break;
        }
    }

  /* Move the node from tx_pending to tx_sending before
   * sending(because dev_send() might call can_txdone()).
   */

  list_delete(&msg_node->list);

  if (&tmp_node->list == &cd_sender->tx_sending)
    {
      list_add_tail(&cd_sender->tx_sending, &msg_node->list);
    }
  else
    {
      list_add_before(&tmp_node->list, &msg_node->list);
    }

#else
  msg = &cd_sender->tx_buffer[cd_sender->tx_queue];

  /* Increment the FIFO queue index before sending (because dev_send()
   * might call can_txdone()).
   */

  cd_sender->tx_queue++;

  if (cd_sender->tx_queue == CONFIG_CAN_TXFIFOSIZE)
    {
      cd_sender->tx_queue = 0;
    }
#endif

  return msg;
}

/****************************************************************************
 * Name: can_revert_msg
 *
 * Description:
 *   Rever msg in sender, because sending failed.
 *
 ****************************************************************************/

void can_revert_msg(FAR struct can_txcache_s *cd_sender,
                    FAR struct can_msg_s *msg)
{
#ifdef CONFIG_CAN_TXPRIORITY
  FAR struct can_msg_node_s *msg_node;

  msg_node = container_of(msg, struct can_msg_node_s, msg);

  list_delete(&msg_node->list);

  list_add_head(&cd_sender->tx_pending, &msg_node->list);
#else
  UNUSED(msg);
  if (cd_sender->tx_queue == 0)
    {
      cd_sender->tx_queue = CONFIG_CAN_TXFIFOSIZE - 1;
    }
  else
    {
      cd_sender->tx_queue--;
    }
#endif
}

/****************************************************************************
 * Name: can_send_done
 *
 * Description:
 *   Release the sender resources, after the tragic message is successfully
 *   sent to the bus.
 *
 ****************************************************************************/

void can_send_done(FAR struct can_txcache_s *cd_sender)
{
#ifdef CONFIG_CAN_TXPRIORITY
  FAR struct list_node *node;

  node = list_remove_head(&cd_sender->tx_sending);
  list_add_head(&cd_sender->tx_free, node);
#else
  /* Remove the message at the head of the xmit FIFO */

  cd_sender->tx_head++;
  if (cd_sender->tx_head >= CONFIG_CAN_TXFIFOSIZE)
    {
      cd_sender->tx_head = 0;
    }
#endif
}
