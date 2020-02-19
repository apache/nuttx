/****************************************************************************
 * net/can/can.h
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

#ifndef __NET_CAN_CAN_H
#define __NET_CAN_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <poll.h>

#include <netpacket/can.h>
#include <nuttx/semaphore.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new packet socket data callback */

#define can_callback_alloc(dev,conn) \
  devif_callback_alloc(dev, &conn->list)
#define can_callback_free(dev,conn,cb) \
  devif_conn_callback_free(dev, cb, &conn->list)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* This "connection" structure describes the underlying state of the socket. */

struct can_conn_s
{
  /* Common prologue of all connection structures. */

  dq_entry_t node;                   /* Supports a doubly linked list */

  /* This is a list of NetLink connection callbacks.  Each callback
   * represents a thread that is stalled, waiting for a device-specific
   * event.
   */

  FAR struct devif_callback_s *list; /* NetLink callbacks */

  FAR struct net_driver_s *dev;      /* Reference to CAN device */

  /* CAN-specific content follows */

  uint8_t protocol;                  /* Selected CAN protocol */
  int16_t crefs;                     /* Reference count */

  /* poll() support */

  FAR sem_t *pollsem;                /* Used to wakeup poll() */
  FAR pollevent_t *pollevent;        /* poll() wakeup event */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

EXTERN const struct sock_intf_s g_can_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct sockaddr_can;  /* Forward reference */

/****************************************************************************
 * Name: can_initialize()
 *
 * Description:
 *   Initialize the NetLink connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void can_initialize(void);

/****************************************************************************
 * Name: can_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized NetLink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct can_conn_s *can_alloc(void);

/****************************************************************************
 * Name: can_free()
 *
 * Description:
 *   Free a NetLink connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void can_free(FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_nextconn()
 *
 * Description:
 *   Traverse the list of allocated NetLink connections
 *
 * Assumptions:
 *   This function is called from NetLink device logic.
 *
 ****************************************************************************/

FAR struct can_conn_s *can_nextconn(FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_callback
 *
 * Description:
 *   Inform the application holding the packet socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

uint16_t can_callback(FAR struct net_driver_s *dev,
                      FAR struct can_conn_s *conn, uint16_t flags);

/****************************************************************************
 * Name: can_poll
 *
 * Description:
 *   Poll a CAN connection structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The CAN "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from network stack logic with the network stack locked
 *
 ****************************************************************************/

void can_poll(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn);

/****************************************************************************
 * Name: can_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection for the
 *   provided NetLink address
 *
 ****************************************************************************/

FAR struct can_conn_s *can_active(FAR struct sockaddr_can *addr);

/****************************************************************************
 * Name: psock_can_send
 *
 * Description:
 *   The psock_can_send() call may be used only when the packet socket is in
 *   a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is returned.  See send() for the complete list
 *   of return values.
 *
 ****************************************************************************/

struct socket;
ssize_t psock_can_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len);


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_CAN */
#endif /* __NET_CAN_CAN_H */
