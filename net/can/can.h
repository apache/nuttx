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

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_CAN

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
 * Name: can_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection for the
 *   provided NetLink address
 *
 ****************************************************************************/

FAR struct can_conn_s *can_active(FAR struct sockaddr_can *addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_CAN */
#endif /* __NET_CAN_CAN_H */
