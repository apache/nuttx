/****************************************************************************
 * net/netlink/netlink.h
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

#ifndef __NET_NETLINK_NETLINK_H
#define __NET_NETLINK_NETLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>
#include <semaphore.h>

#include "devif/devif.h"
#include "socket/socket.h"

#ifdef CONFIG_NET_NETLINK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct netlink_conn_s
{
  dq_entry_t node;                   /* Supports a doubly linked list */
  uint8_t    crefs;                  /* Reference counts on this instance */

  /* Defines the list of netlink callbacks */

  FAR struct devif_callback_s *list;
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

EXTERN const struct sock_intf_s g_netlink_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: netlink_initialize()
 *
 * Description:
 *   Initialize the User Socket connection structures.  Called once and only
 *   from the networking layer.
 *
 ****************************************************************************/

void netlink_initialize(void);

/****************************************************************************
 * Name: netlink_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized netlink connection structure.  This is
 *   normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_alloc(void);

/****************************************************************************
 * Name: netlink_free()
 *
 * Description:
 *   Free a netlink connection structure that is no longer in use. This should
 *   be done by the implementation of close().
 *
 ****************************************************************************/

void netlink_free(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_nextconn()
 *
 * Description:
 *   Traverse the list of allocated netlink connections
 *
 * Assumptions:
 *   This function is called from netlink device logic.
 *
 ****************************************************************************/

FAR struct netlink_conn_s *netlink_nextconn(FAR struct netlink_conn_s *conn);

/****************************************************************************
 * Name: netlink_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection for the
 *   provided netlink address
 *
 * Assumptions:
 *
 ****************************************************************************/

struct sockaddr_nl;  /* Forward reference */
FAR struct netlink_conn_s *netlink_active(FAR struct sockaddr_nl *addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_NETLINK */
#endif /* __NET_NETLINK_NETLINK_H */
