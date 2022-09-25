/****************************************************************************
 * net/pkt/pkt.h
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

#ifndef __NET_PKT_PKT_H
#define __NET_PKT_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/net/net.h>

#ifdef CONFIG_NET_PKT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new packet socket data callback */

#define pkt_callback_alloc(dev,conn) \
  devif_callback_alloc(dev, &conn->sconn.list, &conn->sconn.list_tail)
#define pkt_callback_free(dev,conn,cb) \
  devif_conn_callback_free(dev, cb, &conn->sconn.list, &conn->sconn.list_tail)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a packet socket connection */

struct devif_callback_s; /* Forward reference */

struct pkt_conn_s
{
  /* Common prologue of all connection structures. */

  struct socket_conn_s sconn;

  /* Pkt socket-specific content follows */

  uint8_t    lmac[6];  /* The local Ethernet address in network byte order */
  uint8_t    ifindex;
  uint16_t   proto;
  uint8_t    crefs;    /* Reference counts on this instance */
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

/* The packet socket interface */

EXTERN const struct sock_intf_s g_pkt_sockif;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
struct eth_hdr_s;    /* Forward reference */
struct socket;       /* Forward reference */

/****************************************************************************
 * Name: pkt_initialize()
 *
 * Description:
 *   Initialize the packet socket connection structures.  Called once and
 *   only from the network initialization logic.
 *
 ****************************************************************************/

void pkt_initialize(void);

/****************************************************************************
 * Name: pkt_alloc()
 *
 * Description:
 *   Allocate a new, uninitialized packet socket connection structure. This
 *   is normally something done by the implementation of the socket() API
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_alloc(void);

/****************************************************************************
 * Name: pkt_free()
 *
 * Description:
 *   Free a packet socket connection structure that is no longer in use.
 *   This should be done by the implementation of close().
 *
 ****************************************************************************/

void pkt_free(FAR struct pkt_conn_s *conn);

/****************************************************************************
 * Name: pkt_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate
 *   connection to be used with the provided Ethernet header
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_active(FAR struct eth_hdr_s *buf);

/****************************************************************************
 * Name: pkt_nextconn()
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct pkt_conn_s *pkt_nextconn(FAR struct pkt_conn_s *conn);

/****************************************************************************
 * Name: pkt_callback
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

uint16_t pkt_callback(FAR struct net_driver_s *dev,
                      FAR struct pkt_conn_s *conn, uint16_t flags);

/****************************************************************************
 * Name: pkt_input
 *
 * Description:
 *   Handle incoming packet input
 *
 *   This function provides the interface between Ethernet device drivers and
 *   packet socket logic.  All frames that are received should be provided to
 *   pkt_input() prior to other routing.
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK    The packet has been processed  and can be deleted
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Useful when a packet arrives before a recv call is in
 *         place.
 *
 * Assumptions:
 *   Called from the network diver with the network locked.
 *
 ****************************************************************************/

/* pkt_input() is prototyped in include/nuttx/net/pkt.h */

/****************************************************************************
 * Name: pkt_recvmsg
 *
 * Description:
 *   Implements the socket recvmsg interface for the case of the AF_INET
 *   and AF_INET6 address families.  pkt_recvmsg() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'msg_name' is not NULL, and the underlying protocol provides the
 *   source address, this source address is filled in.  The argument
 *   'msg_namelen' is initialized to the size of the buffer associated with
 *   msg_name, and modified on return to indicate the actual size of the
 *   address stored there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   msg      Buffer to receive the message
 *   flags    Receive flags
 *
 * Returned Value:
 *   On success, returns the number of characters received. If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recvmsg() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvmsg() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t pkt_recvmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags);

/****************************************************************************
 * Name: pkt_find_device
 *
 * Description:
 *   Select the network driver to use with the PKT transaction.
 *
 * Input Parameters:
 *   conn - PKT connection structure (not currently used).
 *
 * Returned Value:
 *   A pointer to the network driver to use.
 *
 ****************************************************************************/

FAR struct net_driver_s *pkt_find_device(FAR struct pkt_conn_s *conn);

/****************************************************************************
 * Name: pkt_poll
 *
 * Description:
 *   Poll a packet "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The packet "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called from the network device interface (devif) with the network
 *   locked.
 *
 ****************************************************************************/

void pkt_poll(FAR struct net_driver_s *dev, FAR struct pkt_conn_s *conn);

/****************************************************************************
 * Name: pkt_sendmsg
 *
 * Description:
 *   The pkt_sendmsg() call may be used only when the packet socket is in
 *   a connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   msg      Message to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error, a negated
 *   errno value is returned (see sendmsg() for the complete list of return
 *   values.
 *
 ****************************************************************************/

ssize_t pkt_sendmsg(FAR struct socket *psock, FAR struct msghdr *msg,
                    int flags);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_PKT */
#endif /* __NET_PKT_PKT_H */
