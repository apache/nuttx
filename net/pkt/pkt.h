/****************************************************************************
 * net/pkt/pkt.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef _NET_PKT_PKT_H
#define _NET_PKT_PKT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <queue.h>

#ifdef CONFIG_NET_PKT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate a new packet socket data callback */

#define pkt_callback_alloc(dev,conn) \
  devif_callback_alloc(dev, &conn->list)
#define pkt_callback_free(dev,conn,cb) \
  devif_conn_callback_free(dev, cb, &conn->list)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a packet socket connection */

struct devif_callback_s; /* Forward reference */

struct pkt_conn_s
{
  dq_entry_t node;     /* Supports a double linked list */
  uint8_t    lmac[6];  /* The local Ethernet address in network byte order */
  uint8_t    ifindex;
  uint16_t   proto;
  uint8_t    crefs;    /* Reference counts on this instance */

  /* Defines the list of packet callbacks */

  struct devif_callback_s *list;
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
 * Name: pkt_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   and AF_INET6 address families.  pkt_recvfrom() receives messages from
 *   a socket, and may be used to receive data on a socket whether or not it
 *   is connection-oriented.
 *
 *   If 'from' is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in.  The argument 'fromlen' is
 *   initialized to the size of the buffer associated with from, and
 *   modified on return to indicate the actual size of the address stored
 *   there.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received.  If no data is
 *   available to be received and the peer has performed an orderly shutdown,
 *   recv() will return 0.  Otherwise, on errors, a negated errno value is
 *   returned (see recvfrom() for the list of appropriate error values).
 *
 ****************************************************************************/

ssize_t pkt_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                     int flags, FAR struct sockaddr *from,
                     FAR socklen_t *fromlen);

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
 * Name: psock_pkt_send
 *
 * Description:
 *   The psock_pkt_send() call may be used only when the packet socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Input Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   a negated errno value is retruend.  See send() for the complete list
 *   of return values.
 *
 ****************************************************************************/

struct socket;
ssize_t psock_pkt_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_PKT */
#endif /* _NET_PKT_PKT_H */
