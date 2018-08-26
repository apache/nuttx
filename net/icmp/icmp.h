/****************************************************************************
 * net/icmp/icmp.h
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

#ifndef __NET_ICMP_ICMP_H
#define __NET_ICMP_ICMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <queue.h>
#include <assert.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_ICMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allocate/free an ICMP data callback */

#define icmp_callback_alloc(dev)   devif_callback_alloc(dev, &(dev)->d_conncb)
#define icmp_callback_free(dev,cb) devif_dev_callback_free(dev, cb)

/****************************************************************************
 * Public types
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
/* Representation of a IPPROTO_ICMP socket connection */

struct devif_callback_s; /* Forward reference */

struct icmp_conn_s
{
  dq_entry_t node;     /* Supports a double linked list */
  uint16_t   id;       /* ICMP ECHO request ID */
  uint8_t    nreqs;    /* Number of requests with no response received */
  uint8_t    crefs;    /* Reference counts on this instance */

  /* The device that the ICMP request was sent on */

  FAR struct net_driver_s *dev;  /* Needed to free the callback structure */

#ifdef CONFIG_MM_IOB
  /* ICMP response read-ahead list.  A singly linked list of type struct
   * iob_qentry_s where the ICMP read-ahead data for the current ID is
   * retained.
   */

  struct iob_queue_s readahead;  /* Read-ahead buffering */
#endif
};
#endif

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

#ifdef CONFIG_NET_ICMP_SOCKET
/* PF_INET socket address family, IPPROTO_ICMP protocol interface */

EXTERN const struct sock_intf_s g_icmp_sockif;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct socket;    /* Forward reference */
struct sockaddr;  /* Forward reference */
struct pollfd;    /* Forward reference */

/****************************************************************************
 * Name: icmp_input
 *
 * Description:
 *   Handle incoming ICMP input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received ICMP
 *         packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmp_input(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: icmp_sock_initialize
 *
 * Description:
 *   Initialize the IPPROTO_ICMP socket connection structures.  Called once
 *   and only from the network initialization layer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_sock_initialize(void);
#endif

/****************************************************************************
 * Name: icmp_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_alloc(void);
#endif

/****************************************************************************
 * Name: icmp_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_free(FAR struct icmp_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmp_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_active(uint16_t id);
#endif

/****************************************************************************
 * Name: icmp_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_nextconn(FAR struct icmp_conn_s *conn);
#endif

/****************************************************************************
 * Name: icmp_findconn
 *
 * Description:
 *   Find an ICMP connection structure that is expecting a ICMP ECHO response
 *   with this ID from this device
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
FAR struct icmp_conn_s *icmp_findconn(FAR struct net_driver_s *dev,
                                      uint16_t id);
#endif

/****************************************************************************
 * Name: icmp_poll
 *
 * Description:
 *   Poll a device "connection" structure for availability of ICMP TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
void icmp_poll(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmp_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the IPPROTO_ICMP
 *   socket.  The 'buf' parameter points to a block of memory that includes
 *   an ICMP request header, followed by any payload that accompanies the
 *   request.  The 'len' parameter includes both the size of the ICMP header
 *   and the following payload.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send_to() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
ssize_t icmp_sendto(FAR struct socket *psock, FAR const void *buf,
                    size_t len, int flags, FAR const struct sockaddr *to,
                    socklen_t tolen);
#endif

/****************************************************************************
 * Name: icmp_recvfrom
 *
 * Description:
 *   Implements the socket recvfrom interface for the case of the AF_INET
 *   data gram socket with the IPPROTO_ICMP protocol.  icmp_recvfrom()
 *   receives ICMP ECHO replies for the a socket.
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

#ifdef CONFIG_NET_ICMP_SOCKET
ssize_t icmp_recvfrom(FAR struct socket *psock, FAR void *buf, size_t len,
                      int flags, FAR struct sockaddr *from,
                      FAR socklen_t *fromlen);
#endif

/****************************************************************************
 * Name: icmp_pollsetup
 *
 * Description:
 *   Setup to monitor events on one ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
int icmp_pollsetup(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

/****************************************************************************
 * Name: icmp_pollteardown
 *
 * Description:
 *   Teardown monitoring of events on an ICMP socket
 *
 * Input Parameters:
 *   psock - The IPPROTO_ICMP socket of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
int icmp_pollteardown(FAR struct socket *psock, FAR struct pollfd *fds);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_ICMP */
#endif /* __NET_ICMP_ICMP_H */
