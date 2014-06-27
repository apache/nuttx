/****************************************************************************
 * net/tcp/tcp.h
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

#ifndef _NET_TCP_TCP_H
#define _NET_TCP_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#ifdef CONFIG_NET_TCP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Defined in tcp_conn.c ****************************************************/

struct tcp_iphdr_s; /* Forward reference */

void tcp_initialize(void);
struct tcp_conn_s *tcp_active(FAR struct tcp_iphdr_s *buf);
struct tcp_conn_s *uip_nexttcpconn(FAR struct tcp_conn_s *conn);
struct tcp_conn_s *tcp_listener(uint16_t portno);
struct tcp_conn_s *tcp_alloc_accept(FAR struct tcp_iphdr_s *buf);

/* Defined in tcp_seqno.c ***************************************************/

void tcp_setsequence(FAR uint8_t *seqno, uint32_t value);
uint32_t tcp_getsequence(FAR uint8_t *seqno);
uint32_t tcp_addsequence(FAR uint8_t *seqno, uint16_t len);
void tcp_initsequence(FAR uint8_t *seqno);
void tcp_nextsequence(void);

/* Defined in tcp_poll.c ****************************************************/

void tcp_poll(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn);

/* Defined in tcp_timer.c ***************************************************/

void tcp_timer(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
               int hsec);

/* Defined in tcp_listen.c **************************************************/

void tcp_listeninit(void);
bool tcp_islistener(uint16_t port);
int tcp_accept_connection(FAR struct net_driver_s *dev,
                          FAR struct tcp_conn_s *conn, uint16_t portno);

/* Defined in tcp_send.c ****************************************************/

void tcp_send(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
              uint16_t flags, uint16_t len);
void tcp_reset(FAR struct net_driver_s *dev);
void tcp_ack(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
             uint8_t ack);

/* Defined in tcp_appsend.c *************************************************/

void tcp_appsend(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                 uint16_t result);
void tcp_rexmit(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn,
                uint16_t result);

/* Defined in tcp_input.c ***************************************************/

void tcp_input(FAR struct net_driver_s *dev);

/* Defined in tcp_callback.c ************************************************/

uint16_t tcp_callback(FAR struct net_driver_s *dev,
                      FAR struct tcp_conn_s *conn, uint16_t flags);
#ifdef CONFIG_NET_TCP_READAHEAD
uint16_t tcp_datahandler(FAR struct tcp_conn_s *conn,
                         FAR uint8_t *buffer, uint16_t nbytes);
#endif

/****************************************************************************
 * Function: psock_tcp_send
 *
 * Description:
 *   The psock_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately:
 *
 *   EAGAIN or EWOULDBLOCK
 *     The socket is marked non-blocking and the requested operation
 *     would block.
 *   EBADF
 *     An invalid descriptor was specified.
 *   ECONNRESET
 *     Connection reset by peer.
 *   EDESTADDRREQ
 *     The socket is not connection-mode, and no peer address is set.
 *   EFAULT
 *      An invalid user space address was specified for a parameter.
 *   EINTR
 *      A signal occurred before any data was transmitted.
 *   EINVAL
 *      Invalid argument passed.
 *   EISCONN
 *     The connection-mode socket was connected already but a recipient
 *     was specified. (Now either this error is returned, or the recipient
 *     specification is ignored.)
 *   EMSGSIZE
 *     The socket type requires that message be sent atomically, and the
 *     size of the message to be sent made this impossible.
 *   ENOBUFS
 *     The output queue for a network interface was full. This generally
 *     indicates that the interface has stopped sending, but may be
 *     caused by transient congestion.
 *   ENOMEM
 *     No memory available.
 *   ENOTCONN
 *     The socket is not connected, and no target has been given.
 *   ENOTSOCK
 *     The argument s is not a socket.
 *   EPIPE
 *     The local end has been shut down on a connection oriented socket.
 *     In this case the process will also receive a SIGPIPE unless
 *     MSG_NOSIGNAL is set.
 *
 * Assumptions:
 *
 ****************************************************************************/

struct socket;
ssize_t psock_tcp_send(FAR struct socket *psock, FAR const void *buf,
                       size_t len);

/****************************************************************************
 * Function: tcp_wrbuffer_initialize
 *
 * Description:
 *   Initialize the list of free write buffers
 *
 * Assumptions:
 *   Called once early initialization.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_initialize(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_alloc
 *
 * Description:
 *   Allocate a TCP write buffer by taking a pre-allocated buffer from
 *   the free list.  This function is called from TCP logic when a buffer
 *   of TCP data is about to sent
 *
 * Input parameters:
 *   None
 *
 * Assumptions:
 *   Called from user logic with interrupts enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
struct tcp_wrbuffer_s;

FAR struct tcp_wrbuffer_s *tcp_wrbuffer_alloc(void);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_release
 *
 * Description:
 *   Release a TCP write buffer by returning the buffer to the free list.
 *   This function is called from user logic after it is consumed the
 *   buffered data.
 *
 * Assumptions:
 *   Called from interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
void tcp_wrbuffer_release(FAR struct tcp_wrbuffer_s *wrb);
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

/****************************************************************************
 * Function: tcp_wrbuffer_dump
 *
 * Description:
 *   Dump the contents of a write buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
#ifdef CONFIG_DEBUG
void tcp_wrbuffer_dump(FAR const char *msg, FAR struct tcp_wrbuffer_s *wrb,
                       unsigned int len, unsigned int offset);
#else
#  define tcp_wrbuffer_dump(msg,wrb)
#endif
#endif /* CONFIG_NET_TCP_WRITE_BUFFERS */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_NET_TCP */
#endif /* _NET_TCP_TCP_H */
