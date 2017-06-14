/****************************************************************************
 * net/sixlowpan/sixlowpan.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef _NET_SIXLOWPAN_SIXLOWPAN_H
#define _NET_SIXLOWPAN_SIXLOWPAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
struct socket;       /* Forward reference */
struct sockaddr;     /* Forward reference */

/****************************************************************************
 * Name: sixlowpan_initialize
 *
 * Description:
 *   sixlowpan_initialize() is called during OS initialization at power-up
 *   reset.  It is called from the common net_setup() function.
 *   sixlowpan_initialize() configures 6loWPAN networking data structures.
 *   It is called prior to platform-specific driver initialization so that
 *   the 6loWPAN networking subsystem is prepared to deal with network
 *   driver initialization actions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sixlowpan_initialize(void);

/****************************************************************************
 * Name: psock_6lowpan_tcp_send
 *
 * Description:
 *   psock_6lowpan_tcp_send() call may be used only when the TCP socket is in a
 *   connected state (so that the intended recipient is known).
 *
 * Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   len   - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send() or
 *   sendto().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
ssize_t psock_6lowpan_tcp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len);
#endif

/****************************************************************************
 * Name: sixlowpan_tcp_send
 *
 * Description:
 *   TCP output comes through three different mechansims.  Either from:
 *
 *   1. TCP socket output.  For the case of TCP output to an
 *      IEEE802.15.4, the TCP output is caught in the socket
 *      send()/sendto() logic and and redirected to psock_6lowpan_tcp_send().
 *   2. TCP output from the TCP state machine.  That will occur
 *      during TCP packet processing by the TCP state meachine.
 *   3. TCP output resulting from TX or timer polling
 *
 *   Cases 2 and 3 will be handled here.  Logic in ipv6_tcp_input(),
 *   devif_poll(), and devif_timer() detect if (1) an attempt to return with
 *   d_len > 0 and (2) that the device is an IEEE802.15.4 MAC network
 *   driver. Under those conditions, this function will be called to create
 *   the IEEE80215.4 frames.
 *
 * Parameters:
 *   dev - An instance of nework device state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void sixlowpan_tcp_send(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: psock_6lowpan_udp_send
 *
 * Description:
 *   psock_6lowpan_udp_send() call may be used with connectionlesss UDP
 *   sockets.
 *
 * Parameters:
 *   psock - An instance of the internal socket structure.
 *   buf   - Data to send
 *   len   - Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.  Returned error numbers
 *   must be consistent with definition of errors reported by send() or
 *   sendto().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
ssize_t psock_6lowpan_udp_send(FAR struct socket *psock, FAR const void *buf,
                               size_t len);
#endif

/****************************************************************************
 * Name: psock_6lowpan_udp_sendto
 *
 * Description:
 *   If sendto() is used on a connection-mode (SOCK_STREAM, SOCK_SEQPACKET)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error,
 *   -1 is returned, and errno is set appropriately.    Returned error
 *   number must be consistent with definition of errors reported by
 *   sendto().
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

ssize_t psock_6lowpan_udp_sendto(FAR struct socket *psock,
                                 FAR const void *buf,
                                 size_t len, int flags,
                                 FAR const struct sockaddr *to,
                                 socklen_t tolen);

#endif /* CONFIG_NET_6LOWPAN */
#endif /* _NET_SIXLOWPAN_SIXLOWPAN_H */
