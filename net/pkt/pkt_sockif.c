/****************************************************************************
 * net/socket/pkt_sockif.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "pkt/pkt.h"

#ifdef CONFIG_NET_PKT

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pkt_setup(FAR struct socket *psock, int protocol);
static int     pkt_bind(FAR struct socket *psock,
                  FAR const struct sockaddr *addr, socklen_t addrlen);
static int     pkt_connect(FAR struct socket *psock,
                  FAR const struct sockaddr *addr, socklen_t addrlen);
static ssize_t pkt_send(FAR struct socket *psock, FAR const void *buf,
                 size_t len, int flags);
static ssize_t pkt_sendto(FAR struct socket *psock, FAR const void *buf,
                 size_t len, int flags, FAR const struct sockaddr *to,
                 socklen_t tolen);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_pkt_sockif =
{
  pkt_setup,    /* si_setup */
  pkt_bind,     /* si_bind */
  pkt_connect,  /* si_connect */
  pkt_send,     /* si_send */
  pkt_sendto,   /* si_sendto */
  pkt_recvfrom  /* si_recvfrom */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_sockif_alloc
 *
 * Description:
 *   Allocate and attach a raw packet connection structure.
 *
 ****************************************************************************/

static int pkt_sockif_alloc(FAR struct socket *psock)
{
  /* Allocate the packet socket connection structure and save in the new
   * socket instance.
   */

  FAR struct pkt_conn_s *conn = pkt_alloc();
  if (conn == NULL)
    {
      /* Failed to reserve a connection structure */

      return -ENOMEM;
    }

  /* Set the reference count on the connection structure.  This reference
   * count will be incremented only if the socket is dup'ed
   */

  DEBUGASSERT(conn->crefs == 0);
  conn->crefs = 1;

  /* Save the pre-allocated connection in the socket structure */

  psock->s_conn = conn;
  return OK;
}

/****************************************************************************
 * Name: pkt_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 * Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negater errno value is
 *   returned.
 *
 ****************************************************************************/

static int pkt_setup(FAR struct socket *psock, int protocol)
{
  /* Allocate the appropriate connection structure.  This reserves the
   * the connection structure is is unallocated at this point.  It will
   * not actually be initialized until the socket is connected.
   *
   * Only SOCK_RAW is supported.
   */

  if (psock->s_type == SOCK_RAW)
    {
      return pkt_sockif_alloc(psock);
    }
  else
    {
      return -EPROTONOSUPPORT;
    }
}

/****************************************************************************
 * Name: pkt_connect
 *
 * Description:
 *   pkt_connect() connects the local socket referred to by the structure
 *   'psock' to the address specified by 'addr'. The addrlen argument
 *   specifies the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'psock'.
 *
 *   If the socket 'psock' is of type SOCK_DGRAM then 'addr' is the address
 *   to which datagrams are sent by default, and the only address from which
 *   datagrams are received. If the socket is of type SOCK_STREAM or
 *   SOCK_SEQPACKET, this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully
 *   pkt_connect() only once; connectionless protocol sockets may use
 *   pkt_connect() multiple times to change their association.
 *   Connectionless sockets may dissolve the association by connecting to
 *   an address with the sa_family member of sockaddr set to AF_UNSPEC.
 *
 * Parameters:
 *   psock     Pointer to a socket structure initialized by psock_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; a negated errno value on failue.  See connect() for the
 *   list of appropriate errno values to be returned.
 *
 ****************************************************************************/

static int pkt_connect(FAR struct socket *psock,
                       FAR const struct sockaddr *addr, socklen_t addrlen)
{
  return -EAFNOSUPPORT;
}

/****************************************************************************
 * Name: pkt_bind
 *
 * Description:
 *   pkt_bind() gives the socket 'psock' the local address 'addr'.  'addr'
 *   is 'addrlen' bytes long.  Traditionally, this is called "assigning a
 *   name to a socket."  When a socket is created with socket(), it exists
 *   in a name space (address family) but has no name assigned.
 *
 * Parameters:
 *   psock    Socket structure of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success;  A negated errno value is returned on failure.  See
 *   bind() for a list a appropriate error values.
 *
 ****************************************************************************/

static int pkt_bind(FAR struct socket *psock, FAR const struct sockaddr *addr,
                    socklen_t addrlen)
{
#if 0
  char hwaddr[6] =  /* our MAC for debugging */
  {
    0x00, 0xa1, 0xb1, 0xc1, 0xd1, 0xe1
  };
#endif
  char hwaddr[6] =  /* MAC from ifconfig */
  {
    0x00, 0xe0, 0xde, 0xad, 0xbe, 0xef
  };
  int ifindex;

  /* Verify that a valid address has been provided */

  if (addr->sa_family != AF_PACKET || addrlen < sizeof(struct sockaddr_ll)
    {
      nerr("ERROR: Invalid address length: %d < %d\n",
           addrlen, sizeof(struct sockaddr_ll);
      return -EBADF;
    }

  /* Bind a raw socket to an network device. */

  if (psock->s_type == SOCK_RAW)
    {
      FAR struct pkt_conn_s *conn = (FAR struct pkt_conn_s *)psock->s_conn;

      /* Look at the addr and identify network interface */

      ifindex = addr->sll_ifindex;

#if 0
      /* Get the MAC address of that interface */

      memcpy(hwaddr, g_netdevices->d_mac.ether, 6);
#endif

      /* Put ifindex and mac address into connection */

      conn->ifindex = ifindex;
      memcpy(conn->lmac, hwaddr, 6);

      /* Mark the socket bound */

      psock->s_flags |= _SF_BOUND;
      return OK;
    }
  else
    {
      return -EBADF;
    }
}

/****************************************************************************
 * Name: pkt_send
 *
 * Description:
 *   Socket send() method for the raw packet socket.
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t pkt_send(FAR struct socket *psock, FAR const void *buf,
                        size_t len, int flags)
{
  ssize_t ret;

  /* Only SOCK_RAW is supported */

  if (psock->s_type == SOCK_RAW)
    {
      /* Raw packet send */

      ret = psock_pkt_send(psock, buf, len);
    }
  else
    {
      /* EDESTADDRREQ.  Signifies that the socket is not connection-mode and no peer
       * address is set.
       */

      ret = -EDESTADDRREQ;
    }

  return ret;
}

/****************************************************************************
 * Name: pkt_sendto
 *
 * Description:
 *   Implements the sendto() operation for the case of the raw packet socket.
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
 *   On success, returns the number of characters sent.  On  error, a negated
 *   errno value is returned (see send_to() for the list of appropriate error
 *   values.
 *
 ****************************************************************************/

static ssize_t pkt_sendto(FAR struct socket *psock, FAR const void *buf,
                          size_t len, int flags, FAR const struct sockaddr *to,
                          socklen_t tolen)
{
  nerr("ERROR: sendto() not supported for raw packet sockets\n");
  return -EAFNOSUPPORT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: 
 *
 * Description:
 *
 * Parameters:
 *
 * Returned Value:
 *
 * Assumptions:
 *
 ****************************************************************************/

#endif /* CONFIG_NET_PKT */
