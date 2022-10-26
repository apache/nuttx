/****************************************************************************
 * net/udp/udp_recvfrom.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/semaphore.h>
#include <nuttx/net/net.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>
#include <netinet/in.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "udp/udp.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct udp_recvfrom_s
{
  FAR struct udp_conn_s   *ir_conn;      /* Connection associated with the socket */
  FAR struct devif_callback_s *ir_cb;    /* Reference to callback instance */
  FAR struct msghdr       *ir_msg;       /* Receive info and buffer */
  sem_t                    ir_sem;       /* Semaphore signals recv completion */
  ssize_t                  ir_recvlen;   /* The received length */
  int                      ir_result;    /* Success:OK, failure:negated errno */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void udp_recvpktinfo(FAR struct udp_recvfrom_s *pstate,
                            FAR void *srcaddr, uint8_t ifindex)
{
  FAR struct msghdr     *msg      = pstate->ir_msg;
  FAR struct udp_conn_s *conn     = pstate->ir_conn;
  FAR struct cmsghdr    *control  = msg->msg_control;
  size_t                 cmsg_len = 0;

  if (!(conn->flags & _UDP_FLAG_PKTINFO))
    {
      goto out;
    }

#ifdef CONFIG_NET_IPv4
  if (conn->domain == PF_INET)
    {
      FAR struct sockaddr_in *infrom  = srcaddr;
      FAR struct in_pktinfo *pkt_info = CMSG_DATA(control);

      if (msg->msg_controllen < CMSG_LEN(sizeof(struct in_pktinfo)))
        {
          goto out;
        }

      cmsg_len                      = CMSG_LEN(sizeof(struct in_pktinfo));
      control->cmsg_level           = IPPROTO_IP;
      control->cmsg_type            = IP_PKTINFO;
      control->cmsg_len             = cmsg_len;
      pkt_info->ipi_ifindex         = ifindex;
      pkt_info->ipi_addr.s_addr     = infrom->sin_addr.s_addr;
      pkt_info->ipi_spec_dst.s_addr = conn->u.ipv4.laddr;
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (conn->domain == PF_INET6)
    {
      FAR struct sockaddr_in6 *infrom  = srcaddr;
      FAR struct in6_pktinfo *pkt_info = CMSG_DATA(control);

      if (msg->msg_controllen < CMSG_LEN(sizeof(struct in6_pktinfo)))
        {
          goto out;
        }

      cmsg_len               = CMSG_LEN(sizeof(struct in6_pktinfo));
      control->cmsg_level    = IPPROTO_IPV6;
      control->cmsg_type     = IPV6_PKTINFO;
      control->cmsg_len      = cmsg_len;
      pkt_info->ipi6_ifindex = ifindex;
      net_ipv6addr_copy(&pkt_info->ipi6_addr, infrom->sin6_addr.s6_addr);
    }
#endif

out:
  msg->msg_controllen = cmsg_len;
}

/****************************************************************************
 * Name: udp_recvfrom_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   The number of bytes taken from the packet.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static size_t udp_recvfrom_newdata(FAR struct net_driver_s *dev,
                                   FAR struct udp_recvfrom_s *pstate)
{
  size_t recvlen;

  /* Get the length of the data to return */

  if (dev->d_len > pstate->ir_msg->msg_iov->iov_len)
    {
      recvlen = pstate->ir_msg->msg_iov->iov_len;
    }
  else
    {
      recvlen = dev->d_len;
    }

  /* Copy the new appdata into the user buffer */

  memcpy(pstate->ir_msg->msg_iov->iov_base, dev->d_appdata, recvlen);
  ninfo("Received %zu bytes (of %" PRIu16 ")\n", recvlen, dev->d_len);

  /* Update the size of the data read */

  pstate->ir_recvlen = recvlen;
  return recvlen;
}

/****************************************************************************
 * Name: udp_newdata
 *
 * Description:
 *   Copy the read data from the packet
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event
 *   pstate   recvfrom state structure
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline void udp_newdata(FAR struct net_driver_s *dev,
                               FAR struct udp_recvfrom_s *pstate)
{
  /* Take as much data from the packet as we can */

  udp_recvfrom_newdata(dev, pstate);

  /* Indicate no data in the buffer */

  dev->d_len = 0;
}

static inline void udp_readahead(struct udp_recvfrom_s *pstate)
{
  FAR struct udp_conn_s *conn = pstate->ir_conn;
  FAR struct iob_s *iob;
  int recvlen;

  /* Check there is any UDP datagram already buffered in a read-ahead
   * buffer.
   */

  pstate->ir_recvlen = -1;

  if ((iob = iob_peek_queue(&conn->readahead)) != NULL)
    {
      FAR struct iob_s *tmp;
#ifdef CONFIG_NET_IPv6
      uint8_t srcaddr[sizeof(struct sockaddr_in6)];
#else
      uint8_t srcaddr[sizeof(struct sockaddr_in)];
#endif
      uint8_t src_addr_size;
      uint8_t offset  = 0;
      uint8_t ifindex = 0;

      DEBUGASSERT(iob->io_pktlen > 0);

      /* Transfer that buffered data from the I/O buffer chain into
       * the user buffer.
       */

      recvlen = iob_copyout(&src_addr_size, iob, sizeof(uint8_t), offset);
      offset += sizeof(uint8_t);
      if (recvlen != sizeof(uint8_t))
        {
          goto out;
        }

      recvlen = iob_copyout(srcaddr, iob, src_addr_size, offset);
      offset += src_addr_size;
      if (recvlen != src_addr_size)
        {
          goto out;
        }

#ifdef CONFIG_NETDEV_IFINDEX
      recvlen = iob_copyout(&ifindex, iob, sizeof(uint8_t), offset);
      offset += sizeof(uint8_t);
      if (recvlen != sizeof(uint8_t))
        {
          goto out;
        }
#endif

      if (pstate->ir_msg->msg_name)
        {
          pstate->ir_msg->msg_namelen =
                src_addr_size > pstate->ir_msg->msg_namelen ?
                pstate->ir_msg->msg_namelen : src_addr_size;

          memcpy(pstate->ir_msg->msg_name, srcaddr,
                 pstate->ir_msg->msg_namelen);
        }

      if (pstate->ir_msg->msg_iov->iov_len > 0)
        {
          recvlen = iob_copyout(pstate->ir_msg->msg_iov->iov_base,
                                iob, pstate->ir_msg->msg_iov->iov_len,
                                offset);

          ninfo("Received %d bytes (of %d)\n", recvlen, iob->io_pktlen);

          /* Update the accumulated size of the data read */

          pstate->ir_recvlen = recvlen;
        }
      else
        {
          pstate->ir_recvlen = 0;
        }

      udp_recvpktinfo(pstate, srcaddr, ifindex);

out:
      /* Remove the I/O buffer chain from the head of the read-ahead
       * buffer queue.
       */

      tmp = iob_remove_queue(&conn->readahead);
      DEBUGASSERT(tmp == iob);
      UNUSED(tmp);

      /* And free the I/O buffer chain */

      iob_free_chain(iob);
    }
}

/****************************************************************************
 * Name: udp_sender
 *
 * Description:
 *   Getting the sender's address from the UDP packet
 *
 * Input Parameters:
 *   dev    - The device driver data structure
 *   pstate - the recvfrom state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static inline void udp_sender(FAR struct net_driver_s *dev,
                              FAR struct udp_recvfrom_s *pstate)
{
#ifdef CONFIG_NET_IPv6
  uint8_t srcaddr[sizeof(struct sockaddr_in6)];
#else
  uint8_t srcaddr[sizeof(struct sockaddr_in)];
#endif
  socklen_t fromlen = 0;

  /* Get the family from the packet type, IP address from the IP header, and
   * the port number from the UDP header.
   */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct sockaddr_in6 *infrom = (FAR struct sockaddr_in6 *)srcaddr;
      FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      infrom->sin6_family = AF_INET6;
      infrom->sin6_port   = udp->srcport;
      fromlen = sizeof(struct sockaddr_in6);

      net_ipv6addr_copy(infrom->sin6_addr.s6_addr, ipv6->srcipaddr);
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
#ifdef CONFIG_NET_IPv6
      FAR struct udp_conn_s *conn = pstate->ir_conn;

      if (conn->domain == PF_INET6)
        {
          /* Hybrid dual-stack IPv6/IPv4 implementations recognize a special
           * class of addresses, the IPv4-mapped IPv6 addresses.
           */

          FAR struct sockaddr_in6 *infrom6 =
            (FAR struct sockaddr_in6 *)srcaddr;
          FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
          in_addr_t ipv4addr;

          /* Encode the IPv4 address as an IPv4-mapped IPv6 address */

          infrom6->sin6_family = AF_INET6;
          infrom6->sin6_port = udp->srcport;
          fromlen  = sizeof(struct sockaddr_in6);
          ipv4addr = net_ip4addr_conv32(ipv6->srcipaddr);
          ip6_map_ipv4addr(ipv4addr, infrom6->sin6_addr.s6_addr16);
        }
      else
#endif
        {
          FAR struct sockaddr_in *infrom = (FAR struct sockaddr_in *)srcaddr;
          FAR struct udp_hdr_s *udp   = UDPIPv4BUF;
          FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

          infrom->sin_family = AF_INET;
          infrom->sin_port   = udp->srcport;
          fromlen = sizeof(struct sockaddr_in);

          net_ipv4addr_copy(infrom->sin_addr.s_addr,
                            net_ip4addr_conv32(ipv4->srcipaddr));
          memset(infrom->sin_zero, 0, sizeof(infrom->sin_zero));
        }
    }
#endif /* CONFIG_NET_IPv4 */

  if (pstate->ir_msg->msg_name)
    {
      pstate->ir_msg->msg_namelen = fromlen > pstate->ir_msg->msg_namelen ?
                                    pstate->ir_msg->msg_namelen : fromlen;
      memcpy(pstate->ir_msg->msg_name, srcaddr, pstate->ir_msg->msg_namelen);
    }

#ifdef CONFIG_NETDEV_IFINDEX
  udp_recvpktinfo(pstate, srcaddr, dev->d_ifindex);
#else
  udp_recvpktinfo(pstate, srcaddr, 0);
#endif
}

/****************************************************************************
 * Name: udp_terminate
 *
 * Description:
 *   Terminate the UDP transfer.
 *
 * Input Parameters:
 *   pstate - The recvfrom state structure
 *   result - The result of the operation
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void udp_terminate(FAR struct udp_recvfrom_s *pstate, int result)
{
  /* Don't allow any further UDP call backs. */

  pstate->ir_cb->flags = 0;
  pstate->ir_cb->priv  = NULL;
  pstate->ir_cb->event = NULL;

  /* Save the result of the transfer */

  pstate->ir_result    = result;

  /* Wake up the waiting thread, returning the number of bytes
   * actually read.
   */

  nxsem_post(&pstate->ir_sem);
}

/****************************************************************************
 * Name: udp_eventhandler
 *
 * Description:
 *   This function is called with the network locked to perform the actual
 *   UDP receive operation via by the lower, device interfacing layer.
 *
 * Input Parameters:
 *   dev      The structure of the network driver that generated the event.
 *   pvpriv   An instance of struct udp_recvfrom_s cast to void*
 *   flags    Set of events describing why the callback was invoked
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static uint16_t udp_eventhandler(FAR struct net_driver_s *dev,
                                 FAR void *pvpriv, uint16_t flags)
{
  FAR struct udp_recvfrom_s *pstate = pvpriv;

  ninfo("flags: %04x\n", flags);

  /* 'priv' might be null in some race conditions (?) */

  if (pstate)
    {
      /* If the network device has gone down, then we will have terminate
       * the wait now with an error.
       */

      if ((flags & NETDEV_DOWN) != 0)
        {
          /* Terminate the transfer with an error. */

          nerr("ERROR: Network is down\n");
          udp_terminate(pstate, -ENETUNREACH);
        }

      /* If new data is available, then complete the read action. */

      else if ((flags & UDP_NEWDATA) != 0)
        {
          /* Copy the data from the packet */

          udp_newdata(dev, pstate);

          /* We are finished. */

          ninfo("UDP done\n");

          /* Save the sender's address in the caller's 'from' location */

          udp_sender(dev, pstate);

          /* Don't allow any further UDP call backs. */

          udp_terminate(pstate, OK);

          /* Indicate that the data has been consumed */

          flags &= ~UDP_NEWDATA;
        }
    }

  return flags;
}

/****************************************************************************
 * Name: udp_recvfrom_initialize
 *
 * Description:
 *   Initialize the state structure
 *
 * Input Parameters:
 *   conn     The UDP connection of interest
 *   msg      Receive info and buffer for receive data
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void udp_recvfrom_initialize(FAR struct udp_conn_s *conn,
                                    FAR struct msghdr *msg,
                                    FAR struct udp_recvfrom_s *pstate)
{
  /* Initialize the state structure. */

  memset(pstate, 0, sizeof(struct udp_recvfrom_s));
  nxsem_init(&pstate->ir_sem, 0, 0); /* Doesn't really fail */

  pstate->ir_msg  = msg;

  /* Set up the start time for the timeout */

  pstate->ir_conn = conn;
}

/* The only un-initialization that has to be performed is destroying the
 * semaphore.
 */

#define udp_recvfrom_uninitialize(s) nxsem_destroy(&(s)->ir_sem)

/****************************************************************************
 * Name: udp_recvfrom_result
 *
 * Description:
 *   Evaluate the result of the recv operations
 *
 * Input Parameters:
 *   result   The result of the net_timedwait operation (may indicate EINTR)
 *   pstate   A pointer to the state structure to be initialized
 *
 * Returned Value:
 *   The result of the recv operation with errno set appropriately
 *
 * Assumptions:
 *
 ****************************************************************************/

static ssize_t udp_recvfrom_result(int result, struct udp_recvfrom_s *pstate)
{
  /* Check for a error/timeout detected by the event handler.  Errors are
   * signaled by negative errno values for the rcv length
   */

  if (pstate->ir_result < 0)
    {
      /* This might return EAGAIN on a timeout or ENOTCONN on loss of
       * connection (TCP only)
       */

      return pstate->ir_result;
    }

  /* If net_timedwait failed, then we were probably reawakened by a signal.
   * In this case, net_timedwait will have returned negated errno
   * appropriately.
   */

  if (result < 0)
    {
      return result;
    }

  return pstate->ir_recvlen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_udp_recvfrom
 *
 * Description:
 *   Perform the recvfrom operation for a UDP SOCK_DGRAM
 *
 * Input Parameters:
 *   psock  Pointer to the socket structure for the SOCK_DRAM socket
 *   msg    Receive info and buffer for receive data
 *
 * Returned Value:
 *   On success, returns the number of characters received.  On  error,
 *   -errno is returned (see recvfrom for list of errnos).
 *
 * Assumptions:
 *
 ****************************************************************************/

ssize_t psock_udp_recvfrom(FAR struct socket *psock, FAR struct msghdr *msg,
                           int flags)
{
  FAR struct udp_conn_s *conn = (FAR struct udp_conn_s *)psock->s_conn;
  FAR struct net_driver_s *dev;
  struct udp_recvfrom_s state;
  int ret;

  /* Perform the UDP recvfrom() operation */

  /* Initialize the state structure.  This is done with the network locked
   * because we don't want anything to happen until we are ready.
   */

  net_lock();
  udp_recvfrom_initialize(conn, msg, &state);

  /* Copy the read-ahead data from the packet */

  udp_readahead(&state);

  /* The default return value is the number of bytes that we just copied
   * into the user buffer.  We will return this if the socket has become
   * disconnected or if the user request was completely satisfied with
   * data from the readahead buffers.
   */

  ret = state.ir_recvlen;

  /* Handle non-blocking UDP sockets */

  if (_SS_ISNONBLOCK(conn->sconn.s_flags) || (flags & MSG_DONTWAIT) != 0)
    {
      /* Return the number of bytes read from the read-ahead buffer if
       * something was received (already in 'ret'); EAGAIN if not.
       */

      if (ret < 0)
        {
          /* Nothing was received */

          ret = -EAGAIN;
        }
    }

  /* It is okay to block if we need to.  If there is space to receive
   * anything more, then we will wait to receive the data. Otherwise
   * return the number of bytes read from the read-ahead buffer
   * (already in 'ret').
   *
   * NOTE: that udp_readahead() may set state.ir_recvlen == -1.
   */

  else if (state.ir_recvlen <= 0)
    {
      /* Get the device that will handle the packet transfers.  This may be
       * NULL if the UDP socket is bound to INADDR_ANY.  In that case, no
       * NETDEV_DOWN notifications will be received.
       */

      dev = udp_find_laddr_device(conn);

      /* Set up the callback in the connection */

      state.ir_cb = udp_callback_alloc(dev, conn);
      if (state.ir_cb)
        {
          /* Set up the callback in the connection */

          state.ir_cb->flags = (UDP_NEWDATA | NETDEV_DOWN);
          state.ir_cb->priv  = (FAR void *)&state;
          state.ir_cb->event = udp_eventhandler;

          /* Wait for either the receive to complete or for an error/timeout
           * to occur.  net_timedwait will also terminate if a signal is
           * received.
           */

          ret = net_timedwait(&state.ir_sem,
                              _SO_TIMEOUT(conn->sconn.s_rcvtimeo));
          if (ret == -ETIMEDOUT)
            {
              ret = -EAGAIN;
            }

          /* Make sure that no further events are processed */

          udp_callback_free(dev, conn, state.ir_cb);
          ret = udp_recvfrom_result(ret, &state);
        }
      else
        {
          ret = -EBUSY;
        }
    }

  net_unlock();
  udp_recvfrom_uninitialize(&state);
  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
