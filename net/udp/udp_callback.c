/****************************************************************************
 * net/udp/udp_callback.c
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

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "udp/udp.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_datahandler
 *
 * Description:
 *   Handle the receipt of UDP data by adding the newly received packet to
 *   the UDP read-ahead buffer.
 *
 ****************************************************************************/

static uint16_t udp_datahandler(FAR struct net_driver_s *dev,
                                FAR struct udp_conn_s *conn,
                                FAR uint8_t *buffer, uint16_t buflen)
{
  FAR struct iob_s *iob;
  int ret;
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 src_addr6 =
  {
    0
  };
#endif

#ifdef CONFIG_NET_IPv4
  FAR struct sockaddr_in src_addr4 =
  {
    0
  };
#endif

  FAR void  *src_addr;
  uint8_t src_addr_size;
  uint8_t offset = 0;

#if CONFIG_NET_RECV_BUFSIZE > 0
  while (iob_get_queue_size(&conn->readahead) > conn->rcvbufs)
    {
      iob = iob_remove_queue(&conn->readahead);
      iob_free_chain(iob);
    }
#endif

  /* Allocate on I/O buffer to start the chain (throttling as necessary).
   * We will not wait for an I/O buffer to become available in this context.
   */

  iob = iob_tryalloc(true);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to create new I/O buffer chain\n");
      return 0;
    }

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
      FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;

      src_addr6.sin6_family = AF_INET6;
      src_addr6.sin6_port   = udp->srcport;

      net_ipv6addr_copy(src_addr6.sin6_addr.s6_addr, ipv6->srcipaddr);

      src_addr_size = sizeof(src_addr6);
      src_addr = &src_addr6;
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
#ifdef CONFIG_NET_IPv6
      /* Hybrid dual-stack IPv6/IPv4 implementations recognize a special
       * class of addresses, the IPv4-mapped IPv6 addresses.
       */

      if (conn->domain == PF_INET6)
        {
          FAR struct udp_hdr_s *udp   = UDPIPv6BUF;
          FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
          in_addr_t ipv4addr;

          /* Encode the IPv4 address as an IPv-mapped IPv6 address */

          src_addr6.sin6_family = AF_INET6;
          src_addr6.sin6_port = udp->srcport;

          ipv4addr = net_ip4addr_conv32(ipv6->srcipaddr);
          ip6_map_ipv4addr(ipv4addr, src_addr6.sin6_addr.s6_addr16);

          src_addr_size = sizeof(src_addr6);
          src_addr = &src_addr6;
        }
      else
#endif
        {
          FAR struct udp_hdr_s *udp   = UDPIPv4BUF;
          FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;

          src_addr4.sin_family = AF_INET;
          src_addr4.sin_port   = udp->srcport;

          net_ipv4addr_copy(src_addr4.sin_addr.s_addr,
                            net_ip4addr_conv32(ipv4->srcipaddr));
          memset(src_addr4.sin_zero, 0, sizeof(src_addr4.sin_zero));

          src_addr_size = sizeof(src_addr4);
          src_addr = &src_addr4;
        }
    }
#endif /* CONFIG_NET_IPv4 */

  /* Copy the src address info into the I/O buffer chain.  We will not wait
   * for an I/O buffer to become available in this context.  It there is
   * any failure to allocated, the entire I/O buffer chain will be discarded.
   */

  ret = iob_trycopyin(iob, (FAR const uint8_t *)&src_addr_size,
                      sizeof(uint8_t), offset, true);
  offset += sizeof(uint8_t);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
      return 0;
    }

  ret = iob_trycopyin(iob, (FAR const uint8_t *)src_addr, src_addr_size,
                      offset, true);
  offset += src_addr_size;
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
      return 0;
    }

#ifdef CONFIG_NETDEV_IFINDEX
  ret = iob_trycopyin(iob, &dev->d_ifindex, sizeof(uint8_t), offset, true);
  offset += sizeof(uint8_t);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nerr("ERROR: Failed to add dindex to the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
      return 0;
    }
#endif

  if (buflen > 0)
    {
      /* Copy the new appdata into the I/O buffer chain */

      ret = iob_trycopyin(iob, buffer, buflen, offset, true);
      if (ret < 0)
        {
          /* On a failure, iob_trycopyin return a negated error value but
           * does not free any I/O buffers.
           */

          nerr("ERROR: Failed to add data to the I/O buffer chain: %d\n",
               ret);
          iob_free_chain(iob);
          return 0;
        }
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
      return 0;
    }

#ifdef CONFIG_NET_UDP_NOTIFIER
  /* Provided notification(s) that additional UDP read-ahead data is
   * available.
   */

  udp_readahead_signal(conn);
#endif

  ninfo("Buffered %d bytes\n", buflen);
  return buflen;
}

/****************************************************************************
 * Name: net_dataevent
 *
 * Description:
 *   Handling the network UDP_NEWDATA event.
 *
 ****************************************************************************/

static inline uint16_t
net_dataevent(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn,
              uint16_t flags)
{
  uint16_t ret;
  uint8_t *buffer = dev->d_appdata;
  int      buflen = dev->d_len;
  uint16_t recvlen;

  ret = (flags & ~UDP_NEWDATA);

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with UDP_NEWDATA set just to cause an ACK).
   */

  ninfo("No receive on connection\n");

  /* Save as the packet data as in the read-ahead buffer.  NOTE that
   * partial packets will not be buffered.
   */

  recvlen = udp_datahandler(dev, conn, buffer, buflen);
  if (recvlen < buflen)
    {
      /* There is no handler to receive new data and there are no free
       * read-ahead buffers to retain the data -- drop the packet.
       */

     ninfo("Dropped %d bytes\n", dev->d_len);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.udp.drop++;
#endif
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_callback
 *
 * Description:
 *   Inform the application holding the UDP socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function must be called with the network locked.
 *
 ****************************************************************************/

uint16_t udp_callback(FAR struct net_driver_s *dev,
                      FAR struct udp_conn_s *conn, uint16_t flags)
{
  ninfo("flags: %04x\n", flags);

  /* Some sanity checking */

  if (conn)
    {
      /* Perform the callback */

      flags = devif_conn_event(dev, flags, conn->sconn.list);

      if ((flags & UDP_NEWDATA) != 0)
        {
          /* Data was not handled.. dispose of it appropriately */

          flags = net_dataevent(dev, conn, flags);
        }
    }

  return flags;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
