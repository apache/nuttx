/****************************************************************************
 * net/udp/udp_callback.c
 *
 *   Copyright (C) 2007-2009, 2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "iob/iob.h"
#include "udp/udp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4BUF    ((FAR struct ipv4_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])
#define IPv6BUF    ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

#define UDPIPv4BUF ((FAR struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv4_HDRLEN])
#define UDPIPv6BUF ((FAR struct udp_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev) + IPv6_HDRLEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: udp_datahandler
 *
 * Description:
 *   Handle the receipt of UDP data by adding the newly received packet to
 *   the UDP read-ahead buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP_READAHEAD
static uint16_t udp_datahandler(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn,
                                FAR uint8_t *buffer, uint16_t buflen)
{
  FAR struct iob_s *iob;
  int ret;
#ifdef CONFIG_NET_IPv6
  FAR struct sockaddr_in6 src_addr6;
#endif
#ifdef CONFIG_NET_IPv4
  FAR struct sockaddr_in src_addr4;
#endif
  FAR void  *src_addr;
  uint8_t src_addr_size;

  /* Allocate on I/O buffer to start the chain (throttling as necessary).
   * We will not wait for an I/O buffer to become available in this context.
   */

  iob = iob_tryalloc(true);
  if (iob == NULL)
    {
      nlldbg("ERROR: Failed to create new I/O buffer chain\n");
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

          src_addr_size = sizeof(src_addr4);
          src_addr = &src_addr4;
        }
    }
#endif /* CONFIG_NET_IPv4 */

  /* Copy the src address info into the I/O buffer chain.  We will not wait
   * for an I/O buffer to become available in this context.  It there is
   * any failure to allocated, the entire I/O buffer chain will be discarded.
   */

  ret = iob_trycopyin(iob, (FAR const uint8_t*)&src_addr_size,
                      sizeof(uint8_t), 0, true);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nlldbg("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  ret = iob_trycopyin(iob, (FAR const uint8_t*)src_addr, src_addr_size,
                      sizeof(uint8_t), true);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nlldbg("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  /* Copy the new appdata into the I/O buffer chain */

  ret = iob_trycopyin(iob, buffer, buflen, src_addr_size + sizeof(uint8_t),
                      true);
  if (ret < 0)
    {
      /* On a failure, iob_trycopyin return a negated error value but does
       * not free any I/O buffers.
       */

      nlldbg("ERROR: Failed to add data to the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  /* Add the new I/O buffer chain to the tail of the read-ahead queue */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nlldbg("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      (void)iob_free_chain(iob);
      return 0;
    }

  nllvdbg("Buffered %d bytes\n", buflen);
  return buflen;
}
#endif /* CONFIG_NET_UDP_READAHEAD */

/****************************************************************************
 * Function: net_dataevent
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

  ret = (flags & ~UDP_NEWDATA);

  /* Is there new data?  With non-zero length?  (Certain connection events
   * can have zero-length with UDP_NEWDATA set just to cause an ACK).
   */

  if (dev->d_len > 0)
    {
#ifdef CONFIG_NET_UDP_READAHEAD
      uint8_t *buffer = dev->d_appdata;
      int      buflen = dev->d_len;
      uint16_t recvlen;
#endif

      nllvdbg("No receive on connection\n");

#ifdef CONFIG_NET_UDP_READAHEAD
      /* Save as the packet data as in the read-ahead buffer.  NOTE that
       * partial packets will not be buffered.
       */

      recvlen = udp_datahandler(dev, conn, buffer, buflen);
      if (recvlen < buflen)
#endif
        {
          /* There is no handler to receive new data and there are no free
           * read-ahead buffers to retain the data -- drop the packet.
           */

         nllvdbg("Dropped %d bytes\n", dev->d_len);

 #ifdef CONFIG_NET_STATISTICS
          g_netstats.udp.drop++;
#endif
        }
    }

  /* In any event, the new data has now been handled */

  dev->d_len = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: udp_callback
 *
 * Description:
 *   Inform the application holding the UDP socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function is called at the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

uint16_t udp_callback(FAR struct net_driver_s *dev,
                      FAR struct udp_conn_s *conn, uint16_t flags)
{
  nllvdbg("flags: %04x\n", flags);

  /* Some sanity checking */

  if (conn)
    {
      /* Perform the callback */

      flags = devif_conn_event(dev, conn, flags, conn->list);

      if ((flags & UDP_NEWDATA) != 0)
        {
          /* Data was not handled.. dispose of it appropriately */

          flags = net_dataevent(dev, conn, flags);
        }
    }

  return flags;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
