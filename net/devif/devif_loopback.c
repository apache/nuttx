/****************************************************************************
 * net/devif/devif_loopback.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

#include <debug.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/pkt.h>
#include <nuttx/net/netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is a helper pointer for accessing the contents of the ip header */

#define IPv4BUF ((FAR struct ipv4_hdr_s *)(dev->d_buf + dev->d_llhdrlen))
#define IPv6BUF ((FAR struct ipv6_hdr_s *)(dev->d_buf + dev->d_llhdrlen))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool is_loopback(FAR struct net_driver_s *dev)
{
  if (dev->d_len > 0)
    {
#ifdef CONFIG_NET_IPv4
      if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION &&
           net_ipv4addr_hdrcmp(IPv4BUF->destipaddr, &dev->d_ipaddr))
        {
          return true;
        }
#endif

#ifdef CONFIG_NET_IPv6
      if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION &&
          net_ipv6addr_hdrcmp(IPv6BUF->destipaddr, dev->d_ipv6addr))
        {
          return true;
        }
#endif
    }

  return false;
}

/****************************************************************************
 * Name: devif_loopback
 *
 * Description:
 *   This function should be called before sending out a packet. The function
 *   checks the destination address of the packet to see whether the target of
 *   packet is ourself and then consume the packet directly by calling input
 *   process functions.
 *
 * Returned Value:
 *   Zero is returned if the packet don't loop back to ourself, otherwise
 *   a no zero value is returned.
 *
 ****************************************************************************/

int devif_loopback(FAR struct net_driver_s *dev)
{
  if (!is_loopback(dev))
    {
      return 0;
    }

  /* Loop while if there is data "sent" to ourself.
   * Sending, of course, just means relaying back through the network.
   */

  do
    {
       NETDEV_TXPACKETS(dev);
       NETDEV_RXPACKETS(dev);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the packet tap */

       pkt_input(dev);
#endif

      /* We only accept IP packets of the configured type */

#ifdef CONFIG_NET_IPv4
      if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
        {
          ninfo("IPv4 frame\n");

          NETDEV_RXIPV4(dev);
          ipv4_input(dev);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
        {
          ninfo("Iv6 frame\n");

          NETDEV_RXIPV6(dev);
          ipv6_input(dev);
        }
      else
#endif
        {
          NETDEV_RXDROPPED(dev);
        }

      NETDEV_TXDONE(dev);

      /* Add the data link header length for the next loop */

      if (dev->d_len != 0)
        {
          dev->d_len += dev->d_llhdrlen;
        }
    }
  while (dev->d_len > 0);

  return 1;
}

