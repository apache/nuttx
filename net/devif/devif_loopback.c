/****************************************************************************
 * net/devif/devif_loopback.c
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

#include <string.h>
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
 *   checks the destination address of the packet to see whether the target
 *   of packet is ourself and then consume the packet directly by calling
 *   input process functions.
 *
 * Returned Value:
 *   Zero is returned if the packet don't loop back to ourself, otherwise
 *   a non-zero value is returned.
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
      /* When packet sockets are enabled, feed the frame into the tap */

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
          ninfo("IPv6 frame\n");

          NETDEV_RXIPV6(dev);
          ipv6_input(dev);
        }
      else
#endif
        {
          NETDEV_RXDROPPED(dev);
        }

      NETDEV_TXDONE(dev);

      /* Add the link layer header length for the next loop */

      if (dev->d_len != 0)
        {
          dev->d_len += dev->d_llhdrlen;
        }
    }
  while (dev->d_len > 0);

  return 1;
}
