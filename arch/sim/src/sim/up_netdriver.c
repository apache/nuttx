/****************************************************************************
 * arch/sim/src/sim/up_netdriver.c
 *
 *   Copyright (C) 2007, 2009-2012, 2015-2016 Gregory Nutt.
 *   All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on code from uIP which also has a BSD-like license:
 *
 *   Copyright (c) 2001, Adam Dunkels.
 *   All rights reserved.
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Net driver worker */

static struct work_s g_timer_work;
static struct work_s g_avail_work;
static struct work_s g_recv_work;

/* Ethernet peripheral state */

static struct net_driver_s g_sim_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void netdriver_reply(FAR struct net_driver_s *dev)
{
  /* If the receiving resulted in data that should be sent out on
   * the network, the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(dev->d_flags))
#endif
        {
          arp_out(dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(dev);
        }
#endif /* CONFIG_NET_IPv6 */

      /* Send the packet */

      NETDEV_TXPACKETS(dev);
      netdev_send(dev->d_buf, dev->d_len);
      NETDEV_TXDONE(dev);
    }
}

static void netdriver_recv_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;
  FAR struct eth_hdr_s *eth;

  net_lock();

  /* netdev_read will return 0 on a timeout event and > 0
   * on a data received event
   */

  dev->d_len = netdev_read((FAR unsigned char *)dev->d_buf,
                           dev->d_pktsize);
  if (dev->d_len > 0)
    {
      NETDEV_RXPACKETS(dev);

      /* Data received event.  Check for valid Ethernet header with
       * destination == our MAC address
       */

      eth = (FAR struct eth_hdr_s *)dev->d_buf;
      if (dev->d_len > ETH_HDRLEN)
        {
#ifdef CONFIG_NET_PKT
          /* When packet sockets are enabled, feed the frame into the packet
           * tap.
           */

          pkt_input(dev);
#endif /* CONFIG_NET_PKT */

          /* We only accept IP packets of the configured type
           * and ARP packets
           */

#ifdef CONFIG_NET_IPv4
          if (eth->type == HTONS(ETHTYPE_IP))
            {
              ninfo("IPv4 frame\n");
              NETDEV_RXIPV4(dev);

              /* Handle ARP on input then give the IPv4 packet to the network
               * layer
               */

              arp_ipin(dev);
              ipv4_input(dev);

              /* Check for a reply to the IPv4 packet */

              netdriver_reply(dev);
            }
          else
#endif /* CONFIG_NET_IPv4 */
#ifdef CONFIG_NET_IPv6
          if (eth->type == HTONS(ETHTYPE_IP6))
            {
              ninfo("IPv6 frame\n");
              NETDEV_RXIPV6(dev);

              /* Give the IPv6 packet to the network layer */

              ipv6_input(dev);

              /* Check for a reply to the IPv6 packet */

              netdriver_reply(dev);
            }
          else
#endif/* CONFIG_NET_IPv6 */
#ifdef CONFIG_NET_ARP
          if (eth->type == htons(ETHTYPE_ARP))
            {
              ninfo("ARP frame\n");
              NETDEV_RXARP(dev);

              arp_arpin(dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the global variable
               * d_len is set to a value > 0.
               */

              if (dev->d_len > 0)
                {
                  netdev_send(dev->d_buf, dev->d_len);
                }
            }
          else
#endif
            {
              NETDEV_RXDROPPED(dev);
              nwarn("WARNING: Unsupported Ethernet type %u\n", eth->type);
            }
        }
      else
        {
          NETDEV_RXERRORS(dev);
        }
    }

  net_unlock();
}

static int netdriver_txpoll(FAR struct net_driver_s *dev)
{
  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(dev->d_flags))
#endif
        {
          arp_out(dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(dev))
        {
          /* Send the packet */

          NETDEV_TXPACKETS(dev);
          netdev_send(dev->d_buf, dev->d_len);
          NETDEV_TXDONE(dev);
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

static void netdriver_timer_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;

  net_lock();
  if (IFF_IS_UP(dev->d_flags))
    {
      work_queue(LPWORK, &g_timer_work, netdriver_timer_work, dev, CLK_TCK);
      devif_timer(dev, CLK_TCK, netdriver_txpoll);
    }

  net_unlock();
}

static int netdriver_ifup(FAR struct net_driver_s *dev)
{
  netdev_ifup(dev->d_ipaddr);
  work_queue(LPWORK, &g_timer_work, netdriver_timer_work, dev, CLK_TCK);
  return OK;
}

static int netdriver_ifdown(FAR struct net_driver_s *dev)
{
  work_cancel(LPWORK, &g_timer_work);
  netdev_ifdown();
  return OK;
}

static void netdriver_txavail_work(FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;

  net_lock();
  if (IFF_IS_UP(dev->d_flags))
    {
      devif_poll(dev, netdriver_txpoll);
    }

  net_unlock();
}

static int netdriver_txavail(FAR struct net_driver_s *dev)
{
  if (work_available(&g_avail_work))
    {
      work_queue(LPWORK, &g_avail_work, netdriver_txavail_work, dev, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int netdriver_init(void)
{
  FAR struct net_driver_s *dev = &g_sim_dev;
  void *pktbuf;
  int pktsize;

  /* Internal initialization */

  netdev_init();

  /* Update the buffer size */

  pktsize = dev->d_pktsize ? dev->d_pktsize :
            (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE);

  /* Allocate packet buffer */

  pktbuf = kmm_malloc(pktsize);
  if (pktbuf == NULL)
    {
      return -ENOMEM;
    }

  /* Set callbacks */

  dev->d_buf     = pktbuf;
  dev->d_ifup    = netdriver_ifup;
  dev->d_ifdown  = netdriver_ifdown;
  dev->d_txavail = netdriver_txavail;

  /* Register the device with the OS so that socket IOCTLs can be performed */

  return netdev_register(dev, NET_LL_ETHERNET);
}

void netdriver_setmacaddr(unsigned char *macaddr)
{
  memcpy(g_sim_dev.d_mac.ether.ether_addr_octet, macaddr, IFHWADDRLEN);
}

void netdriver_setmtu(int mtu)
{
  g_sim_dev.d_pktsize = mtu;
}

void netdriver_loop(void)
{
  if (work_available(&g_recv_work) && netdev_avail())
    {
      work_queue(LPWORK, &g_recv_work, netdriver_recv_work, &g_sim_dev, 0);
    }
}
