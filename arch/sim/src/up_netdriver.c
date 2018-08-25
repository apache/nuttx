/****************************************************************************
 * arch/sim/src/up_netdriver.c
 *
 *   Copyright (C) 2007, 2009-2012, 2015-2016 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET_ETHERNET

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sched.h>
#include <nuttx/net/net.h>

#include <net/ethernet.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUF ((struct eth_hdr_s *)g_sim_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct timer
{
  uint32_t interval;
  uint32_t start;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timer g_periodic_timer;

/* A single packet buffer is used */

static uint8_t g_pktbuf[MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE];

/* Ethernet peripheral state */

static struct net_driver_s g_sim_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void timer_set(struct timer *t, unsigned int interval)
{
  t->interval = interval;
  t->start    = up_getwalltime();
}

static bool timer_expired(struct timer *t)
{
  return (up_getwalltime() - t->start) >= t->interval;
}

void timer_reset(struct timer *t)
{
  t->start += t->interval;
}

static int sim_txpoll(struct net_driver_s *dev)
{
  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (g_sim_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(&g_sim_dev))
#endif
        {
          arp_out(&g_sim_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&g_sim_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&g_sim_dev))
        {
          /* Send the packet */

          NETDEV_TXPACKETS(dev);
          netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
          NETDEV_TXDONE(dev);
        }
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void netdriver_loop(void)
{
  FAR struct eth_hdr_s *eth;

  /* Check for new frames.  If so, then poll the network for new XMIT data */

  net_lock();
  (void)devif_poll(&g_sim_dev, sim_txpoll);
  net_unlock();

  /* netdev_read will return 0 on a timeout event and >0 on a data received event */

  g_sim_dev.d_len = netdev_read((FAR unsigned char *)g_sim_dev.d_buf,
                                CONFIG_NET_ETH_PKTSIZE);

  /* Disable preemption through to the following so that it behaves a little more
   * like an interrupt (otherwise, the following logic gets pre-empted an behaves
   * oddly.
   */

  sched_lock();
  if (g_sim_dev.d_len > 0)
    {
      NETDEV_RXPACKETS(&g_sim_dev);

      /* Data received event.  Check for valid Ethernet header with destination == our
       * MAC address
       */

      eth = BUF;
      if (g_sim_dev.d_len > ETH_HDRLEN)
        {
#ifdef CONFIG_NET_PKT
          /* When packet sockets are enabled, feed the frame into the packet
           * tap.
           */

          pkt_input(&g_sim_dev);
#endif /* CONFIG_NET_PKT */

          /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
          if (eth->type == HTONS(ETHTYPE_IP))
            {
              ninfo("IPv4 frame\n");
              NETDEV_RXIPV4(&g_sim_dev);

              /* Handle ARP on input then give the IPv4 packet to the network
               * layer
               */

              arp_ipin(&g_sim_dev);
              ipv4_input(&g_sim_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the global variable
               * d_len is set to a value > 0.
               */

              if (g_sim_dev.d_len > 0)
                {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
                  if (IFF_IS_IPv4(g_sim_dev.d_flags))
#endif
                    {
                      arp_out(&g_sim_dev);
                    }
#ifdef CONFIG_NET_IPv6
                  else
                    {
                      neighbor_out(&g_sim_dev);
                    }
#endif

                  /* And send the packet */

                  netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
          else
#endif /* CONFIG_NET_IPv4 */
#ifdef CONFIG_NET_IPv6
          if (eth->type == HTONS(ETHTYPE_IP6))
            {
              ninfo("Iv6 frame\n");
              NETDEV_RXIPV6(&g_sim_dev);

              /* Give the IPv6 packet to the network layer */

              ipv6_input(&g_sim_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the global variable
               * d_len is set to a value > 0.
               */

              if (g_sim_dev.d_len > 0)
               {
                  /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
                  if (IFF_IS_IPv4(g_sim_dev.d_flags))
                    {
                      arp_out(&g_sim_dev);
                    }
                  else
#endif
#ifdef CONFIG_NET_IPv6
                    {
                      neighbor_out(&g_sim_dev);
                    }
#endif /* CONFIG_NET_IPv6 */

                  /* And send the packet */

                  netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
          else
#endif/* CONFIG_NET_IPv6 */
#ifdef CONFIG_NET_ARP
          if (eth->type == htons(ETHTYPE_ARP))
            {
              ninfo("ARP frame\n");
              NETDEV_RXARP(&g_sim_dev);

              arp_arpin(&g_sim_dev);

              /* If the above function invocation resulted in data that
               * should be sent out on the network, the global variable
               * d_len is set to a value > 0.
               */

              if (g_sim_dev.d_len > 0)
                {
                  netdev_send(g_sim_dev.d_buf, g_sim_dev.d_len);
                }
            }
          else
#endif
           {
             NETDEV_RXDROPPED(&g_sim_dev);
             nwarn("WARNING: Unsupported Ethernet type %u\n", eth->type);
           }
        }
      else
        {
          NETDEV_RXERRORS(&g_sim_dev);
        }
    }

  /* Otherwise, it must be a timeout event */

  else if (timer_expired(&g_periodic_timer))
    {
      timer_reset(&g_periodic_timer);
      devif_timer(&g_sim_dev, sim_txpoll);
    }

  sched_unlock();
}

int netdriver_ifup(struct net_driver_s *dev)
{
  netdev_ifup(dev->d_ipaddr);
  return OK;
}

int netdriver_ifdown(struct net_driver_s *dev)
{
  netdev_ifdown();
  return OK;
}

int netdriver_init(void)
{
  /* Internal initalization */

  timer_set(&g_periodic_timer, 500);
  netdev_init();

  /* Set callbacks */

  g_sim_dev.d_buf    = g_pktbuf;         /* Single packet buffer */
  g_sim_dev.d_ifup   = netdriver_ifup;
  g_sim_dev.d_ifdown = netdriver_ifdown;

  /* Register the device with the OS so that socket IOCTLs can be performed */

  (void)netdev_register(&g_sim_dev, NET_LL_ETHERNET);
  return OK;
}

int netdriver_setmacaddr(unsigned char *macaddr)
{
  (void)memcpy(g_sim_dev.d_mac.ether.ether_addr_octet, macaddr, IFHWADDRLEN);
  return 0;
}

#endif /* CONFIG_NET_ETHERNET */

