/****************************************************************************
 * arch/sim/src/sim/sim_netdriver.c
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

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/pkt.h>

#include "sim_internal.h"

#if CONFIG_IOB_BUFSIZE >= (MAX_NETDEV_PKTSIZE + \
    CONFIG_NET_GUARDSIZE + CONFIG_NET_LL_GUARDSIZE)
#  define SIM_NETDEV_IOB_OFFLOAD
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void netdriver_txdone_interrupt(void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Net driver worker */

static struct work_s g_avail_work[CONFIG_SIM_NETDEV_NUMBER];
static struct work_s g_recv_work[CONFIG_SIM_NETDEV_NUMBER];

/* Ethernet peripheral state */

static struct net_driver_s g_sim_dev[CONFIG_SIM_NETDEV_NUMBER];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void netdriver_send(struct net_driver_s *dev)
{
  int devidx = (intptr_t)dev->d_private;
#ifdef SIM_NETDEV_IOB_OFFLOAD
  uint8_t *buf = &dev->d_iob->io_data[CONFIG_NET_LL_GUARDSIZE -
                                      NET_LL_HDRLEN(dev)];
#else
  uint8_t *buf = dev->d_buf;
#endif

  UNUSED(devidx);

  sim_netdev_send(devidx, buf, dev->d_len);
}

static void netdriver_reply(struct net_driver_s *dev)
{
  /* If the receiving resulted in data that should be sent out on
   * the network, the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Send the packet */

      NETDEV_TXPACKETS(dev);
      netdriver_send(dev);
      NETDEV_TXDONE(dev);
    }
}

static void netdriver_recv_work(void *arg)
{
  struct net_driver_s *dev = arg;
  struct eth_hdr_s *eth;
  int devidx = (intptr_t)dev->d_private;

  UNUSED(devidx);

  net_lock();

  /* Retrieve all the queued RX frames from the network device
   * to prevent RX data stream congestion.
   */

  while (sim_netdev_avail(devidx))
    {
#ifdef SIM_NETDEV_IOB_OFFLOAD
      if (netdev_iob_prepare(dev, false, 0) != OK)
        {
          netdriver_txdone_interrupt(dev);
          break;
        }
#endif

      /* sim_netdev_read will return 0 on a timeout event and > 0
       * on a data received event
       */

      dev->d_len = sim_netdev_read(devidx,
                               (unsigned char *)dev->d_buf,
                               dev->d_pktsize);
      if (dev->d_len > 0)
        {
          NETDEV_RXPACKETS(dev);

#ifdef SIM_NETDEV_IOB_OFFLOAD
          iob_update_pktlen(dev->d_iob, dev->d_len - NET_LL_HDRLEN(dev));
#endif

          /* Data received event.  Check for valid Ethernet header with
           * destination == our MAC address
           */

          eth = (struct eth_hdr_s *)dev->d_buf;
          if (dev->d_len > ETH_HDRLEN)
            {
#ifdef CONFIG_NET_PKT
              /* When packet sockets are enabled, feed the frame into
               * the packet tap.
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

                  /* Receive an IPv4 packet from the network device */

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
              if (eth->type == HTONS(ETHTYPE_ARP))
                {
                  ninfo("ARP frame\n");
                  NETDEV_RXARP(dev);

                  arp_input(dev);

                  /* If the above function invocation resulted in data that
                   * should be sent out on the network, the global variable
                   * d_len is set to a value > 0.
                   */

                  if (dev->d_len > 0)
                    {
                      netdriver_send(dev);
                    }
                }
              else
#endif
                {
                  NETDEV_RXDROPPED(dev);
                  nwarn("WARNING: Unsupported Ethernet type %u\n",
                        eth->type);
                }
            }
          else
            {
              NETDEV_RXERRORS(dev);
            }
        }

#ifdef SIM_NETDEV_IOB_OFFLOAD
      netdev_iob_release(dev);
#endif
    }

  net_unlock();
}

static int netdriver_txpoll(struct net_driver_s *dev)
{
  /* Send the packet */

  NETDEV_TXPACKETS(dev);
  netdriver_send(dev);
  NETDEV_TXDONE(dev);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

static int netdriver_ifup(struct net_driver_s *dev)
{
  int devidx = (intptr_t)dev->d_private;

  UNUSED(devidx);
#ifdef CONFIG_NET_IPv4
  sim_netdev_ifup(devidx, &dev->d_ipaddr);
#else /* CONFIG_NET_IPv6 */
  sim_netdev_ifup(devidx, &dev->d_ipv6addr);
#endif /* CONFIG_NET_IPv4 */
  netdev_carrier_on(dev);
  return OK;
}

static int netdriver_ifdown(struct net_driver_s *dev)
{
  int devidx = (intptr_t)dev->d_private;

  UNUSED(devidx);
  netdev_carrier_off(dev);
  sim_netdev_ifdown(devidx);
  return OK;
}

static void netdriver_txavail_work(void *arg)
{
  struct net_driver_s *dev = arg;

  net_lock();
  if (IFF_IS_UP(dev->d_flags))
    {
      devif_poll(dev, netdriver_txpoll);
    }

  net_unlock();
}

static int netdriver_txavail(struct net_driver_s *dev)
{
  int devidx = (intptr_t)dev->d_private;
  if (work_available(&g_avail_work[devidx]))
    {
      work_queue(LPWORK, &g_avail_work[devidx], netdriver_txavail_work,
                 dev, 0);
    }

  return OK;
}

static void netdriver_txdone_interrupt(void *priv)
{
  struct net_driver_s *dev = (struct net_driver_s *)priv;
  int devidx = (intptr_t)dev->d_private;
  if (work_available(&g_avail_work[devidx]))
    {
      work_queue(LPWORK, &g_avail_work[devidx], netdriver_txavail_work,
                 dev, 0);
    }
}

static void netdriver_rxready_interrupt(void *priv)
{
  struct net_driver_s *dev = (struct net_driver_s *)priv;
  int devidx = (intptr_t)dev->d_private;
  if (work_available(&g_recv_work[devidx]))
    {
      work_queue(LPWORK, &g_recv_work[devidx], netdriver_recv_work, dev, 0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_netdriver_init(void)
{
  struct net_driver_s *dev;
  int devidx;

  for (devidx = 0; devidx < CONFIG_SIM_NETDEV_NUMBER; devidx++)
    {
      dev = &g_sim_dev[devidx];

      /* Internal initialization */

      sim_netdev_init(devidx, dev,
                  netdriver_txdone_interrupt,
                  netdriver_rxready_interrupt);

      /* Allocate packet buffer */

#ifndef SIM_NETDEV_IOB_OFFLOAD
      dev->d_buf = kmm_malloc(dev->d_pktsize != 0 ?
                              dev->d_pktsize :
                              (MAX_NETDEV_PKTSIZE +
                               CONFIG_NET_GUARDSIZE));
      if (dev->d_buf == NULL)
        {
          return -ENOMEM;
        }
#endif

      dev->d_ifup    = netdriver_ifup;
      dev->d_ifdown  = netdriver_ifdown;
      dev->d_txavail = netdriver_txavail;
      dev->d_private = (void *)(intptr_t)devidx;

      /* Register the device with the OS so that socket IOCTLs can be
       * performed
       */

      netdev_register(dev, NET_LL_ETHERNET);
    }

  return OK;
}

void sim_netdriver_setmacaddr(int devidx, unsigned char *macaddr)
{
  memcpy(g_sim_dev[devidx].d_mac.ether.ether_addr_octet, macaddr,
         IFHWADDRLEN);
}

void sim_netdriver_setmtu(int devidx, int mtu)
{
  g_sim_dev[devidx].d_pktsize = mtu + ETH_HDRLEN;
}

void sim_netdriver_loop(void)
{
  int devidx;
  for (devidx = 0; devidx < CONFIG_SIM_NETDEV_NUMBER; devidx++)
    {
      if (work_available(&g_recv_work[devidx]) && sim_netdev_avail(devidx))
        {
          work_queue(LPWORK, &g_recv_work[devidx], netdriver_recv_work,
                     &g_sim_dev[devidx], 0);
        }
    }
}
