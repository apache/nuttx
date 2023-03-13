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
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/pkt.h>

#include "sim_internal.h"

#define SIM_NETDEV_BUFSIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* We don't know packet length before receiving, so we can only offload it
 * when netpkt's buffer is long enough.
 */

#if NETPKT_BUFLEN >= SIM_NETDEV_BUFSIZE
#  define SIM_NETDEV_RECV_OFFLOAD
#endif

/* Get index / buffer from dev pointer. */

#define DEVIDX(p) ((struct sim_netdev_s *)(p) - g_sim_dev)
#define DEVBUF(p) (((struct sim_netdev_s *)(p))->buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_netdev_s
{
  struct netdev_lowerhalf_s dev;
  uint8_t buf[SIM_NETDEV_BUFSIZE]; /* Used when packet buffer is fragmented */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int netdriver_send(struct netdev_lowerhalf_s *dev, netpkt_t *pkt);
static netpkt_t *netdriver_recv(struct netdev_lowerhalf_s *dev);
static int netdriver_ifup(struct netdev_lowerhalf_s *dev);
static int netdriver_ifdown(struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Ethernet peripheral state */

static struct sim_netdev_s g_sim_dev[CONFIG_SIM_NETDEV_NUMBER];
static const struct netdev_ops_s g_ops =
{
  netdriver_ifup,   /* ifup */
  netdriver_ifdown, /* ifdown */
  netdriver_send,   /* transmit */
  netdriver_recv    /* receive */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int netdriver_send(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  unsigned int len  = netpkt_getdatalen(dev, pkt);

  if (netpkt_is_fragmented(pkt))
    {
      netpkt_copyout(dev, DEVBUF(dev), pkt, len, 0);
      sim_netdev_send(DEVIDX(dev), DEVBUF(dev), len);
    }
  else
    {
      sim_netdev_send(DEVIDX(dev), netpkt_getdata(dev, pkt), len);
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  return OK;
}

static netpkt_t *netdriver_recv(struct netdev_lowerhalf_s *dev)
{
  netpkt_t *pkt = NULL;
  unsigned int len;

  if (sim_netdev_avail(DEVIDX(dev)))
    {
      pkt = netpkt_alloc(dev, NETPKT_RX);
      if (pkt == NULL)
        {
          return NULL;
        }

      /* sim_netdev_read will return 0 on a timeout event and > 0
       * on a data received event
       */

#ifdef SIM_NETDEV_RECV_OFFLOAD
      len = sim_netdev_read(DEVIDX(dev), netpkt_getbase(pkt),
                            SIM_NETDEV_BUFSIZE);
#else
      len = sim_netdev_read(DEVIDX(dev), DEVBUF(dev), SIM_NETDEV_BUFSIZE);
#endif
      if (len == 0)
        {
          netpkt_free(dev, pkt, NETPKT_RX);
          return NULL;
        }

#ifdef SIM_NETDEV_RECV_OFFLOAD
      netpkt_reset_reserved(dev, pkt, 0); /* No overhead before data. */
      netpkt_setdatalen(dev, pkt, len);
#else
      netpkt_copyin(dev, pkt, DEVBUF(dev), len, 0);
#endif
    }

  return pkt;
}

static int netdriver_ifup(struct netdev_lowerhalf_s *dev)
{
#ifdef CONFIG_NET_IPv4
  sim_netdev_ifup(DEVIDX(dev), &dev->netdev.d_ipaddr);
#else /* CONFIG_NET_IPv6 */
  sim_netdev_ifup(DEVIDX(dev), &dev->netdev.d_ipv6addr);
#endif /* CONFIG_NET_IPv4 */
  netdev_lower_carrier_on(dev);
  return OK;
}

static int netdriver_ifdown(struct netdev_lowerhalf_s *dev)
{
  netdev_lower_carrier_off(dev);
  sim_netdev_ifdown(DEVIDX(dev));
  return OK;
}

static void netdriver_txdone_interrupt(void *priv)
{
  struct netdev_lowerhalf_s *dev = (struct netdev_lowerhalf_s *)priv;
  netdev_lower_txdone(dev);
}

static void netdriver_rxready_interrupt(void *priv)
{
  struct netdev_lowerhalf_s *dev = (struct netdev_lowerhalf_s *)priv;
  netdev_lower_rxready(dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sim_netdriver_init(void)
{
  struct netdev_lowerhalf_s *dev;
  int devidx;

  for (devidx = 0; devidx < CONFIG_SIM_NETDEV_NUMBER; devidx++)
    {
      dev = &g_sim_dev[devidx].dev;

      /* Internal initialization */

      sim_netdev_init(devidx, dev,
                      netdriver_txdone_interrupt,
                      netdriver_rxready_interrupt);

      /* 1TX + 1RX is enough for sim. */

      dev->quota[NETPKT_TX] = 1;
      dev->quota[NETPKT_RX] = 1;
      dev->ops              = &g_ops;

      /* Register the device with the OS so that socket IOCTLs can be
       * performed
       */

      netdev_lower_register(dev, NET_LL_ETHERNET);
    }

  return OK;
}

void sim_netdriver_setmacaddr(int devidx, unsigned char *macaddr)
{
  memcpy(g_sim_dev[devidx].dev.netdev.d_mac.ether.ether_addr_octet, macaddr,
         IFHWADDRLEN);
}

void sim_netdriver_setmtu(int devidx, int mtu)
{
  g_sim_dev[devidx].dev.netdev.d_pktsize = mtu + ETH_HDRLEN;
}

void sim_netdriver_loop(void)
{
  int devidx;
  for (devidx = 0; devidx < CONFIG_SIM_NETDEV_NUMBER; devidx++)
    {
      if (sim_netdev_avail(devidx))
        {
          netdev_lower_rxready(&g_sim_dev[devidx].dev);
        }
    }
}
