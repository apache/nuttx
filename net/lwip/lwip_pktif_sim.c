/****************************************************************************
 * net/lwip/lwip_pktif_sim.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#if defined(CONFIG_NET_LWIP) && defined(CONFIG_ARCH_BOARD_SIM)

#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/pbuf.h"
#include "lwip/ip4_addr.h"
#include "lwip_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef IFHWADDRLEN
#  define IFHWADDRLEN 6
#endif

#ifndef IFNAMSIZ
#  define IFNAMSIZ 16
#endif

#ifndef CONFIG_NET_LWIP_SIM_IFNAME
#  define CONFIG_NET_LWIP_SIM_IFNAME "eth0"
#endif

#ifndef CONFIG_NET_LWIP_SIM_FRAMEBUF_SIZE
#  define CONFIG_NET_LWIP_SIM_FRAMEBUF_SIZE 1600
#endif

#ifndef CONFIG_NET_LWIP_SIM_RXQLEN
#  define CONFIG_NET_LWIP_SIM_RXQLEN 16
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

extern int sim_netdriver_l2tap_register(int devidx,
                                        void (*cb)(void *arg,
                                                   const uint8_t *buf,
                                                   unsigned int len),
                                        void *arg);
extern int sim_netdriver_l2tap_unregister(int devidx,
                                          void (*cb)(void *arg,
                                                     const uint8_t *buf,
                                                     unsigned int len),
                                          void *arg);
extern int sim_netdriver_l2tap_xmit(int devidx,
                                    const uint8_t *buf,
                                    unsigned int len);
extern int sim_netdriver_l2tap_getmac(int devidx, uint8_t *mac);
extern int sim_netdriver_ifname_to_devidx(const char *ifname);

struct sim_lwip_rxslot_s
{
  uint16_t len;
  uint8_t  data[CONFIG_NET_LWIP_SIM_FRAMEBUF_SIZE];
};

struct sim_lwip_pktif_s
{
  struct nuttx_lwip_if lif;
  volatile bool        running;
  int                  devidx;
  uint8_t              mac[IFHWADDRLEN];
  char                 ifname[IFNAMSIZ];

  pthread_mutex_t      qlock;
  unsigned int         qhead;
  unsigned int         qtail;
  unsigned int         qcount;
  struct sim_lwip_rxslot_s rxq[CONFIG_NET_LWIP_SIM_RXQLEN];
};

static struct sim_lwip_pktif_s g_sim_lwip0;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sim_lwip_rxhook(void *arg, const uint8_t *buf, unsigned int len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sim_lwip_make_ip4addr(ip4_addr_t *addr, uint32_t value)
{
  IP4_ADDR(addr,
           (value >> 24) & 0xff,
           (value >> 16) & 0xff,
           (value >>  8) & 0xff,
           (value >>  0) & 0xff);
}

static int sim_lwip_get_mac(void *arg, uint8_t *mac)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;

  if (priv == NULL || mac == NULL)
    {
      return -EINVAL;
    }

  memcpy(mac, priv->mac, IFHWADDRLEN);
  return 0;
}

static void sim_lwip_default_ipv4(ip4_addr_t *ipaddr,
                                  ip4_addr_t *netmask,
                                  ip4_addr_t *gw)
{
#ifdef CONFIG_NETINIT_IPADDR
  uint32_t raw_ip = (uint32_t)CONFIG_NETINIT_IPADDR + 1u;
#else
  uint32_t raw_ip = 0x0a000103; /* 10.0.1.3 */
#endif

#ifdef CONFIG_NETINIT_NETMASK
  uint32_t raw_nm = (uint32_t)CONFIG_NETINIT_NETMASK;
#else
  uint32_t raw_nm = 0xffffff00; /* 255.255.255.0 */
#endif

#ifdef CONFIG_NETINIT_DRIPADDR
  uint32_t raw_gw = (uint32_t)CONFIG_NETINIT_DRIPADDR;
#else
  uint32_t raw_gw = 0x0a000101; /* 10.0.1.1 */
#endif

  sim_lwip_make_ip4addr(ipaddr, raw_ip);
  sim_lwip_make_ip4addr(netmask, raw_nm);
  sim_lwip_make_ip4addr(gw, raw_gw);
}

static void sim_lwip_rxhook(void *arg, const uint8_t *buf, unsigned int len)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;
  struct sim_lwip_rxslot_s *slot;

  if (priv == NULL || !priv->running || buf == NULL)
    {
      return;
    }

  if (len < 14 || len > CONFIG_NET_LWIP_SIM_FRAMEBUF_SIZE)
    {
      return;
    }

  /* Filter self-originated frames */

  if (memcmp(buf + 6, priv->mac, IFHWADDRLEN) == 0)
    {
      return;
    }

  pthread_mutex_lock(&priv->qlock);

  if (priv->qcount >= CONFIG_NET_LWIP_SIM_RXQLEN)
    {
      pthread_mutex_unlock(&priv->qlock);
      return;
    }

  slot = &priv->rxq[priv->qtail];
  memcpy(slot->data, buf, len);
  slot->len = len;

  priv->qtail = (priv->qtail + 1) % CONFIG_NET_LWIP_SIM_RXQLEN;
  priv->qcount++;

  pthread_mutex_unlock(&priv->qlock);

  nuttx_lwipif_rx_ready(&priv->lif);
}

static int sim_lwip_ifup(void *arg)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;
  int ret;

  priv->devidx = sim_netdriver_ifname_to_devidx(priv->ifname);
  if (priv->devidx < 0)
    {
      return priv->devidx;
    }

  ret = sim_netdriver_l2tap_getmac(priv->devidx, priv->mac);
  if (ret < 0)
    {
      return ret;
    }

  pthread_mutex_init(&priv->qlock, NULL);
  priv->qhead = 0;
  priv->qtail = 0;
  priv->qcount = 0;
  priv->running = true;

  ret = sim_netdriver_l2tap_register(priv->devidx, sim_lwip_rxhook, priv);
  if (ret < 0)
    {
      priv->running = false;
      pthread_mutex_destroy(&priv->qlock);
      return ret;
    }

  return 0;
}

static int sim_lwip_ifdown(void *arg)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;

  priv->running = false;
  sim_netdriver_l2tap_unregister(priv->devidx, sim_lwip_rxhook, priv);
  pthread_mutex_destroy(&priv->qlock);
  return 0;
}

static int sim_lwip_get_hwaddr(void *arg, uint8_t *mac, size_t maclen)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;

  if (mac == NULL || maclen < IFHWADDRLEN)
    {
      return -EINVAL;
    }

  memcpy(mac, priv->mac, IFHWADDRLEN);
  return 0;
}

static int sim_lwip_linkoutput(void *arg, struct pbuf *p)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;
  uint8_t stackbuf[CONFIG_NET_LWIP_SIM_FRAMEBUF_SIZE];
  uint8_t *buf = stackbuf;
  int ret;

  if (!priv->running || p == NULL)
    {
      return -ENODEV;
    }

  if (p->tot_len > sizeof(stackbuf))
    {
      buf = malloc(p->tot_len);
      if (buf == NULL)
        {
          return -ENOMEM;
        }
    }

  pbuf_copy_partial(p, buf, p->tot_len, 0);

  ret = sim_netdriver_l2tap_xmit(priv->devidx, buf, p->tot_len);

  if (buf != stackbuf)
    {
      free(buf);
    }

  return ret;
}

static struct pbuf *sim_lwip_linkinput(void *arg)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;
  struct sim_lwip_rxslot_s slot;
  struct pbuf *p;

  pthread_mutex_lock(&priv->qlock);

  if (priv->qcount == 0)
    {
      pthread_mutex_unlock(&priv->qlock);
      return NULL;
    }

  slot = priv->rxq[priv->qhead];
  priv->qhead = (priv->qhead + 1) % CONFIG_NET_LWIP_SIM_RXQLEN;
  priv->qcount--;

  pthread_mutex_unlock(&priv->qlock);

  p = pbuf_alloc(PBUF_RAW, slot.len, PBUF_POOL);
  if (p == NULL)
    {
      return NULL;
    }

  if (pbuf_take(p, slot.data, slot.len) != ERR_OK)
    {
      pbuf_free(p);
      return NULL;
    }

  return p;
}

static bool sim_lwip_is_link_up(void *arg)
{
  struct sim_lwip_pktif_s *priv = (struct sim_lwip_pktif_s *)arg;

  return priv->running;
}

static const struct nuttx_lwip_link_ops g_sim_lwip_ops =
{
  .ifup       = sim_lwip_ifup,
  .ifdown     = sim_lwip_ifdown,
  .get_hwaddr = sim_lwip_get_hwaddr,
  .linkoutput = sim_lwip_linkoutput,
  .linkinput  = sim_lwip_linkinput,
  .is_link_up = sim_lwip_is_link_up,
  .get_mac    = sim_lwip_get_mac,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_lwip_netif_init(void)
{
  ip4_addr_t ipaddr;
  ip4_addr_t netmask;
  ip4_addr_t gw;
  static bool initialized;

  if (initialized)
    {
      return 0;
    }

  memset(&g_sim_lwip0, 0, sizeof(g_sim_lwip0));
  strlcpy(g_sim_lwip0.ifname, CONFIG_NET_LWIP_SIM_IFNAME,
          sizeof(g_sim_lwip0.ifname));

#ifdef CONFIG_NET_LWIP_DHCP
  ip4_addr_set_zero(&ipaddr);
  ip4_addr_set_zero(&netmask);
  ip4_addr_set_zero(&gw);
#else
  sim_lwip_default_ipv4(&ipaddr, &netmask, &gw);
#endif

  if (nuttx_lwip_add_interface(&g_sim_lwip0.lif, &g_sim_lwip_ops,
                               &g_sim_lwip0,
                               "lw",
                               true,
                               &ipaddr,
                               &netmask,
                               &gw) < 0)
    {
      return -EIO;
    }

  initialized = true;
  return 0;
}

#endif /* CONFIG_NET_LWIP && CONFIG_ARCH_BOARD_SIM */
