/****************************************************************************
 * net/lwip/ethernetif_nuttx.c
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
#ifdef CONFIG_NET_LWIP

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <string.h>

#include "lwip/def.h"
#include "lwip/dhcp.h"
#include "lwip/etharp.h"
#include "netif/ethernet.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/tcpip.h"

#include "lwip_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_LWIP_MTU
#  define CONFIG_NET_LWIP_MTU 1500
#endif

#ifndef CONFIG_NET_LWIP_RXTHREAD_STACKSIZE
#  define CONFIG_NET_LWIP_RXTHREAD_STACKSIZE 16384
#endif

#ifndef CONFIG_NET_LWIP_RXTHREAD_PRIORITY
#  define CONFIG_NET_LWIP_RXTHREAD_PRIORITY 100
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static err_t nuttx_low_level_output(struct netif *netif, struct pbuf *p)
{
  struct nuttx_lwip_if *lif = (struct nuttx_lwip_if *)netif->state;

  if (lif == NULL || lif->ops == NULL || lif->ops->linkoutput == NULL)
    {
      return ERR_IF;
    }

  return lif->ops->linkoutput(lif->priv, p) == 0 ? ERR_OK : ERR_IF;
}

static void nuttx_low_level_init(struct netif *netif)
{
  struct nuttx_lwip_if *lif = (struct nuttx_lwip_if *)netif->state;

  netif->hwaddr_len = ETH_HWADDR_LEN;
  netif->mtu        = CONFIG_NET_LWIP_MTU;
  netif->flags      = NETIF_FLAG_BROADCAST |
                      NETIF_FLAG_ETHARP |
                      NETIF_FLAG_ETHERNET;
#if LWIP_IGMP
  netif->flags |= NETIF_FLAG_IGMP;
#endif

  if (lif != NULL && lif->ops != NULL && lif->ops->get_hwaddr != NULL)
    {
      lif->ops->get_hwaddr(lif->priv, netif->hwaddr, netif->hwaddr_len);
    }
}

err_t nuttx_ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", netif != NULL);

  netif->output     = etharp_output;
  netif->linkoutput = nuttx_low_level_output;
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
  netif->ip6_autoconfig_enabled = 1;
#endif

  nuttx_low_level_init(netif);
  return ERR_OK;
}

static void *nuttx_lwip_rx_thread(void *arg)
{
  struct nuttx_lwip_if *lif = (struct nuttx_lwip_if *)arg;

  while (lif->running)
    {
      while (sem_wait(&lif->rx_sem) < 0 && errno == EINTR)
        {
        }

      if (!lif->running)
        {
          break;
        }

      if (lif->link_pending)
        {
          lif->link_pending = false;
          if (lif->link_up)
            {
              netifapi_netif_set_link_up(&lif->nif);
            }
          else
            {
              netifapi_netif_set_link_down(&lif->nif);
            }
        }

      if (!lif->rx_pending || lif->ops == NULL ||
          lif->ops->linkinput == NULL)
        {
          continue;
        }

      lif->rx_pending = false;

      for (; ; )
        {
          struct pbuf *p = lif->ops->linkinput(lif->priv);

          if (p == NULL)
            {
              break;
            }

          if (lif->nif.input(p, &lif->nif) != ERR_OK)
            {
              pbuf_free(p);
            }
        }
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int nuttx_lwip_start_rx_thread(struct nuttx_lwip_if *lif)
{
  pthread_attr_t attr;
  int ret;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, CONFIG_NET_LWIP_RXTHREAD_STACKSIZE);

  ret = pthread_create(&lif->rx_thread, &attr, nuttx_lwip_rx_thread, lif);
  if (ret != 0)
    {
      pthread_attr_destroy(&attr);
      return -ret;
    }

  pthread_attr_destroy(&attr);
  return 0;
}

int nuttx_lwipif_rx_ready(struct nuttx_lwip_if *lif)
{
  if (lif == NULL)
    {
      return -EINVAL;
    }

  if (lif->rx_pending)
    {
      return 0;
    }

  lif->rx_pending = true;
  return sem_post(&lif->rx_sem);
}

int nuttx_lwipif_link_event(struct nuttx_lwip_if *lif, bool up)
{
  if (lif == NULL)
    {
      return -EINVAL;
    }

  lif->link_up = up;

  if (lif->link_pending)
    {
      return 0;
    }

  lif->link_pending = true;
  return sem_post(&lif->rx_sem);
}

int nuttx_lwip_add_interface(struct nuttx_lwip_if *lif,
                             const struct nuttx_lwip_link_ops *ops,
                             void *priv,
                             const char *ifname,
                             bool default_if,
                             const ip4_addr_t *ipaddr,
                             const ip4_addr_t *netmask,
                             const ip4_addr_t *gw)
{
  err_t ret;

  if (lif == NULL || ops == NULL || ifname == NULL)
    {
      return -EINVAL;
    }

  memset(lif, 0, sizeof(*lif));
  lif->ops        = ops;
  lif->priv       = priv;
  lif->default_if = default_if;
  lif->ifname[0]  = ifname[0];
  lif->ifname[1]  = ifname[1] != '\0' ? ifname[1] : '0';
  lif->ifname[2]  = '\0';
  lif->running    = true;

  if (sem_init(&lif->rx_sem, 0, 0) != 0)
    {
      return -errno;
    }

  lif->nif.state = lif;
  lif->nif.name[0] = lif->ifname[0];
  lif->nif.name[1] = lif->ifname[1];

  ret = netifapi_netif_add(&lif->nif,
                           (ip4_addr_t *)ipaddr,
                           (ip4_addr_t *)netmask,
                           (ip4_addr_t *)gw,
                           lif,
                           nuttx_ethernetif_init,
                           tcpip_input);
  if (ret != ERR_OK)
    {
      sem_destroy(&lif->rx_sem);
      return -EIO;
    }

  if (ops->ifup != NULL && ops->ifup(priv) != 0)
    {
      netifapi_netif_remove(&lif->nif);
      sem_destroy(&lif->rx_sem);
      return -EIO;
    }

  if (ops->get_mac != NULL)
    {
      uint8_t mac[6];

      ret = ops->get_mac(priv, mac);
      if (ret == 0)
        {
          lif->nif.hwaddr_len = 6;
          memcpy(lif->nif.hwaddr, mac, 6);
        }
    }

  if (nuttx_lwip_start_rx_thread(lif) != 0)
    {
      if (ops->ifdown != NULL)
        {
          ops->ifdown(priv);
        }

      netifapi_netif_remove(&lif->nif);
      sem_destroy(&lif->rx_sem);
      return -EIO;
    }

  /* The receive callback may run as soon as the link is enabled. */

  lif->ready = true;

  if (lif->default_if)
    {
      netifapi_netif_set_default(&lif->nif);
    }

  netifapi_netif_set_up(&lif->nif);
  if (ops->is_link_up != NULL)
    {
      if (ops->is_link_up(priv))
        {
          netifapi_netif_set_link_up(&lif->nif);
        }
      else
        {
          netifapi_netif_set_link_down(&lif->nif);
        }
    }

#if LWIP_IPV6
  netif_create_ip6_linklocal_address(&lif->nif, 1);
#endif

#ifdef CONFIG_NET_LWIP_DHCP
  dhcp_start(&lif->nif);
#endif

  return 0;
}

void nuttx_lwip_remove_interface(struct nuttx_lwip_if *lif)
{
  if (lif == NULL)
    {
      return;
    }

#ifdef CONFIG_NET_LWIP_DHCP
  dhcp_stop(&lif->nif);
#endif

  netifapi_netif_set_down(&lif->nif);
  netifapi_netif_remove(&lif->nif);

  lif->running = false;
  sem_post(&lif->rx_sem);
  pthread_join(lif->rx_thread, NULL);
  sem_destroy(&lif->rx_sem);

  if (lif->ops != NULL && lif->ops->ifdown != NULL)
    {
      lif->ops->ifdown(lif->priv);
    }
}

#endif /* CONFIG_NET_LWIP */
