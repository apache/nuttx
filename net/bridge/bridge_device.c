/****************************************************************************
 * net/bridge/bridge_device.c
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

#include <errno.h>
#include <string.h>

#include <net/if.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "bridge/bridge.h"

#ifdef CONFIG_NET_BRIDGE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bridge_ifup(FAR struct net_driver_s *dev);
static int bridge_ifdown(FAR struct net_driver_s *dev);
static int bridge_txavail(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* All bridges, protected by g_bridge_lock */

static FAR struct bridge_s *g_bridges;
static const uint8_t g_zero_mac[ETHER_ADDR_LEN];
static mutex_t g_bridge_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bridge_ifup
 ****************************************************************************/

static int bridge_ifup(FAR struct net_driver_s *dev)
{
  netdev_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_ifdown
 ****************************************************************************/

static int bridge_ifdown(FAR struct net_driver_s *dev)
{
  netdev_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_txavail
 *
 * Description:
 *   The network stack has frames to send through the bridge.  The poll is
 *   done by the bridge work, so that it never runs inside the RX path of
 *   the bridge device.
 *
 ****************************************************************************/

static int bridge_txavail(FAR struct net_driver_s *dev)
{
  bridge_schedule((FAR struct bridge_s *)dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_find
 *
 * Description:
 *   Find a bridge by name.
 *
 * Assumptions:
 *   Called with g_bridge_lock held.
 *
 ****************************************************************************/

static FAR struct bridge_s *bridge_find(FAR const char *name)
{
  FAR struct bridge_s *br;

  for (br = g_bridges; br != NULL; br = br->br_flink)
    {
      if (strncmp(br->br_dev.d_ifname, name, IFNAMSIZ) == 0)
        {
          return br;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: bridge_update_pktsize
 *
 * Description:
 *   The bridge MTU is the smallest MTU of its ports, or the default
 *   Ethernet MTU when it has no ports.
 *
 * Assumptions:
 *   Called with br_cfglock held.
 *
 ****************************************************************************/

static void bridge_update_pktsize(FAR struct bridge_s *br)
{
  uint16_t pktsize = UINT16_MAX;
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct net_driver_s *dev = br->br_ports[i].bp_dev;

      if (dev != NULL && NETDEV_PKTSIZE(dev) < pktsize)
        {
          pktsize = NETDEV_PKTSIZE(dev);
        }
    }

  br->br_dev.d_pktsize = pktsize != UINT16_MAX ? pktsize :
                         CONFIG_NET_ETH_PKTSIZE;
}

/****************************************************************************
 * Name: bridge_delport
 *
 * Description:
 *   Remove a port from its bridge.
 *
 * Assumptions:
 *   Called with br_cfglock held.
 *
 ****************************************************************************/

static void bridge_delport(FAR struct bridge_port_s *port)
{
  FAR struct bridge_s *br = port->bp_bridge;
  FAR struct net_driver_s *dev = port->bp_dev;

  /* Stop the RX path of the port first, then drop its state */

  netdev_lock(dev);
  dev->d_bridge = NULL;
  dev->d_polltype &= ~BRIDGE_POLL;
  netdev_unlock(dev);

  nxmutex_lock(&br->br_lock);
  bridge_port_flush(port);
  port->bp_dev = NULL;
  nxmutex_unlock(&br->br_lock);

  bridge_update_pktsize(br);
  ninfo("%s: removed port %s\n", br->br_dev.d_ifname, dev->d_ifname);
}

/****************************************************************************
 * Name: bridge_addbr
 ****************************************************************************/

static int bridge_addbr(FAR const char *name)
{
  FAR struct bridge_s *br;
  int ret;

  if (name == NULL || name[0] == '\0' || strlen(name) >= IFNAMSIZ ||
      strchr(name, '%') != NULL)
    {
      return -EINVAL;
    }

  if (netdev_findbyname(name) != NULL)
    {
      return -EEXIST;
    }

  br = kmm_zalloc(sizeof(struct bridge_s));
  if (br == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&br->br_cfglock);
  nxmutex_init(&br->br_lock);
  br->br_ageing = SEC2TICK(CONFIG_NET_BRIDGE_AGEING_TIME);

  for (ret = 0; ret < CONFIG_NET_BRIDGE_MAX_PORTS; ret++)
    {
      br->br_ports[ret].bp_bridge = br;
    }

  strlcpy(br->br_dev.d_ifname, name, IFNAMSIZ);
  br->br_dev.d_ifup    = bridge_ifup;
  br->br_dev.d_ifdown  = bridge_ifdown;
  br->br_dev.d_txavail = bridge_txavail;
  br->br_dev.d_private = br;

  ret = netdev_register(&br->br_dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nxmutex_destroy(&br->br_cfglock);
      nxmutex_destroy(&br->br_lock);
      kmm_free(br);
      return ret;
    }

  br->br_flink = g_bridges;
  g_bridges    = br;

  ninfo("%s: created\n", name);
  return OK;
}

/****************************************************************************
 * Name: bridge_delbr
 ****************************************************************************/

static int bridge_delbr(FAR const char *name)
{
  FAR struct bridge_s **pprev;
  FAR struct bridge_s *br;
  int i;

  if (name == NULL)
    {
      return -EINVAL;
    }

  br = bridge_find(name);
  if (br == NULL)
    {
      return -ENODEV;
    }

  /* Like Linux, refuse to delete a bridge that is up */

  if (IFF_IS_UP(br->br_dev.d_flags))
    {
      return -EBUSY;
    }

  for (pprev = &g_bridges; *pprev != br; pprev = &(*pprev)->br_flink);
  *pprev = br->br_flink;

  nxmutex_lock(&br->br_cfglock);
  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      if (br->br_ports[i].bp_dev != NULL)
        {
          bridge_delport(&br->br_ports[i]);
        }
    }

  nxmutex_unlock(&br->br_cfglock);

  work_cancel_sync(LPWORK, &br->br_work);
  netdev_unregister(&br->br_dev);

  nxmutex_destroy(&br->br_cfglock);
  nxmutex_destroy(&br->br_lock);
  kmm_free(br);

  ninfo("%s: deleted\n", name);
  return OK;
}

/****************************************************************************
 * Name: bridge_addif
 ****************************************************************************/

static int bridge_addif(FAR const struct ifreq *req)
{
  FAR struct bridge_port_s *port = NULL;
  FAR struct net_driver_s *dev;
  FAR struct bridge_s *br;
  int i;

  br = bridge_find(req->ifr_name);
  if (br == NULL)
    {
      return -ENODEV;
    }

  dev = netdev_findbyindex(req->ifr_ifindex);
  if (dev == NULL)
    {
      return -ENODEV;
    }

  /* Only Ethernet-like devices that are not a bridge can be ports */

  if ((dev->d_lltype != NET_LL_ETHERNET &&
       dev->d_lltype != NET_LL_IEEE80211) ||
      dev->d_llhdrlen != ETH_HDRLEN || dev->d_txavail == bridge_txavail)
    {
      return -EINVAL;
    }

  if (dev->d_bridge != NULL)
    {
      return -EBUSY;
    }

  nxmutex_lock(&br->br_cfglock);

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      if (br->br_ports[i].bp_dev == NULL)
        {
          port = &br->br_ports[i];
          break;
        }
    }

  if (port == NULL)
    {
      nxmutex_unlock(&br->br_cfglock);
      return -ENOSPC;
    }

  port->bp_dev = dev;
  IOB_QINIT(&port->bp_txq);
  port->bp_txqlen = 0;

  /* Like Linux, a bridge without an address takes the one of its first
   * port.
   */

  if (memcmp(br->br_dev.d_mac.ether.ether_addr_octet,
             g_zero_mac, ETHER_ADDR_LEN) == 0)
    {
      memcpy(br->br_dev.d_mac.ether.ether_addr_octet,
             dev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);
    }

  bridge_update_pktsize(br);

  /* Attach last: from now on the RX path of the port feeds the bridge */

  netdev_lock(dev);
  dev->d_bridge = port;
  netdev_unlock(dev);

  nxmutex_unlock(&br->br_cfglock);

  ninfo("%s: added port %s\n", br->br_dev.d_ifname, dev->d_ifname);
  return OK;
}

/****************************************************************************
 * Name: bridge_delif
 ****************************************************************************/

static int bridge_delif(FAR const struct ifreq *req)
{
  FAR struct net_driver_s *dev;
  FAR struct bridge_s *br;
  int ret = -EINVAL;

  br = bridge_find(req->ifr_name);
  if (br == NULL)
    {
      return -ENODEV;
    }

  dev = netdev_findbyindex(req->ifr_ifindex);
  if (dev == NULL)
    {
      return -ENODEV;
    }

  nxmutex_lock(&br->br_cfglock);
  if (dev->d_bridge != NULL && dev->d_bridge->bp_bridge == br)
    {
      bridge_delport(dev->d_bridge);
      ret = OK;
    }

  nxmutex_unlock(&br->br_cfglock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bridge_ioctl
 ****************************************************************************/

int bridge_ioctl(int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
      case SIOCBRADDBR:
      case SIOCBRDELBR:
      case SIOCBRADDIF:
      case SIOCBRDELIF:
        break;

      default:
        return -ENOTTY;
    }

  if (arg == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_bridge_lock);

  switch (cmd)
    {
      case SIOCBRADDBR:
        ret = bridge_addbr((FAR const char *)(uintptr_t)arg);
        break;

      case SIOCBRDELBR:
        ret = bridge_delbr((FAR const char *)(uintptr_t)arg);
        break;

      case SIOCBRADDIF:
        ret = bridge_addif((FAR const struct ifreq *)(uintptr_t)arg);
        break;

      default:
        ret = bridge_delif((FAR const struct ifreq *)(uintptr_t)arg);
        break;
    }

  nxmutex_unlock(&g_bridge_lock);
  return ret;
}

/****************************************************************************
 * Name: bridge_netdev_unregister
 ****************************************************************************/

void bridge_netdev_unregister(FAR struct net_driver_s *dev)
{
  FAR struct bridge_s *br;

  /* Nothing to do for non-ports, including the bridge devices themselves,
   * which bridge_delbr() unregisters with g_bridge_lock held.
   */

  if (dev->d_bridge == NULL)
    {
      return;
    }

  nxmutex_lock(&g_bridge_lock);
  if (dev->d_bridge != NULL)
    {
      br = dev->d_bridge->bp_bridge;
      nxmutex_lock(&br->br_cfglock);
      bridge_delport(dev->d_bridge);
      nxmutex_unlock(&br->br_cfglock);
    }

  nxmutex_unlock(&g_bridge_lock);
}

#endif /* CONFIG_NET_BRIDGE */
