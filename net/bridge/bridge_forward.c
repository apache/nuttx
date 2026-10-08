/****************************************************************************
 * net/bridge/bridge_forward.c
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

#include <stdbool.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/pkt.h>

#include "devif/devif.h"
#include "netdev/netdev.h"
#include "bridge/bridge.h"

#ifdef CONFIG_NET_BRIDGE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BR_ETHHDR(iob)  ((FAR struct eth_hdr_s *)(IOB_DATA(iob) - ETH_HDRLEN))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bridge_is_multicast
 *
 * Description:
 *   Return true for group (multicast and broadcast) MAC addresses.
 *
 ****************************************************************************/

static inline bool bridge_is_multicast(FAR const uint8_t *mac)
{
  return (mac[0] & 0x01) != 0;
}

/****************************************************************************
 * Name: bridge_is_linklocal
 *
 * Description:
 *   Return true for the IEEE 802.1D reserved group addresses
 *   01-80-C2-00-00-00 to 01-80-C2-00-00-0F, which a bridge must not
 *   forward.
 *
 ****************************************************************************/

static inline bool bridge_is_linklocal(FAR const uint8_t *mac)
{
  return mac[0] == 0x01 && mac[1] == 0x80 && mac[2] == 0xc2 &&
         mac[3] == 0x00 && mac[4] == 0x00 && (mac[5] & 0xf0) == 0x00;
}

/****************************************************************************
 * Name: bridge_is_local
 *
 * Description:
 *   Return true if a unicast address belongs to the bridge device or to one
 *   of its ports.
 *
 ****************************************************************************/

static bool bridge_is_local(FAR struct bridge_s *br, FAR const uint8_t *mac)
{
  int i;

  if (memcmp(mac, br->br_dev.d_mac.ether.ether_addr_octet,
             ETHER_ADDR_LEN) == 0)
    {
      return true;
    }

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct net_driver_s *dev = br->br_ports[i].bp_dev;

      if (dev != NULL &&
          memcmp(mac, dev->d_mac.ether.ether_addr_octet,
                 ETHER_ADDR_LEN) == 0)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: bridge_fdb_lookup
 *
 * Description:
 *   Find the port of a learned MAC address.  Expired entries are released
 *   on the way.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static FAR struct bridge_port_s *
bridge_fdb_lookup(FAR struct bridge_s *br, FAR const uint8_t *mac)
{
  clock_t now = clock_systime_ticks();
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_FDB_SIZE; i++)
    {
      FAR struct bridge_fdb_s *fdb = &br->br_fdb[i];

      if (fdb->fdb_port == NULL)
        {
          continue;
        }

      if (now - fdb->fdb_time > br->br_ageing)
        {
          fdb->fdb_port = NULL;
          continue;
        }

      if (memcmp(fdb->fdb_mac, mac, ETHER_ADDR_LEN) == 0)
        {
          return fdb->fdb_port;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: bridge_fdb_learn
 *
 * Description:
 *   Remember that a source MAC address was seen on a port.  When the
 *   database is full, the least recently seen address is replaced.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static void bridge_fdb_learn(FAR struct bridge_s *br,
                             FAR struct bridge_port_s *port,
                             FAR const uint8_t *mac)
{
  FAR struct bridge_fdb_s *victim = NULL;
  clock_t now = clock_systime_ticks();
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_FDB_SIZE; i++)
    {
      FAR struct bridge_fdb_s *fdb = &br->br_fdb[i];

      if (fdb->fdb_port == NULL)
        {
          if (victim == NULL || victim->fdb_port != NULL)
            {
              victim = fdb;
            }

          continue;
        }

      if (memcmp(fdb->fdb_mac, mac, ETHER_ADDR_LEN) == 0)
        {
          if (fdb->fdb_port != port)
            {
              ninfo("%s: %02x:%02x:%02x:%02x:%02x:%02x moved to %s\n",
                    br->br_dev.d_ifname, mac[0], mac[1], mac[2], mac[3],
                    mac[4], mac[5], port->bp_dev->d_ifname);
            }

          fdb->fdb_port = port;
          fdb->fdb_time = now;
          return;
        }

      if (victim == NULL ||
          (victim->fdb_port != NULL &&
           now - fdb->fdb_time > now - victim->fdb_time))
        {
          victim = fdb;
        }
    }

  DEBUGASSERT(victim != NULL);
  memcpy(victim->fdb_mac, mac, ETHER_ADDR_LEN);
  victim->fdb_port = port;
  victim->fdb_time = now;
}

/****************************************************************************
 * Name: bridge_clone
 *
 * Description:
 *   Copy a frame, including the Ethernet header in front of IOB_DATA().
 *
 ****************************************************************************/

static FAR struct iob_s *bridge_clone(FAR struct iob_s *iob)
{
  FAR struct iob_s *clone;

  clone = iob_tryalloc(false);
  if (clone == NULL)
    {
      return NULL;
    }

  iob_reserve(clone, CONFIG_NET_LL_GUARDSIZE);
  if (iob_clone_partial(iob, iob->io_pktlen, 0, clone, 0, false,
                        false) < 0)
    {
      iob_free_chain(clone);
      return NULL;
    }

  memcpy(BR_ETHHDR(clone), BR_ETHHDR(iob), ETH_HDRLEN);
  return clone;
}

/****************************************************************************
 * Name: bridge_enqueue
 *
 * Description:
 *   Queue a frame for transmission on a port, or drop it.  Takes ownership
 *   of the IOB.
 *
 * Returned Value:
 *   True if the frame was queued.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static bool bridge_enqueue(FAR struct bridge_port_s *port,
                           FAR struct iob_s *iob)
{
  FAR struct net_driver_s *dev = port->bp_dev;

  if (IFF_IS_UP(dev->d_flags) &&
      port->bp_txqlen < CONFIG_NET_BRIDGE_TXQ_LEN &&
      iob->io_pktlen + ETH_HDRLEN <= NETDEV_PKTSIZE(dev) &&
      iob_tryadd_queue(iob, &port->bp_txq) >= 0)
    {
      port->bp_txqlen++;
      return true;
    }

  NETDEV_TXERRORS(dev);
  iob_free_chain(iob);
  return false;
}

/****************************************************************************
 * Name: bridge_flood
 *
 * Description:
 *   Queue copies of a frame on all ports except the ingress port.  The
 *   original IOB is not consumed.
 *
 * Returned Value:
 *   True if at least one frame was queued.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static bool bridge_flood(FAR struct bridge_s *br,
                         FAR struct bridge_port_s *inport,
                         FAR struct iob_s *iob)
{
  bool queued = false;
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct bridge_port_s *port = &br->br_ports[i];
      FAR struct iob_s *clone;

      if (port->bp_dev == NULL || port == inport)
        {
          continue;
        }

      clone = bridge_clone(iob);
      if (clone == NULL)
        {
          NETDEV_TXERRORS(port->bp_dev);
          continue;
        }

      queued |= bridge_enqueue(port, clone);
    }

  return queued;
}

/****************************************************************************
 * Name: bridge_txq_full
 *
 * Description:
 *   Return true if the TX queue of any port is full.  The bridge device is
 *   not polled further then, so that frames of the local host are not
 *   dropped.
 *
 ****************************************************************************/

static bool bridge_txq_full(FAR struct bridge_s *br)
{
  bool full = false;
  int i;

  nxmutex_lock(&br->br_lock);

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct bridge_port_s *port = &br->br_ports[i];

      if (port->bp_dev != NULL &&
          port->bp_txqlen >= CONFIG_NET_BRIDGE_TXQ_LEN)
        {
          full = true;
          break;
        }
    }

  nxmutex_unlock(&br->br_lock);
  return full;
}

/****************************************************************************
 * Name: bridge_txpoll
 *
 * Description:
 *   devif_poll() callback of the bridge device: forward a frame sent by the
 *   local host.
 *
 * Returned Value:
 *   Always 1: stop this poll, so that devif_poll() keeps the poll type
 *   pending and bridge_work() polls again.  TCP sends one segment per
 *   connection and poll, and returning 0 would clear TCP_POLL after the
 *   first segment.
 *
 ****************************************************************************/

static int bridge_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct iob_s *iob = dev->d_iob;

  DEBUGASSERT(dev->d_len > 0 && iob != NULL);

  NETDEV_TXPACKETS(dev);

#ifdef CONFIG_NET_PKT
  pkt_input(dev);
#endif

  netdev_iob_clear(dev);
  bridge_output((FAR struct bridge_s *)dev, iob);
  NETDEV_TXDONE(dev);
  return 1;
}

/****************************************************************************
 * Name: bridge_work
 *
 * Description:
 *   Poll the bridge device for frames from the local host, then notify the
 *   ports that have frames to send.
 *
 ****************************************************************************/

static void bridge_work(FAR void *arg)
{
  FAR struct bridge_s *br = arg;
  FAR struct net_driver_s *dev = &br->br_dev;
  int i;

  nxmutex_lock(&br->br_cfglock);

  /* Take frames from the local host until there are no more, or until a
   * port queue is full.  In the latter case the poll type stays pending and
   * bridge_poll() schedules this work again when the port takes frames.
   */

  netdev_lock(dev);
  if (IFF_IS_UP(dev->d_flags))
    {
      DEBUGASSERT(dev->d_buf == NULL);
      while (dev->d_polltype != 0 && !bridge_txq_full(br) &&
             devif_poll(dev, bridge_txpoll) != 0);
    }

  netdev_unlock(dev);

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct bridge_port_s *port = &br->br_ports[i];
      bool pending;

      nxmutex_lock(&br->br_lock);
      pending = port->bp_dev != NULL && port->bp_txqlen > 0;
      nxmutex_unlock(&br->br_lock);

      if (pending)
        {
          netdev_txnotify_dev(port->bp_dev, BRIDGE_POLL);
        }
    }

  nxmutex_unlock(&br->br_cfglock);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bridge_schedule
 ****************************************************************************/

void bridge_schedule(FAR struct bridge_s *br)
{
  if (work_available(&br->br_work))
    {
      work_queue(LPWORK, &br->br_work, bridge_work, br, 0);
    }
}

/****************************************************************************
 * Name: bridge_port_flush
 ****************************************************************************/

void bridge_port_flush(FAR struct bridge_port_s *port)
{
  FAR struct bridge_s *br = port->bp_bridge;
  int i;

  iob_free_queue(&port->bp_txq);
  port->bp_txqlen = 0;

  for (i = 0; i < CONFIG_NET_BRIDGE_FDB_SIZE; i++)
    {
      if (br->br_fdb[i].fdb_port == port)
        {
          br->br_fdb[i].fdb_port = NULL;
        }
    }
}

/****************************************************************************
 * Name: bridge_input
 ****************************************************************************/

int bridge_input(FAR struct net_driver_s *dev)
{
  FAR struct bridge_port_s *port = dev->d_bridge;
  FAR struct bridge_s *br = port->bp_bridge;
  FAR struct eth_hdr_s *eth;
  FAR struct iob_s *iob;
  FAR uint8_t *buf = NULL;
  bool local = false;
  bool queued = false;

  netdev_lock(dev);

  /* Drivers using a flat buffer pass the frame in d_buf */

  if (dev->d_iob == NULL)
    {
      buf = dev->d_buf;
      if (netdev_iob_prepare(dev, false, 0) != OK ||
          iob_trycopyin(dev->d_iob, buf, dev->d_len, -ETH_HDRLEN,
                        false) != dev->d_len)
        {
          NETDEV_RXDROPPED(dev);
          goto drop;
        }
    }

  if (dev->d_len < ETH_HDRLEN || !IFF_IS_UP(br->br_dev.d_flags))
    {
      NETDEV_RXDROPPED(dev);
      goto drop;
    }

  /* Take the frame */

  iob = dev->d_iob;
  dev->d_iob = NULL;
  eth = BR_ETHHDR(iob);

  nxmutex_lock(&br->br_lock);

  if (!bridge_is_multicast(eth->src))
    {
      bridge_fdb_learn(br, port, eth->src);
    }

  if (bridge_is_multicast(eth->dest))
    {
      /* Group addresses go to the local host and, except for the reserved
       * link-local ones, to all other ports.
       */

      if (!bridge_is_linklocal(eth->dest))
        {
          queued = bridge_flood(br, port, iob);
        }

      local = true;
    }
  else if (bridge_is_local(br, eth->dest))
    {
      local = true;
    }
  else
    {
      FAR struct bridge_port_s *outport = bridge_fdb_lookup(br, eth->dest);

      if (outport == NULL)
        {
          /* Unknown destination, flood it */

          queued = bridge_flood(br, port, iob);
          iob_free_chain(iob);
        }
      else if (outport != port)
        {
          queued = bridge_enqueue(outport, iob);
        }
      else
        {
          /* The destination is on the ingress segment */

          iob_free_chain(iob);
        }
    }

  nxmutex_unlock(&br->br_lock);

  if (queued)
    {
      bridge_schedule(br);
    }

  if (local)
    {
      bridge_local_input(br, iob);
    }

drop:
  netdev_iob_release(dev);
  if (buf != NULL)
    {
      dev->d_buf = buf;
    }

  dev->d_len = 0;
  netdev_unlock(dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_local_input
 ****************************************************************************/

void bridge_local_input(FAR struct bridge_s *br, FAR struct iob_s *iob)
{
  FAR struct net_driver_s *dev = &br->br_dev;
  uint16_t type = BR_ETHHDR(iob)->type;

  netdev_lock(dev);

  netdev_iob_replace_l2(dev, iob);
  NETDEV_RXPACKETS(dev);

#ifdef CONFIG_NET_PKT
  pkt_input(dev);
#endif

#ifdef CONFIG_NET_IPv4
  if (type == HTONS(ETHTYPE_IP))
    {
      NETDEV_RXIPV4(dev);
      ipv4_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (type == HTONS(ETHTYPE_IP6))
    {
      NETDEV_RXIPV6(dev);
      ipv6_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (type == HTONS(ETHTYPE_ARP))
    {
      NETDEV_RXARP(dev);
      arp_input(dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(dev);
      dev->d_len = 0;
    }

  /* Forward the reply, if any */

  if (dev->d_len > 0 && dev->d_iob != NULL)
    {
      iob = dev->d_iob;
      netdev_iob_clear(dev);
      NETDEV_TXPACKETS(dev);
      bridge_output(br, iob);
      NETDEV_TXDONE(dev);
    }

  netdev_iob_release(dev);
  dev->d_len = 0;
  netdev_unlock(dev);
}

/****************************************************************************
 * Name: bridge_output
 ****************************************************************************/

void bridge_output(FAR struct bridge_s *br, FAR struct iob_s *iob)
{
  FAR struct eth_hdr_s *eth = BR_ETHHDR(iob);
  FAR struct bridge_port_s *outport = NULL;
  bool queued;

  nxmutex_lock(&br->br_lock);

  if (!bridge_is_multicast(eth->dest))
    {
      outport = bridge_fdb_lookup(br, eth->dest);
    }

  if (outport != NULL)
    {
      queued = bridge_enqueue(outport, iob);
    }
  else
    {
      queued = bridge_flood(br, NULL, iob);
      iob_free_chain(iob);
    }

  nxmutex_unlock(&br->br_lock);

  if (queued)
    {
      bridge_schedule(br);
    }
}

/****************************************************************************
 * Name: bridge_poll
 ****************************************************************************/

int bridge_poll(FAR struct net_driver_s *dev,
                devif_poll_callback_t callback)
{
  FAR struct bridge_port_s *port = dev->d_bridge;
  FAR struct iob_s *iob;
  bool sent = false;
  int bstop = 0;

  if (port == NULL)
    {
      return 0;
    }

  while (!bstop)
    {
      nxmutex_lock(&port->bp_bridge->br_lock);
      iob = iob_remove_queue(&port->bp_txq);
      if (iob != NULL)
        {
          port->bp_txqlen--;
        }

      nxmutex_unlock(&port->bp_bridge->br_lock);

      if (iob == NULL)
        {
          break;
        }

      netdev_iob_replace_l2(dev, iob);
      sent = true;
      bstop = callback(dev);
    }

  /* Leave a clean buffer for the next poll handlers, the same way as
   * devif_poll_queue() does.
   */

  if (!bstop && sent)
    {
      if (dev->d_iob != NULL)
        {
          iob_update_pktlen(dev->d_iob, 0, false);
        }

      netdev_iob_prepare(dev, true, 0);
    }

  /* The queue has room again, resume a poll of the bridge device that
   * stopped on a full queue.
   */

  if (sent && port->bp_bridge->br_dev.d_polltype != 0)
    {
      bridge_schedule(port->bp_bridge);
    }

  return bstop;
}

#endif /* CONFIG_NET_BRIDGE */
