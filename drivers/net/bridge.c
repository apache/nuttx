/****************************************************************************
 * drivers/net/bridge.c
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
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <net/if.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/mutex.h>
#include <nuttx/net/bridge.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/wqueue.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Locking.  Locks are always taken in this order, never the reverse:
 *
 *   g_bridge_lock -> port device lock -> bridge device lock -> br_lock
 *
 * g_bridge_lock serializes the configuration (ioctl and the unregistering
 * of a port).  br_lock protects the filtering database and the port table,
 * and serializes the transmission to the ports.  Nothing waits for a
 * device lock with br_lock held: bridge_port_send() only tries the lock of
 * the egress port, and reports the port busy when the attempt fails.
 *
 * The transmit operation of a port is called with br_lock held, and a port
 * driver whose transmit completes synchronously reaches bridge_txdone()
 * from there.  Resuming the bridge device takes its device lock, which
 * would invert the order above, so bridge_txdone() defers the resume to
 * the work queue whenever it finds br_lock held by the calling thread.
 */

struct bridge_s;

/* A bridge port */

struct bridge_port_s
{
  FAR struct bridge_s *bp_bridge;         /* The bridge of this port */
  FAR struct netdev_lowerhalf_s *bp_dev;  /* The port device, NULL = unused */
};

/* Like the filtering database, the used ports are always at the head of the
 * port table, so the walks of the table stop at the first unused port.
 */

/* A filtering database entry.  The used entries are always at the head of
 * the table, so the walks of the table stop at the first free entry.
 */

struct bridge_fdb_s
{
  FAR struct bridge_port_s *fdb_port;     /* Port of the address, NULL = free */
  clock_t fdb_time;                       /* Last time the address was seen */
  uint8_t fdb_mac[ETHER_ADDR_LEN];        /* The learned MAC address */
};

/* A bridge */

struct bridge_s
{
  struct netdev_lowerhalf_s br_dev;       /* The bridge device, must be first */
  FAR struct bridge_s *br_flink;          /* Next bridge in the list */
  mutex_t br_lock;                        /* FDB, ports and TX lock */
  struct work_s br_txdone;                /* Deferred resume, see bridge_txdone */
  struct bridge_port_s br_ports[CONFIG_NET_BRIDGE_MAX_PORTS];
  struct bridge_fdb_s br_fdb[CONFIG_NET_BRIDGE_FDB_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bridge_ifup(FAR struct netdev_lowerhalf_s *dev);
static int bridge_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int bridge_transmit(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt);
static FAR netpkt_t *bridge_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int bridge_addmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac);
static int bridge_rmmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_bridge_ops =
{
  bridge_ifup,     /* ifup */
  bridge_ifdown,   /* ifdown */
  bridge_transmit, /* transmit */
  bridge_receive,  /* receive */
#ifdef CONFIG_NET_MCASTGROUP
  bridge_addmac,   /* addmac */
  bridge_rmmac,    /* rmmac */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  NULL             /* ioctl */
#endif
};

/* All bridges, protected by g_bridge_lock */

static FAR struct bridge_s *g_bridges;
static mutex_t g_bridge_lock = NXMUTEX_INITIALIZER;

static const uint8_t g_zero_mac[ETHER_ADDR_LEN];

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
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static bool bridge_is_local(FAR struct bridge_s *br, FAR const uint8_t *mac)
{
  int i;

  if (memcmp(mac, br->br_dev.netdev.d_mac.ether.ether_addr_octet,
             ETHER_ADDR_LEN) == 0)
    {
      return true;
    }

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct netdev_lowerhalf_s *dev = br->br_ports[i].bp_dev;

      if (dev == NULL)
        {
          break;
        }

      if (memcmp(mac, dev->netdev.d_mac.ether.ether_addr_octet,
                 ETHER_ADDR_LEN) == 0)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: bridge_fdb_remove
 *
 * Description:
 *   Release an entry, moving the last used entry into its place to keep the
 *   used entries at the head of the table.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static void bridge_fdb_remove(FAR struct bridge_s *br, int index)
{
  int last = index;

  while (last + 1 < CONFIG_NET_BRIDGE_FDB_SIZE &&
         br->br_fdb[last + 1].fdb_port != NULL)
    {
      last++;
    }

  br->br_fdb[index]         = br->br_fdb[last];
  br->br_fdb[last].fdb_port = NULL;
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
  int i = 0;

  while (i < CONFIG_NET_BRIDGE_FDB_SIZE)
    {
      FAR struct bridge_fdb_s *fdb = &br->br_fdb[i];

      if (fdb->fdb_port == NULL)
        {
          break;
        }

      if (now - fdb->fdb_time > SEC2TICK(CONFIG_NET_BRIDGE_AGEING_TIME))
        {
          /* The last entry took the place of this one, check it too */

          bridge_fdb_remove(br, i);
          continue;
        }

      if (memcmp(fdb->fdb_mac, mac, ETHER_ADDR_LEN) == 0)
        {
          return fdb->fdb_port;
        }

      i++;
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

      /* The first free entry ends the used ones, and is where to add */

      if (fdb->fdb_port == NULL)
        {
          victim = fdb;
          break;
        }

      if (memcmp(fdb->fdb_mac, mac, ETHER_ADDR_LEN) == 0)
        {
          if (fdb->fdb_port != port)
            {
              ninfo("%s: %02x:%02x:%02x:%02x:%02x:%02x moved to %s\n",
                    br->br_dev.netdev.d_ifname, mac[0], mac[1], mac[2],
                    mac[3], mac[4], mac[5], port->bp_dev->netdev.d_ifname);
            }

          fdb->fdb_port = port;
          fdb->fdb_time = now;
          return;
        }

      if (victim == NULL || now - fdb->fdb_time > now - victim->fdb_time)
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
 *   Copy a packet, including the Ethernet header, which precedes the data.
 *
 ****************************************************************************/

static FAR netpkt_t *bridge_clone(FAR struct bridge_s *br,
                                  FAR netpkt_t *pkt)
{
  FAR struct netdev_lowerhalf_s *dev = &br->br_dev;
  FAR netpkt_t *clone;

  clone = iob_tryalloc(false);
  if (clone == NULL)
    {
      return NULL;
    }

  iob_reserve(clone, CONFIG_NET_LL_GUARDSIZE);
  if (iob_clone_partial(pkt, pkt->io_pktlen, 0, clone, 0, false,
                        false) < 0)
    {
      iob_free_chain(clone);
      return NULL;
    }

  memcpy(netpkt_getdata(dev, clone), netpkt_getdata(dev, pkt), ETH_HDRLEN);
  return clone;
}

/****************************************************************************
 * Name: bridge_port_send
 *
 * Description:
 *   Try to send a packet on a port, bypassing the upper half of the port,
 *   which only serves the packets of the local host.  Follows the transmit
 *   contract of the lower half: the port owns the packet when OK is
 *   returned, the caller still owns it on failure.
 *
 * Returned Value:
 *   OK on success, -EBUSY when the port cannot take the packet now and the
 *   same packet can be offered again later, or another negated errno value.
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static int bridge_port_send(FAR struct bridge_port_s *port,
                            FAR netpkt_t *pkt)
{
  FAR struct netdev_lowerhalf_s *dev = port->bp_dev;
  int ret;

  if (!IFF_IS_UP(dev->netdev.d_flags))
    {
      return -ENETDOWN;
    }

  /* The upper half of the port would do this check for its own packets */

  if (netpkt_getdatalen(dev, pkt) > NETDEV_PKTSIZE(&dev->netdev))
    {
      return -EMSGSIZE;
    }

  /* The transmit operation of the port is also reached under the device
   * lock of the port, by the upper half serving the local host, and a port
   * driver may count on that lock to serialize its transmit state.  Waiting
   * for it here would invert the order documented above, as the RX path of
   * a port takes its device lock before br_lock, so a port that is busy
   * transmitting is reported full instead of waited for.
   */

  if (netdev_trylock(&dev->netdev) < 0)
    {
      return -EBUSY;
    }

  NETDEV_TXPACKETS(&dev->netdev);
  ret = dev->ops->transmit(dev, pkt);
  netdev_unlock(&dev->netdev);

  return ret;
}

/****************************************************************************
 * Name: bridge_port_drop
 *
 * Description:
 *   Account and release a packet that a port could not take.
 *
 ****************************************************************************/

static void bridge_port_drop(FAR struct bridge_port_s *port,
                             FAR netpkt_t *pkt)
{
  NETDEV_TXERRORS(&port->bp_dev->netdev);
  iob_free_chain(pkt);
}

/****************************************************************************
 * Name: bridge_flood
 *
 * Description:
 *   Send copies of a packet on all ports except the ingress port.
 *
 * Input Parameters:
 *   br      - The bridge
 *   inport  - The ingress port, NULL for a packet of the local host
 *   pkt     - The packet
 *   consume - True if the packet itself can be sent on the last port, in
 *             which case the packet is owned by the bridge afterwards
 *
 *   A port that has no room drops its copy instead of holding the packet
 *   back: the other ports have already taken theirs, so a retry of the
 *   whole flood would duplicate them.
 *
 * Returned Value:
 *   True if the packet was consumed.  False if the caller still owns it
 *   (always the case when consume is false, or when no port was found).
 *
 * Assumptions:
 *   Called with br_lock held.
 *
 ****************************************************************************/

static bool bridge_flood(FAR struct bridge_s *br,
                         FAR struct bridge_port_s *inport,
                         FAR netpkt_t *pkt, bool consume)
{
  FAR struct bridge_port_s *last = NULL;
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct bridge_port_s *port = &br->br_ports[i];
      FAR netpkt_t *clone;

      if (port->bp_dev == NULL)
        {
          break;
        }

      if (port == inport)
        {
          continue;
        }

      /* Send on the previous port, now that we know there is one more */

      if (last != NULL)
        {
          clone = bridge_clone(br, pkt);
          if (clone != NULL)
            {
              if (bridge_port_send(last, clone) < 0)
                {
                  bridge_port_drop(last, clone);
                }
            }
          else
            {
              NETDEV_TXERRORS(&last->bp_dev->netdev);
            }
        }

      last = port;
    }

  if (last == NULL)
    {
      return false;
    }

  if (consume)
    {
      if (bridge_port_send(last, pkt) < 0)
        {
          bridge_port_drop(last, pkt);
        }

      return true;
    }

  pkt = bridge_clone(br, pkt);
  if (pkt != NULL)
    {
      if (bridge_port_send(last, pkt) < 0)
        {
          bridge_port_drop(last, pkt);
        }
    }
  else
    {
      NETDEV_TXERRORS(&last->bp_dev->netdev);
    }

  return false;
}

/****************************************************************************
 * Name: bridge_ifup
 ****************************************************************************/

static int bridge_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_lower_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_ifdown
 ****************************************************************************/

static int bridge_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: bridge_transmit
 *
 * Description:
 *   Send a packet of the local host through the bridge.
 *
 * Returned Value:
 *   OK when the packet is gone, or -EBUSY to let the upper half queue it
 *   and offer it again when bridge_txdone() reports room on the egress
 *   port.  Only the single-port case can be retried, see bridge_flood().
 *
 ****************************************************************************/

static int bridge_transmit(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt)
{
  FAR struct bridge_s *br = (FAR struct bridge_s *)dev;
  FAR const uint8_t *dest = netpkt_getdata(dev, pkt);
  FAR struct bridge_port_s *outport = NULL;
  int ret = OK;

  nxmutex_lock(&br->br_lock);

  if (!bridge_is_multicast(dest))
    {
      outport = bridge_fdb_lookup(br, dest);
    }

  if (outport != NULL)
    {
      ret = bridge_port_send(outport, pkt);
      if (ret < 0 && ret != -EBUSY)
        {
          bridge_port_drop(outport, pkt);
          ret = OK;
        }
    }
  else if (!bridge_flood(br, NULL, pkt, true))
    {
      /* No port at all, nobody took the packet */

      iob_free_chain(pkt);
    }

  nxmutex_unlock(&br->br_lock);
  return ret;
}

/****************************************************************************
 * Name: bridge_receive
 ****************************************************************************/

static FAR netpkt_t *bridge_receive(FAR struct netdev_lowerhalf_s *dev)
{
  /* The bridge device doesn't receive packets, its ports do. */

  return NULL;
}

/****************************************************************************
 * Name: bridge_addmac
 *
 * Description:
 *   The ports receive all multicast packets, as they are flooded.  Let
 *   them know the group too, in case the hardware filters.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int bridge_addmac(FAR struct netdev_lowerhalf_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct bridge_s *br = (FAR struct bridge_s *)dev;
  int i;

  nxmutex_lock(&br->br_lock);
  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct netdev_lowerhalf_s *port = br->br_ports[i].bp_dev;

      if (port == NULL)
        {
          break;
        }

      if (port->ops->addmac != NULL)
        {
          port->ops->addmac(port, mac);
        }
    }

  nxmutex_unlock(&br->br_lock);
  return OK;
}

/****************************************************************************
 * Name: bridge_rmmac
 ****************************************************************************/

static int bridge_rmmac(FAR struct netdev_lowerhalf_s *dev,
                        FAR const uint8_t *mac)
{
  FAR struct bridge_s *br = (FAR struct bridge_s *)dev;
  int i;

  nxmutex_lock(&br->br_lock);
  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct netdev_lowerhalf_s *port = br->br_ports[i].bp_dev;

      if (port == NULL)
        {
          break;
        }

      if (port->ops->rmmac != NULL)
        {
          port->ops->rmmac(port, mac);
        }
    }

  nxmutex_unlock(&br->br_lock);
  return OK;
}
#endif

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
      if (strncmp(br->br_dev.netdev.d_ifname, name, IFNAMSIZ) == 0)
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
 *   Called with g_bridge_lock held.
 *
 ****************************************************************************/

static void bridge_update_pktsize(FAR struct bridge_s *br)
{
  uint16_t pktsize = UINT16_MAX;
  int i;

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct netdev_lowerhalf_s *dev = br->br_ports[i].bp_dev;

      if (dev == NULL)
        {
          break;
        }

      if (NETDEV_PKTSIZE(&dev->netdev) < pktsize)
        {
          pktsize = NETDEV_PKTSIZE(&dev->netdev);
        }
    }

  br->br_dev.netdev.d_pktsize = pktsize != UINT16_MAX ? pktsize :
                                CONFIG_NET_ETH_PKTSIZE;
}

/****************************************************************************
 * Name: bridge_delport
 *
 * Description:
 *   Remove a port from its bridge.
 *
 * Assumptions:
 *   Called with g_bridge_lock held.
 *
 ****************************************************************************/

static void bridge_delport(FAR struct bridge_port_s *port)
{
  FAR struct bridge_s *br = port->bp_bridge;
  FAR struct netdev_lowerhalf_s *dev = port->bp_dev;
  FAR struct netdev_lowerhalf_s *moved = NULL;
  FAR struct bridge_port_s *last = port;
  int i;

  /* Stop the RX path of the port first, then drop its state */

  netdev_lower_bridge_set(dev, NULL);

  /* The last used port takes the place of this one, to keep the used ports
   * at the head of the table.  The ports are only changed with
   * g_bridge_lock held, so they can be read without br_lock.  The RX path
   * of the moved port is stopped while its port structure changes, its
   * frames go to the network stack in the meantime.
   */

  for (i = port - br->br_ports + 1;
       i < CONFIG_NET_BRIDGE_MAX_PORTS && br->br_ports[i].bp_dev != NULL;
       i++)
    {
      last = &br->br_ports[i];
    }

  if (last != port)
    {
      moved = last->bp_dev;
      netdev_lower_bridge_set(moved, NULL);
    }

  nxmutex_lock(&br->br_lock);

  i = 0;
  while (i < CONFIG_NET_BRIDGE_FDB_SIZE && br->br_fdb[i].fdb_port != NULL)
    {
      if (br->br_fdb[i].fdb_port == port)
        {
          /* The last entry took the place of this one, check it too */

          bridge_fdb_remove(br, i);
        }
      else
        {
          i++;
        }
    }

  port->bp_dev = moved;

  if (moved != NULL)
    {
      for (i = 0; i < CONFIG_NET_BRIDGE_FDB_SIZE &&
                  br->br_fdb[i].fdb_port != NULL; i++)
        {
          if (br->br_fdb[i].fdb_port == last)
            {
              br->br_fdb[i].fdb_port = port;
            }
        }

      last->bp_dev = NULL;
    }

  nxmutex_unlock(&br->br_lock);

  if (moved != NULL)
    {
      netdev_lower_bridge_set(moved, port);
    }

  bridge_update_pktsize(br);
  ninfo("%s: removed port %s\n", br->br_dev.netdev.d_ifname,
        dev->netdev.d_ifname);
}

/****************************************************************************
 * Name: bridge_addbr
 ****************************************************************************/

static int bridge_addbr(FAR const char *name)
{
  FAR struct bridge_s *br;
  int ret;
  int i;

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

  nxmutex_init(&br->br_lock);

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      br->br_ports[i].bp_bridge = br;
    }

  br->br_dev.ops    = &g_bridge_ops;
  br->br_dev.rxtype = NETDEV_RX_DIRECT;
  strlcpy(br->br_dev.netdev.d_ifname, name, IFNAMSIZ);

  ret = netdev_lower_register(&br->br_dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
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

  if (IFF_IS_UP(br->br_dev.netdev.d_flags))
    {
      return -EBUSY;
    }

  for (pprev = &g_bridges; *pprev != br; pprev = &(*pprev)->br_flink);
  *pprev = br->br_flink;

  for (i = CONFIG_NET_BRIDGE_MAX_PORTS - 1; i >= 0; i--)
    {
      if (br->br_ports[i].bp_dev != NULL)
        {
          bridge_delport(&br->br_ports[i]);
        }
    }

  /* A deferred resume may still reference the bridge, and it must be gone
   * before the unregistering frees the upper half it resumes.
   */

  work_cancel_sync(LPWORK, &br->br_txdone);

  netdev_lower_unregister(&br->br_dev);

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
  FAR struct netdev_lowerhalf_s *dev;
  FAR struct net_driver_s *netdev;
  FAR struct bridge_s *br;
  int i;

  br = bridge_find(req->ifr_name);
  if (br == NULL)
    {
      return -ENODEV;
    }

  netdev = netdev_findbyindex(req->ifr_ifindex);
  if (netdev == NULL)
    {
      return -ENODEV;
    }

  /* Only upper half devices can be ports, because their receive path
   * hands the packets to the bridge.
   */

  dev = netdev_lower_get(netdev);
  if (dev == NULL)
    {
      return -EOPNOTSUPP;
    }

  /* Only Ethernet-like devices that are not a bridge can be ports */

  if ((netdev->d_lltype != NET_LL_ETHERNET &&
       netdev->d_lltype != NET_LL_IEEE80211) ||
      netdev->d_llhdrlen != ETH_HDRLEN || dev->ops == &g_bridge_ops)
    {
      return -EINVAL;
    }

  /* A device can only be a port of one bridge.  Checked over all bridges,
   * because attaching it here would overwrite the port it already points
   * to, leaving the other bridge with a port it no longer receives from.
   */

  if (netdev_lower_bridge_get(dev) != NULL)
    {
      return -EBUSY;
    }

  nxmutex_lock(&br->br_lock);

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      if (br->br_ports[i].bp_dev == dev)
        {
          nxmutex_unlock(&br->br_lock);
          return -EBUSY;
        }

      /* The first unused port ends the used ones, and is where to add */

      if (br->br_ports[i].bp_dev == NULL)
        {
          port = &br->br_ports[i];
          break;
        }
    }

  if (port == NULL)
    {
      nxmutex_unlock(&br->br_lock);
      return -ENOSPC;
    }

  /* Like Linux, a bridge without an address takes the one of its first
   * port.
   */

  if (memcmp(br->br_dev.netdev.d_mac.ether.ether_addr_octet,
             g_zero_mac, ETHER_ADDR_LEN) == 0)
    {
      memcpy(br->br_dev.netdev.d_mac.ether.ether_addr_octet,
             netdev->d_mac.ether.ether_addr_octet, ETHER_ADDR_LEN);
    }

  port->bp_dev = dev;
  nxmutex_unlock(&br->br_lock);

  bridge_update_pktsize(br);

  /* Attach last: from now on the RX path of the port feeds the bridge */

  netdev_lower_bridge_set(dev, port);

  ninfo("%s: added port %s\n", br->br_dev.netdev.d_ifname, netdev->d_ifname);
  return OK;
}

/****************************************************************************
 * Name: bridge_delif
 ****************************************************************************/

static int bridge_delif(FAR const struct ifreq *req)
{
  FAR struct bridge_s *br;
  int i;

  br = bridge_find(req->ifr_name);
  if (br == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS; i++)
    {
      FAR struct netdev_lowerhalf_s *dev = br->br_ports[i].bp_dev;

      if (dev == NULL)
        {
          break;
        }

      if (dev->netdev.d_ifindex == req->ifr_ifindex)
        {
          bridge_delport(&br->br_ports[i]);
          return OK;
        }
    }

  return netdev_findbyindex(req->ifr_ifindex) == NULL ? -ENODEV : -EINVAL;
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
 * Name: bridge_input
 ****************************************************************************/

void bridge_input(FAR struct bridge_port_s *port, FAR netpkt_t *pkt)
{
  FAR struct bridge_s *br = port->bp_bridge;
  FAR struct netdev_lowerhalf_s *dev = &br->br_dev;
  FAR struct eth_hdr_s *eth;
  bool local = false;

  if (netpkt_getdatalen(dev, pkt) < ETH_HDRLEN ||
      !IFF_IS_UP(dev->netdev.d_flags))
    {
      NETDEV_RXDROPPED(&port->bp_dev->netdev);
      iob_free_chain(pkt);
      return;
    }

  eth = (FAR struct eth_hdr_s *)netpkt_getdata(dev, pkt);

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
          bridge_flood(br, port, pkt, false);
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

          if (!bridge_flood(br, port, pkt, true))
            {
              iob_free_chain(pkt);
            }
        }
      else if (outport != port)
        {
          if (bridge_port_send(outport, pkt) < 0)
            {
              bridge_port_drop(outport, pkt);
            }
        }
      else
        {
          /* The destination is on the ingress segment */

          iob_free_chain(pkt);
        }
    }

  nxmutex_unlock(&br->br_lock);

  /* The local host is served outside of br_lock: the stack may reply
   * right away, which comes back to bridge_transmit().
   */

  if (local)
    {
      netdev_lower_input(dev, pkt);
    }
}

/****************************************************************************
 * Name: bridge_txdone_work
 ****************************************************************************/

static void bridge_txdone_work(FAR void *arg)
{
  netdev_lower_txdone(&((FAR struct bridge_s *)arg)->br_dev);
}

/****************************************************************************
 * Name: bridge_txdone
 ****************************************************************************/

void bridge_txdone(FAR struct bridge_port_s *port)
{
  FAR struct bridge_s *br = port->bp_bridge;

  /* Resuming the bridge device takes its device lock and enters the network
   * stack, so it cannot be done here in two cases: in an interrupt handler,
   * which a port driver may report a completed transmission from, and with
   * br_lock held, which must not be held when a device lock is taken.  A
   * port whose transmit operation completes synchronously gets here from
   * bridge_port_send(), that is from under br_lock.
   */

  if (up_interrupt_context() || nxmutex_is_hold(&br->br_lock))
    {
      if (work_available(&br->br_txdone))
        {
          work_queue(LPWORK, &br->br_txdone, bridge_txdone_work, br, 0);
        }
    }
  else
    {
      netdev_lower_txdone(&br->br_dev);
    }
}

/****************************************************************************
 * Name: bridge_port_detach_locked
 *
 * Description:
 *   Remove a device from the bridge it is a port of, if any.
 *
 * Assumptions:
 *   Called with g_bridge_lock held.
 *
 ****************************************************************************/

static void bridge_port_detach_locked(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct bridge_s *br;
  int i;

  /* The port structure of the device may have moved since the caller looked
   * at it, so find it again under the lock.
   */

  for (br = g_bridges; br != NULL; br = br->br_flink)
    {
      for (i = 0; i < CONFIG_NET_BRIDGE_MAX_PORTS &&
                  br->br_ports[i].bp_dev != NULL; i++)
        {
          if (br->br_ports[i].bp_dev == dev)
            {
              bridge_delport(&br->br_ports[i]);
              return;
            }
        }
    }
}

/****************************************************************************
 * Name: bridge_port_detach
 ****************************************************************************/

void bridge_port_detach(FAR struct netdev_lowerhalf_s *dev)
{
  /* Reached from netdev_lower_unregister(), which bridge_delbr() calls with
   * g_bridge_lock held to unregister the bridge device itself.  The ports
   * are already gone by then, so there is nothing to look up and the lock
   * must not be taken again.
   */

  if (nxmutex_is_hold(&g_bridge_lock))
    {
      return;
    }

  nxmutex_lock(&g_bridge_lock);
  bridge_port_detach_locked(dev);
  nxmutex_unlock(&g_bridge_lock);
}
