/****************************************************************************
 * drivers/net/vlan.c
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

#ifdef CONFIG_NET_VLAN

#include <nuttx/debug.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/vlan.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VLAN_DEV_NAME_FMT "%.*s.%u"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vlan_device_s
{
  struct netdev_lowerhalf_s dev;

  FAR struct netdev_lowerhalf_s *real;
  uint16_t tci;
  struct iob_queue_s rxq;
  spinlock_t rxlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vlan_ifup(FAR struct netdev_lowerhalf_s *dev);
static int vlan_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int vlan_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt);
static FAR netpkt_t *vlan_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int vlan_addmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac);
static int vlan_rmmac(FAR struct netdev_lowerhalf_s *dev,
                      FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int vlan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                      unsigned long arg);
#endif
static void vlan_reclaim(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_NETPOLL
static void vlan_txpoll(FAR struct netdev_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_vlan_ops =
{
  vlan_ifup,     /* ifup */
  vlan_ifdown,   /* ifdown */
  vlan_transmit, /* transmit */
  vlan_receive,  /* receive */
#ifdef CONFIG_NET_MCASTGROUP
  vlan_addmac,   /* addmac */
  vlan_rmmac,    /* rmmac */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  vlan_ioctl,    /* ioctl */
#endif
  vlan_reclaim,  /* reclaim */
#ifdef CONFIG_NET_NETPOLL
  vlan_txpoll,   /* txpoll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vlan_ifup
 ****************************************************************************/

static int vlan_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  if (IFF_IS_RUNNING(real->netdev.d_flags))
    {
      netdev_lower_carrier_on(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: vlan_ifdown
 ****************************************************************************/

static int vlan_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: vlan_transmit
 ****************************************************************************/

static int vlan_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;
  FAR struct eth_8021qhdr_s     *vlan_hdr;
  FAR uint8_t                   *base = netpkt_getbase(pkt);
  FAR uint8_t                   *data = netpkt_getdata(dev, pkt);
  unsigned int                   reserved = data - base;
  int                            ret;

  /* Check space for the VLAN tag, normally we want user to set bigger
   * CONFIG_NET_LL_GUARDSIZE to allow inserting VLAN tag in the headroom.
   */

  if (reserved < 4)
    {
      /* TODO: Support backup path for too small CONFIG_NET_LL_GUARDSIZE */

      nerr("ERROR: No headroom for VLAN tag, please enlarge "
           "CONFIG_NET_LL_GUARDSIZE\n");
      return -ENOMEM;
    }

  /* Move the data to make space for the VLAN tag */

  netpkt_copyin(dev, pkt, data, offsetof(struct eth_hdr_s, type), -4);
  netpkt_reset_reserved(dev, pkt, reserved - 4);

  /* Set value of the VLAN tag */

  vlan_hdr       = (FAR struct eth_8021qhdr_s *)netpkt_getdata(dev, pkt);
  vlan_hdr->tpid = HTONS(TPID_8021QVLAN);
  vlan_hdr->tci  = HTONS(vlan->tci);

  /* Transmit the packet on the real device */

  /* TODO: Call pkt_input to allow tcpdump capture tx packet on real dev. */

  ret = real->ops->transmit(real, pkt);
  if (ret != OK)
    {
      /* Undo VLAN encapsulation so the packet can be safely retransmitted
       * through vlan_transmit again (e.g. after being re-queued to txq).
       * Move DST+SRC back to original position and restore io_offset.
       */

      data = netpkt_getdata(dev, pkt);
      netpkt_copyin(dev, pkt, data, offsetof(struct eth_hdr_s, type), 4);
      netpkt_reset_reserved(dev, pkt, reserved);
    }

  return ret;
}

/****************************************************************************
 * Name: vlan_receive
 ****************************************************************************/

static FAR netpkt_t *vlan_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vlan_device_s *vlan = (FAR struct vlan_device_s *)dev;
  FAR netpkt_t *pkt;
  irqstate_t flags;

  flags = spin_lock_irqsave(&vlan->rxlock);
  pkt = iob_remove_queue(&vlan->rxq);
  spin_unlock_irqrestore(&vlan->rxlock, flags);

  return pkt;
}

/****************************************************************************
 * Name: vlan_addmac
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int vlan_addmac(FAR struct netdev_lowerhalf_s *dev,
                       FAR const uint8_t *mac)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  if (real->ops->addmac)
    {
      return real->ops->addmac(real, mac);
    }

  return -ENOSYS;
}

/****************************************************************************
 * Name: vlan_rmmac
 ****************************************************************************/

static int vlan_rmmac(FAR struct netdev_lowerhalf_s *dev,
                      FAR const uint8_t *mac)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  if (real->ops->rmmac)
    {
      return real->ops->rmmac(real, mac);
    }

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: vlan_ioctl
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int vlan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                      unsigned long arg)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  /* TODO: Maybe we should only pass some of IOCTL commands. */

  if (real->ops->ioctl)
    {
      return real->ops->ioctl(real, cmd, arg);
    }

  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: vlan_reclaim
 ****************************************************************************/

static void vlan_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  if (real->ops->reclaim != NULL)
    {
      real->ops->reclaim(real);
    }
}

#ifdef CONFIG_NET_NETPOLL
static void vlan_txpoll(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vlan_device_s      *vlan = (FAR struct vlan_device_s *)dev;
  FAR struct netdev_lowerhalf_s *real = vlan->real;

  if (real->ops->txpoll)
    {
      real->ops->txpoll(real);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vlan_register
 *
 * Description:
 *   Create a new VLAN device and register it.
 *
 * Input Parameters:
 *   real - The real device to which the VLAN is attached
 *   vid  - VLAN ID
 *   prio - Default VLAN priority (PCP)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int vlan_register(FAR struct netdev_lowerhalf_s *real, uint16_t vid,
                  uint16_t prio, uint8_t rx_type, uint8_t rx_priority)
{
  FAR struct vlan_device_s *vlan;
  int ret;

  if (real == NULL || real->netdev.d_lltype != NET_LL_ETHERNET)
    {
      nerr("ERROR: Invalid real device\n");
      return -EINVAL;
    }

  if (vid >= VLAN_N_VID || prio > (VLAN_PRIO_MASK >> VLAN_PRIO_SHIFT))
    {
      nerr("ERROR: Invalid VID %" PRIu16 " with PCP %" PRIu16, vid, prio);
      return -EINVAL;
    }

  if (rx_type > NETDEV_RX_DEFAULT)
    {
      nerr("ERROR: Invalid vlan rx_type %d\n", rx_type);
      return -EINVAL;
    }

  /* Create a new VLAN device */

  vlan = kmm_zalloc(sizeof(struct vlan_device_s));
  if (vlan == NULL)
    {
      nerr("ERROR: Failed to allocate memory for VLAN device\n");
      return -ENOMEM;
    }

  /* Init the VLAN device */

  spin_lock_init(&vlan->rxlock);
  vlan->tci           = (vid & VLAN_VID_MASK) |
                        ((prio << VLAN_PRIO_SHIFT) & VLAN_PRIO_MASK);
  vlan->real          = real;
  vlan->dev.quota_ptr = real->quota_ptr;
  vlan->dev.rxtype    = real->rxtype;
  vlan->dev.priority  = real->priority;
  vlan->dev.ops       = &g_vlan_ops;

  /* Copy MTU from the real device */

  vlan->dev.netdev.d_pktsize = real->netdev.d_pktsize;

  /* Set the VLAN device name, use a buffer to make compiler happy */

  snprintf(vlan->dev.netdev.d_ifname, IFNAMSIZ, VLAN_DEV_NAME_FMT,
           IFNAMSIZ - 6, real->netdev.d_ifname, vid);

  /* Copy the MAC address from the real device */

  memcpy(vlan->dev.netdev.d_mac.ether.ether_addr_octet,
         real->netdev.d_mac.ether.ether_addr_octet, IFHWADDRLEN);

  /* Register the VLAN device */

  ret = netdev_lower_vlan_add(real, vid, &vlan->dev);
  if (ret < 0)
    {
      nerr("ERROR: Failed to add VLAN device: %d\n", ret);
      goto errout;
    }

  if (rx_type != NETDEV_RX_DEFAULT)
    {
      vlan->dev.rxtype   = rx_type;
      vlan->dev.priority = rx_priority;
    }

  ret = netdev_lower_register(&vlan->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      netdev_lower_vlan_del(real, vid);
      goto errout;
    }

  return ret;

errout:
  kmm_free(vlan);
  return ret;
}

/****************************************************************************
 * Name: vlan_unregister
 *
 * Description:
 *   Unregister a VLAN device.
 *
 * Input Parameters:
 *   dev - The VLAN device to be unregistered.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void vlan_unregister(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vlan_device_s *vlan = (FAR struct vlan_device_s *)dev;

  if (vlan == NULL)
    {
      nerr("ERROR: Vlan is NULL\n");
      return;
    }

  netdev_lower_vlan_del(vlan->real, vlan->tci & VLAN_VID_MASK);
  iob_free_queue(&vlan->rxq);
  netdev_lower_unregister(dev);
  kmm_free(dev);
}

/****************************************************************************
 * Name: vlan_dispatch
 *
 * Description:
 *   Dispatch the received packet to the VLAN device's RX queue.
 *
 * Input Parameters:
 *   dev - The VLAN device
 *   pkt - The received packet
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int vlan_dispatch(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt)
{
  FAR struct vlan_device_s *vlan = (FAR struct vlan_device_s *)dev;
  irqstate_t flags;
  int ret;

  /* Need to remove the 4 octet VLAN Tag, by moving src and dest addresses
   * 4 octets to the right, and then read the actual ethertype.
   */

  memmove(netpkt_getdata(dev, pkt) + 4, netpkt_getdata(dev, pkt),
          offsetof(struct eth_hdr_s, type));
  pkt = iob_trimhead(pkt, 4);

  /* Enqueue the packet to the VLAN device's RX queue */

  flags = spin_lock_irqsave(&vlan->rxlock);
  ret = iob_tryadd_queue(pkt, &vlan->rxq);
  spin_unlock_irqrestore(&vlan->rxlock, flags);

  if (ret < 0)
    {
      /* Failed to enqueue, drop the packet */

      nerr("ERROR: VLAN RX queue full, dropping packet\n");
      netpkt_free(dev, pkt, NETPKT_RX);
      NETDEV_RXDROPPED(&dev->netdev);
    }

  return ret;
}

#endif /* CONFIG_NET_VLAN */
