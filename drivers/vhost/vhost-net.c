/****************************************************************************
 * drivers/vhost/vhost-net.c
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

/* Device-role virtio network driver ("vhost-net"): implements the DEVICE
 * end of a virtio-net link, so a peer processor running a stock virtio-net
 * DRIVER (e.g. Linux via remoteproc/rproc-virtio) sees this side as a
 * network card.  Registers a NuttX netdev lowerhalf (ethN).
 *
 * Ring layout is fixed by the peer driver's point of view:
 *   vq[0] = peer driver RX queue: the peer posts empty buffers; the
 *           device side fills them to transmit toward the peer.
 *   vq[1] = peer driver TX queue: the peer posts filled buffers; the
 *           device side harvests them as its receive path.
 *
 * No virtio-net features are negotiated (the resource table advertises
 * none), so every packet is prefixed by the legacy 10-byte
 * struct virtio_net_hdr with all fields zero (gso_type NONE).
 *
 * Notification handling follows the NAPI pattern: vhost_net_getbufs()
 * suppresses the peer's kicks as soon as a ring hands out work and only
 * re-arms them once it ran dry, so a busy link costs no cross-core
 * notifications.  Receive completions are batched and published with a
 * single kick per poll burst.
 *
 * The virtqueues are touched from exactly one context, the netdev upper
 * half's work thread (see the NETDEV_RX_WORK note in vhost_net_probe()),
 * so no locking is needed.  The kick callbacks run in interrupt context
 * and must therefore stay clear of the rings -- all they do is wake the
 * upper half.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/vhost/vhost.h>

#include "vhost-net.h"

/* Peer buffers are referenced by the raw 64-bit addresses found in the
 * descriptors.  Normally that is just a physical address, and the arch
 * translation that libmetal (and therefore virtqueue_phys_to_virt()) also
 * relies on is up_addrenv_pa_to_va().
 *
 * ARCH_HAVE_VHOST_IOMAP targets cannot go through libmetal at all: there
 * metal_phys_addr_t (unsigned long) and up_addrenv_pa_to_va() (uintptr_t)
 * are narrower than the descriptor address, so they would truncate it.
 * Those arches expose a translation window through up_vhost_iomap()
 * instead, which reports how many contiguous bytes the returned pointer
 * covers so that callers can split accesses at the window boundary.
 */

#ifdef CONFIG_ARCH_HAVE_VHOST_IOMAP
#  define vhost_net_map(pa, avl) up_vhost_iomap((pa), (avl))
#else
static inline_function FAR void *vhost_net_map(uint64_t pa,
                                               FAR size_t *avail)
{
  if (avail != NULL)
    {
      *avail = SIZE_MAX;
    }

  return up_addrenv_pa_to_va((uintptr_t)pa);
}
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Queue indices (peer driver's numbering, see file header) */

#define VHOST_NET_PEER_RXQ   0    /* device-side transmit lane */
#define VHOST_NET_PEER_TXQ   1    /* device-side receive lane */
#define VHOST_NET_NUM        2

/* Legacy struct virtio_net_hdr (no VIRTIO_NET_F_MRG_RXBUF): flags(1) +
 * gso_type(1) + hdr_len(2) + gso_size(2) + csum_start(2) + csum_offset(2)
 */

#define VHOST_NET_HDRSIZE    10

/* virtio-net feature bits referenced here */

#define VHOST_NET_F_MAC      5

/* netpkt quota per direction and the longest peer descriptor chain we
 * accept on either ring (Linux commonly splits header and payload).
 */

#define VHOST_NET_NPKTS      8
#define VHOST_NET_MAXCHAIN   8

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Leading field of the virtio-net configuration space (virtio 1.2
 * section 5.1.4); only the MAC is read here.
 */

struct vhost_net_config_s
{
  uint8_t mac[IFHWADDRLEN];             /* VIRTIO_NET_F_MAC */
};

struct vhost_net_priv_s
{
  struct netdev_lowerhalf_s lower;      /* Must be first for casts */
  FAR struct virtqueue     *txq;        /* peer RX ring (filled here) */
  FAR struct virtqueue     *rxq;        /* peer TX ring (drained here) */
  bool                      rxpending;  /* Used entries not yet notified */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vhost_net_ifup(FAR struct netdev_lowerhalf_s *dev);
static int vhost_net_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int vhost_net_transmit(FAR struct netdev_lowerhalf_s *dev,
                              FAR netpkt_t *pkt);
static FAR netpkt_t *vhost_net_receive(FAR struct netdev_lowerhalf_s *dev);
static int vhost_net_probe(FAR struct vhost_device *hdev);
static void vhost_net_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_vhost_net_ops =
{
  vhost_net_ifup,
  vhost_net_ifdown,
  vhost_net_transmit,
  vhost_net_receive,
#ifdef CONFIG_NET_MCASTGROUP
  NULL,
  NULL,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  NULL,
#endif
  NULL
};

static struct vhost_driver g_vhost_net_driver =
{
  LIST_INITIAL_VALUE(g_vhost_net_driver.node),  /* node */
  VIRTIO_ID_NETWORK,                            /* device id */
  vhost_net_probe,                              /* probe */
  vhost_net_remove,                             /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_net_getbufs
 *
 * Description:
 *   Fetch the next descriptor chain the peer posted on a ring and leave
 *   the ring's notifications in the right state: suppressed while it keeps
 *   delivering work, armed once it is empty.
 *
 *   The kick that raced with the re-arm was already swallowed, so
 *   virtqueue_enable_cb() reports whether a chain landed in that window
 *   and it is picked up here instead of being stranded.
 *
 * Returned Value:
 *   The head descriptor index, -ENOMEM if the ring is empty, or another
 *   negated errno for a malformed chain (already recycled by the ring
 *   layer, so it must not be completed again).
 *
 ****************************************************************************/

static int vhost_net_getbufs(FAR struct virtqueue *vq,
                             FAR struct vhost_buf_s *vb, size_t vbsize,
                             FAR size_t *cnt)
{
  int head = vhost_get_vq_buffers_pa(vq, vb, vbsize, cnt);

  if (head == -ENOMEM)
    {
      if (!virtqueue_enable_cb(vq))
        {
          return -ENOMEM;
        }

      head = vhost_get_vq_buffers_pa(vq, vb, vbsize, cnt);
      if (head == -ENOMEM)
        {
          return -ENOMEM;
        }
    }

  /* Work in hand: no need for the peer to keep knocking */

  virtqueue_disable_cb(vq);
  return head;
}

/****************************************************************************
 * Name: vhost_net_rxcomplete / vhost_net_rxflush
 *
 * Description:
 *   Hand a receive chain back to the peer.  The used ring is published
 *   immediately but the notification is deferred to vhost_net_rxflush(),
 *   so a burst costs one kick instead of one per frame.
 *
 ****************************************************************************/

static void vhost_net_rxcomplete(FAR struct vhost_net_priv_s *priv,
                                 int head, uint32_t len)
{
  virtqueue_add_consumed_buffer(priv->rxq, head, len);
  priv->rxpending = true;
}

static void vhost_net_rxflush(FAR struct vhost_net_priv_s *priv)
{
  if (priv->rxpending)
    {
      priv->rxpending = false;
      virtqueue_kick(priv->rxq);
    }
}

/****************************************************************************
 * Name: vhost_net_pkt2vb
 *
 * Description:
 *   Serialize the zeroed virtio-net header followed by the frame into the
 *   descriptor chain the peer posted, honoring both descriptor and
 *   translation-window boundaries.
 *
 * Returned Value:
 *   OK on success, -EMSGSIZE if the chain is too short for the frame, or
 *   -EIO if the frame could not be read out of the netpkt.
 *
 ****************************************************************************/

static int vhost_net_pkt2vb(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt,
                            FAR const struct vhost_buf_s *vb, size_t cnt,
                            unsigned int len)
{
  unsigned int total = len + VHOST_NET_HDRSIZE;
  unsigned int pos = 0;
  size_t i;

  for (i = 0; i < cnt && pos < total; i++)
    {
      uint64_t pa = vb[i].addr;
      unsigned int blen = MIN(vb[i].len, total - pos);

      while (blen > 0)
        {
          FAR uint8_t *dst;
          unsigned int hdrlen = 0;
          unsigned int chunk;
          size_t avail;

          dst   = vhost_net_map(pa, &avail);
          chunk = MIN(blen, avail);
          if (chunk == 0)
            {
              return -EIO;
            }

          if (pos < VHOST_NET_HDRSIZE)
            {
              hdrlen = MIN(VHOST_NET_HDRSIZE - pos, chunk);
              memset(dst, 0, hdrlen);
            }

          if (chunk > hdrlen &&
              netpkt_copyout(dev, dst + hdrlen, pkt, chunk - hdrlen,
                             pos + hdrlen - VHOST_NET_HDRSIZE) < 0)
            {
              return -EIO;
            }

          pos  += chunk;
          pa   += chunk;
          blen -= chunk;
        }
    }

  return pos == total ? OK : -EMSGSIZE;
}

/****************************************************************************
 * Name: vhost_net_vb2pkt
 *
 * Description:
 *   Copy a received descriptor chain into a netpkt, skipping the leading
 *   virtio-net header (which the peer may place in its own descriptor)
 *   and honoring translation-window boundaries.
 *
 ****************************************************************************/

static int vhost_net_vb2pkt(FAR struct netdev_lowerhalf_s *dev,
                            FAR netpkt_t *pkt,
                            FAR const struct vhost_buf_s *vb, size_t cnt,
                            unsigned int len)
{
  unsigned int skip = VHOST_NET_HDRSIZE;
  unsigned int pos = 0;
  size_t i;

  for (i = 0; i < cnt && pos < len; i++)
    {
      uint64_t pa = vb[i].addr;
      unsigned int blen = vb[i].len;

      if (skip > 0)
        {
          unsigned int drop = MIN(skip, blen);

          pa   += drop;
          blen -= drop;
          skip -= drop;
        }

      blen = MIN(blen, len - pos);

      while (blen > 0)
        {
          FAR const uint8_t *src;
          unsigned int chunk;
          size_t avail;

          src   = vhost_net_map(pa, &avail);
          chunk = MIN(blen, avail);
          if (chunk == 0 ||
              netpkt_copyin(dev, pkt, src, chunk, pos) < 0)
            {
              return -EIO;
            }

          pos  += chunk;
          pa   += chunk;
          blen -= chunk;
        }
    }

  return pos == len ? OK : -EIO;
}

/****************************************************************************
 * Name: vhost_net_rxready / vhost_net_txdone
 *
 * Description:
 *   Virtqueue kick callbacks.  These run in interrupt context (rptun
 *   delivers notifications straight from its ISR), so they must not touch
 *   the rings -- that is what keeps the ring state single-context and
 *   lock-free.  Waking the upper half is all they do.
 *
 ****************************************************************************/

static void vhost_net_rxready(FAR struct virtqueue *vq)
{
  FAR struct vhost_net_priv_s *priv = vq->vq_dev->priv;

  netdev_lower_rxready(&priv->lower);
}

static void vhost_net_txdone(FAR struct virtqueue *vq)
{
  FAR struct vhost_net_priv_s *priv = vq->vq_dev->priv;

  netdev_lower_txdone(&priv->lower);
}

/****************************************************************************
 * Name: vhost_net_ifup / vhost_net_ifdown
 ****************************************************************************/

static int vhost_net_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;

  /* Arm both lanes; vhost_net_getbufs() suppresses them again as soon as
   * a ring starts delivering work.
   */

  virtqueue_enable_cb(priv->rxq);
  virtqueue_enable_cb(priv->txq);

  netdev_lower_carrier_on(dev);
  return OK;
}

static int vhost_net_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;

  /* The upper half has already cancelled the poll work, so the rings are
   * idle here.  Stop taking notifications, then let the peer know about
   * the receive buffers completed but not yet notified.
   */

  virtqueue_disable_cb(priv->rxq);
  virtqueue_disable_cb(priv->txq);
  vhost_net_rxflush(priv);

  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: vhost_net_transmit
 *
 * Description:
 *   Fill one peer-posted RX chain with the frame and complete it.
 *   Completion is synchronous: the netpkt is consumed and freed before
 *   returning.  Note that txdone is deliberately not signalled from here,
 *   the upper half continues its poll on a successful transmit and
 *   netpkt_free() already returns the quota.
 *
 ****************************************************************************/

static int vhost_net_transmit(FAR struct netdev_lowerhalf_s *dev,
                              FAR netpkt_t *pkt)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;
  struct vhost_buf_s vb[VHOST_NET_MAXCHAIN];
  unsigned int len;
  size_t cnt;
  int head;
  int ret;

  head = vhost_net_getbufs(priv->txq, vb, nitems(vb), &cnt);
  if (head == -ENOMEM)
    {
      /* The peer has no receive buffer posted.  Its notifications are
       * armed again now, so stop the poll and let vhost_net_txdone()
       * resume it; the upper half keeps the packet.
       */

      return -ENOBUFS;
    }

  len = netpkt_getdatalen(dev, pkt);
  if (head < 0)
    {
      /* Malformed chain; the ring layer already recycled it and only the
       * notification below is still owed.
       */

      ret = head;
    }
  else
    {
      /* A failed copy is completed with a zero length, which the peer
       * driver discards, rather than leaking the descriptor.
       */

      ret = vhost_net_pkt2vb(dev, pkt, vb, cnt, len);
      virtqueue_add_consumed_buffer(priv->txq, head,
                                    ret < 0 ? 0 : len + VHOST_NET_HDRSIZE);
    }

  virtqueue_kick(priv->txq);

  if (ret < 0)
    {
      vhosterr("tx dropped: %u bytes, ret=%d\n", len, ret);
      NETDEV_TXERRORS(&dev->netdev);
    }
  else
    {
      NETDEV_TXDONE(&dev->netdev);
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  return OK;
}

/****************************************************************************
 * Name: vhost_net_receive
 *
 * Description:
 *   Harvest the next frame from the peer TX ring, copy it into a fresh
 *   netpkt (stripping the virtio-net header) and return the buffers to
 *   the peer.  Frames rejected by the length checks are skipped inline so
 *   a single malformed chain does not abort the poll.
 *
 ****************************************************************************/

static FAR netpkt_t *vhost_net_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;
  struct vhost_buf_s vb[VHOST_NET_MAXCHAIN];
  FAR netpkt_t *pkt;
  uint64_t total;
  unsigned int len;
  size_t cnt;
  size_t i;
  int head;
  int ret;

  for (; ; )
    {
      head = vhost_net_getbufs(priv->rxq, vb, nitems(vb), &cnt);
      if (head == -ENOMEM)
        {
          /* Ring drained and the peer's notifications are armed again:
           * publish the batched completions with a single kick and let
           * the upper half stop polling.
           */

          vhost_net_rxflush(priv);
          return NULL;
        }

      if (head < 0)
        {
          /* Malformed chain; the ring layer already recycled it, so only
           * the deferred notification is still owed.  Keep draining, the
           * ring advances on every iteration.
           */

          NETDEV_RXERRORS(&dev->netdev);
          priv->rxpending = true;
          continue;
        }

      /* Accumulate in 64 bits: the descriptor lengths are peer controlled
       * 32-bit values and a full chain of them would wrap a narrower sum.
       */

      for (i = 0, total = 0; i < cnt; i++)
        {
          total += vb[i].len;
        }

      /* A valid frame carries the virtio-net header plus at least a
       * complete Ethernet header, and must fit the MTU.  Note that
       * VIRTIO_NET_F_MTU is not negotiated on this link, so the peer sends
       * according to its own MTU: both ends have to be configured to the
       * same frame size (CONFIG_NET_ETH_PKTSIZE here) or the peer's larger
       * frames are dropped rather than handed up with d_len > d_pktsize.
       */

      if (total < VHOST_NET_HDRSIZE + ETH_HDRLEN ||
          total > VHOST_NET_HDRSIZE + NETDEV_PKTSIZE(&dev->netdev))
        {
          vhosterr("rx dropped: bad frame length %" PRIu64 "\n", total);
          NETDEV_RXERRORS(&dev->netdev);
          vhost_net_rxcomplete(priv, head, 0);
          continue;
        }

      len = (unsigned int)total - VHOST_NET_HDRSIZE;

      pkt = netpkt_alloc(dev, NETPKT_RX);
      if (pkt == NULL)
        {
          /* Out of receive quota: drop the frame, re-arm the peer's
           * notifications and stop polling so the stack can release
           * buffers.
           */

          vhosterr("rx dropped: no netpkt for %u bytes\n", len);
          NETDEV_RXDROPPED(&dev->netdev);
          vhost_net_rxcomplete(priv, head, (uint32_t)total);
          vhost_net_rxflush(priv);
          virtqueue_enable_cb(priv->rxq);
          return NULL;
        }

      /* Size the netpkt up front so a short chain is caught before any
       * byte is copied.
       */

      ret = netpkt_setdatalen(dev, pkt, len);
      if (ret < 0 || (unsigned int)ret < len)
        {
          vhosterr("rx dropped: cannot size netpkt to %u\n", len);
          NETDEV_RXDROPPED(&dev->netdev);
          ret = -ENOSPC;
        }
      else
        {
          ret = vhost_net_vb2pkt(dev, pkt, vb, cnt, len);
          if (ret < 0)
            {
              vhosterr("rx dropped: copy failed, ret=%d\n", ret);
              NETDEV_RXERRORS(&dev->netdev);
            }
        }

      vhost_net_rxcomplete(priv, head, (uint32_t)total);
      if (ret >= 0)
        {
          return pkt;
        }

      netpkt_free(dev, pkt, NETPKT_RX);
    }
}

/****************************************************************************
 * Name: vhost_net_probe
 ****************************************************************************/

static int vhost_net_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_net_priv_s *priv;
  FAR const char *vqnames[VHOST_NET_NUM];
  vq_callback callbacks[VHOST_NET_NUM];
  FAR uint8_t *mac;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  hdev->priv = priv;

  vqnames[VHOST_NET_PEER_RXQ]   = "vhost_net_peer_rx";
  vqnames[VHOST_NET_PEER_TXQ]   = "vhost_net_peer_tx";
  callbacks[VHOST_NET_PEER_RXQ] = vhost_net_txdone;
  callbacks[VHOST_NET_PEER_TXQ] = vhost_net_rxready;
  ret = vhost_create_virtqueues(hdev, 0, VHOST_NET_NUM, vqnames,
                                callbacks, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  priv->txq = hdev->vrings_info[VHOST_NET_PEER_RXQ].vq;
  priv->rxq = hdev->vrings_info[VHOST_NET_PEER_TXQ].vq;

  /* Start from a known state rather than whatever the peer left in the
   * shared vring; vhost_net_ifup() arms the notifications.
   */

  virtqueue_disable_cb(priv->txq);
  virtqueue_disable_cb(priv->rxq);

  priv->lower.quota[NETPKT_RX] = VHOST_NET_NPKTS;
  priv->lower.quota[NETPKT_TX] = VHOST_NET_NPKTS;
  priv->lower.ops = &g_vhost_net_ops;

  /* transmit() and receive() must not run concurrently: both walk peer
   * buffers through the arch translation window, a single shared resource
   * (see up_vhost_iomap()).  NETDEV_RX_WORK keeps both on the upper
   * half's work thread, which is also what makes the ring state above
   * single-context.
   */

  priv->lower.rxtype = NETDEV_RX_WORK;

  /* Take the address the peer published in the configuration space when
   * VIRTIO_NET_F_MAC says it is valid.  Otherwise fall back to the fixed
   * address from Kconfig, or generate a random locally administered
   * unicast one when that is left at 0 so that two instances on the same
   * link cannot collide.
   */

  mac = priv->lower.netdev.d_mac.ether.ether_addr_octet;
  if (!vhost_has_feature(hdev, VHOST_NET_F_MAC) ||
      vhost_read_config(hdev, offsetof(struct vhost_net_config_s, mac),
                        mac, IFHWADDRLEN) < 0)
    {
#if CONFIG_DRIVERS_VHOST_NET_MACADDR != 0
      uint64_t macaddr = CONFIG_DRIVERS_VHOST_NET_MACADDR;

      mac[0] = (macaddr >> 40) & 0xff;
      mac[1] = (macaddr >> 32) & 0xff;
      mac[2] = (macaddr >> 24) & 0xff;
      mac[3] = (macaddr >> 16) & 0xff;
      mac[4] = (macaddr >> 8)  & 0xff;
      mac[5] = (macaddr >> 0)  & 0xff;
#else
      arc4random_buf(mac, IFHWADDRLEN);
      mac[0] &= 0xfe;    /* Unicast */
      mac[0] |= 0x02;    /* Locally administered */
#endif
    }

  ret = netdev_lower_register(&priv->lower, NET_LL_ETHERNET);
  if (ret < 0)
    {
      vhosterr("netdev_lower_register failed, ret=%d\n", ret);
      goto err_with_vqs;
    }

  return OK;

err_with_vqs:
  vhost_delete_virtqueues(hdev);
err_with_priv:
  kmm_free(priv);
  hdev->priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: vhost_net_remove
 ****************************************************************************/

static void vhost_net_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_net_priv_s *priv = hdev->priv;

  /* Silence the peer before tearing the netdev down, so a late kick
   * cannot reach an unregistered upper half.
   */

  virtqueue_disable_cb(priv->rxq);
  virtqueue_disable_cb(priv->txq);

  netdev_lower_unregister(&priv->lower);
  vhost_delete_virtqueues(hdev);
  kmm_free(priv);
  hdev->priv = NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_net_driver
 ****************************************************************************/

int vhost_register_net_driver(void)
{
  return vhost_register_driver(&g_vhost_net_driver);
}
