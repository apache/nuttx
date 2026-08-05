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
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/vhost/vhost.h>

#include "vhost-net.h"

/* Peer buffers are referenced by 64-bit guest physical addresses that may
 * exceed the CPU's direct reach; arches that provide a translation window
 * implement up_vhost_iomap() (ARCH_HAVE_VHOST_IOMAP), others use the
 * identity mapping.
 */

#ifdef CONFIG_ARCH_HAVE_VHOST_IOMAP
#  define vhost_net_map(pa, avl) up_vhost_iomap((pa), (avl))
#else
static inline FAR void *vhost_net_map(uint64_t pa, FAR size_t *avail)
{
  if (avail != NULL)
    {
      *avail = SIZE_MAX;
    }

  return (FAR void *)(uintptr_t)pa;
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

/* netpkt quota per direction and the longest peer descriptor chain we
 * accept on receive (Linux commonly splits header and payload).
 */

#define VHOST_NET_NPKTS      8
#define VHOST_NET_MAXCHAIN   8

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_net_priv_s
{
  struct netdev_lowerhalf_s lower;      /* Must be first for casts */
  FAR struct vhost_device  *hdev;
  FAR struct virtqueue     *txq;        /* peer RX ring (filled here) */
  FAR struct virtqueue     *rxq;        /* peer TX ring (drained here) */
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
 * Name: vhost_net_rxready / vhost_net_txdone
 *
 * Description:
 *   Virtqueue kick callbacks (transport notification context, thread
 *   level).  Notify the upper half that ring work is pending; the rings
 *   are processed in transmit()/receive().
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
  netdev_lower_carrier_on(dev);
  return OK;
}

static int vhost_net_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: vhost_net_transmit
 *
 * Description:
 *   Fill one peer-posted RX buffer with the frame and complete it.
 *   Completion is synchronous: the netpkt is consumed and freed before
 *   returning.
 *
 ****************************************************************************/

static int vhost_net_transmit(FAR struct netdev_lowerhalf_s *dev,
                              FAR netpkt_t *pkt)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;
  struct vhost_buf_s vb[1];
  unsigned int len;
  unsigned int pos;
  size_t cnt;
  int head;

  head = vhost_get_vq_buffers_pa(priv->txq, vb, nitems(vb), &cnt);
  if (head < 0)
    {
      /* Peer has not posted buffers (yet).  Re-enable its notifications;
       * enable_cb reports buffers that arrived in the race window (their
       * kick was suppressed), so grab them now if so.
       */

      if (!virtqueue_enable_cb(priv->txq))
        {
          return -ENOBUFS;
        }

      head = vhost_get_vq_buffers_pa(priv->txq, vb, nitems(vb), &cnt);
      if (head < 0)
        {
          return -ENOBUFS;
        }
    }

  len = netpkt_getdatalen(dev, pkt);
  if (len + VHOST_NET_HDRSIZE > vb[0].len)
    {
      /* Frame cannot fit the peer's buffer: complete it empty (drop) */

      vhosterr("frame %u exceeds peer buffer %" PRIu32 ", dropped\n",
               len, vb[0].len);
      len = 0;
    }
  else
    {
      /* Serialize the zero header + frame into the peer buffer through
       * the translation window, honoring window-boundary splits.
       */

      for (pos = 0; pos < len + VHOST_NET_HDRSIZE; )
        {
          size_t avail;
          FAR uint8_t *dst = vhost_net_map(vb[0].addr + pos, &avail);
          unsigned int chunk = MIN(len + VHOST_NET_HDRSIZE - pos, avail);
          unsigned int hdrlen = 0;
          int ret = OK;

          if (pos < VHOST_NET_HDRSIZE)
            {
              hdrlen = MIN(VHOST_NET_HDRSIZE - pos, chunk);
              memset(dst, 0, hdrlen);
            }

          if (chunk > hdrlen)
            {
              ret = netpkt_copyout(dev, dst + hdrlen, pkt, chunk - hdrlen,
                                   pos + hdrlen - VHOST_NET_HDRSIZE);
            }

          if (ret < 0)
            {
              vhosterr("netpkt_copyout failed, ret=%d, dropped\n", ret);
              len = 0;
              break;
            }

          pos += chunk;
        }
    }

  virtqueue_add_consumed_buffer(priv->txq, head,
                                len ? len + VHOST_NET_HDRSIZE : 0);
  virtqueue_kick(priv->txq);

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);
  return OK;
}

/****************************************************************************
 * Name: vhost_net_receive
 *
 * Description:
 *   Harvest one frame (possibly a descriptor chain) from the peer TX
 *   ring, copy it into a fresh netpkt (stripping the virtio-net header)
 *   and return the buffers to the peer.
 *
 ****************************************************************************/

static FAR netpkt_t *vhost_net_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct vhost_net_priv_s *priv = (FAR struct vhost_net_priv_s *)dev;
  struct vhost_buf_s vb[VHOST_NET_MAXCHAIN];
  FAR netpkt_t *pkt = NULL;
  unsigned int total = 0;
  unsigned int skip = VHOST_NET_HDRSIZE;
  int offset = 0;
  size_t cnt;
  size_t i;
  int head;

  head = vhost_get_vq_buffers_pa(priv->rxq, vb, nitems(vb), &cnt);
  if (head < 0)
    {
      /* See vhost_net_transmit() for the enable_cb recheck rationale */

      if (!virtqueue_enable_cb(priv->rxq))
        {
          return NULL;
        }

      head = vhost_get_vq_buffers_pa(priv->rxq, vb, nitems(vb), &cnt);
      if (head < 0)
        {
          return NULL;
        }
    }

  for (i = 0; i < cnt; i++)
    {
      total += vb[i].len;
    }

  if (total > skip)
    {
      pkt = netpkt_alloc(dev, NETPKT_RX);
    }

  if (pkt != NULL &&
      netpkt_setdatalen(dev, pkt, total - skip) < total - skip)
    {
      vhosterr("rx dropped: cannot size netpkt to %u\n", total - skip);
      netpkt_free(dev, pkt, NETPKT_RX);
      pkt = NULL;
    }

  if (pkt != NULL)
    {
      for (i = 0; i < cnt; i++)
        {
          uint64_t pa = vb[i].addr;
          uint32_t blen = vb[i].len;

          if (skip > 0)
            {
              uint32_t skiplen = MIN(skip, blen);

              pa   += skiplen;
              blen -= skiplen;
              skip -= skiplen;
            }

          /* Copy through the translation window, honoring
           * window-boundary splits.
           */

          while (blen > 0)
            {
              size_t avail;
              FAR const uint8_t *src = vhost_net_map(pa, &avail);
              uint32_t chunk = MIN(blen, avail);

              if (netpkt_copyin(dev, pkt, src, chunk, offset) < 0)
                {
                  vhosterr("netpkt_copyin failed, rx dropped\n");
                  netpkt_free(dev, pkt, NETPKT_RX);
                  pkt = NULL;
                  goto out;
                }

              offset += chunk;
              pa     += chunk;
              blen   -= chunk;
            }
        }
    }
  else
    {
      vhosterr("rx dropped: total=%u (no netpkt)\n", total);
    }

out:

  /* Hand the buffers back to the peer either way */

  virtqueue_add_consumed_buffer(priv->rxq, head, total);
  virtqueue_kick(priv->rxq);

  return pkt;
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

  priv->hdev = hdev;
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

  priv->lower.quota[NETPKT_RX] = VHOST_NET_NPKTS;
  priv->lower.quota[NETPKT_TX] = VHOST_NET_NPKTS;
  priv->lower.ops = &g_vhost_net_ops;

  /* Software-assigned MAC (no MAC config space without negotiated
   * features); see CONFIG_DRIVERS_VHOST_NET_MACADDR.
   */

  mac = priv->lower.netdev.d_mac.ether.ether_addr_octet;
  mac[0] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 5)) & 0xff;
  mac[1] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 4)) & 0xff;
  mac[2] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_DRIVERS_VHOST_NET_MACADDR >> (8 * 0)) & 0xff;

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
