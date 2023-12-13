/****************************************************************************
 * drivers/virtio/virtio-net.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-net.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_NET_F_MAC      5

/* Virtio net header size and packet buffer size */

#define VIRTIO_NET_HDRSIZE    (sizeof(struct virtio_net_hdr_s))
#define VIRTIO_NET_LLHDRSIZE  (sizeof(struct virtio_net_llhdr_s))
#define VIRTIO_NET_BUFSIZE    (CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* Virtio net virtqueue index and number */

#define VIRTIO_NET_RX         0
#define VIRTIO_NET_TX         1
#define VIRTIO_NET_NUM        2

#define VIRTIO_NET_MAX_PKT_SIZE \
    ((CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN) + VIRTIO_NET_BUFSIZE)
#define VIRTIO_NET_MAX_NIOB \
    ((VIRTIO_NET_MAX_PKT_SIZE + CONFIG_IOB_BUFSIZE - 1) / CONFIG_IOB_BUFSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Virtio net header, just define it here for further, now only use it
 * to calculate the virto net header size, see marco VIRTIO_NET_HDRSIZE
 */

begin_packed_struct struct virtio_net_hdr_s
{
  uint8_t  flags;
  uint8_t  gso_type;
  uint16_t hdr_len;
  uint16_t gso_size;
  uint16_t csum_start;
  uint16_t csum_offset;
} end_packed_struct;

/* The definition of the struct virtio_net_config refers to the link
 * https://docs.oasis-open.org/virtio/virtio/v1.2/cs01/
 * virtio-v1.2-cs01.html#x1-2230004.
 */

begin_packed_struct struct virtio_net_config_s
{
  uint8_t  mac[IFHWADDRLEN];                 /* VIRTIO_NET_F_MAC */
  uint16_t status;                           /* VIRTIO_NET_F_STATUS */
  uint16_t max_virtqueue_pairs;              /* VIRTIO_NET_F_MQ */
  uint16_t mtu;                              /* VIRTIO_NET_F_MTU */
  uint32_t speed;                            /* VIRTIO_NET_F_SPEED_DUPLEX */
  uint8_t  duplex;
  uint8_t  rss_max_key_size;                 /* VIRTIO_NET_F_RSS */
  uint16_t rss_max_indirection_table_length;
  uint32_t supported_hash_types;
} end_packed_struct;

struct virtio_net_priv_s
{
  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s lower;     /* The netdev lowerhalf */

  /* Virtio device information */

  FAR struct virtio_device *vdev;      /* Virtio device pointer */
  int                       bufnum;    /* TX and RX Buffer number */
};

/* Virtio Link Layer Header, follow shows the iob buffer layout:
 *
 * |<-- CONFIG_NET_LL_GUARDSIZE -->|
 * +---------------+---------------+------------+------+     +-------------+
 * | Virtio Header |  ETH Header   |    data    | free | --> | next netpkt |
 * +---------------+---------------+------------+------+     +-------------+
 * |               |<--------- datalen -------->|
 * ^base           ^data
 *
 * CONFIG_NET_LL_GUARDSIZE >= VIRTIO_NET_LLHDRSIZE + ETH_HDR_SIZE
 *                          = sizeof(uintptr) + 10 + 14
 *                          = 32 (64-Bit)
 *                          = 28 (32-Bit)
 */

begin_packed_struct struct virtio_net_llhdr_s
{
  FAR netpkt_t           *pkt;         /* Netpaket pointer */
  struct virtio_net_hdr_s vhdr;        /* Virtio net header */
} end_packed_struct;

static_assert(CONFIG_NET_LL_GUARDSIZE >= VIRTIO_NET_LLHDRSIZE + ETH_HDRLEN,
              "CONFIG_NET_LL_GUARDSIZE cannot be less than ETH_HDRLEN"
              " + VIRTIO_NET_LLHDRSIZE");

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_net_ifup(FAR struct netdev_lowerhalf_s *dev);
static int virtio_net_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int virtio_net_send(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt);
static netpkt_t *virtio_net_recv(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int virtio_net_addmac(FAR struct netdev_lowerhalf_s *dev,
                             FAR const uint8_t *mac);
static int virtio_net_rmmac(FAR struct netdev_lowerhalf_s *dev,
                            FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int virtio_net_ioctl(FAR struct netdev_lowerhalf_s *dev);
#endif

static int  virtio_net_probe(FAR struct virtio_device *vdev);
static void virtio_net_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_net_driver =
{
  LIST_INITIAL_VALUE(g_virtio_net_driver.node), /* node */
  VIRTIO_ID_NETWORK,                            /* device id */
  virtio_net_probe,                             /* probe */
  virtio_net_remove,                            /* remove */
};

static const struct netdev_ops_s g_virtio_net_ops =
{
  virtio_net_ifup,
  virtio_net_ifdown,
  virtio_net_send,
  virtio_net_recv,
#ifdef CONFIG_NET_MCASTGROUP
  virtio_net_addmac,
  virtio_net_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  virtio_net_ioctl,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_net_rxfill
 ****************************************************************************/

static void virtio_net_rxfill(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_NET_RX].vq;
  FAR struct virtio_net_llhdr_s *hdr;
  struct virtqueue_buf vb[VIRTIO_NET_MAX_NIOB];
  struct iovec iov[VIRTIO_NET_MAX_NIOB];
  FAR netpkt_t *pkt;
  int iov_cnt;
  int i;

  for (i = 0; i < priv->bufnum; i++)
    {
      /* IOB Offload, Alloc buffer from RX netpkt */

      pkt = netpkt_alloc(dev, NETPKT_RX);
      if (pkt == NULL)
        {
          vrtinfo("Has ran out of the RX buffer, i=%d\n", i);
          break;
        }

      /* Preserve data length */

      if (netpkt_setdatalen(dev, pkt, VIRTIO_NET_BUFSIZE) <
          VIRTIO_NET_BUFSIZE)
        {
          vrtwarn("No enough buffer to prepare RX buffer, i=%d\n", i);
          netpkt_free(dev, pkt, NETPKT_RX);
          break;
        }

      /* Convert netpkt to virtqueue_buf */

      iov_cnt = netpkt_to_iov(dev, pkt, iov, VIRTIO_NET_MAX_NIOB);
      for (i = 0; i < iov_cnt; i++)
        {
          vb[i].buf = iov[i].iov_base;
          vb[i].len = iov[i].iov_len;
        }

      /* Alloc cookie and net header from transport layer */

      hdr = (FAR struct virtio_net_llhdr_s *)
              ((FAR uint8_t *)vb[0].buf - VIRTIO_NET_LLHDRSIZE);
      DEBUGASSERT((FAR uint8_t *)hdr >= netpkt_getbase(pkt));
      memset(&hdr->vhdr, 0, sizeof(hdr->vhdr));
      hdr->pkt = pkt;

      /* Buffer 0, the virtio net header */

      vb[0].buf = &hdr->vhdr;
      vb[0].len += VIRTIO_NET_HDRSIZE;

      vrtinfo("Fill rx, hdr=%p, count=%d\n", hdr, iov_cnt);
      virtqueue_add_buffer(vq, vb, 0, iov_cnt, hdr);
    }

  if (i > 0)
    {
      virtqueue_kick(vq);
    }
}

/****************************************************************************
 * Name: virtio_net_txfree
 ****************************************************************************/

static void virtio_net_txfree(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_NET_TX].vq;
  FAR struct virtio_net_llhdr_s *hdr;

  while (1)
    {
      /* Get buffer from tx virtqueue */

      hdr = virtqueue_get_buffer(vq, NULL, NULL);
      if (hdr == NULL)
        {
          break;
        }

      netpkt_free(dev, hdr->pkt, NETPKT_TX);
      vrtinfo("Free, hdr: %p, pkt: %p\n", hdr, hdr->pkt);
    }
}

/****************************************************************************
 * Name: virtio_net_ifup
 ****************************************************************************/

static int virtio_net_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;

#ifdef CONFIG_NET_IPv4
  vrtinfo("Bringing up: %u.%u.%u.%u\n",
          ip4_addr1(dev->netdev.d_ipaddr), ip4_addr2(dev->netdev.d_ipaddr),
          ip4_addr3(dev->netdev.d_ipaddr), ip4_addr4(dev->netdev.d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  vrtinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          dev->netdev.d_ipv6addr[0], dev->netdev.d_ipv6addr[1],
          dev->netdev.d_ipv6addr[2], dev->netdev.d_ipv6addr[3],
          dev->netdev.d_ipv6addr[4], dev->netdev.d_ipv6addr[5],
          dev->netdev.d_ipv6addr[6], dev->netdev.d_ipv6addr[7]);
#endif

  /* Prepare interrupt and packets for receiving */

  virtqueue_enable_cb(priv->vdev->vrings_info[VIRTIO_NET_RX].vq);
  virtio_net_rxfill(dev);

  return netdev_lower_carrier_on(dev);
}

/****************************************************************************
 * Name: virtio_net_ifdown
 ****************************************************************************/

static int virtio_net_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;
  int i;

  /* Disable the Ethernet interrupt */

  for (i = 0; i < VIRTIO_NET_NUM; i++)
    {
      virtqueue_disable_cb(priv->vdev->vrings_info[i].vq);
    }

  return netdev_lower_carrier_off(dev);
}

/****************************************************************************
 * Name: virtio_net_send
 ****************************************************************************/

static int virtio_net_send(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_NET_TX].vq;
  FAR struct virtio_net_llhdr_s *hdr;
  struct virtqueue_buf vb[VIRTIO_NET_MAX_NIOB];
  struct iovec iov[VIRTIO_NET_MAX_NIOB];
  int iov_cnt;
  int i;

  /* Check the send length */

  if (netpkt_getdatalen(dev, pkt) > VIRTIO_NET_BUFSIZE)
    {
      vrterr("net send buffer too large\n");
      return -EINVAL;
    }

  /* Convert netpkt to virtqueue_buf */

  iov_cnt = netpkt_to_iov(dev, pkt, iov, VIRTIO_NET_MAX_NIOB);
  for (i = 0; i < iov_cnt; i++)
    {
      vb[i].buf = iov[i].iov_base;
      vb[i].len = iov[i].iov_len;
    }

  /* Prepare virtio net header */

  hdr = (FAR struct virtio_net_llhdr_s *)
          ((FAR uint8_t *)vb[0].buf - VIRTIO_NET_LLHDRSIZE);
  DEBUGASSERT((FAR uint8_t *)hdr >= netpkt_getbase(pkt));
  hdr->pkt = pkt;
  memset(&hdr->vhdr, 0, sizeof(hdr->vhdr));

  /* Buffer 0 is the virtio net header */

  vb[0].buf = &hdr->vhdr;
  vb[0].len += VIRTIO_NET_HDRSIZE;

  /* Add buffer to vq and notify the other side */

  vrtinfo("Send, hdr=%p, count=%d\n", hdr, iov_cnt);
  virtqueue_add_buffer(vq, vb, iov_cnt, 0, hdr);
  virtqueue_kick(vq);

  /* Try return Netpkt TX buffer to upper-half. */

  virtio_net_txfree(dev);

  /* If we have no buffer left, enable TX done callback. */

  if (netdev_lower_quota_load(dev, NETPKT_TX) <= 0)
    {
      virtqueue_enable_cb(vq);
    }

  return OK;
}

/****************************************************************************
 * Name: virtio_net_recv
 ****************************************************************************/

static netpkt_t *virtio_net_recv(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct virtio_net_priv_s *priv = (FAR struct virtio_net_priv_s *)dev;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[VIRTIO_NET_RX].vq;
  FAR struct virtio_net_llhdr_s *hdr;
  uint32_t len;

  /* Fill the free Netpkt RX buffer to the RX virtqueue */

  virtio_net_rxfill(dev);

  /* Get received buffer form RX virtqueue */

  hdr = virtqueue_get_buffer(vq, &len, NULL);
  if (hdr == NULL)
    {
      /* If we have no buffer left, enable RX callback. */

      virtqueue_enable_cb(vq);

      /* We do transmit after recv, now it's time to free TX buffer.
       * Depends on upper-half order (Call TX after RX).
       *
       * TODO: Find a better way to free TX buffer.
       */

      virtio_net_txfree(&priv->lower);

      vrtinfo("get NULL buffer\n");
      return NULL;
    }

  /* Set the received pkt length */

  netpkt_setdatalen(dev, hdr->pkt, len - VIRTIO_NET_HDRSIZE);
  vrtinfo("Recv, hdr=%p, pkt=%p, len=%" PRIu32 "\n", hdr, hdr->pkt, len);
  return hdr->pkt;
}

#ifdef CONFIG_NET_MCASTGROUP
/****************************************************************************
 * Name: virtio_net_addmac
 ****************************************************************************/

static int virtio_net_addmac(FAR struct netdev_lowerhalf_s *dev,
                             FAR const uint8_t *mac)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: virtio_net_rmmac
 ****************************************************************************/

static int virtio_net_rmmac(FAR struct netdev_lowerhalf_s *dev,
                            FAR const uint8_t *mac)
{
  return -ENOSYS;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
/****************************************************************************
 * Name: virtio_net_ioctl
 ****************************************************************************/

static int virtio_net_ioctl(FAR struct netdev_lowerhalf_s *dev)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: virtio_net_rxready
 ****************************************************************************/

static void virtio_net_rxready(FAR struct virtqueue *vq)
{
  FAR struct virtio_net_priv_s *priv = vq->vq_dev->priv;

  virtqueue_disable_cb(vq);
  netdev_lower_rxready(&priv->lower);
}

/****************************************************************************
 * Name: virtio_net_txdone
 ****************************************************************************/

static void virtio_net_txdone(FAR struct virtqueue *vq)
{
  FAR struct virtio_net_priv_s *priv = vq->vq_dev->priv;

  virtqueue_disable_cb(vq);
  netdev_lower_txdone(&priv->lower);
}

/****************************************************************************
 * Name: virtio_net_init
 ****************************************************************************/

static int virtio_net_init(FAR struct virtio_net_priv_s *priv,
                           FAR struct virtio_device *vdev)
{
  FAR const char *vqnames[VIRTIO_NET_NUM];
  vq_callback callbacks[VIRTIO_NET_NUM];
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev, 1 << VIRTIO_NET_F_MAC);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[VIRTIO_NET_RX]   = "virtio_net_rx";
  vqnames[VIRTIO_NET_TX]   = "virtio_net_tx";
  callbacks[VIRTIO_NET_RX] = virtio_net_rxready;
  callbacks[VIRTIO_NET_TX] = virtio_net_txdone;
  ret = virtio_create_virtqueues(vdev, 0, VIRTIO_NET_NUM, vqnames,
                                 callbacks);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      return ret;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

#if CONFIG_DRIVERS_VIRTIO_NET_BUFNUM > 0
  priv->bufnum = CONFIG_DRIVERS_VIRTIO_NET_BUFNUM;
#else
  /* Calculate the virtio network buffer number:
   * 1/4 for the TX netpkts, 1/4 for the RX netpkts.
   */

  priv->bufnum = CONFIG_IOB_NBUFFERS / VIRTIO_NET_MAX_NIOB / 4;
#endif
  priv->bufnum = MIN(vdev->vrings_info[VIRTIO_NET_RX].info.num_descs,
                     priv->bufnum);
  priv->bufnum = MIN(vdev->vrings_info[VIRTIO_NET_TX].info.num_descs,
                     priv->bufnum);
  return OK;
}

static void virtio_net_set_macaddr(FAR struct virtio_net_priv_s *priv)
{
  FAR struct net_driver_s *dev = &priv->lower.netdev;
  FAR struct virtio_device *vdev = priv->vdev;
  FAR uint8_t *mac = dev->d_mac.ether.ether_addr_octet;

  if (virtio_has_feature(vdev, VIRTIO_NET_F_MAC))
    {
      virtio_read_config(vdev, offsetof(struct virtio_net_config_s, mac),
                         mac, IFHWADDRLEN);
    }
  else
    {
      /* Assign a random locally-created MAC address.
       *
       * TODO:  The generated MAC address should be checked to see if it
       *        conflicts with something else on the network.
       */

      srand(time(NULL) +
#ifdef CONFIG_NETDEV_IFINDEX
            dev->d_ifindex
#else
            dev % 256
#endif
          );

      mac[0] = 0x42;
      mac[1] = rand() % 256;
      mac[2] = rand() % 256;
      mac[3] = rand() % 256;
      mac[4] = rand() % 256;
      mac[5] = rand() % 256;
    }
}

/****************************************************************************
 * Name: virtio_net_probe
 ****************************************************************************/

static int virtio_net_probe(FAR struct virtio_device *vdev)
{
  FAR struct netdev_lowerhalf_s *netdev;
  FAR struct virtio_net_priv_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("Virtio net driver priv alloc failed\n");
      return -ENOMEM;
    }

  ret = virtio_net_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("virtio_net_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Initialize the netdev lower half */

  netdev = &priv->lower;
  netdev->quota[NETPKT_RX] = priv->bufnum;
  netdev->quota[NETPKT_TX] = priv->bufnum;
  netdev->ops = &g_virtio_net_ops;

  /* Register the net deivce */

  ret = netdev_lower_register(netdev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      vrterr("netdev_lower_register failed, ret=%d\n", ret);
      goto err_with_virtqueues;
    }

  virtio_net_set_macaddr(priv);

  return ret;

err_with_virtqueues:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_net_remove
 ****************************************************************************/

static void virtio_net_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_net_priv_s *priv = vdev->priv;

  netdev_lower_unregister(&priv->lower);
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_net_driver
 ****************************************************************************/

int virtio_register_net_driver(void)
{
  return virtio_register_driver(&g_virtio_net_driver);
}
