/****************************************************************************
 * net/vsock/vsock_vhost.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <sys/param.h>

#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/vhost/vhost.h>

#include "vsock/vsock.h"
#include "vsock/vsock_virtio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VSOCK_VQ_TX              0
#define VSOCK_VQ_RX              1
#define VSOCK_VQ_EVENT           2
#define VSOCK_VQ_NUM             3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vsock_vhost_priv_s
{
  FAR struct vhost_device *hdev;
  uint64_t                 guest_cid;
  uint64_t                 host_cid;

  spinlock_t               tx_lock;
  sem_t                    tx_sem;
  struct work_s            rx_work;

  struct vsock_transport_s t;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Vhost driver functions */

static int vsock_vhost_probe(FAR struct vhost_device *hdev);
static void vsock_vhost_remove(FAR struct vhost_device *hdev);

static uint64_t vsock_vhost_get_localcid(FAR struct vsock_transport_s *t);
static uint64_t vsock_vhost_get_remotecid(FAR struct vsock_transport_s *t);
static int vsock_vhost_alloc_pkt(FAR struct vsock_transport_s *t,
                                 FAR struct vsock_pkt_s *pkt,
                                 size_t data_len);
static ssize_t vsock_vhost_send_pkt(FAR struct vsock_transport_s *t,
                                    FAR struct vsock_pkt_s *pkt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vsock_vhost_driver =
{
  LIST_INITIAL_VALUE(g_vsock_vhost_driver.node), /* Node */
  VIRTIO_ID_VSOCK,                               /* Device id */
  vsock_vhost_probe,                             /* Probe */
  vsock_vhost_remove,                            /* Remove */
};

static const struct vsock_transport_ops_s g_vsock_vhost_transport_ops =
{
  .get_localcid  = vsock_vhost_get_localcid,
  .get_remotecid = vsock_vhost_get_remotecid,
  .alloc_pkt     = vsock_vhost_alloc_pkt,
  .send_pkt      = vsock_vhost_send_pkt,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_vhost_trans2priv
 ****************************************************************************/

static FAR struct vsock_vhost_priv_s *
vsock_vhost_trans2priv(FAR struct vsock_transport_s *t)
{
  return container_of(t, struct vsock_vhost_priv_s, t);
}

/****************************************************************************
 * Name: vsock_vhost_get_localcid
 ****************************************************************************/

static uint64_t vsock_vhost_get_localcid(FAR struct vsock_transport_s *t)
{
  FAR struct vsock_vhost_priv_s *priv = vsock_vhost_trans2priv(t);

  return priv->host_cid;
}

/****************************************************************************
 * Name: vsock_vhost_get_remotecid
 ****************************************************************************/

static uint64_t vsock_vhost_get_remotecid(FAR struct vsock_transport_s *t)
{
  FAR struct vsock_vhost_priv_s *priv =
    container_of(t, struct vsock_vhost_priv_s, t);

  return priv->guest_cid;
}

/****************************************************************************
 * Name: vsock_vhost_alloc_pkt
 ****************************************************************************/

static int vsock_vhost_alloc_pkt(FAR struct vsock_transport_s *t,
                                 FAR struct vsock_pkt_s *pkt,
                                 size_t data_len)
{
  FAR struct vsock_vhost_priv_s *priv = vsock_vhost_trans2priv(t);
  FAR struct virtqueue *vq;
  irqstate_t flags;
  size_t vbcnt;
  int head;

  data_len += VIRTIO_VSOCK_HDR_LEN;
  vbcnt = div_round_up(data_len, CONFIG_NET_VSOCK_PKT_BUFSIZE);
  vbcnt = MIN(vbcnt, CONFIG_NET_VSOCK_PKT_BUFCOUNT);

  vq = priv->hdev->vrings_info[VSOCK_VQ_TX].vq;
  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->tx_lock);
      head = vhost_get_vq_buffers(vq, pkt->vb, vbcnt, &pkt->vbcnt);
      spin_unlock_irqrestore(&priv->tx_lock, flags);
      if (head < 0)
        {
          nxsem_wait(&priv->tx_sem);
          continue;
        }

      pkt->vbidx = 0;
      pkt->vboff = VIRTIO_VSOCK_HDR_LEN;
      pkt->priv = (FAR void *)(uintptr_t)head;
      return 0;
    }
}

/****************************************************************************
 * Name: vsock_vhost_send_pkt
 ****************************************************************************/

static ssize_t vsock_vhost_send_pkt(FAR struct vsock_transport_s *t,
                                    FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_vhost_priv_s *priv = vsock_vhost_trans2priv(t);
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  FAR struct virtqueue *vq;
  ssize_t len = hdr->len;
  irqstate_t flags;
  int ret;

  vq = priv->hdev->vrings_info[VSOCK_VQ_TX].vq;
  flags = spin_lock_irqsave(&priv->tx_lock);
  ret = virtqueue_add_consumed_buffer(vq, (uint16_t)(uintptr_t)pkt->priv,
                                      hdr->len + VIRTIO_VSOCK_HDR_LEN);
  if (ret < 0)
    {
      spin_unlock_irqrestore(&priv->tx_lock, flags);
      vhosterr("Add buffer failed ret=%d\n", ret);
      return ret;
    }

  virtqueue_kick(vq);

  spin_unlock_irqrestore(&priv->tx_lock, flags);
  return len;
}

/****************************************************************************
 * Name: vsock_vhost_rx_work
 ****************************************************************************/

static void vsock_vhost_rx_work(FAR void *arg)
{
  FAR struct vsock_vhost_priv_s *priv = arg;
  FAR struct virtqueue *vq;
  struct vsock_pkt_s pkt;
  bool added = false;
  int head;

  vq = priv->hdev->vrings_info[VSOCK_VQ_RX].vq;
  memset(&pkt, 0, sizeof(struct vsock_pkt_s));

  do
    {
      virtqueue_disable_cb(vq);
      for (; ; )
        {
          head = vhost_get_vq_buffers(vq, pkt.vb,
                                      CONFIG_NET_VSOCK_PKT_BUFCOUNT,
                                      &pkt.vbcnt);
          if (head < 0)
            {
              break;
            }

          pkt.vbidx = 0;
          pkt.vboff = VIRTIO_VSOCK_HDR_LEN;
          vsock_recv_pkt(&priv->t, &pkt);
          virtqueue_add_consumed_buffer(vq, head, 0);
          added = true;
        }
    }
  while (virtqueue_enable_cb(vq));

  if (added)
    {
      virtqueue_kick(vq);
    }
}

/****************************************************************************
 * Name: vsock_vhost_handle_rx
 ****************************************************************************/

static void vsock_vhost_handle_rx(FAR struct virtqueue *vq)
{
  FAR struct vsock_vhost_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->rx_work))
    {
      vsock_queue_work(&priv->rx_work, vsock_vhost_rx_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vsock_vhost_handle_tx
 ****************************************************************************/

static void vsock_vhost_handle_tx(FAR struct virtqueue *vq)
{
  FAR struct vsock_vhost_priv_s *priv = vq->vq_dev->priv;

  vsock_post(&priv->tx_sem);
}

/****************************************************************************
 * Name: vsock_vhost_probe
 ****************************************************************************/

static int vsock_vhost_probe(FAR struct vhost_device *hdev)
{
  FAR struct vsock_vhost_priv_s *priv;
  FAR const char *vqnames[VSOCK_VQ_NUM];
  vq_callback callbacks[VSOCK_VQ_NUM];
  uint64_t features;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->hdev = hdev;
  hdev->priv = priv;
  spin_lock_init(&priv->tx_lock);
  nxsem_init(&priv->tx_sem, 0, 0);

  /* Create the virtqueues */

  vqnames[VSOCK_VQ_RX]      = "vsock_vhost_handle_rx";
  vqnames[VSOCK_VQ_TX]      = "vsock_vhost_handle_tx";
  vqnames[VSOCK_VQ_EVENT]   = "vsock_vhost_handle_event";
  callbacks[VSOCK_VQ_RX]    = vsock_vhost_handle_rx;
  callbacks[VSOCK_VQ_TX]    = vsock_vhost_handle_tx;
  callbacks[VSOCK_VQ_EVENT] = NULL;
  ret = vhost_create_virtqueues(hdev, 0, VSOCK_VQ_NUM, vqnames, callbacks,
                                NULL);
  if (ret < 0)
    {
      vhosterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err;
    }

  virtio_get_features(hdev, &features);
  virtio_read_config_member(priv->hdev, struct virtio_vsock_config_s,
                            guest_cid, &priv->guest_cid);

  if (virtio_has_feature(hdev, VIRTIO_VSOCK_F_HOST_CID))
    {
      virtio_read_config_member(priv->hdev, struct virtio_vsock_config_s,
                                host_cid, &priv->host_cid);
    }
  else
    {
      priv->host_cid = VMADDR_CID_HOST;
    }

  /* Register Virtual Socket H2G transport */

  priv->t.ops = &g_vsock_vhost_transport_ops;
  vsock_transport_register(&priv->t);

  virtqueue_enable_cb(hdev->vrings_info[VSOCK_VQ_RX].vq);
  virtqueue_enable_cb(hdev->vrings_info[VSOCK_VQ_TX].vq);
  return ret;

err:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vsock_vhost_remove
 ****************************************************************************/

static void vsock_vhost_remove(FAR struct vhost_device *hdev)
{
  FAR struct vsock_vhost_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[VSOCK_VQ_RX].vq);
  virtqueue_disable_cb(hdev->vrings_info[VSOCK_VQ_TX].vq);
  vhost_delete_virtqueues(hdev);
  nxsem_destroy(&priv->tx_sem);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_vhost_initialize
 ****************************************************************************/

int vsock_vhost_initialize(void)
{
  return vhost_register_driver(&g_vsock_vhost_driver);
}
