/****************************************************************************
 * net/vsock/vsock_virtio.c
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

#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/event.h>
#include <nuttx/list.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>

#include "vsock/vsock.h"
#include "vsock/vsock_virtio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VSOCK_VQ_RX         0
#define VSOCK_VQ_TX         1
#define VSOCK_VQ_EVENT      2
#define VSOCK_VQ_NUM        3

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vsock_virtio_priv_s
{
  FAR struct virtio_device *vdev;           /* Virtio device */
  uint64_t                  guest_cid;
  uint64_t                  host_cid;

  /* Rx buffer management
   * rx_buf_list: rx free buffer list
   * rx_buf_num : rx free buffer number in rx_buf_list
   */

  struct list_node          rx_buf_list;
  int                       rx_buf_num;
  FAR void                 *rx_buf_alloc;

  /* Tx buffer management
   * tx_lock  : protect the tx buffer list
   * tx_buf_list  : hold all the tx free buffer
   * tx_buf_num   : tx free buffer number in tx_buf_list
   */

  spinlock_t                tx_lock;
  struct list_node          tx_buf_list;
  int                       tx_buf_num;
  FAR void                 *tx_buf_alloc;
  sem_t                     tx_sem;

  /* Workers
   * rx_work     : used for packet receive
   */

  struct work_s             rx_work;

  struct vsock_transport_s  t;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Virtio vsock driver probe/remote function */

static int vsock_virtio_probe(FAR struct virtio_device *vdev);
static void vsock_virtio_remove(FAR struct virtio_device *vdev);

static uint64_t vsock_virtio_get_localcid(FAR struct vsock_transport_s *t);
static uint64_t vsock_virtio_get_remotecid(FAR struct vsock_transport_s *t);
static int vsock_virtio_alloc_pkt(FAR struct vsock_transport_s *t,
                                  FAR struct vsock_pkt_s *pkt,
                                  size_t data_len);
static ssize_t vsock_virtio_send_pkt(FAR struct vsock_transport_s *t,
                                     FAR struct vsock_pkt_s *pkt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_vsock_virtio_driver =
{
  LIST_INITIAL_VALUE(g_vsock_virtio_driver.node), /* node */
  VIRTIO_ID_VSOCK,                                /* device id */
  vsock_virtio_probe,                             /* probe */
  vsock_virtio_remove,                            /* remove */
};

static const struct vsock_transport_ops_s g_vsock_virtio_transport_ops =
{
  .get_localcid  = vsock_virtio_get_localcid,
  .get_remotecid = vsock_virtio_get_remotecid,
  .alloc_pkt     = vsock_virtio_alloc_pkt,
  .send_pkt      = vsock_virtio_send_pkt,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_virtio_trans2priv
 ****************************************************************************/

static FAR struct vsock_virtio_priv_s *
vsock_virtio_trans2priv(FAR struct vsock_transport_s *t)
{
  return container_of(t, struct vsock_virtio_priv_s, t);
}

/****************************************************************************
 * Name: vsock_virtio_get_localcid
 ****************************************************************************/

static uint64_t
vsock_virtio_get_localcid(FAR struct vsock_transport_s *t)
{
  FAR struct vsock_virtio_priv_s *priv = vsock_virtio_trans2priv(t);

  return priv->guest_cid;
}

/****************************************************************************
 * Name: vsock_virtio_get_remotecid
 ****************************************************************************/

static uint64_t
vsock_virtio_get_remotecid(FAR struct vsock_transport_s *t)
{
  FAR struct vsock_virtio_priv_s *priv = vsock_virtio_trans2priv(t);

  return priv->host_cid;
}

/****************************************************************************
 * Name: vsock_virtio_alloc_pkt
 ****************************************************************************/

static int vsock_virtio_alloc_pkt(FAR struct vsock_transport_s *t,
                                  FAR struct vsock_pkt_s *pkt,
                                  size_t data_len)
{
  FAR struct vsock_virtio_priv_s *priv = vsock_virtio_trans2priv(t);
  FAR struct list_node *buf;
  irqstate_t flags;
  size_t vbcnt;

  data_len += VIRTIO_VSOCK_HDR_LEN;
  vbcnt = div_round_up(data_len, CONFIG_NET_VSOCK_PKT_BUFSIZE);
  vbcnt = MIN(vbcnt, CONFIG_NET_VSOCK_PKT_BUFCOUNT);

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->tx_lock);
      if (priv->tx_buf_num > 0)
        {
          size_t i;

          vbcnt = MIN(vbcnt, priv->tx_buf_num);
          for (i = 0; i < vbcnt; i++)
            {
              buf = list_remove_head(&priv->tx_buf_list);
              DEBUGASSERT(buf != NULL);
              pkt->vb[i].buf = buf;
              if (data_len >= CONFIG_NET_VSOCK_PKT_BUFSIZE)
                {
                  pkt->vb[i].len = CONFIG_NET_VSOCK_PKT_BUFSIZE;
                  data_len -= CONFIG_NET_VSOCK_PKT_BUFSIZE;
                }
              else
                {
                  pkt->vb[i].len = data_len;
                }
            }

          priv->tx_buf_num -= vbcnt;
          spin_unlock_irqrestore(&priv->tx_lock, flags);

          pkt->vbcnt = vbcnt;
          pkt->vbidx = 0;
          pkt->vboff = VIRTIO_VSOCK_HDR_LEN;
          return 0;
        }

      spin_unlock_irqrestore(&priv->tx_lock, flags);
      nxsem_wait(&priv->tx_sem);
    }
}

/****************************************************************************
 * Name: vsock_virtio_free_rx_pkt
 ****************************************************************************/

static void vsock_virtio_free_rx_pkt(FAR struct vsock_virtio_priv_s *priv,
                                     FAR struct vsock_pkt_s *pkt)
{
  size_t i;

  for (i = 0; i < pkt->vbcnt; i++)
    {
      list_add_head(&priv->rx_buf_list,
                    (FAR struct list_node *)pkt->vb[i].buf);
      priv->rx_buf_num++;
    }
}

/****************************************************************************
 * Name: vsock_virtio_send_pkt_work
 ****************************************************************************/

static ssize_t vsock_virtio_send_pkt(FAR struct vsock_transport_s *t,
                                     FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_virtio_priv_s *priv = vsock_virtio_trans2priv(t);
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq;
  uint32_t len = hdr->len;
  irqstate_t flags;
  int ret;

  vq = vdev->vrings_info[VSOCK_VQ_TX].vq;
  flags = spin_lock_irqsave(&priv->tx_lock);

  ret = virtqueue_add_buffer(vq, &pkt->vb[0], pkt->vbcnt, 0, pkt->vb[0].buf);
  if (ret < 0)
    {
      vrterr("Add buffer failed ret=%d\n", ret);
      spin_unlock_irqrestore(&priv->tx_lock, flags);
      return ret;
    }

  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->tx_lock, flags);
  return len;
}

/****************************************************************************
 * Name: vsock_virtio_rx_fill
 *
 * Description:
 *   Fill the rx buffer to the RX virtqueue.
 *
 ****************************************************************************/

static void vsock_virtio_rx_fill(FAR struct vsock_virtio_priv_s *priv)
{
  FAR struct virtqueue *vq;
  int ret;

  vq = priv->vdev->vrings_info[VSOCK_VQ_RX].vq;

  for (; ; )
    {
      struct virtqueue_buf vb;
      FAR void *buf;

      buf = list_remove_head(&priv->rx_buf_list);
      if (buf == NULL)
        {
          break;
        }

      vb.buf = buf;
      vb.len = VIRTIO_VSOCK_HDR_LEN + CONFIG_NET_VSOCK_PKT_BUFSIZE;

      ret = virtqueue_add_buffer(vq, &vb, 0, 1, buf);
      if (ret < 0)
        {
          vrterr("Add rx buffer %d failed, ret=%d\n", ret);
          list_add_head(&priv->rx_buf_list, (FAR struct list_node *)buf);
          break;
        }

      priv->rx_buf_num--;
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vsock_virtio_rx_work
 ****************************************************************************/

static void vsock_virtio_rx_work(FAR void *arg)
{
  FAR struct vsock_virtio_priv_s *priv = arg;
  FAR struct virtqueue *vq;
  struct vsock_pkt_s pkt;
  int ret;

  vq = priv->vdev->vrings_info[VSOCK_VQ_RX].vq;
  do
    {
      virtqueue_disable_cb(vq);
      for (; ; )
        {
          ret = virtio_get_buffers(vq, pkt.vb,
                                   CONFIG_NET_VSOCK_PKT_BUFCOUNT);
          if (ret < 0)
            {
              break;
            }

          pkt.vbidx = 0;
          pkt.vboff = VIRTIO_VSOCK_HDR_LEN;
          pkt.vbcnt = ret;
          vsock_recv_pkt(&priv->t, &pkt);
          vsock_virtio_free_rx_pkt(priv, &pkt);
        }
    }
  while (virtqueue_enable_cb(vq));

  vsock_virtio_rx_fill(priv);
}

/****************************************************************************
 * Name: vsock_virtio_rx_done
 *
 * Description:
 *   The virtio vsock rx virtqueue callback function, called when receive
 *   message.
 *
 * Input Parameters:
 *   vq - the rx virtqueue
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void vsock_virtio_rx_done(FAR struct virtqueue *vq)
{
  FAR struct vsock_virtio_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->rx_work))
    {
      vsock_queue_work(&priv->rx_work, vsock_virtio_rx_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vsock_virtio_tx_done
 *
 * Description:
 *   The virtio vsock tx virtqueue callback function, called when the sent
 *   message returned by other size.
 *
 * Input Parameters:
 *   vq - the tx virtqueue
 *
 * Returned Value:
 *   void
 *
 ****************************************************************************/

static void vsock_virtio_tx_done(struct virtqueue *vq)
{
  FAR struct vsock_virtio_priv_s *priv = vq->vq_dev->priv;
  struct virtqueue_buf vb[CONFIG_NET_VSOCK_PKT_BUFCOUNT];
  irqstate_t flags;
  int ret;
  int i;

  flags = spin_lock_irqsave(&priv->tx_lock);
  do
    {
      virtqueue_disable_cb(vq);
      for (; ; )
        {
          ret = virtio_get_buffers(vq, vb, CONFIG_NET_VSOCK_PKT_BUFCOUNT);
          if (ret < 0)
            {
              break;
            }

          for (i = 0; i < ret; i++)
            {
              list_add_head(&priv->tx_buf_list, vb[i].buf);
              priv->tx_buf_num++;
            }
        }
    }
  while (virtqueue_enable_cb(vq));

  spin_unlock_irqrestore(&priv->tx_lock, flags);
  vsock_post(&priv->tx_sem);
}

/****************************************************************************
 * Name: vsock_virtio_tx_buf_init
 ****************************************************************************/

static int vsock_virtio_tx_buf_init(FAR struct vsock_virtio_priv_s *priv)
{
  size_t size;
  int i;

  spin_lock_init(&priv->tx_lock);
  list_initialize(&priv->tx_buf_list);
  nxsem_init(&priv->tx_sem, 0, 0);

  size = VIRTIO_VSOCK_HDR_LEN + CONFIG_NET_VSOCK_PKT_BUFSIZE;
  priv->tx_buf_alloc = virtio_malloc_buf(priv->vdev,
                                         size * priv->tx_buf_num, 16);
  if (priv->tx_buf_alloc == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < priv->tx_buf_num; i++)
    {
      list_add_tail(&priv->tx_buf_list,
                    (FAR struct list_node *)(priv->tx_buf_alloc + i * size));
    }

  return 0;
}

/****************************************************************************
 * Name: vsock_virtio_rx_buf_init
 ****************************************************************************/

static int vsock_virtio_rx_buf_init(FAR struct vsock_virtio_priv_s *priv)
{
  size_t size;
  int i;

  list_initialize(&priv->rx_buf_list);

  size = VIRTIO_VSOCK_HDR_LEN + CONFIG_NET_VSOCK_PKT_BUFSIZE;
  priv->rx_buf_alloc = virtio_malloc_buf(priv->vdev,
                                         priv->rx_buf_num * size, 16);
  if (priv->rx_buf_alloc == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < priv->rx_buf_num; i++)
    {
      list_add_tail(&priv->rx_buf_list,
                   (FAR struct list_node *)(priv->rx_buf_alloc + i * size));
    }

  return 0;
}

/****************************************************************************
 * Name: vsock_virtio_init
 ****************************************************************************/

static int vsock_virtio_init(FAR struct vsock_virtio_priv_s *priv,
                             FAR struct virtio_device *vdev)
{
  FAR const char *vqname[VSOCK_VQ_NUM];
  vq_callback callback[VSOCK_VQ_NUM];
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev, 1UL << VIRTIO_VSOCK_F_HOST_CID, NULL);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqname[VSOCK_VQ_RX]      = "vsock_virtio_vq_rx";
  vqname[VSOCK_VQ_TX]      = "vsock_virtio_vq_tx";
  vqname[VSOCK_VQ_EVENT]   = "vsock_virtio_vq_event";
  callback[VSOCK_VQ_RX]    = vsock_virtio_rx_done;
  callback[VSOCK_VQ_TX]    = vsock_virtio_tx_done;
  callback[VSOCK_VQ_EVENT] = NULL;
  ret = virtio_create_virtqueues(vdev, 0, VSOCK_VQ_NUM, vqname, callback,
                                 NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      return ret;
    }

  /* Update the cid */

  virtio_read_config_member(priv->vdev, struct virtio_vsock_config_s,
                            guest_cid, &priv->guest_cid);

  if (virtio_has_feature(vdev, VIRTIO_VSOCK_F_HOST_CID))
    {
      virtio_read_config_member(priv->vdev, struct virtio_vsock_config_s,
                                host_cid, &priv->host_cid);
    }
  else
    {
      priv->host_cid = VMADDR_CID_HOST;
    }

  /* Get tx/tx vq size */

  priv->tx_buf_num = MIN(vdev->vrings_info[VSOCK_VQ_TX].vq->vq_nentries,
                         CONFIG_NET_VSOCK_VQ_BUFCOUNT);
  priv->rx_buf_num = MIN(vdev->vrings_info[VSOCK_VQ_RX].vq->vq_nentries,
                         CONFIG_NET_VSOCK_VQ_BUFCOUNT);

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

  /* Init the tx buffer list */

  ret = vsock_virtio_tx_buf_init(priv);
  if (ret < 0)
    {
      vrterr("vsock_virtio_tx_buf_init failed, ret=%d\n", ret);
      goto err_with_tx;
    }

  /* Fill the rx buffer list and virtqueue */

  ret = vsock_virtio_rx_buf_init(priv);
  if (ret < 0)
    {
      vrterr("vsock_virtio_rx_buf_init failed, ret=%d\n", ret);
      goto err_with_rx;
    }

  vsock_virtio_rx_fill(priv);

  /* Enable the virtqueue interrupt */

  virtqueue_enable_cb(vdev->vrings_info[VSOCK_VQ_RX].vq);
  virtqueue_enable_cb(vdev->vrings_info[VSOCK_VQ_TX].vq);
  return ret;

err_with_rx:
  nxsem_destroy(&priv->tx_sem);
  virtio_free_buf(priv->vdev, priv->tx_buf_alloc);
err_with_tx:
  virtio_delete_virtqueues(vdev);
  return ret;
}

/****************************************************************************
 * Name: vsock_virtio_uninit
 ****************************************************************************/

static void vsock_virtio_uninit(FAR struct vsock_virtio_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;

  virtqueue_disable_cb(vdev->vrings_info[VSOCK_VQ_TX].vq);
  virtqueue_disable_cb(vdev->vrings_info[VSOCK_VQ_RX].vq);
  virtio_delete_virtqueues(vdev);
  virtio_reset_device(vdev);
  nxsem_destroy(&priv->tx_sem);
  virtio_free_buf(priv->vdev, priv->rx_buf_alloc);
  virtio_free_buf(priv->vdev, priv->tx_buf_alloc);
}

/****************************************************************************
 * Name: vsock_virtio_probe
 ****************************************************************************/

static int vsock_virtio_probe(FAR struct virtio_device *vdev)
{
  FAR struct vsock_virtio_priv_s *priv;
  int ret;

  /* Alloc the virtio block driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the virtio block driver */

  ret = vsock_virtio_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("vsock_virtio_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Register Virtual Socket transport */

  priv->t.ops = &g_vsock_virtio_transport_ops;
  vsock_transport_register(&priv->t);
  return ret;

err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vsock_virtio_remove
 ****************************************************************************/

static void vsock_virtio_remove(FAR struct virtio_device *vdev)
{
  FAR struct vsock_virtio_priv_s *priv = vdev->priv;

  vsock_virtio_uninit(priv);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_virtio_initialize
 ****************************************************************************/

int vsock_virtio_initialize(void)
{
  return virtio_register_driver(&g_vsock_virtio_driver);
}
