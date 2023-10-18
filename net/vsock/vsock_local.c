/****************************************************************************
 * net/vsock/vsock_local.c
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
#include <stdio.h>

#include <nuttx/list.h>
#include <nuttx/nuttx.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>

#include "vsock/vsock.h"
#include "vsock/vsock_virtio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vsock_local_priv_s
{
  struct vsock_transport_s t;

  spinlock_t               buf_lock;
  struct list_node         buf_list;
  int                      buf_num;
  sem_t                    buf_sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint64_t vsock_local_get_localcid(FAR struct vsock_transport_s *t);
static uint64_t vsock_local_get_remotecid(FAR struct vsock_transport_s *t);
static int vsock_local_alloc_pkt(FAR struct vsock_transport_s *t,
                                 FAR struct vsock_pkt_s *pkt,
                                 size_t data_len);
static ssize_t vsock_local_send_pkt(FAR struct vsock_transport_s *t,
                                    FAR struct vsock_pkt_s *pkt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct vsock_transport_ops_s g_vsock_local_transport_ops =
{
  .get_localcid  = vsock_local_get_localcid,
  .get_remotecid = vsock_local_get_remotecid,
  .alloc_pkt     = vsock_local_alloc_pkt,
  .send_pkt      = vsock_local_send_pkt,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_virtio_trans2priv
 ****************************************************************************/

static FAR struct vsock_local_priv_s *
vsock_virtio_trans2priv(FAR struct vsock_transport_s *t)
{
  return container_of(t, struct vsock_local_priv_s, t);
}

/****************************************************************************
 * Name: vsock_local_get_localcid
 ****************************************************************************/

static uint64_t vsock_local_get_localcid(FAR struct vsock_transport_s *t)
{
  return VMADDR_CID_LOCAL;
}

/****************************************************************************
 * Name: vsock_local_get_remotecid
 ****************************************************************************/

static uint64_t vsock_local_get_remotecid(FAR struct vsock_transport_s *t)
{
  return VMADDR_CID_LOCAL;
}

/****************************************************************************
 * Name: vsock_local_alloc_pkt
 ****************************************************************************/

static int vsock_local_alloc_pkt(FAR struct vsock_transport_s *t,
                                 FAR struct vsock_pkt_s *pkt,
                                 size_t data_len)
{
  FAR struct vsock_local_priv_s *priv = vsock_virtio_trans2priv(t);
  irqstate_t flags;
  size_t vbcnt;

  data_len += VIRTIO_VSOCK_HDR_LEN;
  vbcnt = div_round_up(data_len, CONFIG_NET_VSOCK_PKT_BUFSIZE);
  vbcnt = MIN(vbcnt, CONFIG_NET_VSOCK_PKT_BUFCOUNT);

  for (; ; )
    {
      flags = spin_lock_irqsave(&priv->buf_lock);
      if (priv->buf_num > 0)
        {
          size_t i;

          vbcnt = MIN(vbcnt, priv->buf_num);
          for (i = 0; i < vbcnt; i++)
            {
              pkt->vb[i].buf = list_remove_head(&priv->buf_list);
              DEBUGASSERT(pkt->vb[i].buf != NULL);
              if (data_len >= CONFIG_NET_VSOCK_PKT_BUFSIZE)
                {
                  pkt->vb[i].len = CONFIG_NET_VSOCK_PKT_BUFSIZE;
                  data_len -= CONFIG_NET_VSOCK_PKT_BUFSIZE;
                }
              else
                {
                  pkt->vb[i].len = data_len;
                }

              pkt->len += pkt->vb[i].len;
            }

          priv->buf_num -= vbcnt;
          spin_unlock_irqrestore(&priv->buf_lock, flags);

          pkt->vbcnt = vbcnt;
          pkt->vbidx = 0;
          pkt->vboff = VIRTIO_VSOCK_HDR_LEN;
          return 0;
        }

      spin_unlock_irqrestore(&priv->buf_lock, flags);
      nxsem_wait(&priv->buf_sem);
    }
}

/****************************************************************************
 * Name: vsock_local_send_pkt
 ****************************************************************************/

static ssize_t vsock_local_send_pkt(FAR struct vsock_transport_s *t,
                                    FAR struct vsock_pkt_s *pkt)
{
  FAR struct vsock_local_priv_s *priv = vsock_virtio_trans2priv(t);
  FAR struct vsock_hdr_s *hdr = vsock_pkt2hdr(pkt);
  ssize_t len = hdr->len;
  irqstate_t flags;
  size_t i;

  pkt->vbidx = 0;
  pkt->vboff = VIRTIO_VSOCK_HDR_LEN;
  vsock_recv_pkt(&priv->t, pkt);

  flags = spin_lock_irqsave(&priv->buf_lock);
  for (i = 0; i < pkt->vbcnt; i++)
    {
      list_add_head(&priv->buf_list, pkt->vb[i].buf);
      priv->buf_num++;
    }

  spin_unlock_irqrestore(&priv->buf_lock, flags);
  vsock_post(&priv->buf_sem);
  return len;
}

/****************************************************************************
 * Name: vsock_local_buf_init
 ****************************************************************************/

static int vsock_local_buf_init(FAR struct vsock_local_priv_s *priv)
{
  FAR char *buf;
  size_t size;
  int i;

  spin_lock_init(&priv->buf_lock);
  list_initialize(&priv->buf_list);
  nxsem_init(&priv->buf_sem, 0, 0);

  priv->buf_num = CONFIG_NET_VSOCK_VQ_BUFCOUNT;
  size = VIRTIO_VSOCK_HDR_LEN + CONFIG_NET_VSOCK_PKT_BUFSIZE;
  buf = kmm_memalign(16, priv->buf_num * size);
  if (buf == NULL)
    {
      nxsem_destroy(&priv->buf_sem);
      return -ENOMEM;
    }

  for (i = 0; i < priv->buf_num; i++)
    {
      list_add_tail(&priv->buf_list, (FAR struct list_node *)buf);
      buf += size;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vsock_local_initialize
 ****************************************************************************/

int vsock_local_initialize(void)
{
  FAR struct vsock_local_priv_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  ret = vsock_local_buf_init(priv);
  if (ret < 0)
    {
      vrterr("vsock_local_buf_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Register Virtual Socket Local transport */

  priv->t.ops = &g_vsock_local_transport_ops;
  vsock_transport_register(&priv->t);
  return ret;

err_with_priv:
  kmm_free(priv);
  return ret;
}
