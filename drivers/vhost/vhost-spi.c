/****************************************************************************
 * drivers/vhost/vhost-spi.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/param.h>
#include <sys/endian.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/percpu.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-spi.h>
#include <nuttx/wqueue.h>

#include "vhost-spi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_spi_priv_s
{
  FAR struct vhost_device *hdev;
  struct file              bus;      /* Local SPI bus forwarded to */
  struct work_s            work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vhost_spi_probe(FAR struct vhost_device *hdev);
static void vhost_spi_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DEFINE_PER_CPU_BMP(struct vhost_driver, g_vhost_spi_driver) =
{
  VIRTIO_ID_SPI,    /* Device id */
  vhost_spi_probe,  /* Probe */
  vhost_spi_remove, /* Remove */
};
#define g_vhost_spi_driver this_cpu_var_bmp(g_vhost_spi_driver)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_spi_decode
 *
 * Description:
 *   Decode a VirtIO SPI request into an SPI sequence with one transfer,
 *   mapping the CS_HIGH/LSB_FIRST wire mode flags to hardware features of
 *   the transfer; -ENOSYS when a flag is not mappable on this build.  The
 *   data length left in trans->nwords by vhost_spi_split is converted
 *   from bytes to words.
 *
 ****************************************************************************/

static int virtio_spi_decode(FAR const struct spi_transfer_head_s *head,
                             FAR struct spi_sequence_s *seq,
                             FAR struct spi_trans_s *trans)
{
  uint32_t mode = le32toh(head->mode);

  if ((mode & ~(VIRTIO_SPI_CPHA | VIRTIO_SPI_CPOL |
                VIRTIO_SPI_CS_HIGH | VIRTIO_SPI_MODE_LSB_FIRST |
                VIRTIO_SPI_MODE_LOOP)) != 0 ||
      head->bits_per_word == 0 || head->cs_change > 1 ||
      !virtio_spi_bus_width_valid(head->tx_nbits) ||
      !virtio_spi_bus_width_valid(head->rx_nbits))
    {
      return -EINVAL;
    }

  seq->frequency = le32toh(head->freq);
  seq->dev = SPIDEV_USER(head->chip_select_id);
  seq->mode = virtio_spi_nuttx_mode(mode);
  seq->nbits = head->bits_per_word;
#ifdef CONFIG_SPI_DELAY_CONTROL
  seq->a = le32toh(head->cs_setup_ns);
  seq->b = le32toh(head->cs_delay_hold_ns);
  seq->c = le32toh(head->cs_change_delay_inactive_ns);
  seq->i = le32toh(head->word_delay_ns);
#endif

  trans->deselect = head->cs_change != 0;
  trans->nwords /= virtio_spi_wordsize(seq->nbits);

#ifdef CONFIG_SPI_HWFEATURES
#  ifdef CONFIG_SPI_CS_CONTROL
  if ((mode & VIRTIO_SPI_CS_HIGH) != 0)
    {
      trans->hwfeat |= HWFEAT_INVERT_CS_LEVEL;
      mode &= ~VIRTIO_SPI_CS_HIGH;
    }
#  endif

#  ifdef CONFIG_SPI_BITORDER
  if ((mode & VIRTIO_SPI_MODE_LSB_FIRST) != 0)
    {
      trans->hwfeat |= HWFEAT_LSBFIRST;
      mode &= ~VIRTIO_SPI_MODE_LSB_FIRST;
    }
#  endif
#endif

  /* Any flag left unmapped (loopback always) is unsupported here. */

  return (mode & (VIRTIO_SPI_CS_HIGH | VIRTIO_SPI_MODE_LSB_FIRST |
                  VIRTIO_SPI_MODE_LOOP)) != 0 ? -ENOSYS : OK;
}

/****************************************************************************
 * Name: vhost_spi_split
 *
 * Description:
 *   Split the descriptor chain at idx into its role buffers: the head and
 *   the optional tx buffer are device-readable, the optional rx buffer and
 *   the result byte are device-writable.  Each role is its own descriptor,
 *   so a frontend that maps the buffers separately (non-contiguous) works
 *   too.  result and total are filled even on a malformed chain, so the
 *   caller can consume it and report an error.  Returns true when the chain
 *   is well-formed.
 *
 ****************************************************************************/

static bool vhost_spi_split(FAR struct virtqueue *vq, FAR uint8_t *buf,
                            uint32_t len, uint16_t idx,
                            FAR struct spi_transfer_head_s **head,
                            FAR struct spi_trans_s *trans,
                            FAR struct spi_transfer_result_s **result,
                            FAR size_t *total)
{
  FAR struct vring_desc *desc = vq->vq_ring.desc;
  FAR uint8_t *rdbuf = NULL;    /* First readable descriptor is the head */
  FAR uint8_t *wrbuf = NULL;    /* Last writable descriptor holds result */
  size_t rdlen = 0;
  size_t wrlen = 0;
  size_t txlen = 0;
  size_t rxlen = 0;
  uint16_t i = idx;

  memset(trans, 0, sizeof(*trans));
  *result = NULL;
  *total = 0;

  for (; ; )
    {
      *total += len;

      if ((desc[i].flags & VRING_DESC_F_WRITE) == 0)
        {
          /* Readable: the head, then the optional tx buffer. */

          if (rdbuf == NULL)
            {
              rdbuf = buf;
              rdlen = len;
            }
          else
            {
              trans->txbuffer = buf;
              txlen = len;
            }
        }
      else if (wrbuf == NULL)
        {
          /* First writable: held until we know if another one follows. */

          wrbuf = buf;
          wrlen = len;
        }
      else
        {
          /* A writable descriptor follows, so the one held was the rx
           * buffer and this new one carries the result byte.
           */

          trans->rxbuffer = wrbuf;
          rxlen = wrlen;
          wrbuf = buf;
          wrlen = len;
        }

      buf = virtqueue_get_next_avail_buffer(vq, i, &i, &len);
      if (buf == NULL)
        {
          break;
        }
    }

  /* The result byte is the tail of the last writable descriptor. */

  if (wrbuf != NULL && wrlen >= sizeof(struct spi_transfer_result_s))
    {
      *result = (FAR struct spi_transfer_result_s *)
                (wrbuf + wrlen - sizeof(struct spi_transfer_result_s));
    }

  /* Data length in bytes; virtio_spi_decode converts it to words. */

  trans->nwords = MAX(txlen, rxlen);
  *head = (FAR struct spi_transfer_head_s *)rdbuf;
  return rdbuf != NULL && rdlen >= sizeof(struct spi_transfer_head_s) &&
         *result != NULL;
}

/****************************************************************************
 * Name: vhost_spi_one
 *
 * Description:
 *   Split one VirtIO SPI request from its descriptor chain, decode it,
 *   drive the local SPI lower-half, and write the result byte back.  A
 *   malformed chain still gets its result byte written when a writable
 *   descriptor is present.  Returns the chain length in bytes so the
 *   caller can consume it.
 *
 ****************************************************************************/

static size_t vhost_spi_one(FAR struct file *spi, FAR struct virtqueue *vq,
                            FAR void *buf, uint32_t len, uint16_t idx)
{
  FAR struct spi_transfer_head_s *head;
  FAR struct spi_transfer_result_s *result;
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  size_t total;
  int ret;

  if (!vhost_spi_split(vq, buf, len, idx, &head, &trans, &result, &total))
    {
      if (result != NULL)
        {
          result->result = VIRTIO_SPI_PARAM_ERR;
        }

      return total;
    }

  memset(&seq, 0, sizeof(seq));
  seq.ntrans = 1;
  seq.trans = &trans;

  ret = virtio_spi_decode(head, &seq, &trans);
  if (ret >= 0)
    {
      /* cs_change=0 chains into the next request: keep CS asserted past
       * this sequence.  The bus lock is per sequence, so chained requests
       * are CS-continuous, not bus-exclusive.
       */

      seq.keepcs = !trans.deselect;
      ret = file_ioctl(spi, SPIIOC_TRANSFER, &seq);
    }

  result->result = virtio_spi_errno_to_status(ret);
  return total;
}

/****************************************************************************
 * Name: vhost_spi_work
 ****************************************************************************/

static void vhost_spi_work(FAR void *arg)
{
  FAR struct vhost_spi_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[0].vq;
  FAR void *buf;
  size_t total;
  uint16_t idx;
  uint32_t len;

  /* Only this worker touches the virtqueue: no locking needed */

  for (; ; )
    {
      buf = virtqueue_get_first_avail_buffer(vq, &idx, &len);
      if (buf == NULL)
        {
          break;
        }

      total = vhost_spi_one(&priv->bus, vq, buf, len, idx);
      virtqueue_add_consumed_buffer(vq, idx, total);
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vhost_spi_handler
 ****************************************************************************/

static void vhost_spi_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_spi_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, vhost_spi_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_spi_probe
 ****************************************************************************/

static int vhost_spi_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_spi_priv_s *priv;
  struct vhost_spi_config_s cfg;
  FAR const char *vqnames[1];
  vq_callback callback[1];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->hdev = hdev;

  /* Open the local SPI device node named in the device config space. */

  vhost_read_config(hdev, 0, &cfg, sizeof(cfg));
  cfg.path[sizeof(cfg.path) - 1] = '\0';

  ret = file_open(&priv->bus, cfg.path, O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      vhosterr("Failed to open %s: %d\n", cfg.path, ret);
      kmm_free(priv);
      return ret;
    }

  hdev->priv = priv;

  vqnames[0]  = "virtio_spi_req";
  callback[0] = vhost_spi_handler;
  ret = vhost_create_virtqueues(hdev, 0, 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      file_close(&priv->bus);
      kmm_free(priv);
      return ret;
    }

  virtqueue_enable_cb(hdev->vrings_info[0].vq);

  vhost_spi_work(priv);
  return OK;
}

/****************************************************************************
 * Name: vhost_spi_remove
 ****************************************************************************/

static void vhost_spi_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_spi_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[0].vq);
  work_cancel_sync(HPWORK, &priv->work);
  vhost_delete_virtqueues(hdev);
  file_close(&priv->bus);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_spi_driver
 ****************************************************************************/

int vhost_register_spi_driver(void)
{
  return vhost_register_driver(&g_vhost_spi_driver);
}
