/****************************************************************************
 * drivers/virtio/virtio-spi.c
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

#include <debug.h>
#include <errno.h>
#include <sys/endian.h>
#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_virtio.h>
#include <nuttx/virtio/virtio-spi.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-spi.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_spi_priv_s
{
  struct spi_dev_s           spi;      /* Must be the first member */
  FAR struct virtio_device  *vdev;
  dq_entry_t                 node;     /* Entry in the frontend registry */
  int                        idx;      /* VirtIO SPI bus index */
  mutex_t                    buslock;  /* SPI_LOCK exclusive bus access */
  sem_t                      done;     /* Request completion, one in flight */
  struct virtio_spi_config_s hwcfg;    /* Device configuration space */

  /* Wire-format request header, kept up to date by the SPI lower-half
   * ops; cs_change holds the after-transfer CS state and is cleared by
   * virtio_spi_exchange for intermediate chunks.
   */

  struct spi_transfer_head_s head;
};

/* Registry of probed frontends, published for virtio_spi_get_master() */

struct virtio_spi_bus_s
{
  spinlock_t        lock;
  struct dq_queue_s devices;
  int               idx;      /* Next bus index */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void virtio_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                              bool selected);
static uint32_t virtio_spi_setfrequency(FAR struct spi_dev_s *dev,
                                        uint32_t frequency);
#ifdef CONFIG_SPI_DELAY_CONTROL
static int virtio_spi_setdelay(FAR struct spi_dev_s *dev,
                               uint32_t startdelay, uint32_t stopdelay,
                               uint32_t csdelay, uint32_t ifdelay);
#endif
static void virtio_spi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void virtio_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int virtio_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                 spi_hwfeatures_t features);
#endif
static uint8_t virtio_spi_status(FAR struct spi_dev_s *dev, uint32_t devid);
static uint32_t virtio_spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static int virtio_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int virtio_spi_trigger(FAR struct spi_dev_s *dev);
#endif
static int virtio_spi_registercallback(FAR struct spi_dev_s *dev,
                                       spi_mediachange_t callback,
                                       FAR void *arg);

static int virtio_spi_probe(FAR struct virtio_device *vdev);
static void virtio_spi_remove(FAR struct virtio_device *vdev);
static void virtio_spi_dump(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_spi_driver =
{
  VIRTIO_ID_SPI,     /* device id */
  virtio_spi_probe,  /* probe */
  virtio_spi_remove, /* remove */
  virtio_spi_dump,   /* dump */
};

static struct virtio_spi_bus_s g_virtio_spi_bus =
{
  SP_UNLOCKED,
};

static const struct spi_ops_s g_virtio_spi_ops =
{
  virtio_spi_lock,
  virtio_spi_select,
  virtio_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  virtio_spi_setdelay,
#endif
  virtio_spi_setmode,
  virtio_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  virtio_spi_hwfeatures,
#endif
  virtio_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  NULL,
#endif
  virtio_spi_send,
  virtio_spi_exchange,
  NULL,                    /* exchangeseqs */
  NULL,                    /* cancelseqs */
#ifdef CONFIG_SPI_TRIGGER
  virtio_spi_trigger,
#endif
  virtio_spi_registercallback
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_spi_done
 *
 * Description:
 *   Request virtqueue completion callback: wake the waiter of every
 *   finished request.
 *
 ****************************************************************************/

static void virtio_spi_done(FAR struct virtqueue *vq)
{
  FAR sem_t *sem;

  /* One request in flight: never concurrent with the submitter */

  sem = virtqueue_get_buffer(vq, NULL, NULL);
  if (sem != NULL)
    {
      nxsem_post(sem);
    }
}

/****************************************************************************
 * Name: virtio_spi_transfer
 *
 * Description:
 *   Run one SPI transfer as a single VirtIO SPI request.  The header is
 *   the wire-format image cached in the driver state, kept up to date by
 *   the SPI lower-half ops; the
 *   result lives on the stack and the transmit and receive buffers are
 *   the caller's.  Each is mapped for the device: on MMIO/PCI the mapping
 *   is a no-op (zero copy), on a bouncing transport it is bounced through
 *   shared memory.  The descriptor chain is the header, an optional
 *   transmit buffer, an optional receive buffer, and the result byte.
 *
 ****************************************************************************/

static int virtio_spi_transfer(FAR struct virtio_spi_priv_s *priv,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t txlen,
                               size_t rxlen)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  struct spi_transfer_result_s result;
  struct virtqueue_buf vb[4];
  FAR void *headmap;
  FAR void *txmap;
  FAR void *rxmap;
  FAR void *resultmap;
  int readable = 0;
  int writable = 0;
  int ret;

  /* Callers hold the bus lock, serializing the single in-flight request
   * and the cached header it snapshots.
   */

  result.result = VIRTIO_SPI_TRANS_ERR;

  ret = virtio_map_buf(vdev, &priv->head, sizeof(priv->head),
                       VIRTIO_DMA_TO_DEVICE, &headmap);
  if (ret < 0)
    {
      return ret;
    }

  vb[readable + writable].buf = headmap;
  vb[readable + writable].len = sizeof(priv->head);
  readable++;

  if (txlen > 0)
    {
      ret = virtio_map_buf(vdev, (FAR void *)txbuffer, txlen,
                           VIRTIO_DMA_TO_DEVICE, &txmap);
      if (ret < 0)
        {
          goto out_head;
        }

      vb[readable + writable].buf = txmap;
      vb[readable + writable].len = txlen;
      readable++;
    }

  if (rxlen > 0)
    {
      ret = virtio_map_buf(vdev, rxbuffer, rxlen, VIRTIO_DMA_FROM_DEVICE,
                           &rxmap);
      if (ret < 0)
        {
          goto out_tx;
        }

      vb[readable + writable].buf = rxmap;
      vb[readable + writable].len = rxlen;
      writable++;
    }

  ret = virtio_map_buf(vdev, &result, sizeof(result),
                       VIRTIO_DMA_FROM_DEVICE, &resultmap);
  if (ret < 0)
    {
      goto out_rx;
    }

  vb[readable + writable].buf = resultmap;
  vb[readable + writable].len = sizeof(result);
  writable++;

  ret = virtqueue_add_buffer(vq, vb, readable, writable, &priv->done);
  if (ret >= 0)
    {
      virtqueue_kick(vq);
      nxsem_wait_uninterruptible(&priv->done);
    }

  /* unmap copies device-written data (rx, result) back to their buffers;
   * only after that are they valid on a bouncing transport.
   */

  virtio_unmap_buf(vdev, &result, sizeof(result), VIRTIO_DMA_FROM_DEVICE,
                   resultmap);
out_rx:
  if (rxlen > 0)
    {
      virtio_unmap_buf(vdev, rxbuffer, rxlen, VIRTIO_DMA_FROM_DEVICE, rxmap);
    }

out_tx:
  if (txlen > 0)
    {
      virtio_unmap_buf(vdev, (FAR void *)txbuffer, txlen,
                       VIRTIO_DMA_TO_DEVICE, txmap);
    }

out_head:
  virtio_unmap_buf(vdev, &priv->head, sizeof(priv->head),
                   VIRTIO_DMA_TO_DEVICE, headmap);

  if (ret >= 0)
    {
      ret = virtio_spi_status_to_errno(result.result);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_spi_lock
 ****************************************************************************/

static int virtio_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;

  return lock ? nxmutex_lock(&priv->buslock) :
                nxmutex_unlock(&priv->buslock);
}

/****************************************************************************
 * Name: virtio_spi_select
 ****************************************************************************/

static void virtio_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                              bool selected)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;
  uint32_t cs = SPIDEVID_INDEX(devid);

  /* CS is driven per request through head.cs_change: just latch the
   * device id, ignoring de-selects and ids the device cannot address
   * (head.chip_select_id is one byte).
   */

  if (selected && cs <= UINT8_MAX &&
      (priv->hwcfg.cs_max_number == 0 || cs < priv->hwcfg.cs_max_number))
    {
      priv->head.chip_select_id = cs;
    }
}

/****************************************************************************
 * Name: virtio_spi_setfrequency
 ****************************************************************************/

static uint32_t virtio_spi_setfrequency(FAR struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;

  if (priv->hwcfg.max_freq_hz != 0 && frequency > priv->hwcfg.max_freq_hz)
    {
      frequency = priv->hwcfg.max_freq_hz;
    }

  priv->head.freq = htole32(frequency);
  return frequency;
}

/****************************************************************************
 * Name: virtio_spi_setdelay
 ****************************************************************************/

#ifdef CONFIG_SPI_DELAY_CONTROL
static int virtio_spi_setdelay(FAR struct spi_dev_s *dev,
                               uint32_t startdelay, uint32_t stopdelay,
                               uint32_t csdelay, uint32_t ifdelay)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;

  priv->head.cs_setup_ns = htole32(startdelay);
  priv->head.cs_delay_hold_ns = htole32(stopdelay);
  priv->head.cs_change_delay_inactive_ns = htole32(csdelay);
  priv->head.word_delay_ns = htole32(ifdelay);
  return OK;
}
#endif

/****************************************************************************
 * Name: virtio_spi_setmode
 ****************************************************************************/

static void virtio_spi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;

  if (mode <= SPIDEV_MODE3)
    {
      uint32_t m = le32toh(priv->head.mode);

      m &= ~(VIRTIO_SPI_CPHA | VIRTIO_SPI_CPOL);
      priv->head.mode = htole32(m | virtio_spi_mode(mode));
    }
}

/****************************************************************************
 * Name: virtio_spi_setbits
 ****************************************************************************/

static void virtio_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;

  if (nbits > 0 && nbits <= 32 &&
      (priv->hwcfg.bits_per_word_mask == 0 ||
       (priv->hwcfg.bits_per_word_mask & (1u << (nbits - 1))) != 0))
    {
      priv->head.bits_per_word = nbits;
    }
}

/****************************************************************************
 * Name: virtio_spi_hwfeatures
 *
 * Description:
 *   Map hardware features to wire mode flags and cs_change forcing;
 *   -ENOSYS when a feature has no mapping or the device lacks support.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int virtio_spi_hwfeatures(FAR struct spi_dev_s *dev,
                                 spi_hwfeatures_t features)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;
  uint32_t mf = priv->hwcfg.mode_func_supported;
  spi_hwfeatures_t valid = 0;
  uint32_t flags = 0;
  uint32_t m;

#ifdef CONFIG_SPI_CS_CONTROL
  valid |= HWFEAT_FORCE_CS_INACTIVE_AFTER_TRANSFER |
           HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER;
  if (mf == 0 || (mf & VIRTIO_SPI_MF_SUPPORT_CS_HIGH) != 0)
    {
      valid |= HWFEAT_INVERT_CS_LEVEL;
    }
#endif

#ifdef CONFIG_SPI_BITORDER
  if (mf == 0 || (mf & VIRTIO_SPI_MF_SUPPORT_LSB_FIRST) != 0)
    {
      valid |= HWFEAT_LSBFIRST;
    }
#endif

  if ((features & ~valid) != 0)
    {
      return -ENOSYS;
    }

#ifdef CONFIG_SPI_CS_CONTROL
  if ((features & HWFEAT_INVERT_CS_LEVEL) != 0)
    {
      flags |= VIRTIO_SPI_CS_HIGH;
    }

  priv->head.cs_change =
    (features & HWFEAT_FORCE_CS_ACTIVE_AFTER_TRANSFER) != 0 ? 0 : 1;
#endif

#ifdef CONFIG_SPI_BITORDER
  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      flags |= VIRTIO_SPI_MODE_LSB_FIRST;
    }
#endif

  m = le32toh(priv->head.mode) &
      ~(VIRTIO_SPI_CS_HIGH | VIRTIO_SPI_MODE_LSB_FIRST);
  priv->head.mode = htole32(m | flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: virtio_spi_status
 ****************************************************************************/

static uint8_t virtio_spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: virtio_spi_send
 ****************************************************************************/

static uint32_t virtio_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  uint32_t rxword = 0;
  int ret;

  /* Exchange the low wordsize bytes of the word, in native byte order. */

  ret = virtio_spi_exchange(dev, &wd, &rxword, 1);
  return ret < 0 ? 0 : rxword;
}

/****************************************************************************
 * Name: virtio_spi_exchange
 ****************************************************************************/

static int virtio_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer,
                               FAR void *rxbuffer, size_t nwords)
{
  FAR struct virtio_spi_priv_s *priv = (FAR struct virtio_spi_priv_s *)dev;
  FAR const uint8_t *tx = txbuffer;
  FAR uint8_t *rx = rxbuffer;
  size_t wordsize = virtio_spi_wordsize(priv->head.bits_per_word);
  uint8_t cs_change = priv->head.cs_change;
  size_t largest;
  size_t maxdata;
  size_t offset = 0;
  size_t total;
  int ndirs;
  int ret;

  /* Split into chunks the transport can map at once; cs_change=0 makes the
   * device hold CS between them.  virtio_free_largest is SIZE_MAX on
   * MMIO/PCI, so this runs as a single zero-copy pass there.
   */

  ndirs = (tx != NULL) + (rx != NULL);
  if (ndirs == 0)
    {
      ndirs = 1;
    }

  largest = virtio_free_largest(priv->vdev);
  if (largest <= VIRTIO_SPI_REQ_SIZE(0, 0))
    {
      return -ENOMEM;
    }

  maxdata = (largest - VIRTIO_SPI_REQ_SIZE(0, 0)) / ndirs / wordsize *
            wordsize;
  if (maxdata == 0)
    {
      return -ENOMEM;
    }

  total = nwords * wordsize;

  do
    {
      size_t chunk = total - offset;

      /* Intermediate chunks keep CS asserted; the last chunk carries the
       * configured after-transfer state.
       */

      priv->head.cs_change = 0;
      if (chunk > maxdata)
        {
          chunk = maxdata;
        }
      else
        {
          priv->head.cs_change = cs_change;
        }

      ret = virtio_spi_transfer(priv,
                                tx != NULL ? tx + offset : NULL,
                                rx != NULL ? rx + offset : NULL,
                                tx != NULL ? chunk : 0,
                                rx != NULL ? chunk : 0);
      if (ret < 0)
        {
          priv->head.cs_change = cs_change;
          return ret;
        }

      offset += chunk;
    }
  while (offset < total);

  return OK;
}

#ifdef CONFIG_SPI_TRIGGER

/****************************************************************************
 * Name: virtio_spi_trigger
 ****************************************************************************/

static int virtio_spi_trigger(FAR struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: virtio_spi_registercallback
 ****************************************************************************/

static int virtio_spi_registercallback(FAR struct spi_dev_s *dev,
                                       spi_mediachange_t callback,
                                       FAR void *arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: virtio_spi_probe
 ****************************************************************************/

static int virtio_spi_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_spi_priv_s *priv;
  FAR const char *vqnames[1];
  vq_callback callback[1];
  irqstate_t flags;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->spi.ops = &g_virtio_spi_ops;
  priv->vdev = vdev;
  priv->head.bits_per_word = 8;
  priv->head.tx_nbits = VIRTIO_SPI_SINGLE;
  priv->head.rx_nbits = VIRTIO_SPI_SINGLE;
  priv->head.freq = htole32(1000000);
  priv->head.cs_change = 1;            /* De-select after transfer */
  vdev->priv = priv;
  nxmutex_init(&priv->buslock);
  nxsem_init(&priv->done, 0, 0);

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[0]  = "virtio_spi_req";
  callback[0] = virtio_spi_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);

  /* Read the SPI configuration space exposed by the device */

  virtio_read_config(vdev, 0, &priv->hwcfg, sizeof(priv->hwcfg));

  /* Publish the lower-half so board/in-kernel code can fetch it by bus index
   * with virtio_spi_get_master().  Exposing it as a /dev/spiN character
   * device with spi_register() is left to the board, as for any other SPI
   * lower-half.
   */

  flags = spin_lock_irqsave(&g_virtio_spi_bus.lock);
  priv->idx = g_virtio_spi_bus.idx++;
  dq_addlast(&priv->node, &g_virtio_spi_bus.devices);
  spin_unlock_irqrestore(&g_virtio_spi_bus.lock, flags);

  return OK;

err_with_priv:
  nxsem_destroy(&priv->done);
  nxmutex_destroy(&priv->buslock);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_spi_remove
 ****************************************************************************/

static void virtio_spi_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_spi_priv_s *priv = vdev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_virtio_spi_bus.lock);
  dq_rem(&priv->node, &g_virtio_spi_bus.devices);
  spin_unlock_irqrestore(&g_virtio_spi_bus.lock, flags);

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  nxsem_destroy(&priv->done);
  nxmutex_destroy(&priv->buslock);
  kmm_free(priv);
}

/****************************************************************************
 * Name: virtio_spi_dump
 ****************************************************************************/

static void virtio_spi_dump(FAR struct virtio_device *vdev)
{
  FAR struct virtio_spi_priv_s *priv = vdev->priv;

  syslog(LOG_EMERG, "    idx=%d\n", priv->idx);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_spi_driver
 ****************************************************************************/

int virtio_register_spi_driver(void)
{
  return virtio_register_driver(&g_virtio_spi_driver);
}

/****************************************************************************
 * Name: virtio_spi_get_master
 *
 * Description:
 *   Return the SPI lower-half probed by the VirtIO SPI frontend for bus
 *   index, or NULL if no such bus has been probed.  Board/in-kernel code
 *   uses this to obtain the instance and may then pass it to device drivers
 *   or register it with spi_register() as /dev/spiN.
 *
 ****************************************************************************/

FAR struct spi_dev_s *virtio_spi_get_master(int index)
{
  FAR struct virtio_spi_priv_s *priv;
  FAR struct spi_dev_s *spi = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_virtio_spi_bus.lock);
  dq_for_every_entry(&g_virtio_spi_bus.devices, priv,
                     struct virtio_spi_priv_s, node)
    {
      if (priv->idx == index)
        {
          spi = &priv->spi;
          break;
        }
    }

  spin_unlock_irqrestore(&g_virtio_spi_bus.lock, flags);
  return spi;
}
