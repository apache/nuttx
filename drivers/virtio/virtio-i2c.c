/****************************************************************************
 * drivers/virtio/virtio-i2c.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_virtio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-i2c.h>

#include "virtio-i2c.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_i2c_priv_s
{
  struct i2c_master_s       i2c;      /* Must be the first member */
  FAR struct virtio_device *vdev;
  dq_entry_t                node;     /* Entry in the frontend registry */
  int                       idx;      /* VirtIO I2C bus index */
  mutex_t                   lock;     /* Serialize transfers (bus owner) */
};

struct virtio_i2c_bus_s
{
  spinlock_t        lock;
  struct dq_queue_s devices;
  int               idx;      /* Next bus index */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int virtio_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count);
static int virtio_i2c_setup(FAR struct i2c_master_s *dev);
static int virtio_i2c_shutdown(FAR struct i2c_master_s *dev);
static uint8_t virtio_i2c_status(FAR struct i2c_master_s *dev);

static int virtio_i2c_probe(FAR struct virtio_device *vdev);
static void virtio_i2c_remove(FAR struct virtio_device *vdev);
static void virtio_i2c_dump(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_i2c_driver =
{
  VIRTIO_ID_I2C_ADAPTER, /* device id */
  virtio_i2c_probe,      /* probe */
  virtio_i2c_remove,     /* remove */
  virtio_i2c_dump,       /* dump */
};

/* Registry of probed VirtIO I2C frontends, so board/in-kernel code can
 * obtain the I2C lower-half through virtio_i2c_get_master().
 */

static struct virtio_i2c_bus_s g_virtio_i2c_bus =
{
  SP_UNLOCKED,
};

static const struct i2c_ops_s g_virtio_i2c_ops =
{
  virtio_i2c_transfer, /* transfer */
#ifdef CONFIG_I2C_RESET
  NULL,                /* reset */
#endif
  virtio_i2c_setup,    /* setup */
  virtio_i2c_shutdown, /* shutdown */
  virtio_i2c_status    /* status */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_i2c_encode
 *
 * Description:
 *   Encode a NuttX I2C message as a VirtIO I2C request.
 *
 ****************************************************************************/

static int virtio_i2c_encode(FAR struct virtio_i2c_out_hdr_s *out,
                             FAR const struct i2c_msg_s *msg, bool fail_next)
{
  uint32_t flags = 0;
  uint16_t addr;

  if (msg->flags & I2C_M_TEN)
    {
      /* 10-bit address mode: A7..A0 in bits 15-8, the 11110 marker in
       * bits 7-3, A9 A8 in bits 2-1.  Bit 0 is unused; the read/write
       * direction is carried by VIRTIO_I2C_FLAGS_M_RD.
       */

      if (msg->addr > 0x3ff)
        {
          return -EINVAL;
        }

      addr = ((msg->addr & 0xff) << 8) | 0xf0 |
             (((msg->addr >> 8) & 0x03) << 1);
    }
  else
    {
      /* 7-bit address mode: A6..A0 in bits 7-1.  The I2C standard reserves
       * 0x78..0x7b as the 10-bit addressing prefix, so they are not valid
       * 7-bit addresses.
       */

      if (msg->addr > 0x7f || (msg->addr & 0x7c) == 0x78)
        {
          return -EINVAL;
        }

      addr = msg->addr << 1;
    }

  if (fail_next)
    {
      flags |= VIRTIO_I2C_FLAGS_FAIL_NEXT;
    }

  if ((msg->flags & I2C_M_READ) != 0)
    {
      flags |= VIRTIO_I2C_FLAGS_M_RD;
    }

  out->addr = htole16(addr);
  out->padding = 0;
  out->flags = htole32(flags);
  return OK;
}

/****************************************************************************
 * Name: virtio_i2c_done
 *
 * Description:
 *   Request virtqueue completion callback.  Drain every completed request
 *   and wake its waiter.
 *
 ****************************************************************************/

static void virtio_i2c_done(FAR struct virtqueue *vq)
{
  FAR sem_t *sem;

  for (; ; )
    {
      sem = virtqueue_get_buffer(vq, NULL, NULL);
      if (sem == NULL)
        {
          break;
        }

      nxsem_post(sem);
    }
}

/****************************************************************************
 * Name: virtio_i2c_transfer_one
 *
 * Description:
 *   Submit a single OASIS VirtIO I2C request as one descriptor chain (out
 *   header, optional data buffer, in header) and wait for completion.  The
 *   out and in headers live on the stack; the data buffer is the caller's.
 *   Each is mapped for the device: a no-op (zero copy) on MMIO/PCI, bounced
 *   through shared memory on a bouncing transport.
 *
 ****************************************************************************/

static int virtio_i2c_transfer_one(FAR struct virtio_i2c_priv_s *priv,
                                   FAR struct i2c_msg_s *msg, bool fail_next)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  sem_t sem;
  struct virtio_i2c_out_hdr_s out;
  struct virtio_i2c_in_hdr_s in;
  struct virtqueue_buf vb[3];
  enum virtio_dma_map datadir = VIRTIO_DMA_TO_DEVICE;
  FAR void *outmap;
  FAR void *datamap = NULL;
  FAR void *inmap;
  bool read = (msg->flags & I2C_M_READ) != 0;
  int readable = 0;
  int writable = 0;
  int ret;

  ret = virtio_i2c_encode(&out, msg, fail_next);
  if (ret < 0)
    {
      return ret;
    }

  in.status = VIRTIO_I2C_MSG_ERR;

  /* The out header is always device-readable and the in header always
   * device-writable.  The data buffer is device-readable for a write and
   * device-writable for a read.
   */

  ret = virtio_map_buf(vdev, &out, sizeof(out), VIRTIO_DMA_TO_DEVICE,
                       &outmap);
  if (ret < 0)
    {
      return ret;
    }

  vb[readable + writable].buf = outmap;
  vb[readable + writable].len = sizeof(out);
  readable++;

  if (msg->length > 0)
    {
      datadir = read ? VIRTIO_DMA_FROM_DEVICE : VIRTIO_DMA_TO_DEVICE;
      ret = virtio_map_buf(vdev, msg->buffer, msg->length, datadir,
                           &datamap);
      if (ret < 0)
        {
          goto out_hdr;
        }

      vb[readable + writable].buf = datamap;
      vb[readable + writable].len = msg->length;
      if (read)
        {
          writable++;
        }
      else
        {
          readable++;
        }
    }

  ret = virtio_map_buf(vdev, &in, sizeof(in), VIRTIO_DMA_FROM_DEVICE,
                       &inmap);
  if (ret < 0)
    {
      goto out_data;
    }

  vb[readable + writable].buf = inmap;
  vb[readable + writable].len = sizeof(in);
  writable++;

  nxsem_init(&sem, 0, 0);

  ret = virtqueue_add_buffer(vq, vb, readable, writable, &sem);
  if (ret >= 0)
    {
      virtqueue_kick(vq);
      nxsem_wait_uninterruptible(&sem);
    }

  nxsem_destroy(&sem);

  /* unmap copies device-written data (read payload, status) back to their
   * buffers; only after that are they valid on a bouncing transport.
   */

  virtio_unmap_buf(vdev, &in, sizeof(in), VIRTIO_DMA_FROM_DEVICE, inmap);
out_data:
  if (msg->length > 0)
    {
      virtio_unmap_buf(vdev, msg->buffer, msg->length, datadir, datamap);
    }

out_hdr:
  virtio_unmap_buf(vdev, &out, sizeof(out), VIRTIO_DMA_TO_DEVICE, outmap);

  if (ret >= 0)
    {
      ret = virtio_i2c_result(in.status);
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_i2c_transfer
 *
 * Description:
 *   Submit each I2C message of the transaction in order, one VirtIO request
 *   per message.  The FAIL_NEXT flag is propagated so the device can
 *   associate the messages of a transaction, but the messages are not
 *   submitted as a single batch, so strict repeated-start atomicity is not
 *   guaranteed at the transport level.
 *
 ****************************************************************************/

static int virtio_i2c_transfer(FAR struct i2c_master_s *dev,
                               FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct virtio_i2c_priv_s *priv =
    (FAR struct virtio_i2c_priv_s *)dev;
  int ret = OK;
  int i;

  if (msgs == NULL || count <= 0)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      if (msgs[i].length < 0 ||
          (msgs[i].length > 0 && msgs[i].buffer == NULL))
        {
          return -EINVAL;
        }
    }

  nxmutex_lock(&priv->lock);

  for (i = 0; i < count; i++)
    {
      ret = virtio_i2c_transfer_one(priv, &msgs[i], i + 1 < count);
      if (ret < 0)
        {
          break;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_i2c_setup
 ****************************************************************************/

static int virtio_i2c_setup(FAR struct i2c_master_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: virtio_i2c_shutdown
 ****************************************************************************/

static int virtio_i2c_shutdown(FAR struct i2c_master_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: virtio_i2c_status
 ****************************************************************************/

static uint8_t virtio_i2c_status(FAR struct i2c_master_s *dev)
{
  return I2C_STATUS_PRESENT;
}

/****************************************************************************
 * Name: virtio_i2c_probe
 ****************************************************************************/

static int virtio_i2c_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_i2c_priv_s *priv;
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

  priv->i2c.ops = &g_virtio_i2c_ops;
  priv->vdev = vdev;
  vdev->priv = priv;
  nxmutex_init(&priv->lock);

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev,
                            1UL << VIRTIO_I2C_F_ZERO_LENGTH_REQUEST, NULL);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[0]  = "virtio_i2c_req";
  callback[0] = virtio_i2c_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_create_virtqueues failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);

  /* Publish the lower-half so board/in-kernel code can fetch it by bus index
   * with virtio_i2c_get_master().  Exposing it as a /dev/i2cN character
   * device with i2c_register() is left to the board, as for any other I2C
   * lower-half.
   */

  flags = spin_lock_irqsave(&g_virtio_i2c_bus.lock);
  priv->idx = g_virtio_i2c_bus.idx++;
  dq_addlast(&priv->node, &g_virtio_i2c_bus.devices);
  spin_unlock_irqrestore(&g_virtio_i2c_bus.lock, flags);

  return OK;

err_with_priv:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_i2c_remove
 ****************************************************************************/

static void virtio_i2c_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_i2c_priv_s *priv = vdev->priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_virtio_i2c_bus.lock);
  dq_rem(&priv->node, &g_virtio_i2c_bus.devices);
  spin_unlock_irqrestore(&g_virtio_i2c_bus.lock, flags);

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
}

/****************************************************************************
 * Name: virtio_i2c_dump
 ****************************************************************************/

static void virtio_i2c_dump(FAR struct virtio_device *vdev)
{
  FAR struct virtio_i2c_priv_s *priv = vdev->priv;

  syslog(LOG_EMERG, "    idx=%d\n", priv->idx);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_i2c_driver
 ****************************************************************************/

int virtio_register_i2c_driver(void)
{
  return virtio_register_driver(&g_virtio_i2c_driver);
}

/****************************************************************************
 * Name: virtio_i2c_get_master
 *
 * Description:
 *   Return the I2C lower-half probed by the VirtIO I2C frontend for bus
 *   index, or NULL if no such bus has been probed.  Board/in-kernel code
 *   uses this to obtain the instance and may then pass it to device drivers
 *   or register it with i2c_register() as /dev/i2cN.
 *
 ****************************************************************************/

FAR struct i2c_master_s *virtio_i2c_get_master(int index)
{
  FAR struct virtio_i2c_priv_s *priv;
  FAR struct i2c_master_s *i2c = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_virtio_i2c_bus.lock);
  dq_for_every_entry(&g_virtio_i2c_bus.devices, priv,
                     struct virtio_i2c_priv_s, node)
    {
      if (priv->idx == index)
        {
          i2c = &priv->i2c;
          break;
        }
    }

  spin_unlock_irqrestore(&g_virtio_i2c_bus.lock, flags);
  return i2c;
}
