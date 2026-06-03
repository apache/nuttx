/****************************************************************************
 * drivers/vhost/vhost-i2c.c
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
#include <sys/endian.h>
#include <sys/param.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-i2c.h>
#include <nuttx/wqueue.h>

#include "vhost-i2c.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_i2c_priv_s
{
  FAR struct vhost_device *hdev;
  struct file              bus;      /* Local I2C bus forwarded to */
  struct work_s            work;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vhost_i2c_probe(FAR struct vhost_device *hdev);
static void vhost_i2c_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_i2c_driver =
{
  LIST_INITIAL_VALUE(g_vhost_i2c_driver.node),
  VIRTIO_ID_I2C_ADAPTER, /* Device id */
  vhost_i2c_probe,       /* Probe */
  vhost_i2c_remove,      /* Remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_i2c_decode
 *
 * Description:
 *   Decode a VirtIO I2C request into a NuttX I2C message.
 *
 ****************************************************************************/

static int virtio_i2c_decode(FAR const struct virtio_i2c_out_hdr_s *out,
                             FAR void *data, size_t buflen,
                             FAR struct i2c_msg_s *msg)
{
  uint32_t flags;
  uint16_t raw;

  if (out == NULL || (buflen > 0 && data == NULL))
    {
      return -EINVAL;
    }

  raw = le16toh(out->addr);
  if ((raw & 1) != 0 || le16toh(out->padding) != 0)
    {
      return -EINVAL;
    }

  flags = le32toh(out->flags);
  if ((flags & ~(VIRTIO_I2C_FLAGS_FAIL_NEXT | VIRTIO_I2C_FLAGS_M_RD)) != 0)
    {
      return -EINVAL;
    }

  msg->frequency = 0;
  msg->flags = (flags & VIRTIO_I2C_FLAGS_M_RD) != 0 ? I2C_M_READ : 0;
  if ((raw & 0x00f8) == 0x00f0)
    {
      /* 11110 marker in bits 7-3 => 10-bit address mode. */

      msg->flags |= I2C_M_TEN;
      msg->addr = (((raw >> 1) & 0x03) << 8) | ((raw >> 8) & 0xff);
    }
  else
    {
      msg->addr = (raw >> 1) & 0x7f;
    }

  msg->buffer = data;
  msg->length = buflen;

  return OK;
}

/****************************************************************************
 * Name: vhost_i2c_work
 *
 * Description:
 *   Process available I2C requests.  Each descriptor chain carries a single
 *   OASIS VirtIO I2C request laid out as an out header, an optional data
 *   buffer, and an in header.  Each role is its own descriptor, so a
 *   frontend that maps the buffers separately (non-contiguous) works too.
 *
 ****************************************************************************/

static void vhost_i2c_work(FAR void *arg)
{
  FAR struct vhost_i2c_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[0].vq;

  for (; ; )
    {
      FAR struct virtio_i2c_in_hdr_s *in = NULL;
      struct virtqueue_buf vb[3];
      struct i2c_msg_s msg;
      size_t vbcnt = 0;
      size_t reqlen = 0;
      int head;
      int ret;
      size_t i;

      head = vhost_get_vq_buffers(vq, vb, nitems(vb), &vbcnt);
      if (head < 0)
        {
          break;
        }

      for (i = 0; i < vbcnt; i++)
        {
          reqlen += vb[i].len;
        }

      /* Layout: out header, optional data, in header.  Grab the in header
       * (the last descriptor) first so a rejected request can still report
       * an error.  vhost_get_vq_buffers caps vbcnt at nitems(vb), so a valid
       * request has two or three descriptors.
       */

      if (vbcnt >= 2 && vb[vbcnt - 1].len >= sizeof(*in))
        {
          in = vb[vbcnt - 1].buf;
        }

      if (in != NULL && vb[0].len >= sizeof(struct virtio_i2c_out_hdr_s))
        {
          FAR void *data = vbcnt == 3 ? vb[1].buf : NULL;
          size_t buflen = vbcnt == 3 ? vb[1].len : 0;

          ret = virtio_i2c_decode(vb[0].buf, data, buflen, &msg);
          if (ret >= 0)
            {
              struct i2c_transfer_s xfer;

              xfer.msgv = &msg;
              xfer.msgc = 1;
              ret = file_ioctl(&priv->bus, I2CIOC_TRANSFER, &xfer);
            }

          in->status = ret >= 0 ? VIRTIO_I2C_MSG_OK : VIRTIO_I2C_MSG_ERR;
        }
      else if (in != NULL)
        {
          in->status = VIRTIO_I2C_MSG_ERR;
        }

      virtqueue_add_consumed_buffer(vq, head, reqlen);
    }

  virtqueue_kick(vq);
}

/****************************************************************************
 * Name: vhost_i2c_handler
 ****************************************************************************/

static void vhost_i2c_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_i2c_priv_s *priv = vq->vq_dev->priv;

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, vhost_i2c_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_i2c_probe
 ****************************************************************************/

static int vhost_i2c_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_i2c_priv_s *priv;
  struct vhost_i2c_config_s cfg;
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

  /* Open the local I2C device node named in the device config space. */

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

  vqnames[0]  = "virtio_i2c_req";
  callback[0] = vhost_i2c_handler;
  ret = vhost_create_virtqueues(hdev, 0, 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      file_close(&priv->bus);
      kmm_free(priv);
      return ret;
    }

  virtqueue_enable_cb(hdev->vrings_info[0].vq);

  /* Process any requests already queued by the remote frontend. */

  vhost_i2c_work(priv);
  return OK;
}

/****************************************************************************
 * Name: vhost_i2c_remove
 ****************************************************************************/

static void vhost_i2c_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_i2c_priv_s *priv = hdev->priv;

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
 * Name: vhost_register_i2c_driver
 ****************************************************************************/

int vhost_register_i2c_driver(void)
{
  return vhost_register_driver(&g_vhost_i2c_driver);
}
