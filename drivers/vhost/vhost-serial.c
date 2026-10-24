/****************************************************************************
 * drivers/vhost/vhost-serial.c
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
#include <fcntl.h>
#include <stdio.h>

#include <nuttx/lib/lib.h>
#include <nuttx/kthread.h>
#include <nuttx/spinlock.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-config.h>
#include <nuttx/virtio/virtio-serial.h>

#include "vhost-serial.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_serial_priv_s
{
  FAR struct vhost_device   *hdev;
  struct file                serial;
  sem_t                      semrx;
  sem_t                      semtx;
  pid_t                      pidrx;
  pid_t                      pidtx;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  vhost_serial_probe(FAR struct vhost_device *hdev);
static void vhost_serial_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_serial_driver =
{
  LIST_INITIAL_VALUE(g_vhost_serial_driver.node),      /* node */
  VIRTIO_ID_CONSOLE,                                   /* device id */
  vhost_serial_probe,                                  /* probe */
  vhost_serial_remove,                                 /* remove */
};

static struct vhost_driver g_vhost_rprocserial_driver =
{
  LIST_INITIAL_VALUE(g_vhost_rprocserial_driver.node), /* node */
  VIRTIO_ID_RPROC_SERIAL,                              /* device id */
  vhost_serial_probe,                                  /* probe */
  vhost_serial_remove,                                 /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t vhost_serial_safe_write(FAR struct vhost_serial_priv_s *priv,
                                       FAR void *buf, uint32_t len)
{
  size_t written = 0;
  ssize_t ret;

  while (written < len)
    {
      ret = file_write(&priv->serial, buf + written,
                        len - written);
      if (ret <= 0)
        {
          vhosterr("File write data failed %zd\n", ret);
          return ret;
        }

      written += ret;
    }

  return written;
}

/****************************************************************************
 * Name: vhost_serial_tx_worker
 ****************************************************************************/

static void vhost_serial_tx_worker(FAR struct vhost_serial_priv_s *priv)
{
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VIRTIO_SERIAL_TX].vq;
  struct virtqueue_buf vb[2];
  bool added = false;
  uint32_t len;
  ssize_t idx;
  ssize_t ret;
  size_t vbcnt;
  size_t i;

  /* Write data to actual serial port */

  for (; ; )
    {
      idx = vhost_get_vq_buffers(vq, vb, 2, &vbcnt);
      if (idx < 0)
        {
          break;
        }

      for (i = 0, len = 0; i < vbcnt; i++)
        {
          ret = vhost_serial_safe_write(priv, vb[i].buf, vb[i].len);
          if (ret <= 0)
            {
              break;
            }

          len += ret;
        }

      virtqueue_add_consumed_buffer(vq, idx, len);
      added = true;
    }

  if (added)
    {
      virtqueue_kick(vq);
    }
}

/****************************************************************************
 * Name: vhost_serial_tx_thread
 ****************************************************************************/

static int vhost_serial_tx_thread(int argc, FAR char *argv[])
{
  FAR struct vhost_serial_priv_s *priv = (FAR struct vhost_serial_priv_s *)
    ((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->semtx);
      vhost_serial_tx_worker(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: vhost_serial_tx_handler
 ****************************************************************************/

static void vhost_serial_tx_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_serial_priv_s *priv = vq->vq_dev->priv;
  int semcount;

  nxsem_get_value(&priv->semtx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semtx);
    }
}

/****************************************************************************
 * Name: vhost_serial_rx_worker
 ****************************************************************************/

static void vhost_serial_rx_worker(FAR struct vhost_serial_priv_s *priv)
{
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VIRTIO_SERIAL_RX].vq;
  struct virtqueue_buf vb[2];
  bool added = false;
  uint32_t len;
  ssize_t idx;
  ssize_t ret;
  size_t vbcnt;
  size_t i;

  for (; ; )
    {
      idx = vhost_get_vq_buffers(vq, vb, 2, &vbcnt);
      if (idx < 0)
        {
          break;
        }

      for (i = 0, len = 0; i < vbcnt; i++)
        {
          ret = file_read(&priv->serial, vb[i].buf, vb[i].len);
          if (ret < 0)
            {
              vhosterr("File read data failed %zd\n", ret);
              break;
            }

          len += ret;
          if (ret < vb[i].len)
            {
              break;
            }
        }

      virtqueue_add_consumed_buffer(vq, idx, len);
      added = true;
    }

  if (added)
    {
      virtqueue_kick(vq);
    }
}

/****************************************************************************
 * Name: vhost_serial_rx_thread
 ****************************************************************************/

static int vhost_serial_rx_thread(int argc, FAR char *argv[])
{
  FAR struct vhost_serial_priv_s *priv = (FAR struct vhost_serial_priv_s *)
    ((uintptr_t)strtoul(argv[1], NULL, 16));

  for (; ; )
    {
      nxsem_wait_uninterruptible(&priv->semrx);
      vhost_serial_rx_worker(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: vhost_serial_rx_handler
 ****************************************************************************/

static void vhost_serial_rx_handler(FAR struct virtqueue *vq)
{
  FAR struct vhost_serial_priv_s *priv = vq->vq_dev->priv;
  int semcount;

  nxsem_get_value(&priv->semrx, &semcount);
  while (semcount++ < 1)
    {
      nxsem_post(&priv->semrx);
    }
}

/****************************************************************************
 * Name: vhost_serial_probe
 ****************************************************************************/

static int vhost_serial_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_serial_priv_s *priv;
  FAR const char *vqnames[VIRTIO_SERIAL_NUM];
  vq_callback callbacks[VIRTIO_SERIAL_NUM];
  char path[VIRTIO_SERIAL_PATH_SIZE];
  FAR char *name;
  FAR char *argv[2];
  char arg0[32];
  int ret;

  /* Alloc the virtio serial driver and uart buffer */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->hdev = hdev;
  hdev->priv = priv;
  nxsem_init(&priv->semrx, 0, 0);
  nxsem_init(&priv->semtx, 0, 0);

  /* Initialize actual serial port path */

  vhost_read_config_member(priv->hdev, struct vhost_serial_config_s,
                           path, &path);
  ret = file_open(&priv->serial, path, O_RDWR | O_NOCTTY | O_SYNC);
  if (ret < 0)
    {
      vhosterr("Open %s failed, ret=%d\n", path, ret);
      goto err_file_open;
    }

  /* Initialize the vhost device */

  vqnames[VIRTIO_SERIAL_RX]   = "vhost_serial_rx";
  vqnames[VIRTIO_SERIAL_TX]   = "vhost_serial_tx";
  callbacks[VIRTIO_SERIAL_RX] = vhost_serial_rx_handler;
  callbacks[VIRTIO_SERIAL_TX] = vhost_serial_tx_handler;
  ret = vhost_create_virtqueues(hdev, 0, VIRTIO_SERIAL_NUM, vqnames,
                                callbacks, NULL);
  if (ret < 0)
    {
      vhosterr("vhost_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_init_vq;
    }

  snprintf(arg0, sizeof(arg0), "%p", priv);
  argv[0] = arg0;
  argv[1] = NULL;

  name = lib_get_tempbuffer(PATH_MAX);
  if (name == NULL)
    {
      ret = -ENOMEM;
      goto err_init_name;
    }

  snprintf(name, PATH_MAX, "vhost_serial_rx_%s", path);
  priv->pidrx = kthread_create(name,
                               CONFIG_DRIVERS_VHOST_SERIAL_PRIORITY,
                               CONFIG_DRIVERS_VHOST_SERIAL_STACKSIZE,
                               vhost_serial_rx_thread, argv);
  if (priv->pidrx < 0)
    {
      vhosterr("vhost_serial_rx thread create failed %d\n", priv->pidrx);
      goto err_rx_thread;
    }

  snprintf(name, PATH_MAX, "vhost_serial_tx_%s", path);
  priv->pidtx = kthread_create(name,
                               CONFIG_DRIVERS_VHOST_SERIAL_PRIORITY,
                               CONFIG_DRIVERS_VHOST_SERIAL_STACKSIZE,
                               vhost_serial_tx_thread, argv);
  if (priv->pidtx < 0)
    {
      vhosterr("vhost_serial_tx thread create failed %d\n", priv->pidtx);
      goto err_tx_thread;
    }

  lib_put_tempbuffer(name);
  return ret;

err_tx_thread:
  kthread_delete(priv->pidrx);
err_rx_thread:
  lib_put_tempbuffer(name);
err_init_name:
  vhost_delete_virtqueues(hdev);
err_init_vq:
  file_close(&priv->serial);
err_file_open:
  nxsem_destroy(&priv->semrx);
  nxsem_destroy(&priv->semtx);
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_serial_remove
 ****************************************************************************/

static void vhost_serial_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_serial_priv_s *priv = hdev->priv;

  kthread_delete(priv->pidtx);
  kthread_delete(priv->pidrx);
  vhost_delete_virtqueues(hdev);
  file_close(&priv->serial);
  nxsem_destroy(&priv->semrx);
  nxsem_destroy(&priv->semtx);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_serial_driver
 ****************************************************************************/

int vhost_register_serial_driver(void)
{
  int ret1 = vhost_register_driver(&g_vhost_serial_driver);
  int ret2 = vhost_register_driver(&g_vhost_rprocserial_driver);

  return ret1 < 0 ? ret1 : ret2;
}
