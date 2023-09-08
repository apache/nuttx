/****************************************************************************
 * drivers/virtio/virtio-rng.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-rng.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_rng_cookie_s
{
  sem_t  sem;
  size_t len;
};

struct virtio_rng_priv_s
{
  FAR struct virtio_device *vdev;
  char                      name[NAME_MAX];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RNG file operation function */

static ssize_t virtio_rng_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);

/* Virtio driver functions */

static int virtio_rng_probe(FAR struct virtio_device *vdev);
static void virtio_rng_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_rng_driver =
{
  LIST_INITIAL_VALUE(g_virtio_rng_driver.node), /* node */
  VIRTIO_ID_ENTROPY,                            /* device id */
  virtio_rng_probe,                             /* probe */
  virtio_rng_remove,                            /* remove */
};

static const struct file_operations g_virtio_rng_ops =
{
  NULL,            /* open */
  NULL,            /* close */
  virtio_rng_read, /* read */
  NULL,            /* write */
  NULL,            /* seek */
  NULL,            /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  NULL,            /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,            /* unlink */
#endif
};

static int g_virtio_rng_idx = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_rng_done
 ****************************************************************************/

static void virtio_rng_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_rng_cookie_s *cookie;
  uint32_t len;

  /* Get the buffer, virtqueue_get_buffer() return the cookie added in
   * virtio_rng_read().
   */

  cookie = virtqueue_get_buffer(vq, &len, NULL);
  if (cookie != NULL)
    {
      /* Assign the return length */

      cookie->len = len;

      /* Read completed, post the sem */

      nxsem_post(&cookie->sem);
    }
}

/****************************************************************************
 * Name: virtio_rng_read
 ****************************************************************************/

static ssize_t virtio_rng_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct virtio_rng_priv_s *priv = filep->f_inode->i_private;
  FAR struct virtqueue *vq = priv->vdev->vrings_info[0].vq;
  struct virtio_rng_cookie_s cookie;
  struct virtqueue_buf vb;
  int ret;

  /* Init the cookie */

  cookie.len = 0;
  nxsem_init(&cookie.sem, 0, 0);

  /* Add the input buffer to the virtqueue, and the cookie as the virtqueue
   * cookie. (virtqueue_get_buffer() will return cookie).
   */

  vb.buf = buffer;
  vb.len = buflen;
  ret = virtqueue_add_buffer(vq, &vb, 0, 1, &cookie);
  if (ret < 0)
    {
      return ret;
    }

  /* Notify the other side to process the added virtqueue buffer */

  virtqueue_kick(vq);

  /* Wait fot completion */

  nxsem_wait_uninterruptible(&cookie.sem);
  return cookie.len;
}

/****************************************************************************
 * Name: virtio_rng_probe
 ****************************************************************************/

static int virtio_rng_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_rng_priv_s *priv;
  FAR const char *vqnames[1];
  vq_callback callback[1];
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->vdev = vdev;
  vdev->priv = priv;

  /* Call openamp api to intialize the virtio deivce */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqnames[0]  = "virtio_rng_rx";
  callback[0] = virtio_rng_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqnames, callback);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);

  /* Register NuttX driver */

  if (g_virtio_rng_idx == 0)
    {
      strlcpy(priv->name, "/dev/random", NAME_MAX);
    }
  else
    {
      snprintf(priv->name, NAME_MAX, "/dev/random%d", g_virtio_rng_idx);
    }

  ret = register_driver(priv->name, &g_virtio_rng_ops, 0444, priv);
  if (ret < 0)
    {
      vrterr("Register NuttX driver failed, ret=%d\n", ret);
      goto err_with_virtqueue;
    }

  g_virtio_rng_idx++;
  return ret;

err_with_virtqueue:
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_rng_remove
 ****************************************************************************/

static void virtio_rng_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_rng_priv_s *priv = vdev->priv;

  unregister_driver(priv->name);
  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devrandom_register
 *
 * Description:
 *   Initialize the RNG hardware and register the /dev/random driver.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_RANDOM
void weak_function devrandom_register(void)
{
  /* Nothing, implement it here just avoid the compile error, the driver
   * /dev/random will be registered in the virtio rng driver.
   */
}
#endif

/****************************************************************************
 * Name: virtio_register_rng_driver
 ****************************************************************************/

int virtio_register_rng_driver(void)
{
  return virtio_register_driver(&g_virtio_rng_driver);
}
