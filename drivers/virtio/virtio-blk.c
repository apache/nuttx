/****************************************************************************
 * drivers/virtio/virtio-blk.c
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
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/virtio/virtio.h>

#include "virtio-blk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_BLK_REQ_HEADER_SIZE  sizeof(struct virtio_blk_req_s)
#define VIRTIO_BLK_RESP_HEADER_SIZE sizeof(struct virtio_blk_resp_s)

/* Block request type */

#define VIRTIO_BLK_T_IN             0  /* READ */
#define VIRTIO_BLK_T_OUT            1  /* WRITE */
#define VIRTIO_BLK_T_FLUSH          4  /* FLUSH */

/* Block request return status */

#define VIRTIO_BLK_S_OK             0
#define VIRTIO_BLK_S_IOERR          1
#define VIRTIO_BLK_S_UNSUPP         2

/* Block device sector size */

#define VIRTIO_BLK_SECTOR_SIZE      512

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Block request out header */

begin_packed_struct struct virtio_blk_req_s
{
  uint32_t type;
  uint32_t reserved;
  uint64_t sector;
} end_packed_struct;

/* Block request in header */

begin_packed_struct struct virtio_blk_resp_s
{
  uint8_t status;
} end_packed_struct;

begin_packed_struct struct virtio_blk_config_s
{
  uint64_t capacity;
  uint32_t size_max;
  uint32_t seg_max;
  uint16_t cylinders; /* block geometry */
  uint8_t  heads;     /* block geometry */
  uint8_t  sectors;   /* block geometry */
  uint32_t blk_size;
  uint8_t  physical_block_exp;
  uint8_t  alignment_offset;
  uint16_t min_io_size;
  uint32_t opt_io_size;
  uint8_t  writeback;
  uint8_t  unused0;
  uint16_t num_queues;
  uint32_t max_discard_sectors;
  uint32_t max_discard_seg;
  uint32_t discard_sector_alignment;
  uint32_t max_write_zeroes_sectors;
  uint32_t max_write_zeroes_seg;
  uint8_t  write_zeroes_may_unmap;
  uint8_t  unused1[3];
  uint32_t max_secure_erase_sectors;
  uint32_t max_secure_erase_seg;
  uint32_t secure_erase_sector_alignment;
} end_packed_struct;

struct virtio_blk_priv_s
{
  FAR struct virtio_device     *vdev;           /* Virtio deivce */
  FAR struct virtio_blk_req_s  *req;            /* Virtio block out header */
  FAR struct virtio_blk_resp_s *resp;           /* Virtio block in header */
  mutex_t                       lock;           /* Lock */
  uint64_t                      nsectors;       /* Sectore numbers */
  char                          name[NAME_MAX]; /* Device name */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLK block_operations functions and they helper function */

static ssize_t virtio_blk_rdwr(FAR struct virtio_blk_priv_s *priv,
                               FAR void *buffer, blkcnt_t startsector,
                               unsigned int nsectors, bool write);
static int     virtio_blk_open(FAR struct inode *inode);
static int     virtio_blk_close(FAR struct inode *inode);
static ssize_t virtio_blk_read(FAR struct inode *inode,
                               FAR unsigned char *buffer,
                               blkcnt_t startsector, unsigned int nsectors);
static ssize_t virtio_blk_write(FAR struct inode *inode,
                                FAR const unsigned char *buffer,
                                blkcnt_t startsector, unsigned int nsectors);
static int     virtio_blk_geometry(FAR struct inode *inode,
                                   FAR struct geometry *geometry);
static int     virtio_blk_ioctl(FAR struct inode *inode, int cmd,
                                unsigned long arg);
static int     virtio_blk_flush(FAR struct virtio_blk_priv_s *priv);

/* Other functions */

static int  virtio_blk_init(FAR struct virtio_blk_priv_s *priv,
                            FAR struct virtio_device *vdev);
static void virtio_blk_uninit(FAR struct virtio_blk_priv_s *priv);
static void virtio_blk_done(FAR struct virtqueue *vq);
static int  virtio_blk_probe(FAR struct virtio_device *vdev);
static void virtio_blk_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_blk_driver =
{
  LIST_INITIAL_VALUE(g_virtio_blk_driver.node), /* node */
  VIRTIO_ID_BLOCK,                              /* device id */
  virtio_blk_probe,                             /* probe */
  virtio_blk_remove,                            /* remove */
};

static const struct block_operations g_virtio_blk_bops =
{
  virtio_blk_open,     /* open     */
  virtio_blk_close,    /* close    */
  virtio_blk_read,     /* read     */
  virtio_blk_write,    /* write    */
  virtio_blk_geometry, /* geometry */
  virtio_blk_ioctl     /* ioctl    */
};

static int g_virtio_blk_idx = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_blk_rdwr
 *
 * Description:
 *   Common function for read and write
 *
 ****************************************************************************/

static ssize_t virtio_blk_rdwr(FAR struct virtio_blk_priv_s *priv,
                               FAR void *buffer, blkcnt_t startsector,
                               unsigned int nsectors, bool write)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct virtqueue_buf vb[3];
  sem_t respsem;
  ssize_t ret;
  int readnum;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_init(&respsem, 0, 0);

  /* Build the block request */

  priv->req->type     = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
  priv->req->reserved = 0;
  priv->req->sector   = startsector;
  priv->resp->status  = VIRTIO_BLK_S_IOERR;

  /* Fill the virtqueue buffer:
   * Buffer 0: the block out header;
   * Buffer 1: the read/write buffer;
   * Buffer 2: the block in header, return the status.
   */

  vb[0].buf = priv->req;
  vb[0].len = VIRTIO_BLK_REQ_HEADER_SIZE;
  vb[1].buf = buffer;
  vb[1].len = nsectors * VIRTIO_BLK_SECTOR_SIZE;
  vb[2].buf = priv->resp;
  vb[2].len = VIRTIO_BLK_RESP_HEADER_SIZE;
  readnum = write ? 2 : 1;
  ret = virtqueue_add_buffer(vq, vb, readnum, 3 - readnum, &respsem);
  if (ret < 0)
    {
      vrterr("virtqueue_add_buffer failed, ret=%zd\n", ret);
      goto err;
    }

  virtqueue_kick(vq);

  /* Wait for the request completion */

  nxsem_wait_uninterruptible(&respsem);
  if (priv->resp->status != VIRTIO_BLK_S_OK)
    {
      vrterr("%s Error\n", write ? "Write" : "Read");
      ret = -EIO;
    }

err:
  nxmutex_unlock(&priv->lock);
  return ret >= 0 ? nsectors : ret;
}

/****************************************************************************
 * Name: virtio_blk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int virtio_blk_open(FAR struct inode *inode)
{
  DEBUGASSERT(inode->i_private);
  return OK;
}

/****************************************************************************
 * Name: virtio_blk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int virtio_blk_close(FAR struct inode *inode)
{
  DEBUGASSERT(inode->i_private);
  return OK;
}

/****************************************************************************
 * Name: virtio_blk_read
 *
 * Description:
 *   Read the specified number of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t virtio_blk_read(FAR struct inode *inode,
                               FAR unsigned char *buffer,
                               blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtio_blk_priv_s *priv;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;
  return virtio_blk_rdwr(priv, buffer, startsector, nsectors, false);
}

/****************************************************************************
 * Name: virtio_blk_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

static ssize_t virtio_blk_write(FAR struct inode *inode,
                                FAR const unsigned char *buffer,
                                blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtio_blk_priv_s *priv;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;
  return virtio_blk_rdwr(priv, (FAR void *)buffer, startsector, nsectors,
                         true);
}

/****************************************************************************
 * Name: virtio_blk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int virtio_blk_geometry(FAR struct inode *inode,
                               FAR struct geometry *geometry)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret = -EINVAL;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  if (geometry)
    {
      geometry->geo_available    = true;
      geometry->geo_mediachanged = false;
      geometry->geo_writeenabled = true;
      geometry->geo_nsectors     = priv->nsectors;
      geometry->geo_sectorsize   = VIRTIO_BLK_SECTOR_SIZE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_blk_ioctl
 ****************************************************************************/

static int virtio_blk_flush(FAR struct virtio_blk_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct virtqueue_buf vb[2];
  sem_t respsem;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  nxsem_init(&respsem, 0, 0);

  /* Build the block request */

  priv->req->type     = VIRTIO_BLK_T_FLUSH;
  priv->req->reserved = 0;
  priv->req->sector   = 0;
  priv->resp->status  = VIRTIO_BLK_S_IOERR;

  vb[0].buf = priv->req;
  vb[0].len = VIRTIO_BLK_REQ_HEADER_SIZE;
  vb[1].buf = priv->resp;
  vb[1].len = VIRTIO_BLK_RESP_HEADER_SIZE;
  ret = virtqueue_add_buffer(vq, vb, 1, 1, &respsem);
  if (ret < 0)
    {
      goto err;
    }

  virtqueue_kick(vq);

  /* Wait for the request completion */

  nxsem_wait_uninterruptible(&respsem);
  if (priv->resp->status != VIRTIO_BLK_S_OK)
    {
      vrterr("Flush Error\n");
      ret = -EIO;
    }

err:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_blk_ioctl
 ****************************************************************************/

static int virtio_blk_ioctl(FAR struct inode *inode, int cmd,
                            unsigned long arg)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret = -ENOTTY;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  switch (cmd)
    {
      case BIOC_FLUSH:
        ret = virtio_blk_flush(priv);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_blk_done
 ****************************************************************************/

static void virtio_blk_done(FAR struct virtqueue *vq)
{
  FAR sem_t *respsem;

  respsem = virtqueue_get_buffer(vq, NULL, NULL);
  if (respsem != NULL)
    {
      nxsem_post(respsem);
    }
}

/****************************************************************************
 * Name: virtio_blk_init
 ****************************************************************************/

static int virtio_blk_init(FAR struct virtio_blk_priv_s *priv,
                           FAR struct virtio_device *vdev)
{
  FAR const char *vqname[1];
  vq_callback callback[1];
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;
  nxmutex_init(&priv->lock);

  /* Alloc the request and in header from tansport layer */

  priv->req = virtio_alloc_buf(vdev, sizeof(*priv->req), 16);
  if (priv->req == NULL)
    {
      ret = -ENOMEM;
      goto err_with_lock;
    }

  priv->resp = virtio_alloc_buf(vdev, sizeof(*priv->resp), 16);
  if (priv->resp == NULL)
    {
      ret = -ENOMEM;
      goto err_with_req;
    }

  /* Initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_set_features(vdev, 0);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqname[0]   = "virtio_blk_vq";
  callback[0] = virtio_blk_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqname, callback);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err_with_resp;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);
  return OK;

err_with_resp:
  virtio_free_buf(vdev, priv->resp);
err_with_req:
  virtio_free_buf(vdev, priv->req);
err_with_lock:
  nxmutex_destroy(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: virtio_blk_uninit
 ****************************************************************************/

static void virtio_blk_uninit(FAR struct virtio_blk_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
  virtio_free_buf(vdev, priv->resp);
  virtio_free_buf(vdev, priv->req);
  nxmutex_destroy(&priv->lock);
}

/****************************************************************************
 * Name: virtio_blk_probe
 ****************************************************************************/

static int virtio_blk_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret;

  /* Alloc the virtio block driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the virtio block driver */

  ret = virtio_blk_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("virtio_blk_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Read the block config and save the capacity to nsectors */

  virtio_read_config_member(priv->vdev, struct virtio_blk_config_s, capacity,
                            &priv->nsectors);
  vrtinfo("Virio blk capacity=%" PRIu64 " sectors\n", priv->nsectors);

  /* Register block driver */

  snprintf(priv->name, NAME_MAX, "/dev/virtblk%d", g_virtio_blk_idx);
  ret = register_blockdriver(priv->name, &g_virtio_blk_bops, 0660, priv);
  if (ret < 0)
    {
      vrterr("Register block driver failed, ret=%d\n", ret);
      goto err_with_init;
    }

  g_virtio_blk_idx++;
  return ret;

err_with_init:
  virtio_blk_uninit(priv);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_blk_remove
 ****************************************************************************/

static void virtio_blk_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_blk_priv_s *priv = vdev->priv;

  unregister_driver(priv->name);
  virtio_blk_uninit(priv);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_blk_driver
 ****************************************************************************/

int virtio_register_blk_driver(void)
{
  return virtio_register_driver(&g_virtio_blk_driver);
}
