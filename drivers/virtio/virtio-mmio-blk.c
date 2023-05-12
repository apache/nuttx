/****************************************************************************
 * drivers/virtio/virtio-mmio-blk.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/virtio/virtio-mmio.h>

#include "virtio-mmio-blk.h"

#ifdef CONFIG_DRIVERS_VIRTIO_BLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_BLK_REQ_HEADER_SIZE 16
#define VIRTIO_BLK_REQ_FOOTER_SIZE 1

#define VIRTIO_BLK_IN    0  /* read */
#define VIRTIO_BLK_OUT   1  /* write */

#define VIRTIO_BLK_Q     0

#define VIRTIO_BLK_SECTOR_SIZE 512

/* VIRTIO_BLK_NINTERFACES determines the number of
 * physical interfaces that will be supported.
 */

#ifndef VIRTIO_BLK_NINTERFACES
# define VIRTIO_BLK_NINTERFACES 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Block driver methods *****************************************************/

static int     virtblk_open(FAR struct inode *inode);
static int     virtblk_close(FAR struct inode *inode);
static ssize_t virtblk_read(FAR struct inode *inode,
                            FAR unsigned char *buffer,
                            blkcnt_t startsector, unsigned int nsectors);
static ssize_t virtblk_write(FAR struct inode *inode,
                             FAR const unsigned char *buffer,
                             blkcnt_t startsector,
                             unsigned int nsectors);
static int     virtblk_geometry(FAR struct inode *inode,
                                FAR struct geometry *geometry);
static int     virtblk_ioctl(FAR struct inode *inode, int cmd,
                             unsigned long arg);

static int     virtblk_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_blk_req
{
  uint32_t type;
  uint32_t reserved;
  uint64_t sector;
  uint8_t  status;
  uint8_t  pad[3];
  uint32_t descriptor;
};

struct virtblk_driver_s
{
  int  irq;

  FAR struct virtio_mmio_regs *regs; /* virtio_mmio registers */
  FAR struct virtqueue   *txq;       /* TX queue */

  uint64_t nsectors;     /* number of sectors on device */
  sem_t    req_sem;      /* semaphore for virtio request */
};

struct virtio_blk_config
{
  uint64_t capacity;
  uint32_t size_max;
  uint32_t seg_max;
  uint32_t chs;
  uint32_t blk_size;
  uint64_t topology;
  uint8_t writeback;
};

/* Driver state structure */

static struct virtblk_driver_s g_virtblk[VIRTIO_BLK_NINTERFACES];

static uint32_t g_virtblk_ninterfaces = 0;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_virtblk_bops =
{
  virtblk_open,     /* open     */
  virtblk_close,    /* close    */
  virtblk_read,     /* read     */
  virtblk_write,    /* write    */
  virtblk_geometry, /* geometry */
  virtblk_ioctl     /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtblk_rw_common
 *
 * Description:
 *   Common function for read and write
 *
 ****************************************************************************/

static ssize_t virtblk_rw_common(FAR struct virtblk_driver_s *priv,
                                 FAR void *buffer,
                                 blkcnt_t startsector,
                                 unsigned int nsectors,
                                 bool write)
{
  struct virtio_blk_req req[2];
  uint16_t idx;
  uint32_t d1;
  uint32_t d2;
  uint32_t d3;

  memset(req, 0, sizeof(req));
  idx = priv->txq->avail->idx;

  if (write)
    {
      req[0].type = VIRTIO_BLK_OUT;
    }
  else
    {
      req[0].type = VIRTIO_BLK_IN;
    }

  req[0].sector = startsector;

  /* Allocate descriptor for header */

  d1 = virtq_alloc_desc(priv->txq, idx,
                        (FAR void *)&req[0]);
  d2 = virtq_alloc_desc(priv->txq, idx + 1,
                        (FAR void *)buffer);
  DEBUGASSERT(d1 != d2);

  req[0].descriptor = d1;

  d3 = virtq_alloc_desc(priv->txq, idx + 2,
                        (FAR void *)&req[1]);
  DEBUGASSERT(d2 != d3);

  /* Set up the descriptor for d1 (header) */

  priv->txq->desc[d1].len   = VIRTIO_BLK_REQ_HEADER_SIZE;
  priv->txq->desc[d1].flags = VIRTQ_DESC_F_NEXT;
  priv->txq->desc[d1].next  = d2;

  /* Set up the descriptor for d2 (sector buffer) */

  priv->txq->desc[d2].len   = VIRTIO_BLK_SECTOR_SIZE * nsectors;
  priv->txq->desc[d2].flags = VIRTQ_DESC_F_NEXT;

  if (!write)
    {
      /* Make the sector buffer writable for read operation */

      priv->txq->desc[d2].flags |= VIRTQ_DESC_F_WRITE;
    }

  priv->txq->desc[d2].next  = d3;

  /* Set up the descriptor for d3 (status) */

  priv->txq->desc[d3].len   = 1;
  priv->txq->desc[d3].flags = VIRTQ_DESC_F_WRITE;
  priv->txq->desc[d3].next  = 0;

  /* Set the first descriptor to the avail->ring */

  priv->txq->avail->ring[d1] = d1;

  /* Increment the avail->idx for each three descriptors */

  virtio_mb();
  priv->txq->avail->idx += 1;

  /* Send request */

  virtio_putreg32(VIRTIO_BLK_Q, &priv->regs->queue_notify);

  finfo("*** finish updating queue_notify\n");

  /* Wait for the request completion */

  nxsem_wait_uninterruptible(&priv->req_sem);

  /* On success, return the number of blocks read */

  return nsectors;
}

/****************************************************************************
 * Name: virtblk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int virtblk_open(FAR struct inode *inode)
{
  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  return OK;
}

/****************************************************************************
 * Name: virtblk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int virtblk_close(FAR struct inode *inode)
{
  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  return OK;
}

/****************************************************************************
 * Name: virtblk_read
 *
 * Description:
 *   Read the specified number of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t virtblk_read(FAR struct inode *inode,
                            FAR unsigned char *buffer,
                            blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtblk_driver_s *priv;

  finfo("Entry: nsectors=%d \n", nsectors);
  DEBUGASSERT(inode && inode->i_private);

  priv = (FAR struct virtblk_driver_s *)inode->i_private;
  return virtblk_rw_common(priv,
                           buffer,
                           startsector, nsectors, false);
}

/****************************************************************************
 * Name: virtblk_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

static ssize_t virtblk_write(FAR struct inode *inode,
                             FAR const unsigned char *buffer,
                             blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtblk_driver_s *priv;

  finfo("Entry: nsectors=%d \n", nsectors);
  DEBUGASSERT(inode && inode->i_private);

  priv = (FAR struct virtblk_driver_s *)inode->i_private;

  return virtblk_rw_common(priv,
                           (FAR void *)buffer,
                           startsector, nsectors, true);
}

/****************************************************************************
 * Name: virtblk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int virtblk_geometry(FAR struct inode *inode,
                            FAR struct geometry *geometry)
{
  FAR struct virtblk_driver_s *priv;
  int ret = -EINVAL;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  priv = (FAR struct virtblk_driver_s *)inode->i_private;

  if (geometry)
    {
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = priv->nsectors;
      geometry->geo_sectorsize    = VIRTIO_BLK_SECTOR_SIZE;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: virtblk_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int virtblk_ioctl(FAR struct inode *inode,
                         int cmd, unsigned long arg)
{
  int ret = -ENOTTY;

  finfo("Entry\n");
  DEBUGASSERT(inode && inode->i_private);

  return ret;
}

/****************************************************************************
 * Name: virtblk_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static int virtblk_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct virtblk_driver_s *priv = (FAR struct virtblk_driver_s *)arg;
  uint32_t stat;

  DEBUGASSERT(priv != NULL);

  /* Get and clear interrupt status bits */

  stat = virtio_getreg32(&priv->regs->interrupt_status);
  virtio_putreg32(stat, &priv->regs->interrupt_ack);
  finfo("+++ called (stat=0x%" PRIx32 ")\n", stat);

  nxsem_post(&priv->req_sem);
  return OK;
}

/****************************************************************************
 * Name: virtblk_initialize
 *
 * Description:
 *   Initialize the virt-blk driver
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

static int virtblk_initialize(FAR struct virtio_mmio_regs *regs, int irq)
{
  FAR struct virtio_blk_config *blk_config;
  FAR struct virtblk_driver_s  *priv;
  char devname[16];
  int ret = -ENOMEM;

  priv = &g_virtblk[g_virtblk_ninterfaces];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct virtblk_driver_s));

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  priv->irq = irq;

  if (irq_attach(priv->irq, virtblk_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Setup virtio related */

  priv->regs = regs;
  priv->txq  = virtq_create(CONFIG_DRIVERS_VIRTIO_BLK_QUEUE_LEN);

  virtq_add_to_mmio_device(regs, priv->txq, VIRTIO_BLK_Q);

  nxsem_init(&priv->req_sem, 0, 0);

  /* Create a ramdisk device name */

  snprintf(devname, 16, "/dev/virtblk%" PRId32, g_virtblk_ninterfaces);

  blk_config = (struct virtio_blk_config *)regs->config;

  finfo("capacity=%" PRId64 " (sectors) \n", blk_config->capacity);

  /* Save the capacity to nsectors */

  priv->nsectors = blk_config->capacity;

  /* Register the device with the OS */

  ret = register_blockdriver(devname, &g_virtblk_bops, 0, priv);
  up_enable_irq(priv->irq);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_mmio_blk_init
 *
 * Description:
 *   Called from virtio-mmio.c to initialize virtblk
 *
 ****************************************************************************/

int virtio_mmio_blk_init(FAR struct virtio_mmio_regs *regs, uint32_t irq)
{
  int ret = OK;

  /* TODO: feature negotiation */

  /* Set STATUS_FEATURE_OK */

  virtio_putreg32(virtio_getreg32(&regs->status) | VIRTIO_STATUS_FEATURES_OK,
                  &regs->status);
  virtio_mb();

  ret = virtblk_initialize(regs, irq);

  if (OK != ret)
    {
      vrterr("error: virtblk_initialize() returned %d \n", ret);
      return ret;
    }

  /* Set STATUS_FRIVER_OK */

  virtio_putreg32(virtio_getreg32(&regs->status) | VIRTIO_STATUS_DRIVER_OK,
                  &regs->status);
  virtio_mb();

  g_virtblk_ninterfaces++;

  return ret;
}

#endif /* CONFIG_DRIVERS_VIRTIO_BLK */
