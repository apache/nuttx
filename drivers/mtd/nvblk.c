/****************************************************************************
 * drivers/mtd/nvblk.c
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

#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/lib/lib.h>

#include <nvblk/nvblk.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NVBLK_MIN_LBS 128 /* Minimal logical block size */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nvblk_dev_s
{
  struct nvb_config     cfg;         /* nvblk configuration */
  struct nvb_info       info;        /* nvblk data */
  mutex_t               lock;

  FAR struct mtd_dev_s *mtd;         /* Contained MTD interface */
  struct mtd_geometry_s geo;         /* Device geometry */
  uint16_t              refs;        /* Number of references */
  uint8_t               log2_bpiob;  /* (logical) blocks per IO block */
  uint8_t               log2_ppb;    /* pages per (logical) block */
  bool                  unlinked;    /* The driver has been unlinked */

  /* Two pagesize buffer first is for working temp buffer
   * second is for journel use
   */

  FAR uint8_t *pagebuf;
};

typedef struct nvblk_dev_s nvblk_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     nvblk_open(FAR struct inode *inode);
static int     nvblk_close(FAR struct inode *inode);
static ssize_t nvblk_read(FAR struct inode *inode,
                          FAR unsigned char *buffer,
                          blkcnt_t start_sector,
                          unsigned int nsectors);
static ssize_t nvblk_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t start_sector,
                           unsigned int nsectors);
static int     nvblk_geometry(FAR struct inode *inode,
                              FAR struct geometry *geometry);
static int     nvblk_ioctl(FAR struct inode *inode,
                           int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     nvblk_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_nvblk_bops =
{
  nvblk_open,     /* open     */
  nvblk_close,    /* close    */
  nvblk_read,     /* read     */
  nvblk_write,    /* write    */
  nvblk_geometry, /* geometry */
  nvblk_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , nvblk_unlink  /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int nvblk_convert_result(int err)
{
  switch (err)
    {
      case 0:
        return 0;
      case -NVB_ENOENT:
        return -ENOENT;
      case -NVB_EINVAL:
        return -EINVAL;
      case -NVB_EROFS:
        return -EROFS;
      case -NVB_EAGAIN:
        return -EAGAIN;
      case -NVB_ENOSPC:
        return -ENOSPC;
      default:
        return -EFAULT;
    }
}

/****************************************************************************
 * Name: nvblk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int nvblk_open(FAR struct inode *inode)
{
  FAR nvblk_dev_t *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
  nxmutex_lock(&dev->lock);
  dev->refs++;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: nvblk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int nvblk_close(FAR struct inode *inode)
{
  FAR nvblk_dev_t *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
  nxmutex_lock(&dev->lock);
  (void)nvb_sync(&dev->info);
  dev->refs--;
  nxmutex_unlock(&dev->lock);

  if (dev->refs == 0 && dev->unlinked)
    {
      nxmutex_destroy(&dev->lock);
      kmm_free(dev->pagebuf);
      kmm_free(dev);
    }

  return 0;
}

/****************************************************************************
 * Name: nvblk_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t nvblk_read(FAR struct inode *inode,
                          FAR unsigned char *buffer,
                          blkcnt_t start_sector,
                          unsigned int nsectors)
{
  FAR nvblk_dev_t *dev;
  uint16_t bstart;
  uint16_t bcnt;
  uint16_t n;
  int ret = 0;

  finfo("Read %lld %zd\n", (long long)start_sector, nsectors);

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
  nxmutex_lock(&dev->lock);
  bstart = start_sector << dev->log2_bpiob;
  bcnt = nsectors << dev->log2_bpiob;

  for (n = 0; n < bcnt; n++)
    {
      ret = nvb_read(&dev->info, buffer, bstart, 1);
      if (ret == -NVB_ENOENT)
        {
          memset(buffer, 0xff, (1 << dev->cfg.log2_bs));
          ret = 0;
        }

      if (ret < 0)
        {
          break;
        }

      buffer += (1 << dev->cfg.log2_bs);
      bstart++;
    }

  if (ret < 0)
    {
      ret = nvblk_convert_result(ret);
      ferr("Read startblock %lld failed nsectors %zd err: %d\n",
           (long long)start_sector, nsectors, ret);
    }

  nxmutex_unlock(&dev->lock);
  return ret < 0 ? ret : nsectors;
}

/****************************************************************************
 * Name: nvblk_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t nvblk_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t start_sector,
                           unsigned int nsectors)
{
  FAR nvblk_dev_t *dev;
  uint16_t bstart;
  uint16_t bcnt;
  int ret = 0;

  finfo("Write %lld %zd\n", (long long)start_sector, nsectors);

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
  nxmutex_lock(&dev->lock);
  bstart = start_sector << dev->log2_bpiob;
  bcnt = nsectors << dev->log2_bpiob;
  ret = nvblk_convert_result(nvb_write(&dev->info, buffer, bstart, bcnt));
  if (ret < 0)
    {
      ferr("Write startblock %lld failed nsectors %zd err: %d\n",
           (long long)start_sector, nsectors, ret);
    }

  nxmutex_unlock(&dev->lock);
  return ret < 0 ? ret : nsectors;
}

/****************************************************************************
 * Name: nvblk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int nvblk_geometry(FAR struct inode *inode,
                          FAR struct geometry *geometry)
{
  FAR nvblk_dev_t *dev;
  uint32_t blkcnt;
  int ret = -EINVAL;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  if (!geometry)
    {
      return ret;
    }

  nxmutex_lock(&dev->lock);
  ret = nvb_ioctl(&dev->info, NVB_CMD_GET_BLK_COUNT, &blkcnt);

  if (ret < 0)
    {
      ret = nvblk_convert_result(ret);
      goto end;
    }

  geometry->geo_available    = true;
  geometry->geo_mediachanged = false;
  geometry->geo_writeenabled = true;
  geometry->geo_nsectors     = blkcnt >> dev->log2_bpiob;
  geometry->geo_sectorsize   = (1 << (dev->log2_bpiob + dev->cfg.log2_bs));
  strlcpy(geometry->geo_model, dev->geo.model, sizeof(geometry->geo_model));

end:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: nvblk_ioctl
 *
 * Description:
 *   Set/Get option to/from block device.
 *
 * No ioctl commands are supported.
 *
 ****************************************************************************/

static int nvblk_ioctl(FAR struct inode *inode,
                       int cmd,
                       unsigned long arg)
{
  FAR nvblk_dev_t *dev;
  int ret;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  nxmutex_lock(&dev->lock);
  switch (cmd)
    {
      case BIOC_FLUSH:
        {
          ret = nvb_sync(&dev->info);
          if (ret < 0)
            {
              ferr("sync failed: %d\n", ret);
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: nvblk_unlink
 *
 * Description: Unlink the device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int nvblk_unlink(FAR struct inode *inode)
{
  FAR nvblk_dev_t *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
  nxmutex_lock(&dev->lock);
  dev->unlinked = true;
  nxmutex_unlock(&dev->lock);

  if (dev->refs == 0)
    {
      nxmutex_destroy(&dev->lock);
      kmm_free(dev->pagebuf);
      kmm_free(dev);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* nvblk cfg interface implementation */

int nvblk_cfg_read(FAR const struct nvb_config *cfg,
                   uint32_t p, void *buf)
{
  FAR nvblk_dev_t *dev = container_of(cfg, FAR nvblk_dev_t, cfg);

  if (MTD_BREAD(dev->mtd, p << dev->log2_ppb, 1 << dev->log2_ppb, buf) < 0)
    {
      return -NVB_EFAULT;
    }

  return 0;
}

int nvblk_cfg_prog(FAR const struct nvb_config *cfg,
                   uint32_t p, const void *buf)
{
  FAR nvblk_dev_t *dev = container_of(cfg, FAR nvblk_dev_t, cfg);

  if ((p % (1 << cfg->log2_bpeb)) == 0)
    {
      if (MTD_ERASE(dev->mtd, p >> cfg->log2_bpeb, 1U) < 0)
        {
          return -NVB_EFAULT;
        }
    }

  if (MTD_BWRITE(dev->mtd, p << dev->log2_ppb, 1 << dev->log2_ppb, buf) < 0)
    {
      return -NVB_EFAULT;
    }

  return 0;
}

/****************************************************************************
 * Name: nvblk_initialize_by_path
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   path - The block device path.
 *   mtd  - The MTD device that supports the FLASH interface.
 *   lbs  - The logical blocksize (size of the nvblk blocks).
 *   iobs - The input output blocksize (multiple of lbs).
 *   speb - The number of spare erase blocks.
 *
 ****************************************************************************/

int nvblk_initialize(FAR const char *path,
                     FAR struct mtd_dev_s *mtd,
                     uint32_t lbs,
                     uint32_t iobs,
                     uint32_t speb)
{
  FAR nvblk_dev_t *dev;
  int ret;

  /* Sanity check */

  if (path == NULL || mtd == NULL || lbs < NVBLK_MIN_LBS || lbs > iobs ||
      iobs == 0U || (iobs & (iobs - 1U)) != 0U ||
      lbs == 0U || (lbs & (lbs - 1U)) != 0U)
    {
      return -EINVAL;
    }

  finfo("path=\"%s\"\n", path);

  /* Allocate a NVBLK_MTDBLOCK device structure */

  dev = (FAR nvblk_dev_t *)kmm_zalloc(sizeof(nvblk_dev_t));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&dev->lock);

  /* Initialize the NVBLK_MTDBLOCK device structure */

  dev->mtd = mtd;

  /* Get the device geometry. (casting to uintptr_t first
   * eliminates complaints on some architectures where the
   * sizeof long is different
   * from the size of a pointer).
   */

  ret = MTD_IOCTL(mtd,
                  MTDIOC_GEOMETRY,
                  (unsigned long)((uintptr_t)&dev->geo));
  if (ret < 0)
    {
      ferr("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
      goto err;
    }

  /* Validate the number of R/W blocks per erase block */

  if ((dev->geo.erasesize % lbs) != 0 || (lbs % dev->geo.blocksize) != 0 ||
      (dev->geo.neraseblocks <= speb))
    {
      ferr("NVBLK bad config\n");
      ret = -EINVAL;
      goto err;
    }

  /* Init nvblk */

  dev->log2_bpiob = fls(iobs / lbs) - 1;
  dev->log2_ppb = fls(lbs / dev->geo.blocksize) - 1;
  dev->cfg.log2_bs = fls(lbs) - 1;
  dev->cfg.log2_bpeb = fls(dev->geo.erasesize / lbs) - 1;
  dev->cfg.eb = dev->geo.neraseblocks;
  dev->cfg.sp_eb = speb;

  dev->pagebuf = kmm_zalloc(lbs * 2);
  if (!dev->pagebuf)
    {
      ret = -ENOMEM;
      goto err;
    }

  dev->cfg.mb = &dev->pagebuf[0];
  dev->cfg.gb = &dev->pagebuf[lbs];
  dev->cfg.read = nvblk_cfg_read;
  dev->cfg.prog = nvblk_cfg_prog;

  finfo("Initializing nvblk\n");
  ret = nvb_init(&dev->info, &dev->cfg);
  if (ret < 0)
    {
      ferr("failed to initialize nvblk: %d\n", ret);
      goto err;
    }

  finfo("succeeded initializing nvblk\n");
  finfo("Physical Size: ebcnt [%d] bcnt [%d]", dev->cfg.eb,
        dev->cfg.eb << dev->cfg.log2_bpeb);
  finfo("Head at mblock [%d], Tail at mblock [%d]\n", dev->info.head,
        dev->info.tail);
  finfo("Root at mblock [%d]\n", dev->info.root);
  finfo("cpe [%d], tail_cpe [%d]\n", dev->info.cpe, dev->info.tail_cpe);
  finfo("used [%d], pass [%x]\n", dev->info.used, dev->info.pass);

  /* Inode private data is a reference to the
   * NVBLK_MTDBLOCK device structure
   */

  ret = register_blockdriver(path, &g_nvblk_bops, 0666, dev);
  if (ret < 0)
    {
      ferr("register_blockdriver failed: %d\n", ret);
      goto err;
    }

  return ret;

err:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev->pagebuf);
  kmm_free(dev);
  return ret;
}
