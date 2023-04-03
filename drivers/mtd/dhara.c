/****************************************************************************
 * drivers/mtd/dhara.c
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

#include <dhara/map.h>
#include <dhara/nand.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dhara_pagecache_s
{
  dq_entry_t   node;
  dhara_page_t page;
  FAR uint8_t *buffer;
};

typedef struct dhara_pagecache_s dhara_pagecache_t;

struct dhara_dev_s
{
  struct dhara_nand     nand;
  struct dhara_map      map;
  mutex_t               lock;

  FAR struct mtd_dev_s *mtd;      /* Contained MTD interface */
  struct mtd_geometry_s geo;      /* Device geometry */
  uint16_t              blkper;   /* R/W blocks per erase block */
  uint16_t              refs;     /* Number of references */
  bool                  unlinked; /* The driver has been unlinked */

  /* Two pagesize buffer first is for working temp buffer
   * second is for journel use
   */

  FAR uint8_t *pagebuf;

  /* Read cache for accelerate read dhara meta data */

  struct dq_queue_s readcache;
  dhara_pagecache_t readpage[CONFIG_DHARA_READ_NCACHES];
};

typedef struct dhara_dev_s dhara_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     dhara_open(FAR struct inode *inode);
static int     dhara_close(FAR struct inode *inode);
static ssize_t dhara_read(FAR struct inode *inode,
                          FAR unsigned char *buffer,
                          blkcnt_t start_sector,
                          unsigned int nsectors);
static ssize_t dhara_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t start_sector,
                           unsigned int nsectors);
static int     dhara_geometry(FAR struct inode *inode,
                              FAR struct geometry *geometry);
static int     dhara_ioctl(FAR struct inode *inode,
                           int cmd,
                           unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     dhara_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_dhara_bops =
{
  dhara_open,     /* open     */
  dhara_close,    /* close    */
  dhara_read,     /* read     */
  dhara_write,    /* write    */
  dhara_geometry, /* geometry */
  dhara_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , dhara_unlink  /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dhara_convert_result(dhara_error_t err)
{
  switch (err)
    {
      case DHARA_E_NONE:
        return 0;

      case DHARA_E_BAD_BLOCK:
      case DHARA_E_ECC:
      case DHARA_E_TOO_BAD:
      case DHARA_E_CORRUPT_MAP:
        return -EBADMSG;

      case DHARA_E_JOURNAL_FULL:
      case DHARA_E_MAP_FULL:
        return -ENOSPC;

      case DHARA_E_NOT_FOUND:
        return -ENOENT;

      default:
        return  -EFAULT;
    }
}

static bool dhara_check_ff(FAR const uint8_t *buf, size_t size)
{
  FAR const uint8_t *p = buf;
  size_t len;

  for (len = 0; len < 16; len++)
    {
      if (!size)
          return true;

      if (*p != 0xff)
          return false;

      p++;
      size--;
    }

  return memcmp(buf, p, size) == 0;
}

static int dhara_init_readcache(FAR dhara_dev_t *dev)
{
  FAR dq_queue_t *q = &dev->readcache;
  int i = 0;

  dq_init(q);

  do
    {
      dev->readpage[i].page = DHARA_PAGE_NONE;
      dev->readpage[i].buffer = kmm_malloc(dev->geo.blocksize);
      dq_addfirst(&dev->readpage[i].node, q);
    }
  while (dev->readpage[i++].buffer && i < CONFIG_DHARA_READ_NCACHES);

  return i == CONFIG_DHARA_READ_NCACHES ? 0 : -ENOMEM;
}

static void dhara_deinit_readcache(FAR dhara_dev_t *dev)
{
  int i;

  for (i = 0; i < CONFIG_DHARA_READ_NCACHES; i++)
    {
      if (dev->readpage[i].buffer)
        {
          kmm_free(dev->readpage[i].buffer);
        }
    }
}

static uint8_t *dhara_find_readcache(FAR dhara_dev_t *dev,
                                     dhara_page_t page)
{
  FAR dq_queue_t *q = &dev->readcache;
  FAR dhara_pagecache_t *cache;
  FAR dq_entry_t *c;

  for (c = dq_peek(q); c; c = dq_next(c))
    {
      cache = (FAR dhara_pagecache_t *)c;
      if (cache->page == page)
        {
          dq_rem(c, q);
          dq_addfirst(c, q);
          return cache->buffer;
        }
    }

  return NULL;
}

static dhara_pagecache_t *dhara_grab_readcache(FAR dhara_dev_t *dev)
{
  FAR dq_queue_t *q = &dev->readcache;
  FAR dhara_pagecache_t *cache;
  FAR dq_entry_t *c;

  c = dq_tail(q);
  dq_rem(c, q);
  cache = (FAR dhara_pagecache_t *)c;
  cache->page = DHARA_PAGE_NONE;
  return cache;
}

static void dhara_insert_readcache(FAR dhara_dev_t *dev,
                                   dhara_pagecache_t *cache)
{
  FAR dq_queue_t *q = &dev->readcache;

  if (cache->page != DHARA_PAGE_NONE)
      dq_addfirst((dq_entry_t *)cache, q);
  else
      dq_addlast((dq_entry_t *)cache, q);
}

static void dhara_discard_readcache(FAR dhara_dev_t *dev,
                                    dhara_page_t page)
{
  FAR dhara_pagecache_t *cache;
  FAR dq_queue_t *q = &dev->readcache;
  FAR dq_entry_t *c;

  for (c = dq_peek(q); c; c = dq_next(c))
    {
      cache = (FAR dhara_pagecache_t *)c;
      if (cache->page == page)
        {
          cache->page = DHARA_PAGE_NONE;
          dq_rem(c, q);
          dq_addlast(c, q);
          break;
        }
    }
}

static void dhara_update_readcache(FAR dhara_dev_t *dev,
                                    dhara_page_t page,
                                    FAR const uint8_t *data)
{
  FAR dhara_pagecache_t *cache;
  FAR dq_queue_t *q = &dev->readcache;
  FAR dq_entry_t *c;

  for (c = dq_peek(q); c; c = dq_next(c))
    {
      cache = (FAR dhara_pagecache_t *)c;
      if (cache->page == page)
        {
          cache->page = page;
          memcpy(cache->buffer, data, dev->geo.blocksize);
          dq_rem(c, q);
          dq_addfirst(c, q);
          break;
        }
    }
}

/****************************************************************************
 * Name: dhara_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int dhara_open(FAR struct inode *inode)
{
  FAR dhara_dev_t *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *) inode->i_private;
  nxmutex_lock(&dev->lock);
  dev->refs++;
  nxmutex_unlock(&dev->lock);

  return 0;
}

/****************************************************************************
 * Name: dhara_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int dhara_close(FAR struct inode *inode)
{
  FAR dhara_dev_t *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *) inode->i_private;
  nxmutex_lock(&dev->lock);
  dev->refs--;
  nxmutex_unlock(&dev->lock);

  if (dev->refs == 0 && dev->unlinked)
    {
      nxmutex_destroy(&dev->lock);
      dhara_deinit_readcache(dev);
      kmm_free(dev->pagebuf);
      kmm_free(dev);
    }

  return 0;
}

/****************************************************************************
 * Name: dhara_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t dhara_read(FAR struct inode *inode,
                          FAR unsigned char *buffer,
                          blkcnt_t start_sector,
                          unsigned int nsectors)
{
  FAR dhara_dev_t *dev;
  size_t nread = 0;
  int ret = 0;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *)inode->i_private;

  nxmutex_lock(&dev->lock);
  while (nsectors-- > 0)
    {
      dhara_error_t err;
      ret = dhara_map_read(&dev->map,
                           start_sector,
                           buffer,
                           &err);
      if (ret < 0)
        {
          ret = dhara_convert_result(err);
          ferr("Read startblock %lld failed nread %zd err: %s\n",
               (long long)start_sector, nread, dhara_strerror(err));
          break;
        }

      nread++;
      start_sector++;
      buffer += dev->geo.blocksize;
    }

  nxmutex_unlock(&dev->lock);
  return nread ? nread : ret;
}

/****************************************************************************
 * Name: dhara_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

static ssize_t dhara_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t start_sector,
                           unsigned int nsectors)
{
  FAR dhara_dev_t *dev;
  size_t nwrite = 0;
  int ret = 0;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *)inode->i_private;

  nxmutex_lock(&dev->lock);
  while (nsectors-- > 0)
    {
      dhara_error_t err;
      ret = dhara_map_write(&dev->map,
                            start_sector,
                            buffer,
                            &err);
      if (ret < 0)
        {
          ret = dhara_convert_result(err);
          ferr("Write starting at block %lld failed nwrite %zu err %s\n",
               (long long)start_sector, nwrite, dhara_strerror(err));
          break;
        }

      nwrite++;
      start_sector++;
      buffer += dev->geo.blocksize;
    }

  nxmutex_unlock(&dev->lock);
  return nwrite ? nwrite : ret;
}

/****************************************************************************
 * Name: dhara_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int dhara_geometry(FAR struct inode *inode,
                          FAR struct geometry *geometry)
{
  FAR dhara_dev_t *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *)inode->i_private;

  if (geometry)
    {
      geometry->geo_available    = true;
      geometry->geo_mediachanged = false;
      geometry->geo_writeenabled = true;
      geometry->geo_nsectors     = dev->geo.neraseblocks * dev->blkper;
      geometry->geo_sectorsize   = dev->geo.blocksize;

      strcpy(geometry->geo_model, dev->geo.model);
      nxmutex_unlock(&dev->lock);
      return 0;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: dhara_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int dhara_ioctl(FAR struct inode *inode,
                       int cmd,
                       unsigned long arg)
{
  FAR dhara_dev_t *dev;
  int ret;

  DEBUGASSERT(inode && inode->i_private);
  dev = (dhara_dev_t *)inode->i_private;

  /* No other block driver ioctl commands are not recognized by this
   * driver.  Other possible MTD driver ioctl commands are passed through
   * to the MTD driver (unchanged).
   */

  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0 && ret != -ENOTTY)
    {
      ferr("MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: dhara_unlink
 *
 * Description: Unlink the device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int dhara_unlink(FAR struct inode *inode)
{
  FAR dhara_dev_t *dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR dhara_dev_t *)inode->i_private;
  nxmutex_lock(&dev->lock);
  dev->unlinked = true;
  nxmutex_unlock(&dev->lock);

  if (dev->refs == 0)
    {
      nxmutex_destroy(&dev->lock);
      dhara_deinit_readcache(dev);
      kmm_free(dev->pagebuf);
      kmm_free(dev);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* dhara nand interface implement */

int dhara_nand_is_bad(FAR const struct dhara_nand *n,
                      dhara_block_t bno)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  return MTD_ISBAD(dev->mtd, bno);
}

void dhara_nand_mark_bad(FAR const struct dhara_nand *n,
                         dhara_block_t bno)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  MTD_MARKBAD(dev->mtd, bno);
}

int dhara_nand_erase(FAR const struct dhara_nand *n,
                     dhara_block_t bno,
                     FAR dhara_error_t *err)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  dhara_page_t pno = bno << n->log2_ppb;
  int ret;
  int i;

  ret = MTD_ERASE(dev->mtd, bno, 1);
  if (ret < 0)
    {
      dhara_set_error(err, DHARA_E_BAD_BLOCK);
      return ret;
    }

  for (i = 0; i < dev->blkper; i++)
    {
      dhara_discard_readcache(dev, pno + i);
    }

  return 0;
}

int dhara_nand_prog(FAR const struct dhara_nand *n,
                    dhara_page_t p,
                    FAR const uint8_t *data,
                    FAR dhara_error_t *err)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  int ret;

  ret = MTD_BWRITE(dev->mtd, p, 1, data);
  if (ret < 0)
    {
      dhara_set_error(err, DHARA_E_BAD_BLOCK);
      return ret;
    }

  dhara_update_readcache(dev, p, data);
  return 0;
}

int dhara_nand_is_free(FAR const struct dhara_nand *n,
                       dhara_page_t p)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  size_t page_size = 1 << n->log2_page_size;
  FAR uint8_t *buf = dev->pagebuf;
  dhara_error_t err;

  if (dhara_nand_read(n, p, 0, page_size, buf, &err) < 0)
    {
      ferr("Fail to read page for free check err %s\n",
            dhara_strerror(err));
      return 0;
    }

  return dhara_check_ff(buf, page_size);
}

int dhara_nand_read(FAR const struct dhara_nand *n,
                    dhara_page_t p,
                    size_t offset,
                    size_t length,
                    FAR uint8_t *data,
                    dhara_error_t *err)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  FAR dhara_pagecache_t *cache;
  FAR uint8_t *buf;
  int ret;

  buf = dhara_find_readcache(dev, p);
  if (buf)
    {
      memcpy(data, buf + offset, length);
      return 0;
    }

  cache = dhara_grab_readcache(dev);
  ret = MTD_BREAD(dev->mtd, p, 1, cache->buffer);
  if (ret == -EUCLEAN)
    {
      ret = 0; /* Ignore the correctable ECC error */
    }

  if (ret < 0)
    {
      dhara_insert_readcache(dev, cache);
      dhara_set_error(err, DHARA_E_ECC);
      return ret;
    }

  memcpy(data, cache->buffer + offset, length);
  cache->page = p;
  dhara_insert_readcache(dev, cache);
  return ret;
}

int dhara_nand_copy(FAR const struct dhara_nand *n,
                    dhara_page_t src,
                    dhara_page_t dst,
                    FAR dhara_error_t *err)
{
  FAR dhara_dev_t *dev = (FAR dhara_dev_t *)n;
  size_t page_size = 1 << n->log2_page_size;
  FAR uint8_t *buf = dev->pagebuf;
  int ret;

  ret = dhara_nand_read(n, src, 0, page_size, buf, err);
  if (ret < 0)
    {
      ferr("src_page %d, ret %d failed: %s\n",
            src, ret, dhara_strerror(*err));
      return ret;
    }

  ret = dhara_nand_prog(n, dst, buf, err);
  if (ret < 0)
    {
      ferr("dst_page %d, ret %d failed: %s\n",
            dst, ret, dhara_strerror(*err));
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: dhara_initialize_by_path
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   path - The block device path.
 *   mtd  - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int dhara_initialize_by_path(FAR const char *path,
                             FAR struct mtd_dev_s *mtd)
{
  FAR dhara_dev_t *dev;
  int ret;

  /* Sanity check */

  if (path == NULL || mtd == NULL)
    {
      return -EINVAL;
    }

  /* Allocate a DHARA_MTDBLOCK device structure */

  dev = (FAR dhara_dev_t *)
                 kmm_zalloc(sizeof(dhara_dev_t));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&dev->lock);

  /* Initialize the DHARA_MTDBLOCK device structure */

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

  /* Get the number of R/W blocks per erase block */

  dev->blkper = dev->geo.erasesize / dev->geo.blocksize;
  DEBUGASSERT(dev->blkper * dev->geo.blocksize == dev->geo.erasesize);

  /* Init dhara */

  dev->nand.log2_page_size = fls(dev->geo.blocksize) - 1;
  dev->nand.log2_ppb = fls(dev->blkper) - 1;
  dev->nand.num_blocks = dev->geo.neraseblocks;

  dev->pagebuf = kmm_zalloc(dev->geo.blocksize * 2);
  if (!dev->pagebuf)
    {
      ret = -ENOMEM;
      goto err;
    }

  ret = dhara_init_readcache(dev);
  if (ret != 0)
    {
      goto err;
    }

  dhara_map_init(&dev->map, &dev->nand,
                 dev->pagebuf + dev->geo.blocksize,
                 CONFIG_DHARA_GC_RATIO);

  dhara_map_resume(&dev->map, NULL);

  /* Inode private data is a reference to the
   * DHARA_MTDBLOCK device structure
   */

  ret = register_blockdriver(path, &g_dhara_bops, 0666, dev);
  if (ret < 0)
    {
      ferr("register_blockdriver failed: %d\n", ret);
      goto err;
    }

  return ret;

err:
  nxmutex_destroy(&dev->lock);
  dhara_deinit_readcache(dev);
  kmm_free(dev->pagebuf);
  kmm_free(dev);
  return ret;
}

/****************************************************************************
 * Name: dhara_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *           registered as as /dev/mtdblockN where N is the minor number.
 *   mtd   - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int dhara_initialize(int minor, FAR struct mtd_dev_s *mtd)
{
  char path[PATH_MAX];

#ifdef CONFIG_DEBUG_FEATURES
  /* Sanity check */

  if (minor < 0 || minor > 255)
    {
      return -EINVAL;
    }
#endif

  /* Do the real work by dhara_mtdblock_initialize_by_path */

  snprintf(path, sizeof(path), "/dev/mtdblock%d", minor);
  return dhara_initialize_by_path(path, mtd);
}
