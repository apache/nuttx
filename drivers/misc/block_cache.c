/****************************************************************************
 * drivers/misc/block_cache.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <sys/mount.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>

#include <nuttx/drivers/block_cache.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_BLOCK_CACHE_DEBUG)
# define bcinfo _info
#else
# define bcinfo _none
#endif

#if defined(CONFIG_FS_LARGEFILE)
#define PRIBLKC PRIu64
#define PRIOFFS PRId64
#else
#define PRIBLKC PRIu32
#define PRIOFFS PRId32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum block_flags
{
  BLOCK_FREE  = 0x1,                  /* The window is marked as free */
  BLOCK_DIRTY = 0x2,                  /* The window contains dirty blocks */
};

struct cache_window_s
{
  blkcnt_t          start;            /* The initial block index */
  enum block_flags  flags;            /* Any flags for this window */
  uint8_t *         buffer;           /* The underlying block buffer */
};

struct block_cache_dev_s
{
  FAR struct inode * inode;               /* Contained block device */
  struct geometry geo;                    /* Contained device geometry */
  size_t geo_multiple;                    /* External block multiple */
  struct cache_window_s * cache;          /* Array of block caches */
  off_t                 last_index;       /* The last index evicted */
  uint16_t              refs;             /* Number of references */
  size_t                cache_width;      /* The width of a cache */
  size_t                cache_count;      /* The number of caches */
  size_t                cache_used_count; /* Number of used caches */
  size_t                aligned_mask;     /* The mask for block alignment */
  mutex_t               lock;             /* Lock */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Block device handles */

static int     block_cache_open(FAR struct inode *inode);
static int     block_cache_close(FAR struct inode *inode);
static ssize_t block_cache_read(FAR struct inode *inode,
                              FAR unsigned char *buffer,
                              blkcnt_t start_sector,
                              unsigned int nsectors);
static ssize_t block_cache_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, blkcnt_t start_block,
                 unsigned int nblocks);
static int     block_cache_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     block_cache_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);

/* Interface to the underlying block device */

static ssize_t block_cache_reload(struct block_cache_dev_s * dev,
                            FAR uint8_t *buffer,
                            off_t startblock, size_t nblocks);
static ssize_t block_cache_sync(struct block_cache_dev_s * dev,
                            FAR const uint8_t *buffer,
                            off_t startblock, size_t nblocks);

/* Internal use  */

int block_cache_write_internal(struct block_cache_dev_s * dev,
                            const unsigned char * buffer,
                            blkcnt_t startblock, unsigned int nblocks);
int block_cache_read_internal(struct block_cache_dev_s * dev,
                            unsigned char * buffer,
                            blkcnt_t startblock, unsigned int nblocks);

int block_cache_get_next(struct block_cache_dev_s * dev,
                                blkcnt_t aligned_start);
int block_cache_prime(struct block_cache_dev_s * dev,
                                blkcnt_t aligned_start);
int block_cache_evict(struct block_cache_dev_s * dev);
int block_cache_get_free(struct block_cache_dev_s * dev);
int block_cache_get_primed(struct block_cache_dev_s * dev,
                                blkcnt_t aligned_start);
int block_cache_flush(struct block_cache_dev_s * dev,
                                size_t cache_index);
void block_cache_deinit(struct block_cache_dev_s * dev);
int block_cache_init(struct block_cache_dev_s * dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  block_cache_open,     /* open     */
  block_cache_close,    /* close    */
  block_cache_read,     /* read     */
  block_cache_write,    /* write    */
  block_cache_geometry, /* geometry */
  block_cache_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL  /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: block_cache_init
 *
 * Description:
 *  Initialize the array of windows used for caching blocks.
 *
 ****************************************************************************/

int block_cache_init(struct block_cache_dev_s * dev)
{
  dev->cache = (struct cache_window_s *)
                kmm_zalloc(dev->cache_count * sizeof(struct cache_window_s));

  if (!dev->cache)
    {
      return -ENOMEM;
    }

  for (int i = 0; i < dev->cache_count; i++)
    {
      dev->cache[i].start = -1;
      dev->cache[i].flags = BLOCK_FREE;
      dev->cache[i].buffer = (uint8_t *)kmm_zalloc(
                            dev->geo.geo_sectorsize * dev->cache_width);

      /* Unwind any existing allocated buffers */

      if (!dev->cache[i].buffer)
        {
          for (int j = (i - 1); j >= 0; j--)
            {
              kmm_free(dev->cache[j].buffer);
            }

          kmm_free(dev->cache);
          return -ENOMEM;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: block_cache_deinit
 *
 * Description:
 *  Deinitialize the array of cache windows used.
 *  Any blocks not flushed are lost.
 *
 ****************************************************************************/

void block_cache_deinit(struct block_cache_dev_s * dev)
{
  for (int i = 0; i < dev->cache_count; i++)
    {
      if (dev->cache[i].buffer)
        {
          kmm_free(dev->cache[i].buffer);
        }
    }

    kmm_free(dev->cache);
}

/****************************************************************************
 * Name: block_cache_flush
 *
 * Description:
 *  Flush the given cache index to the device, if the cache contains 'dirty'
 *  blocks, which don't match that of the underlying device.
 *
 ****************************************************************************/

int block_cache_flush(struct block_cache_dev_s * dev, size_t cache_index)
{
  int ret = OK;

  if (dev->cache[cache_index].flags & BLOCK_DIRTY)
    {
      bcinfo("flush: %" PRIuPTR ", origin %" PRIBLKC "\n", cache_index,
                                        dev->cache[cache_index].start);
      ret = block_cache_sync(
                dev,
                dev->cache[cache_index].buffer,
                dev->cache[cache_index].start,
                dev->cache_width);
      if (ret)
        {
          dev->cache[cache_index].flags &= ~BLOCK_DIRTY;
        }
    }
  else
    {
      bcinfo("skip flush: %" PRIuPTR ", origin %" PRIBLKC "\n", cache_index,
                                            dev->cache[cache_index].start);
    }

  return ret;
}

/****************************************************************************
 * Name: block_cache_get_primed
 *
 * Description:
 *  Search all cache windows for a block which matches the aligned_start.
 *  Return the index is found.
 *
 ****************************************************************************/

int block_cache_get_primed(struct block_cache_dev_s * dev,
                                                     blkcnt_t aligned_start)
{
  int cache_index = -1;
  for (size_t i = 0; i < dev->cache_count; i++)
    {
      if ((dev->cache[i].start == aligned_start))
        {
          cache_index = i;
          bcinfo("found: %" PRIuPTR ", origin: %" PRIBLKC "\n",
                 i, aligned_start);
          break;
        }
    }

  return cache_index;
}

/****************************************************************************
 * Name: block_cache_get_free
 *
 * Description:
 *  Find any cache index which is currently not used.
 *
 ****************************************************************************/

int block_cache_get_free(struct block_cache_dev_s * dev)
{
  int cache_index = -1;
  if (dev->cache_used_count == dev->cache_count)
    {
      return cache_index;
    }

  for (int i = 0; i < dev->cache_count; i++)
    {
      if (dev->cache[i].flags & BLOCK_FREE)
        {
          bcinfo("empty: %" PRIdPTR "\n", i);
          dev->cache_used_count++;
          cache_index = i;
          break;
        }
    }

  return cache_index;
}

/****************************************************************************
 * Name: block_cache_evict
 *
 * Description:
 *  Determine and evict a cache window in preparation for use with new
 *  blocks.
 *
 ****************************************************************************/

int block_cache_evict(struct block_cache_dev_s * dev)
{
  int ret;
  int cache_index;

  cache_index = dev->last_index;

  bcinfo("evict: %" PRIdPTR ", origin: %" PRIBLKC "\n", cache_index,
                                    dev->cache[cache_index].start);
  ret = block_cache_flush(dev, cache_index);

  if (ret >= 0)
    {
      ret = cache_index;
      dev->last_index = (dev->last_index + 1) % dev->cache_count;
    }

  return ret;
}

/****************************************************************************
 * Name: block_cache_prime
 *
 * Description:
 *  Reload a cache window with blocks from the encapsulated device.
 *  This either:
 *  - Finds an empty unused window.
 *  - Evicts an existing window and reloads it with the start block from
 *    the encapsulated device.
 *
 ****************************************************************************/

int block_cache_prime(struct block_cache_dev_s * dev, blkcnt_t aligned_start)
{
  int cache_index = -1;
  int ret;

  cache_index = block_cache_get_free(dev);

  if (cache_index < 0)
    {
      cache_index = block_cache_evict(dev);
    }

  if (cache_index >= 0)
    {
      bcinfo("prime: %" PRIdPTR ", origin %" PRIBLKC,
             cache_index, aligned_start);
      ret = block_cache_reload(dev,
                               dev->cache[cache_index].buffer,
                               aligned_start, dev->cache_width);
      if (ret > 0)
        {
          dev->cache[cache_index].start = aligned_start;
          dev->cache[cache_index].flags &= ~BLOCK_FREE;
        }
    }

  return cache_index;
}

/****************************************************************************
 * Name: block_cache_get_next
 *
 * Description:
 *  Get the next available cache index to used for a block aligned to the
 *  start of a window. This either:
 *  - Finds an already available index which starts with aligned_start.
 *  - Finds an empty unused window.
 *  - Evicts an existing window and reloads it with the start block from
 *    the encapsulated device.
 *
 ****************************************************************************/

int block_cache_get_next(struct block_cache_dev_s * dev,
                                              blkcnt_t aligned_start)
{
  int cache_number;
  cache_number = block_cache_get_primed(dev, aligned_start);
  if (cache_number < 0)
    {
      cache_number = block_cache_prime(dev, aligned_start);
    }

#if defined(CONFIG_DEBUG_ASSERTIONS) && defined(CONFIG_BLOCK_CACHE_DEBUG)
  if (cache_number >= 0)
    {
      /* Check to see if any caches have the same start block number */

      for (int i = cache_number + 1; i < dev->cache_count; i++)
        {
          DEBUGASSERT(dev->cache[cache_number].start != dev->cache[i].start);
        }
    }
#endif

  return cache_number;
}

/****************************************************************************
 * Name: block_cache_read_internal
 *
 * Description:
 *  Internal cached block functionality. Performs the following:
 *    - Write to an existing window if the start block is already cached.
 *    - Write to a new window if the start block is not already cached.
 *    - Repeat as necessary to cache the entire buffer.
 *
 * Returned Value:
 *   The number of blocks written of a negative value on error.
 *
 ****************************************************************************/

int block_cache_read_internal(struct block_cache_dev_s * dev,
                              unsigned char * buffer,
                              blkcnt_t startblock, unsigned int nblocks)
{
  blkcnt_t aligned_start;
  unsigned int remaining;
  unsigned int read;
  off_t read_count;
  off_t blocks_remaining;
  int cache_number;
  blkcnt_t offset;

  bcinfo("startblock: %" PRIBLKC ", nblocks: %" PRIuPTR "\n",
         startblock, nblocks);

  /* Calculate the aligned start block and offset within that block */

  aligned_start = startblock & ~(dev->aligned_mask);
  offset = startblock & dev->aligned_mask;
  remaining = nblocks;
  read = 0;

  /** Iterate until all blocks are read */

  while (remaining > 0)
    {
      /** Get the cache window index for the aligned_start block */

      cache_number = block_cache_get_next(dev, aligned_start);

      /** Check for errors in cache retrieval */

      if (cache_number < 0)
        {
          return cache_number;
        }

      /** Calculate the number of blocks that can be read in this operation */

      blocks_remaining = dev->cache_width - offset;
      read_count = remaining > blocks_remaining ?
                                          blocks_remaining : remaining;

      bcinfo("use %" PRIOFFS "blocks from offset %" PRIBLKC "\n",
             read_count, offset);

      /** Copy data from the cache window to the buffer */

      memcpy(
        &buffer[read * dev->geo.geo_sectorsize],
        &dev->cache[cache_number].buffer[offset * dev->geo.geo_sectorsize],
        read_count * dev->geo.geo_sectorsize
      );

      /** Update remaining block counts and offsets */

      remaining -= read_count;
      read += read_count;
      aligned_start += dev->cache_width;
      offset = 0;
    }

  DEBUGASSERT(read == nblocks);
  return read;
}

/****************************************************************************
 * Name: block_cache_write_internal
 *
 * Description:
 *  Internal cached block functionality. Performs the following:
 *    - Write to an existing window if the start block is already cached.
 *    - Write to a new window if the start block is not already cached.
 *    - Repeat as necessary to cache the entire buffer.
 *
 * Returned Value:
 *   The number of blocks written of a negative value on error.
 *
 ****************************************************************************/

int block_cache_write_internal(struct block_cache_dev_s * dev,
                               const unsigned char * buffer,
                               blkcnt_t startblock, unsigned int nblocks)
{
  blkcnt_t aligned_start;
  unsigned int remaining;
  unsigned int written;
  off_t write_count;
  off_t blocks_remaining;
  int cache_number;
  blkcnt_t offset;

  bcinfo("startblock: %" PRIBLKC ", nblocks: %" PRIuPTR "\n",
         startblock, nblocks);

  /** Calculate the aligned start block and offset within that block */

  aligned_start = startblock & ~(dev->aligned_mask);
  offset = startblock & dev->aligned_mask;
  remaining = nblocks;
  written = 0;

  /** Iterate until all blocks are written */

  while (remaining > 0)
    {
      /** Get the cache window index for the aligned_start block */

      cache_number = block_cache_get_next(dev, aligned_start);

      /** Check for errors in cache retrieval */

      if (cache_number < 0)
        {
          return cache_number;
        }

      /** Number of blocks that can be written in this operation */

      blocks_remaining = dev->cache_width - offset;
      write_count = remaining > blocks_remaining ?
                                      blocks_remaining : remaining;

      bcinfo("use %" PRIOFFS " blocks from offset %" PRIBLKC "\n",
             write_count, offset);

      /** Copy data from the buffer to the cache window */

      memcpy(
        &dev->cache[cache_number].buffer[offset * dev->geo.geo_sectorsize],
        &buffer[written * dev->geo.geo_sectorsize],
        write_count * dev->geo.geo_sectorsize
      );

      /** Mark the cache window as dirty to indicate changes */

      dev->cache[cache_number].flags |= BLOCK_DIRTY;

      /** Update remaining block counts and offsets */

      remaining -= write_count;
      written += write_count;
      aligned_start += dev->cache_width;
      offset = 0;
    }

  DEBUGASSERT(written == nblocks);

  return written;
}

/****************************************************************************
 * Name: block_cache_open
 *
 * Description:
 *  Exposed block open interface. Just increments reference count.
 *
 ****************************************************************************/

static int block_cache_open(FAR struct inode * inode)
{
  int ret;
  FAR struct block_cache_dev_s * dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  dev->refs++;
  nxmutex_unlock(&dev->lock);

  return OK;
}

/****************************************************************************
 * Name: block_cache_close
 *
 * Description:
 *  Exposed block close interface. Decrement reference count and flush the
 *  cache when it reaches zero.
 *
 ****************************************************************************/

static int block_cache_close(FAR struct inode * inode)
{
  int ret;
  FAR struct block_cache_dev_s * dev;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (--dev->refs == 0)
    {
      for (int i = 0; i < dev->cache_count; i++)
        {
          ret = block_cache_flush(dev, i);
          DEBUGASSERT(ret >= 0);

          if (ret < 0)
            {
              break;
            }
        }
    }

  nxmutex_unlock(&dev->lock);

  return OK;
}

/****************************************************************************
 * Name: block_cache_reload
 *
 * Description:
 *  Read the specified number of sectors to the encapsulated
 *  block device.
 *
 ****************************************************************************/

static ssize_t block_cache_reload(struct block_cache_dev_s * dev,
                      FAR uint8_t *buffer, off_t startblock, size_t nblocks)
{
  return dev->inode->u.i_bops->read(dev->inode, buffer, startblock, nblocks);
}

/****************************************************************************
 * Name: block_cache_read
 *
 * Description:
 *  Read the specified number of sectors. Utilizing the read ahead buffer
 *  where possible.
 *
 ****************************************************************************/

static ssize_t block_cache_read(FAR struct inode * inode,
                      unsigned char * buffer, blkcnt_t startblock,
                      unsigned int nblocks)
{
  int ret;
  FAR struct block_cache_dev_s * dev;

  DEBUGASSERT(inode && inode->i_private);

  dev = (FAR struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = block_cache_read_internal(
            dev,
            buffer,
            startblock * dev->geo_multiple,
            nblocks * dev->geo_multiple
        );
  if (ret > 0)
    {
      ret /= dev->geo_multiple;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: block_cache_sync
 *
 * Description:
 *  Sync the specified number of sectors to the encapsulated
 *  block device.
 *
 ****************************************************************************/

static ssize_t block_cache_sync(struct block_cache_dev_s * dev,
                                FAR const uint8_t *buffer,
                                off_t startblock, size_t nblocks)
{
  return dev->inode->u.i_bops->write(dev->inode, buffer,
                                           startblock, nblocks);
}

/****************************************************************************
 * Name: block_cache_write
 *
 * Description:
 *  Exposed block write interface. Attempt to cache the incoming buffer
 *  to an existing or available cache window. Perform any synchronisation
 *  needed.
 *
 ****************************************************************************/

static ssize_t block_cache_write(FAR struct inode *inode,
                         FAR const unsigned char *buffer,
                         blkcnt_t startblock, unsigned int nblocks)
{
  int ret;
  struct block_cache_dev_s *dev;

  DEBUGASSERT(inode && inode->i_private);

  dev = (struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret =  block_cache_write_internal(
             dev,
             buffer,
             startblock * dev->geo_multiple,
             nblocks * dev->geo_multiple
        );
  if (ret > 0)
    {
      ret /= dev->geo_multiple;
    }

  nxmutex_unlock(&dev->lock);

  return ret;
}

/****************************************************************************
 * Name: block_cache_geometry
 *
 * Description:
 *  Retrieve the device geometry multiplied by the scaling factor between
 *  the encapsulated and exposed block device.
 *
 ****************************************************************************/

static int block_cache_geometry(FAR struct inode *inode,
                        FAR struct geometry *geometry)
{
  int ret;
  FAR struct block_cache_dev_s *dev;

  DEBUGASSERT(inode && geometry && inode->i_private);
  dev = (struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = dev->inode->u.i_bops->geometry(dev->inode, &dev->geo);

  if (ret >= 0)
    {
      geometry->geo_available     = dev->geo.geo_available;
      geometry->geo_mediachanged  = dev->geo.geo_mediachanged;
      geometry->geo_writeenabled  = dev->geo.geo_writeenabled;
      geometry->geo_nsectors      = dev->geo.geo_nsectors
                                                / dev->geo_multiple;
      geometry->geo_sectorsize    = dev->geo.geo_sectorsize
                                                * dev->geo_multiple;
    }

  nxmutex_unlock(&dev->lock);

  return ret;
}

/****************************************************************************
 * Name: block_cache_ioctl
 *
 * Description:
 *  Exposed ioctl interface. All ioctl commands except for flush are
 *  forwarded through to the encapsulated device.
 *
 ****************************************************************************/

static int block_cache_ioctl(FAR struct inode *inode, int cmd,
                                                    unsigned long arg)
{
  FAR struct block_cache_dev_s *dev;
  int ret;

  DEBUGASSERT(inode && inode->i_private);

  dev = (struct block_cache_dev_s *)inode->i_private;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (cmd == BIOC_FLUSH)
    {
      for (int i = 0; i < dev->cache_count; i++)
        {
          ret = block_cache_flush(dev, i);
          if (ret < 0)
            {
              nxmutex_unlock(&dev->lock);
              return ret;
            }
        }
    }

  ret = dev->inode->u.i_bops->ioctl(dev->inode, cmd, arg);
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: block_cache_initialize
 *
 * Description:
 *   Initialize to provide a cached block device wrapper around an existing
 *    block device
 *
 * Input Parameters:
 *   source:         The source block driver to encapsulate
 *   destination:    The path to the exposed block driver
 *   cache_width:    The number of blocks in a single cache window.
 *   cache_count:    The number of cache windows
 *   geo_multiplier: A multiplier applied between the encapsulated and
 *                   exposed block devices.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int block_cache_initialize(FAR const char *source,
                           FAR const char *destination,
                           const size_t cache_width,
                           const size_t cache_count,
                           const size_t geo_multiplyer)
{
  int ret;
  struct inode *inode;
  struct block_cache_dev_s *dev;

  /* Minimal sanity checks */

  DEBUGASSERT(source && destination);

  /* The cache width should be a power of two */

  DEBUGASSERT(popcount(cache_width) == 1);

  /* The multiplyer is applied to the geometry, zero doesn't make sense */

  DEBUGASSERT(geo_multiplyer > 0);

  finfo("bind \"%s\", to \"%s\"\n", source, destination);

  ret = open_blockdriver(source, 0666, &inode);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a block buffer device structure */

  dev = (FAR struct block_cache_dev_s *)
                      kmm_zalloc(sizeof(struct block_cache_dev_s));
  if (dev)
    {
      /* Initialize the block buffer device structure */

      nxmutex_init(&dev->lock);
      dev->inode = inode;

      ret = dev->inode->u.i_bops->geometry(dev->inode, &dev->geo);
      if (ret < 0)
        {
          ferr("ERROR: node geometry failed: %" PRIdPTR "\n", ret);
          kmm_free(dev);
          return ret;
        }

      dev->cache_width = cache_width;
      dev->cache_count = cache_count;
      dev->geo_multiple = geo_multiplyer;
      dev->cache_used_count = 0;
      dev->last_index = 0;
      dev->aligned_mask = ((dev->cache_width) - 1);
      block_cache_init(dev);

      ret = register_blockdriver(destination, &g_bops, 0666, dev);
      if (ret < 0)
        {
          ferr("ERROR: register_blockdriver failed: %" PRIdPTR "\n", -ret);
          block_cache_deinit(dev);
          kmm_free(dev);
        }
    }
  else
    {
        close_blockdriver(inode);
    }

  return ret;
}
