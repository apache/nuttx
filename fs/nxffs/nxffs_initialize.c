/****************************************************************************
 * fs/nxffs/nxffs_initialize.c
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
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "nxffs.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations nxffs_operations =
{
  nxffs_open,        /* open */
  nxffs_close,       /* close */
  nxffs_read,        /* read */
  nxffs_write,       /* write */
  NULL,              /* seek -- Use f_pos in struct file */
  nxffs_ioctl,       /* ioctl */

  NULL,              /* sync -- No buffered data */
  nxffs_dup,         /* dup */
  nxffs_fstat,       /* fstat */
  NULL,              /* fchstat */
#ifdef __NO_TRUNCATE_SUPPORT__
  nxffs_truncate,    /* truncate */
#else
  NULL,              /* truncate */
#endif

  nxffs_opendir,     /* opendir */
  nxffs_closedir,    /* closedir */
  nxffs_readdir,     /* readdir */
  nxffs_rewinddir,   /* rewinddir */

  nxffs_bind,        /* bind */
  nxffs_unbind,      /* unbind */
  nxffs_statfs,      /* statfs */

  nxffs_unlink,      /* unlink */
  NULL,              /* mkdir -- no directories */
  NULL,              /* rmdir -- no directories */
  NULL,              /* rename -- cannot rename in place if name is longer */
  nxffs_stat,        /* stat */
  NULL               /* chstat */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The magic number that appears that the beginning of each NXFFS (logical)
 * block
 */

const uint8_t g_blockmagic[NXFFS_MAGICSIZE] =
{
  'B', 'l', 'c', 'k'
};

/* The magic number that appears that the beginning of each NXFFS inode */

const uint8_t g_inodemagic[NXFFS_MAGICSIZE] =
{
  'I', 'n', 'o', 'd'
};

/* The magic number that appears that the beginning of each NXFFS inode
 * data block.
 */

const uint8_t g_datamagic[NXFFS_MAGICSIZE] =
{
  'D', 'a', 't', 'a'
};

/* If CONFIG_NXFFS_PREALLOCATED is defined, then this is the single, pre-
 * allocated NXFFS volume instance.
 */

#ifdef CONFIG_NXFFS_PREALLOCATED
struct nxffs_volume_s g_volume;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_initialize
 *
 * Description:
 *   Initialize to provide NXFFS on an MTD interface
 *
 * Input Parameters:
 *   mtd - The MTD device that supports the FLASH interface.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_initialize(FAR struct mtd_dev_s *mtd)
{
  FAR struct nxffs_volume_s *volume;
#ifdef CONFIG_NXFFS_SCAN_VOLUME
  struct nxffs_blkstats_s stats;
  off_t threshold;
#endif
  int ret;

  /* If CONFIG_NXFFS_PREALLOCATED is defined, then this is the single, pre-
   * allocated NXFFS volume instance.
   */

#ifdef CONFIG_NXFFS_PREALLOCATED

  volume = &g_volume;
  memset(volume, 0, sizeof(struct nxffs_volume_s));

#else

  /* Allocate a NXFFS volume structure */

  volume = kmm_zalloc(sizeof(struct nxffs_volume_s));
  if (!volume)
    {
      return -ENOMEM;
    }
#endif

  /* Initialize the NXFFS volume structure */

  volume->mtd    = mtd;
  volume->cblock = (off_t)-1;
  nxmutex_init(&volume->lock);
  nxsem_init(&volume->wrsem, 0, 1);

  /* Get the volume geometry. (casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                  (unsigned long)((uintptr_t)&volume->geo));
  if (ret < 0)
    {
      ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", -ret);
      goto errout_with_volume;
    }

  /* Allocate one I/O block buffer to general files system access */

  volume->cache = (FAR uint8_t *)kmm_malloc(volume->geo.blocksize);
  if (!volume->cache)
    {
      ferr("ERROR: Failed to allocate an erase block buffer\n");
      ret = -ENOMEM;
      goto errout_with_volume;
    }

  /* Pre-allocate one, full, in-memory erase block.  This is needed for
   * filesystem packing (but is useful in other places as well). This buffer
   * is not needed often, but is best to have pre-allocated and in-place.
   */

  volume->pack = (FAR uint8_t *)kmm_malloc(volume->geo.erasesize);
  if (!volume->pack)
    {
      ferr("ERROR: Failed to allocate an I/O block buffer\n");
      ret = -ENOMEM;
      goto errout_with_cache;
    }

  /* Get the number of R/W blocks per erase block and the total number o
   * R/W blocks
   */

  volume->blkper  = volume->geo.erasesize / volume->geo.blocksize;
  volume->nblocks = volume->geo.neraseblocks * volume->blkper;
  DEBUGASSERT((off_t)volume->blkper * volume->geo.blocksize ==
              volume->geo.erasesize);

#ifdef CONFIG_NXFFS_SCAN_VOLUME
  /* Check if there is a valid NXFFS file system on the flash */

  ret = nxffs_blockstats(volume, &stats);
  if (ret < 0)
    {
      ferr("ERROR: Failed to collect block statistics: %d\n", -ret);
      goto errout_with_buffer;
    }

  /* If the proportion of good blocks is low or the proportion of unformatted
   * blocks is high, then reformat the FLASH.
   */

  threshold = (stats.nblocks * CONFIG_NXFFS_REFORMAT_THRESH) / 100;
  if (stats.ngood < threshold || stats.nunformat > threshold)
    {
      /* Reformat the volume */

      ret = nxffs_reformat(volume);
      if (ret < 0)
        {
          ferr("ERROR: Failed to reformat the volume: %d\n", -ret);
          goto errout_with_buffer;
        }

      /* Get statistics on the re-formatted volume */

#if defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_FS)
      ret = nxffs_blockstats(volume, &stats);
      if (ret < 0)
        {
          ferr("ERROR: Failed to collect block statistics: %d\n", -ret);
          goto errout_with_buffer;
        }
#endif
    }
#endif /* CONFIG_NXFFS_SCAN_VOLUME */

  /* Get the file system limits */

  ret = nxffs_limits(volume);
  if (ret == OK)
    {
      return OK;
    }

  /* We may need to format the volume.  Try that before giving up. */

  fwarn("WARNING: Failed to calculate file system limits: %d\n", -ret);
  ret = nxffs_reformat(volume);
  if (ret < 0)
    {
      ferr("ERROR: Failed to reformat the volume: %d\n", -ret);
      goto errout_with_buffer;
    }

  /* Get statistics on the re-formatted volume */

#if defined(CONFIG_NXFFS_SCAN_VOLUME) && defined(CONFIG_DEBUG_FEATURES) && defined(CONFIG_DEBUG_FS)
  ret = nxffs_blockstats(volume, &stats);
  if (ret < 0)
    {
      ferr("ERROR: Failed to collect block statistics: %d\n", -ret);
      goto errout_with_buffer;
    }
#endif

  /* Now try to get the file system limits again */

  ret = nxffs_limits(volume);
  if (ret == OK)
    {
      return OK;
    }

  /* Now give up */

  ferr("ERROR: Failed to calculate file system limits: %d\n", -ret);

errout_with_buffer:
  kmm_free(volume->pack);
errout_with_cache:
  kmm_free(volume->cache);
errout_with_volume:
  nxmutex_destroy(&volume->lock);
  nxsem_destroy(&volume->wrsem);
#ifndef CONFIG_NXFFS_PREALLOCATED
  kmm_free(volume);
#endif
  return ret;
}

/****************************************************************************
 * Name: nxffs_limits
 *
 * Description:
 *   Recalculate file system limits:  (1) the FLASH offset to the first,
 *   valid inode, and (2) the FLASH offset to the first, unused byte after
 *   the last inode (invalid or not).
 *
 *   The first, lower limit must be recalculated: (1) initially, (2)
 *   whenever the first inode is deleted, or (3) whenever inode is moved
 *   as part of the file system packing operation.
 *
 *   The second, upper limit must be (1) incremented whenever new file
 *   data is written, or (2) recalculated as part of the file system packing
 *   operation.
 *
 * Input Parameters:
 *   volume - Identifies the NXFFS volume
 *
 * Returned Value:
 *   Zero on success. Otherwise, a negated error is returned indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nxffs_limits(FAR struct nxffs_volume_s *volume)
{
  FAR struct nxffs_entry_s entry;
  off_t block;
  off_t offset;
  bool noinodes = false;
  int nerased;
  int ret;

  /* Get the offset to the first valid block on the FLASH */

  block = 0;
  ret = nxffs_validblock(volume, &block);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find a valid block: %d\n", -ret);
      return ret;
    }

  /* Then find the first valid inode in or beyond the first valid block */

  offset = block * volume->geo.blocksize;
  ret = nxffs_nextentry(volume, offset, &entry);
  if (ret < 0)
    {
      /* The value -ENOENT is special.  This simply means that the FLASH
       * was searched to the end and no valid inode was found... the file
       * system is empty (or, in more perverse cases, all inodes are
       * deleted or corrupted).
       */

      if (ret != -ENOENT)
        {
          ferr("ERROR: nxffs_nextentry failed: %d\n", -ret);
          return ret;
        }

      /* Set a flag the just indicates that no inodes were found.  Later,
       * we will set the location of the first inode to be the same as
       * the location of the free FLASH region.
       */

      finfo("No inodes found\n");
      noinodes = true;
    }
  else
    {
      /* Save the offset to the first inode */

      volume->inoffset = entry.hoffset;
      finfo("First inode at offset %jd\n", (intmax_t)volume->inoffset);

      /* Discard this entry and set the next offset. */

      offset = nxffs_inodeend(volume, &entry);
      nxffs_freeentry(&entry);
    }

  /* Now, search for the last valid entry */

  if (!noinodes)
    {
      while (nxffs_nextentry(volume, offset, &entry) == OK)
        {
          /* Discard the entry and guess the next offset. */

          offset = nxffs_inodeend(volume, &entry);
          nxffs_freeentry(&entry);
        }

      finfo("Last inode before offset %jd\n", (intmax_t)offset);
    }

  /* No inodes were found after this offset.  Now search for a block of
   * erased flash.
   */

  nxffs_ioseek(volume, offset);
  nerased = 0;
  for (; ; )
    {
      int ch = nxffs_getc(volume, 1);
      if (ch < 0)
        {
          /* Failed to read the next byte... this could mean that the FLASH
           * is full?
           */

          if (volume->ioblock + 1 >= volume->nblocks &&
              volume->iooffset + 1 >= volume->geo.blocksize)
            {
              /* Yes.. the FLASH is full.  Force the offsets to the end of
               * FLASH
               */

              volume->froffset = volume->nblocks * volume->geo.blocksize;
              finfo("Assume no free FLASH, froffset: %jd\n",
                    (intmax_t)volume->froffset);
              if (noinodes)
                {
                  volume->inoffset = volume->froffset;
                  finfo("No inodes, inoffset: %jd\n",
                        (intmax_t)volume->inoffset);
                }

              return OK;
            }

          /* No?  Then it is some other failure that we do not know how to
           * handle
           */

          ferr("ERROR: nxffs_getc failed: %d\n", -ch);
          return ch;
        }

      /* Check for another erased byte */

      else if (ch == CONFIG_NXFFS_ERASEDSTATE)
        {
          /* If we have encountered NXFFS_NERASED number of consecutive
           * erased bytes, then presume we have reached the end of valid
           * data.
           */

          if (++nerased >= NXFFS_NERASED)
            {
              /* Okay.. we have a long stretch of erased FLASH in a valid
               * FLASH block.  Let's say that this is the beginning of
               * the free FLASH region.
               */

              volume->froffset = offset;
              finfo("Free FLASH region begins at offset: %jd\n",
                    (intmax_t)volume->froffset);
              if (noinodes)
                {
                  volume->inoffset = offset;
                  finfo("First inode at offset %jd\n",
                        (intmax_t)volume->inoffset);
                }

              return OK;
            }
        }
      else
        {
          offset += nerased + 1;
          nerased = 0;
        }
    }

  /* Won't get here */

  return OK;
}

/****************************************************************************
 * Name: nxffs_bind
 *
 * Description:
 *   This function implements a portion of the mount operation. Normmally,
 *   the bind() method allocates and initializes the mountpoint private data
 *   then binds the blockdriver inode to the filesystem private data.  The
 *   final binding of the private data (containing the blockdriver) to the
 *   mountpoint is performed by mount().
 *
 *   For the NXFFS, this sequence is quite different for the following
 *   reasons:
 *
 *   1. A block driver is not used.  Instead, an MTD instance was provided
 *      to nxfs_initialize prior to mounting.  So, in this sense, the NXFFS
 *      file system is already bound.
 *
 *   2. Since the volume was already bound to the MTD driver, all allocations
 *      and initializations have already been performed.  Essentially, all
 *      mount operations have been bypassed and now we just need to provide
 *      the pre-allocated volume instance.
 *
 *   3. The tricky thing is that there is no mechanism to associate multiple
 *      NXFFS volumes to the multiple volumes bound to different MTD drivers.
 *      Hence, the limitation of a single NXFFS volume.
 *
 ****************************************************************************/

int nxffs_bind(FAR struct inode *blkdriver, FAR const void *data,
               FAR void **handle)
{
#ifndef CONFIG_NXFFS_PREALLOCATED
#  error "No design to support dynamic allocation of volumes"
#else

  /* If CONFIG_NXFFS_PREALLOCATED is defined, then this is the single, pre-
   * allocated NXFFS volume instance.
   */

  DEBUGASSERT(g_volume.cache);
  *handle = &g_volume;
#endif
  return OK;
}

/****************************************************************************
 * Name: nxffs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

int nxffs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                 unsigned int flags)
{
#ifndef CONFIG_NXFFS_PREALLOCATED
#  error "No design to support dynamic allocation of volumes"
#else
  /* This implementation currently only supports unmounting if there are no
   * open file references.
   */

  if (flags != 0)
    {
      return -ENOSYS;
    }

  return g_volume.ofiles ? -EBUSY : OK;
#endif
}
