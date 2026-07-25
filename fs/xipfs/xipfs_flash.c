/****************************************************************************
 * fs/xipfs/xipfs_flash.c
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
 *
 * Every media access in xipfs funnels through this file.  That is
 * deliberate: on a single NOR die without read-while-write, an erase stalls
 * all fetches from the die, so erase slicing, running the erase routine
 * from RAM and erase-suspend/resume all have to be introduced here once the
 * actual NOR part is known.  Keeping the chokepoint from the start means
 * adding them later is a local change rather than a refactor.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>

#include "xipfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_flash_fault
 *
 * Description:
 *   Test hook modelling a power loss at an arbitrary flash operation.  When
 *   the countdown reaches zero the operation fails and the medium latches
 *   dead for the remainder of the run, which is what a real power cut looks
 *   like to the code above: no further write or erase ever completes.
 *
 *   A real cut tears exactly the one operation that was in flight at the
 *   instant power dropped; everything nominally after it simply never runs.
 *   So only the first failing operation reports XIPFS_OP_FAIL_TORN, and only
 *   when the caller armed torn mode -- every operation past the dead latch
 *   fails clean, having never touched the medium.
 *
 ****************************************************************************/

#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
enum xipfs_fault_e
{
  XIPFS_OP_PROCEED = 0,   /* Not the injected point; run the operation      */
  XIPFS_OP_FAIL_CLEAN,    /* Fail with the medium left untouched            */
  XIPFS_OP_FAIL_TORN      /* Fail after half of the operation has landed    */
};

static enum xipfs_fault_e xipfs_flash_fault(FAR struct xipfs_mount_s *fs)
{
  if (fs->media_dead)
    {
      return XIPFS_OP_FAIL_CLEAN;
    }

  if (fs->fault_countdown < 0)
    {
      return XIPFS_OP_PROCEED;
    }

  if (fs->fault_countdown == 0)
    {
      finfo("xipfs: injected %s media failure\n",
            fs->fault_mode == XIPFS_FAULT_TORN ? "torn" : "clean");
      fs->media_dead = true;
      return fs->fault_mode == XIPFS_FAULT_TORN ?
             XIPFS_OP_FAIL_TORN : XIPFS_OP_FAIL_CLEAN;
    }

  fs->fault_countdown--;
  return XIPFS_OP_PROCEED;
}

/****************************************************************************
 * Name: xipfs_flash_tear_write
 *
 * Description:
 *   Model a NOR page program cut short by power loss: the first half of the
 *   page reaches the medium and the rest is left at the erased value, so a
 *   reader sees a page that is neither its old contents nor the intended new
 *   ones -- which is what makes a torn metadata generation fail its CRC
 *   rather than look valid.  Every xipfs write is a single read/write block,
 *   so the tear is expressed by reprogramming that block with its tail
 *   replaced by the erased value; a transient allocation failure simply
 *   leaves the operation a clean one, still a legitimate power-loss outcome.
 *
 ****************************************************************************/

static void xipfs_flash_tear_write(FAR struct xipfs_mount_s *fs,
                                   uint32_t block, uint32_t offset,
                                   FAR const void *buffer, size_t nbytes)
{
  off_t        byteoff = (off_t)block * fs->geo.erasesize + offset;
  size_t       half    = nbytes / 2;
  FAR uint8_t *torn;

  torn = kmm_malloc(nbytes);
  if (torn == NULL)
    {
      return;
    }

  memcpy(torn, buffer, half);
  memset(torn + half, 0xff, nbytes - half);

  MTD_BWRITE(fs->mtd, byteoff / fs->geo.blocksize,
             nbytes / fs->geo.blocksize, torn);

  kmm_free(torn);
}

/****************************************************************************
 * Name: xipfs_flash_tear_erase
 *
 * Description:
 *   Model a sector erase cut short: the leading half of the sector reaches
 *   the erased state while the rest still holds its old contents.  Expressed
 *   by programming the erased value over the first half of the read/write
 *   blocks, which is a simulation the RAM-backed test medium accepts even
 *   though real NOR could not be erased this way.
 *
 ****************************************************************************/

static void xipfs_flash_tear_erase(FAR struct xipfs_mount_s *fs,
                                   uint32_t block)
{
  uint32_t     blks_per_sector = fs->geo.erasesize / fs->geo.blocksize;
  off_t        startblk = (off_t)block * blks_per_sector;
  uint32_t     half = blks_per_sector / 2;
  FAR uint8_t *ones;
  uint32_t     i;

  if (half == 0)
    {
      return;
    }

  ones = kmm_malloc(fs->geo.blocksize);
  if (ones == NULL)
    {
      return;
    }

  memset(ones, 0xff, fs->geo.blocksize);

  for (i = 0; i < half; i++)
    {
      MTD_BWRITE(fs->mtd, startblk + i, 1, ones);
    }

  kmm_free(ones);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_flash_probe
 *
 * Description:
 *   Read the MTD geometry and resolve the direct address of the media.
 *
 *   A media that cannot answer BIOC_XIPBASE is still perfectly usable as a
 *   filesystem; it simply cannot serve execute-in-place mappings, and
 *   strict XIP requests against it fail loudly rather than silently
 *   degrading into a RAM copy.
 *
 ****************************************************************************/

int xipfs_flash_probe(FAR struct xipfs_mount_s *fs)
{
  FAR void *xipbase = NULL;
  int ret;

  ret = MTD_IOCTL(fs->mtd, MTDIOC_GEOMETRY,
                  (unsigned long)(uintptr_t)&fs->geo);
  if (ret < 0)
    {
      ferr("ERROR: MTDIOC_GEOMETRY failed: %d\n", ret);
      return ret;
    }

  if (fs->geo.blocksize == 0 || fs->geo.erasesize == 0 ||
      (fs->geo.erasesize % fs->geo.blocksize) != 0)
    {
      ferr("ERROR: Bad geometry: blocksize=%" PRIu32 " erasesize=%" PRIu32
           "\n", fs->geo.blocksize, fs->geo.erasesize);
      return -EINVAL;
    }

  if (fs->geo.neraseblocks < XIPFS_MIN_BLOCKS)
    {
      ferr("ERROR: Volume too small: %" PRIu32 " blocks\n",
           fs->geo.neraseblocks);
      return -EINVAL;
    }

  /* The metadata generation header occupies the first read/write block of a
   * ring block and the dirents follow it, so a ring block must hold at
   * least the header plus one dirent.
   */

  if (fs->geo.erasesize <= fs->geo.blocksize)
    {
      ferr("ERROR: erasesize must exceed blocksize for the metadata ring\n");
      return -EINVAL;
    }

  ret = MTD_IOCTL(fs->mtd, BIOC_XIPBASE, (unsigned long)(uintptr_t)&xipbase);
  if (ret >= 0 && xipbase != NULL)
    {
      fs->xipbase = (FAR uint8_t *)xipbase;
      finfo("xipfs: XIP base %p\n", fs->xipbase);
    }
  else
    {
      fs->xipbase = NULL;
      fwarn("xipfs: media is not memory mapped; XIP unavailable\n");
    }

  return OK;
}

/****************************************************************************
 * Name: xipfs_flash_addr
 *
 * Description:
 *   Return the direct address of an erase block, or NULL when the media is
 *   not memory mapped.
 *
 ****************************************************************************/

FAR uint8_t *xipfs_flash_addr(FAR struct xipfs_mount_s *fs, uint32_t block)
{
  if (fs->xipbase == NULL)
    {
      return NULL;
    }

  return fs->xipbase + (size_t)block * fs->geo.erasesize;
}

/****************************************************************************
 * Name: xipfs_flash_read
 *
 * Description:
 *   Read nbytes at 'offset' within erase block 'block'.
 *
 ****************************************************************************/

int xipfs_flash_read(FAR struct xipfs_mount_s *fs, uint32_t block,
                     uint32_t offset, FAR void *buffer, size_t nbytes)
{
  off_t byteoff;
  ssize_t nread;

  DEBUGASSERT(offset + nbytes <= fs->geo.erasesize);

  if (nbytes == 0)
    {
      return OK;
    }

  byteoff = (off_t)block * fs->geo.erasesize + offset;

  /* Prefer the byte oriented read when the driver offers one.  Otherwise
   * fall back to reading whole read/write blocks through the staging
   * buffer.
   */

  if (fs->mtd->read != NULL)
    {
      nread = MTD_READ(fs->mtd, byteoff, nbytes, buffer);
      if (nread < 0)
        {
          return (int)nread;
        }

      return nread == (ssize_t)nbytes ? OK : -EIO;
    }

  /* Block oriented fallback.  Only whole read/write blocks can be
   * transferred, so the caller's range must be aligned.  Every internal
   * caller either reads a whole block or reads from a byte capable driver,
   * so this is an assertion rather than a supported partial path.
   */

  if ((offset % fs->geo.blocksize) != 0 ||
      (nbytes % fs->geo.blocksize) != 0)
    {
      return -EINVAL;
    }

  nread = MTD_BREAD(fs->mtd, byteoff / fs->geo.blocksize,
                    nbytes / fs->geo.blocksize, buffer);
  if (nread < 0)
    {
      return (int)nread;
    }

  return nread == (ssize_t)(nbytes / fs->geo.blocksize) ? OK : -EIO;
}

/****************************************************************************
 * Name: xipfs_flash_write
 *
 * Description:
 *   Write nbytes at 'offset' within erase block 'block'.  Both must be
 *   multiples of the read/write block size: NOR is programmed in whole
 *   pages, and every xipfs write path is structured to respect that.
 *
 ****************************************************************************/

int xipfs_flash_write(FAR struct xipfs_mount_s *fs, uint32_t block,
                      uint32_t offset, FAR const void *buffer,
                      size_t nbytes)
{
  off_t byteoff;
  ssize_t nwritten;

  DEBUGASSERT(offset + nbytes <= fs->geo.erasesize);

  if (nbytes == 0)
    {
      return OK;
    }

  if ((offset % fs->geo.blocksize) != 0 ||
      (nbytes % fs->geo.blocksize) != 0)
    {
      ferr("ERROR: Unaligned write: offset=%" PRIu32 " nbytes=%zu\n",
           offset, nbytes);
      return -EINVAL;
    }

#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
  switch (xipfs_flash_fault(fs))
    {
      case XIPFS_OP_FAIL_TORN:
        xipfs_flash_tear_write(fs, block, offset, buffer, nbytes);

        /* Fall through: half the page has landed; report the cut. */

      case XIPFS_OP_FAIL_CLEAN:
        return -EIO;

      default:
        break;
    }
#endif

  byteoff = (off_t)block * fs->geo.erasesize + offset;

  nwritten = MTD_BWRITE(fs->mtd, byteoff / fs->geo.blocksize,
                        nbytes / fs->geo.blocksize, buffer);
  if (nwritten < 0)
    {
      return (int)nwritten;
    }

  return nwritten == (ssize_t)(nbytes / fs->geo.blocksize) ? OK : -EIO;
}

/****************************************************************************
 * Name: xipfs_flash_erase
 *
 * Description:
 *   Erase nblocks erase blocks starting at 'block'.
 *
 *   On a single NOR die this is the operation that stalls instruction
 *   fetches, so this is where erase slicing and erase-suspend belong once
 *   the part is known.  Callers must already have guaranteed that no live
 *   XIP mapping covers the range: defrag does so by refusing to touch a
 *   pinned extent.
 *
 ****************************************************************************/

int xipfs_flash_erase(FAR struct xipfs_mount_s *fs, uint32_t block,
                      uint32_t nblocks)
{
  uint32_t i;
  int ret;

  if (nblocks == 0)
    {
      return OK;
    }

  DEBUGASSERT(block + nblocks <= fs->geo.neraseblocks);

  /* Erase one block at a time.  On real hardware this loop is the natural
   * place to bound interrupt latency; keeping the granularity here from the
   * beginning means the callers already tolerate a partial erase followed
   * by an error.
   */

  for (i = 0; i < nblocks; i++)
    {
#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
      switch (xipfs_flash_fault(fs))
        {
          case XIPFS_OP_FAIL_TORN:
            xipfs_flash_tear_erase(fs, block + i);

            /* Fall through: half the sector is erased; report the cut. */

          case XIPFS_OP_FAIL_CLEAN:
            return -EIO;

          default:
            break;
        }
#endif

      ret = MTD_ERASE(fs->mtd, block + i, 1);
      if (ret < 0)
        {
          ferr("ERROR: Erase of block %" PRIu32 " failed: %d\n",
               block + i, ret);
          return ret;
        }
    }

  return OK;
}
