/****************************************************************************
 * fs/nxffs/nxffs_dump.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/ioctl.h>
#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_analyze
 *
 * Description:
 *   Analyze the content of one block.
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
static inline void nxffs_analyze(FAR uint8_t *buffer, size_t buflen, off_t blockno)
{
  FAR struct nxffs_block_s *blkhdr;

  /* Verify that there is a header on the block */

  blkhdr = (FAR struct nxffs_block_s *)buffer;
  if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0)
    {
      fdbg("Block %d: ERROR -- no header\n", blockno);
    }
  else if (blkhdr->state == BLOCK_STATE_GOOD)
    {
      fdbg("Block %d: GOOD\n", blockno);
    }
  else if (blkhdr->state == BLOCK_STATE_BAD)
    {
      fdbg("Block %d: BAD\n", blockno);
    }
  else
    {
      fdbg("Block %d: ERROR -- bad state\n", blockno);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_dump
 *
 * Description:
 *   Dump a summary of the contents of an NXFFS file system.  CONFIG_DEBUG
 *   and CONFIG_DEBUG_FS must be enabled for this function to do anything.
 *
 * Input Parameters:
 *   mtd - The MTD device that provides the interface to NXFFS-formatted media.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_dump(FAR struct mtd_dev_s *mtd)
{
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_FS)
  struct mtd_geometry_s geo;
  FAR uint8_t *buffer;
  off_t nblocks;
  off_t block;
  int ret;

  /* Get the volume geometry. (casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", -ret);
      goto errout;
    }

  /* Allocate a buffer to hold one block */

  buffer = (FAR uint8_t *)kmalloc(geo.blocksize);
  if (!buffer)
    {
      fdbg("Failed to allocate block cache\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Now read every block on the device */

  nblocks = geo.erasesize * geo.neraseblocks / geo.blocksize;
  for (block = 0; block < nblocks; block++)
    {
      /* Read the next block */

      ret = MTD_BREAD(mtd, block, 1, buffer);
      if (ret < 0)
        {
          fdbg("Failed to read block %d\n", block);
          goto errout_with_block;
        }

      /* Analyze the block */

      nxffs_analyze(buffer, geo.blocksize, block);
    }

errout_with_block:
  kfree(buffer);
errout:
  return ret;

#else
  return -ENOSYS;
#endif
}
