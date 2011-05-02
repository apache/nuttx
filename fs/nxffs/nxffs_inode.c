/****************************************************************************
 * fs/nxffs/nxffs_inode.c
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
#include <crc32.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rdentry
 *
 * Description:
 *   Read the inode entry at this offset.  Called only from nxffs_nextentry().
 *
 * Input Parameters:
 *   volume - Describes the current volume.
 *   offset - The byte offset from the beginning of FLASH where the inode
 *     header is expected.
 *   entry  - A memory location to return the expanded inode header
 *     information.
 *
 * Returned Value:
 *   Zero on success.  Otherwise, a negater errno value is returned
 *   indicating the nature of the failure.
 *
 ****************************************************************************/

static int nxffs_rdentry(FAR struct nxffs_volume_s *volume, off_t offset,
                         FAR struct nxffs_entry_s *entry)
{
  struct nxffs_inode_s inode;
  uint32_t ecrc;
  uint32_t crc;
  int namlen;
  int ret;

  DEBUGASSERT(volume && entry);
  memset(entry, 0, sizeof(struct nxffs_entry_s));

  /* Read the header at the FLASH offset */

  nxffs_ioseek(volume, offset);
  memcpy(&inode, &volume->cache[volume->iooffset], SIZEOF_NXFFS_INODE_HDR);

  /* Check if the file is marked as deleted. */

  if (inode.state != INODE_STATE_FILE)
    {
      return -ENOENT;
    }
 
  /* Copy the packed header into the user-friendly buffer */

  entry->hoffset = offset;
  entry->noffset = nxffs_rdle32(inode.noffs);
  entry->doffset = nxffs_rdle32(inode.doffs);
  entry->utc     = nxffs_rdle32(inode.utc);
  entry->datlen  = nxffs_rdle32(inode.datlen);

  /* Modify the packed header and perform the (partial) CRC calculation */

  ecrc           = nxffs_rdle32(inode.crc);
  inode.state    = CONFIG_NXFFS_ERASEDSTATE;
  memset(inode.crc, 0, 4);
  crc            = crc32((FAR const uint8_t *)&inode, SIZEOF_NXFFS_INODE_HDR);

  /* Allocate memory to hold the variable-length file name */

  namlen = inode.namlen;
  entry->name = (FAR char *)kmalloc(namlen + 1);
  if (!entry->name)
    {
      fdbg("Failed to allocate name, namlen: %d\n", namlen);
      return -ENOMEM;
    }
  
  /* Seek to the expected location of the name in FLASH */

  nxffs_ioseek(volume, entry->noffset);

  /* Make sure that the block is in memory (the name may not be in the
   * same block as the inode header.
   */
 
  ret = nxffs_rdcache(volume, volume->ioblock, 1);
  if (ret < 0)
    {
      fdbg("nxffsx_rdcache failed: %d\n", -ret);
      return ret;
    }

  /* Read the file name from the expected offset in FLASH */

  memcpy(entry->name, &volume->cache[volume->iooffset], namlen);
  entry->name[namlen] = '\0';

  /* Finish the CRC calculation and verify the entry */

  crc = crc32part((FAR const uint8_t *)entry->name, namlen, crc);
  if (crc != ecrc)
    {
      fdbg("CRC entry: %08x CRC calculated: %08x\n", ecrc, crc);
      nxffs_freeentry(entry);
      return -EIO;
    }
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_freeentry
 *
 * Description:
 *   The inode values returned by nxffs_nextentry() include allocated memory
 *   (specifically, the file name string).  This function should be called
 *   to dispose of that memory when the inode entry is no longer needed.
 *
 *   Note that the nxffs_entry_s containing structure is not freed.  The
 *   caller may call kfree upon return of this function if necessary to
 *   free the entry container.
 *
 * Input parameters:
 *   entry  - The entry to be freed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxffs_freeentry(FAR struct nxffs_entry_s *entry)
{
  if (entry->name)
    {
      kfree(entry->name);
      entry->name = NULL;
    }
}

/****************************************************************************
 * Name: nxffs_nextentry
 *
 * Description:
 *   Search for the next valid inode starting at the provided FLASH offset.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   offset - The FLASH memory offset to begin searching.
 *   entry  - A pointer to memory provided by the caller in which to return
 *     the inode description.
 *  
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 ****************************************************************************/

int nxffs_nextentry(FAR struct nxffs_volume_s *volume, off_t offset,
                    FAR struct nxffs_entry_s *entry)
{
  int nmagic;
  int ch;
  int nerased;
  int ret;

  /* Seek to the first FLASH offset provided by the caller. */

  nxffs_ioseek(volume, offset);

  /* Then begin searching */
  
  nerased = 0;
  nmagic  = 0;
  for (;;)
    {
      /* Read the next character */

      ch = nxffs_getc(volume);
      if (ch < 0)
        {
          fvdbg("nxffs_getc failed: %d\n", -ch);
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
              fvdbg("No entry found\n");
              return -ENOENT;
            }
        }
      else
        {
          nerased = 0;

          /* Check for the magic sequence indicating the start of an NXFFS
           * inode. There is the possibility of this magic sequnce occurring
           * in FLASH data.  However, the header CRC should distinguish
           * between real NXFFS inode headers and such false alarms.
           */

          if (ch != g_inodemagic[nmagic])
            {
              nmagic = 0;
            }
          else if (nmagic < NXFFS_MAGICSIZE - 1)
            {
              nmagic++;
            }

          /* We have found the magic sequence in the FLASH data that may
           * indicate the beginning of an NXFFS inode.
           */

          else 
            {
              /* The the FLASH offset where we found the matching magic number */

              offset = nxffs_iotell(volume) - NXFFS_MAGICSIZE;

              /* Try to extract the inode header from that position */

              ret = nxffs_rdentry(volume, offset, entry);
              if (ret == OK)
                {
                  fvdbg("Found a valid fileheader, offset: %d\n", offset);
                  return OK;
                }

              /* False alarm.. keep looking */

              nmagic = 0;
            }
        }
    }

  /* We won't get here, but to keep some compilers happy: */

  return -ENOENT;
}

/****************************************************************************
 * Name: nxffs_findinode
 *
 * Description:
 *   Search for an inode with the provided name starting with the first
 *   valid inode and proceeding to the end FLASH or until the matching
 *   inode is found.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   name   - The name of the inode to find
 *   entry  - The location to return information about the inode.
 *
 * Returned Value:
 *   Zero is returned on success. Otherwise, a negated errno is returned
 *   that indicates the nature of the failure.
 *
 ****************************************************************************/

int nxffs_findinode(FAR struct nxffs_volume_s *volume, FAR const char *name,
                    FAR struct nxffs_entry_s *entry)
{
  off_t offset;
  int ret;

  /* Start with the first valid inode that was discovered when the volume
   * was created (or modified after the last file system re-packing).
   */

  offset = volume->inoffset;

  /* Loop, checking each NXFFS inode until either: (1) we find the NXFFS inode
   * with the matching name, or (2) we reach the end of data written on the
   * media.
   */

  for (;;)
   {
      /* Get the next, valid NXFFS inode entry */

      ret = nxffs_nextentry(volume, offset, entry);
      if (ret < 0)
        {
          fvdbg("No inode found: %d\n", -ret);
          return ret;
        }

      /* Is this the NXFFS inode we are looking for? */

      else if (strcmp(name, entry->name) == 0)
        {
          /* Yes, return success with the entry data in 'entry' */

          return OK;
        }

      /* Discard this entry and try the next one.  Here we set the
       * next offset using the raw data length as the offset
       * increment.  This is, of course, not accurate because it
       * does not account for the data headers that enclose the
       * data.  But it is guaranteed to be less than or equal to
       * the correct offset and, hence, better then searching
       * byte-for-byte.
       */

      offset = entry->doffset + entry->datlen;
      nxffs_freeentry(entry);
    }

  /* We won't get here, but for some compilers: */

  return -ENOENT;
}
