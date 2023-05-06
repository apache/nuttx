/****************************************************************************
 * fs/nxffs/nxffs_write.c
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
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/crc32.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_hdrpos
 *
 * Description:
 *   Find a valid location for the data block header.  A valid location will
 *   have these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire header
 *      PLUS some meaningful amount of data (NXFFS_MINDATA).
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the checks of 1) and 2).
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Contains the current guess for the header position.  On
 *     successful return, this field will hold the selected header
 *     position.
 *   size - The minimum size of the current write.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *     wrfile->doffset - Flash offset to candidate data block header position
 *     volume->ioblock - Read/write block number of the block containing the
 *       header position
 *     volume->iooffset - The offset in the block to the candidate header
 *       position.
 *     volume->froffset - Updated offset to the first free FLASH block.
 *
 ****************************************************************************/

static inline int nxffs_hdrpos(FAR struct nxffs_volume_s *volume,
                               FAR struct nxffs_wrfile_s *wrfile,
                               size_t size)
{
  int ret;

  /* Reserve memory for the object */

  ret = nxffs_wrreserve(volume, SIZEOF_NXFFS_DATA_HDR + size);
  if (ret == OK)
    {
      /* Save the offset to the FLASH region reserved for the data block
       * header
       */

      wrfile->doffset = nxffs_iotell(volume);
    }

  return ret;
}

/****************************************************************************
 * Name: nxffs_hdrerased
 *
 * Description:
 *   Find a valid location for the data block header.  A valid location will
 *   have these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire header
 *      PLUS some meaningful amount of data (NXFFS_MINDATA).
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the check 3). On entry it assumes:
 *
 *     volume->ioblock  - Read/write block number of the block containing the
 *       header position
 *     volume->iooffset - The offset in the block to the candidate header
 *       position.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Contains the current guess for the header position.  On
 *     successful return, this field will hold the selected header
 *     position.
 *   size - The minimum size of the current write.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *     wrfile->doffset - Flash offset to candidate data block header position
 *     volume->ioblock - Read/write block number of the block containing the
 *       header position
 *     volume->iooffset - The offset in the block to the candidate header
 *       position.
 *     volume->froffset - Updated offset to the first free FLASH block.
 *
 ****************************************************************************/

static inline int nxffs_hdrerased(FAR struct nxffs_volume_s *volume,
                                  FAR struct nxffs_wrfile_s *wrfile,
                                  size_t size)
{
  int ret;

  /* Find a valid location to save the inode header */

  ret = nxffs_wrverify(volume, SIZEOF_NXFFS_DATA_HDR + size);
  if (ret == OK)
    {
      /* This is where we will put the data block header */

      wrfile->doffset = nxffs_iotell(volume);
    }

  return ret;
}

/****************************************************************************
 * Name: nxffs_wralloc
 *
 * Description:
 *   Allocate FLASH memory for the data block.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Describes the open file to be written.
 *   size   - Size of the current write operation.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_wralloc(FAR struct nxffs_volume_s *volume,
                                FAR struct nxffs_wrfile_s *wrfile,
                                size_t size)
{
  bool packed;
  int ret;

  /* Allocate FLASH memory for the data block.
   *
   * Loop until the data block header is configured or until a failure
   * occurs. Note that nothing is written to FLASH.  The data block header
   * is not written until either (1) the file is closed, or (2) the data
   * region is fully populated.
   */

  packed = false;
  for (; ; )
    {
      size_t mindata = MIN(NXFFS_MINDATA, size);

      /* File a valid location to position the data block.  Start with
       * the first byte in the free FLASH region.
       */

      ret = nxffs_hdrpos(volume, wrfile, mindata);
      if (ret == OK)
        {
          /* Find a region of memory in the block that is fully erased */

          ret = nxffs_hdrerased(volume, wrfile, mindata);
          if (ret == OK)
            {
              /* Valid memory for the data block was found.  Return
               * success.
               */

              return OK;
            }
        }

      /* If no valid memory is found searching to the end of the volume,
       * then -ENOSPC will be returned.  Other errors are not handled.
       */

      if (ret != -ENOSPC || packed)
        {
          ferr("ERROR: Failed to find inode header memory: %d\n", -ret);
          return -ENOSPC;
        }

      /* -ENOSPC is a special case..  It means that the volume is full.
       * Try to pack the volume in order to free up some space.
       */

      ret = nxffs_pack(volume);
      if (ret < 0)
        {
          ferr("ERROR: Failed to pack the volume: %d\n", -ret);
          return ret;
        }

      /* After packing the volume, froffset will be updated to point to the
       * new free flash region.  Try again.
       */

      nxffs_ioseek(volume, volume->froffset);
      packed = true;
    }

  /* Can't get here */

  return OK;
}

/****************************************************************************
 * Name: nxffs_reverify
 *
 * Description:
 *   Verify that the partial flash data block in the volume cache is valid.
 *   On entry, this function assumes:
 *
 *   volume->ioblock  - Read/write block number of the block containing the
 *     data block.
 *   volume->iooffset - The offset in the block to the data block.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Describes the open file to be written.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_reverify(FAR struct nxffs_volume_s *volume,
                                 FAR struct nxffs_wrfile_s *wrfile)
{
  uint32_t crc;
  off_t offset;

  if (wrfile->datlen > 0)
    {
      /* Get the offset to the start of the data */

      offset = volume->iooffset + SIZEOF_NXFFS_DATA_HDR;
      DEBUGASSERT(offset + wrfile->datlen <= volume->geo.blocksize);

      /* Calculate the CRC of the partial data block */

      crc = crc32(&volume->cache[offset], wrfile->datlen);

      /* It must match the previously calculated CRC value */

      if (crc != wrfile->crc)
        {
          ferr("ERROR: CRC failure\n");
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nxffs_wrappend
 *
 * Description:
 *   Append FLASH data to the data block.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Describes the open file to be written.
 *   buffer - Address of buffer of data to be written.
 *   buflen - The number of bytes remaining to be written
 *
 * Returned Value:
 *   The number of bytes written is returned on success.  Otherwise, a
 *   negated errno value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

static inline ssize_t nxffs_wrappend(FAR struct nxffs_volume_s *volume,
                                     FAR struct nxffs_wrfile_s *wrfile,
                                     FAR const char *buffer, size_t buflen)
{
  ssize_t maxsize;
  size_t nbytestowrite;
  ssize_t nbytesleft;
  off_t offset;
  int ret;

  /* Get the offset to the start of unwritten data */

  offset = volume->iooffset + wrfile->datlen + SIZEOF_NXFFS_DATA_HDR;

  /* Determine that maximum amount of data that can be written to this
   * block.
   */

  maxsize = volume->geo.blocksize - offset;
  DEBUGASSERT(maxsize > 0);

  /* But don't try to write over any unerased bytes */

  maxsize = nxffs_erased(&volume->cache[offset], maxsize);

  /* Write as many bytes as we can into the data buffer */

  nbytestowrite = MIN(maxsize, buflen);
  nbytesleft    = maxsize - nbytestowrite;

  if (nbytestowrite > 0)
    {
      /* Copy the data into the volume write cache */

      memcpy(&volume->cache[offset], buffer, nbytestowrite);

      /* Increment the number of bytes written to the data block */

      wrfile->datlen += nbytestowrite;

      /* Re-calculate the CRC */

      offset = volume->iooffset + SIZEOF_NXFFS_DATA_HDR;
      wrfile->crc = crc32(&volume->cache[offset], wrfile->datlen);

      /* And write the partial write block to FLASH -- unless the data
       * block is full.  In that case, the block will be written below.
       */

      if (nbytesleft > 0)
        {
          ret = nxffs_wrcache(volume);
          if (ret < 0)
            {
              ferr("ERROR: nxffs_wrcache failed: %d\n", -ret);
              return ret;
            }
        }
    }

  /* Check if the data block is now full */

  if (nbytesleft <= 0)
    {
      /* The data block is full, write the block to FLASH */

      ret = nxffs_wrblkhdr(volume, wrfile);
      if (ret < 0)
        {
          ferr("ERROR: nxffs_wrblkdhr failed: %d\n", -ret);
          return ret;
        }
    }

  /* Return the number of bytes written to FLASH this time */

  return nbytestowrite;
}

/****************************************************************************
 * Name: nxffs_zappend
 *
 * Description:
 *   Zero-extend FLASH data to the data block.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   wrfile - Describes the open file to be written.
 *   nzeros - The number of bytes of zeroed data to be written
 *
 * Returned Value:
 *   The number of zero bytes written is returned on success.  Otherwise, a
 *   negated errno value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

#ifdef __NO_TRUNCATE_SUPPORT__
static inline ssize_t nxffs_zappend(FAR struct nxffs_volume_s *volume,
                                    FAR struct nxffs_wrfile_s *wrfile,
                                    off_t nzeros)
{
  ssize_t maxsize;
  size_t nbytestoclear;
  ssize_t nbytesleft;
  off_t offset;
  int ret;

  /* Get the offset to the start of unwritten data */

  offset = volume->iooffset + wrfile->datlen + SIZEOF_NXFFS_DATA_HDR;

  /* Determine that maximum amount of data that can be written to this
   * block.
   */

  maxsize = volume->geo.blocksize - offset;
  DEBUGASSERT(maxsize > 0);

  /* Write as many bytes as we can into the data buffer */

  nbytestoclear = MIN(maxsize, nzeros);
  nbytesleft    = maxsize - nbytestoclear;

  if (nbytestoclear > 0)
    {
      /* Zero the data into the volume write cache */

      memset(&volume->cache[offset], 0, nbytestoclear);

      /* Increment the number of bytes written to the data block */

      wrfile->datlen += nbytestoclear;

      /* Re-calculate the CRC */

      offset = volume->iooffset + SIZEOF_NXFFS_DATA_HDR;
      wrfile->crc = crc32(&volume->cache[offset], wrfile->datlen);

      /* And write the partial write block to FLASH -- unless the data
       * block is full.  In that case, the block will be written below.
       */

      if (nbytesleft > 0)
        {
          ret = nxffs_wrcache(volume);
          if (ret < 0)
            {
              ferr("ERROR: nxffs_wrcache failed: %d\n", -ret);
              return ret;
            }
        }
    }

  /* Check if the data block is now full */

  if (nbytesleft <= 0)
    {
      /* The data block is full, write the block to FLASH */

      ret = nxffs_wrblkhdr(volume, wrfile);
      if (ret < 0)
        {
          ferr("ERROR: nxffs_wrblkdhr failed: %d\n", -ret);
          return ret;
        }
    }

  return nbytestoclear;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_write
 *
 * Description:
 *   This is an implementation of the NuttX standard file system write
 *   method.
 *
 ****************************************************************************/

ssize_t nxffs_write(FAR struct file *filep, FAR const char *buffer,
                    size_t buflen)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_wrfile_s *wrfile;
  ssize_t remaining;
  ssize_t nwritten;
  ssize_t total;
  int ret;

  finfo("Write %zd bytes to offset %jd\n", buflen, (intmax_t)filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  wrfile = (FAR struct nxffs_wrfile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume lock
   * protects the open file list.
   */

  ret = nxmutex_lock(&volume->lock);
  if (ret < 0)
    {
      ferr("ERROR: nxmutex_lock failed: %d\n", ret);
      goto errout;
    }

  /* Check if the file was opened with write access */

  if ((wrfile->ofile.oflags & O_WROK) == 0)
    {
      ferr("ERROR: File not open for write access\n");
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Loop until we successfully appended all of the data to the file (or an
   * error occurs)
   */

  for (total = 0; total < buflen; )
    {
      remaining = buflen - total;

      /* Have we already allocated the data block? */

      if (wrfile->doffset == 0)
        {
          /* No, allocate the data block now, re-packing if necessary. */

          wrfile->datlen = 0;
          ret = nxffs_wralloc(volume, wrfile, remaining);
          if (ret < 0)
            {
              ferr("ERROR: Failed to allocate a data block: %d\n", -ret);
              goto errout_with_lock;
            }
        }

      /* Seek to the FLASH block containing the data block */

      nxffs_ioseek(volume, wrfile->doffset);

      /* Verify that the FLASH data that was previously written is still
       * intact
       */

      ret = nxffs_reverify(volume, wrfile);
      if (ret < 0)
        {
          ferr("ERROR: Failed to verify FLASH data block: %d\n", -ret);
          goto errout_with_lock;
        }

      /* Append the data to the end of the data block and write the updated
       * block to flash.
       */

      nwritten = nxffs_wrappend(volume, wrfile, &buffer[total], remaining);
      if (nwritten < 0)
        {
          ferr("ERROR: Failed to append to FLASH to a data block: %d\n",
               -ret);
          goto errout_with_lock;
        }

      /* Decrement the number of bytes remaining to be written */

      total += nwritten;
    }

  /* Success.. return the number of bytes written */

  ret          = total;
  filep->f_pos = wrfile->datlen;

errout_with_lock:
  nxmutex_unlock(&volume->lock);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_wrextend
 *
 * Description:
 *   Zero-extend a file.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   entry  - Describes the new inode entry
 *   length - The new, extended length of the file
 *
 * Assumptions:
 *   The caller holds the NXFFS semaphore.
 *   The caller has verified that the file is writable.
 *
 ****************************************************************************/

#ifdef __NO_TRUNCATE_SUPPORT__
int nxffs_wrextend(FAR struct nxffs_volume_s *volume,
                   FAR struct nxffs_wrfile_s *wrfile, off_t length)
{
  ssize_t remaining;
  ssize_t nwritten;
  off_t oldsize;
  int ret;

  finfo("Extend file to %ld bytes to offset %d\n", (long)length);
  DEBUGASSERT(volume != NULL && wrfile != NULL);

  oldsize = wrfile->ofile.entry.datlen;
  DEBUGASSERT(length > oldsize);

  /* Loop until we successfully appended all of the data to the file (or an
   * error occurs)
   */

  remaining = length - oldsize;
  while (remaining > 0)
    {
      /* Have we already allocated the data block? */

      if (wrfile->doffset == 0)
        {
          /* No, allocate the data block now, re-packing if necessary. */

          wrfile->datlen = 0;
          ret = nxffs_wralloc(volume, wrfile, remaining);
          if (ret < 0)
            {
              ferr("ERROR: Failed to allocate a data block: %d\n", -ret);
              return ret;
            }
        }

      /* Seek to the FLASH block containing the data block */

      nxffs_ioseek(volume, wrfile->doffset);

      /* Verify that the FLASH data that was previously written is still
       * intact
       */

      ret = nxffs_reverify(volume, wrfile);
      if (ret < 0)
        {
          ferr("ERROR: Failed to verify FLASH data block: %d\n", -ret);
          return ret;
        }

      /* Append the data to the end of the data block and write the updated
       * block to flash.
       */

      nwritten = nxffs_zappend(volume, wrfile, remaining);
      if (nwritten < 0)
        {
          ferr("ERROR: Failed to zero extend FLASH data block: %d\n", -ret);
          return (int)nwritten;
        }

      /* Decrement the number of bytes remaining to be written */

      remaining -= nwritten;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: nxffs_wrreserve
 *
 * Description:
 *   Find a valid location for a file system object of 'size'.  A valid
 *   location will have these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire object
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the checks of 1) and 2).  The
 *   end-of-filesystem offset, froffset, is update past this memory which,
 *   in effect, reserves the memory.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   size - The size of the object to be reserved.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     candidate object position
 *   volume->iooffset - The offset in the block to the candidate object
 *     position.
 *   volume->froffset - Updated offset to the first free FLASH block after
 *     the reserved memory.
 *
 ****************************************************************************/

int nxffs_wrreserve(FAR struct nxffs_volume_s *volume, size_t size)
{
  int ret;

  /* Seek to the beginning of the free FLASH region */

  nxffs_ioseek(volume, volume->froffset);

  /* Check for a seek past the end of the volume */

  if (volume->ioblock >= volume->nblocks)
    {
      /* Return -ENOSPC to indicate that the volume is full */

      return -ENOSPC;
    }

  /* Skip over block headers */

  if (volume->iooffset < SIZEOF_NXFFS_BLOCK_HDR)
    {
      volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
    }

  /* Make sure that there is space there to hold the entire object */

  if (volume->iooffset + size > volume->geo.blocksize)
    {
      /* We will need to skip to the next block.  But first, check if we are
       * already at the final block.
       */

      if (volume->ioblock + 1 >= volume->nblocks)
        {
          /* Return -ENOSPC to indicate that the volume is full */

          ferr("ERROR: No space in last block\n");
          return -ENOSPC;
        }

      /* This is not the last block in the volume, so just seek to the
       * beginning of the next, valid block.
       */

      volume->ioblock++;
      ret = nxffs_validblock(volume, &volume->ioblock);
      if (ret < 0)
        {
          ferr("ERROR: No more valid blocks\n");
          return ret;
        }

      volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
    }

  /* Update the pointer to the first next free FLASH memory -- reserving this
   * block of memory.
   */

  volume->froffset = nxffs_iotell(volume) + size;
  return OK;
}

/****************************************************************************
 * Name: nxffs_wrverify
 *
 * Description:
 *   Find a valid location for the object.  A valid location will have
 *   these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire header
 *      (excluding the file name which may lie in the next block).
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the check 3). On entry it assumes the
 *   following settings (left by nxffs_wrreserve()):
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     candidate object position
 *   volume->iooffset - The offset in the block to the candidate object
 *     position.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   size - The size of the object to be verified.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     verified object position
 *   volume->iooffset - The offset in the block to the verified object
 *     position.
 *   volume->froffset - Updated offset to the first free FLASH block.
 *
 ****************************************************************************/

int nxffs_wrverify(FAR struct nxffs_volume_s *volume, size_t size)
{
  uint16_t iooffset;
  int nerased;
  int ret;
  int i;

  /* Search to the very last block in the volume if we have to */

  while (volume->ioblock < volume->nblocks)
    {
      /* Make sure that the block is in memory */

      ret = nxffs_rdcache(volume, volume->ioblock);
      if (ret < 0)
        {
          /* Ignore the error... just skip to the next block.  This should
           * never happen with normal FLASH, but could occur with NAND if
           * the block has uncorrectable bit errors.
           */

          ferr("ERROR: Failed to read block %jd: %d\n",
               (intmax_t)volume->ioblock, -ret);
        }

      /* Search to the very end of this block if we have to */

      else
        {
          iooffset = volume->iooffset;
          nerased  = 0;

          for (i = volume->iooffset; i < volume->geo.blocksize; i++)
            {
              /* Is this byte erased? */

              if (volume->cache[i] == CONFIG_NXFFS_ERASEDSTATE)
                {
                  /* Yes.. increment the count of contiguous, erased bytes */

                  nerased++;

                  /* Is the whole header memory erased? */

                  if (nerased >= size)
                    {
                      /* Yes.. this is where we will put the object */

                      off_t offset =
                        volume->ioblock * volume->geo.blocksize + iooffset;

                      /* Update the free flash offset and return success */

                      volume->froffset = offset + size;
                      return OK;
                    }
                }

              /* This byte is not erased!  (It should be unless the block is
               * bad)
               */

              else
                {
                  nerased  = 0;
                  iooffset = i + 1;
                }
            }
        }

      /* If we get here, then either (1) this block is not read-able, or
       * (2) we have looked at every byte in the block and did not find
       * any sequence of erased bytes long enough to hold the object.
       * Skip to the next, valid block.
       */

      volume->ioblock++;
      ret = nxffs_validblock(volume, &volume->ioblock);
      if (ret < 0)
        {
          ferr("ERROR: No more valid blocks\n");
          return ret;
        }

      volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
      volume->froffset = volume->ioblock * volume->geo.blocksize +
                         SIZEOF_NXFFS_BLOCK_HDR;
    }

  /* Return -ENOSPC if there is no erased memory left in the volume for
   * the object.
   */

  ferr("ERROR: Not enough memory left to hold the file header\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: nxffs_wrblkhdr
 *
 * Description:
 *   Write the block header information.  This is done (1) whenever the end-
 *   block is encountered and (2) also when the file is closed in order to
 *   flush the final block of data to FLASH.
 *
 * Input Parameters:
 *   volume - Describes the state of the NXFFS volume
 *   wrfile - Describes the state of the open file
 *
 * Returned Value:
 *   Zero is returned on success; Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_wrblkhdr(FAR struct nxffs_volume_s *volume,
                   FAR struct nxffs_wrfile_s *wrfile)
{
  FAR struct nxffs_data_s *dathdr;
  int ret;

  /* Write the data block header to memory */

  nxffs_ioseek(volume, wrfile->doffset);
  dathdr = (FAR struct nxffs_data_s *)&volume->cache[volume->iooffset];
  memcpy(dathdr->magic, g_datamagic, NXFFS_MAGICSIZE);
  nxffs_wrle32(dathdr->crc, 0);
  nxffs_wrle16(dathdr->datlen, wrfile->datlen);

  /* Update the entire data block CRC (including the header) */

  wrfile->crc = crc32(&volume->cache[volume->iooffset],
                      wrfile->datlen + SIZEOF_NXFFS_DATA_HDR);
  nxffs_wrle32(dathdr->crc, wrfile->crc);

  /* And write the data block to FLASH */

  ret = nxffs_wrcache(volume);
  if (ret < 0)
    {
      ferr("ERROR: nxffs_wrcache failed: %d\n", -ret);
      goto errout;
    }

  /* After the block has been successfully written to flash, update the inode
   * statistics and reset the write state.
   *
   * volume:
   *   froffset - The offset the next free FLASH region.  Set to just after
   *     the inode data block that we just wrote.  This is where we will
   *     begin the search for the next inode header or data block.
   */

  volume->froffset = (wrfile->doffset + wrfile->datlen +
                      SIZEOF_NXFFS_DATA_HDR);

  /* wrfile->file.entry:
   *   datlen:  Total file length accumulated so far.  When the file is
   *     closed, this will hold the file length.
   *   doffset: Offset to the first data block.  Only the offset to the
   *     first data block is saved.
   */

  wrfile->ofile.entry.datlen += wrfile->datlen;
  if (wrfile->ofile.entry.doffset == 0)
    {
      wrfile->ofile.entry.doffset = wrfile->doffset;
    }

  /* Return success */

  ret = OK;

errout:
  wrfile->crc     = 0;
  wrfile->doffset = 0;
  wrfile->datlen  = 0;
  return ret;
}
