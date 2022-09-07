/****************************************************************************
 * drivers/mtd/mtd_config.c
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
 * dev_config provides an interface for saving and retrieving
 *            application configuration data to a standard MTD partition.
 *            It accepts an MTD pointer (presumable a partition) and
 *            publishes a /dev/config device file for accessing via
 *            defined ioctl calls to set and get config data.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/fs/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>

#ifdef CONFIG_MTD_CONFIG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define the current format version */

#ifdef CONFIG_MTD_CONFIG_NAMED
#  define CONFIGDATA_FORMAT_VERSION     1
#  define MTD_ERASED_ID(dev)            ((dev)->erasestate)
#else
#  define CONFIGDATA_FORMAT_VERSION     2
#  define MTD_ERASED_ID(dev)            (((dev)->erasestate << 8) | (dev)->erasestate)
#endif
#define CONFIGDATA_BLOCK_HDR_SIZE       3
#define MTD_ERASED_FLAGS(dev)           ((dev)->erasestate)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mtdconfig_struct_s
{
  FAR struct mtd_dev_s *mtd;  /* Contained MTD interface */
  sem_t        exclsem;       /* Supports mutual exclusion */
  uint32_t     blocksize;     /* Size of blocks in contained MTD */
  uint32_t     erasesize;     /* Size of erase block  in contained MTD */
  size_t       nblocks;       /* Number of blocks available */
  size_t       neraseblocks;  /* Number of erase blocks available */
  uint8_t      erasestate;    /* Erased value */
  off_t        readoff;       /* Read offset (for hexdump) */
  FAR uint8_t *buffer;        /* Temp block read buffer */
};

begin_packed_struct struct mtdconfig_header_s
{
  uint8_t      flags;         /* Entry control flags */
#ifdef CONFIG_MTD_CONFIG_NAMED
  char         name[CONFIG_MTD_CONFIG_NAME_LEN];
#else
  uint8_t      instance;      /* Instance of the item */
  uint16_t     id;            /* ID of the config data item */
#endif
  uint16_t     len;           /* Length of the data block */
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mtdconfig_open(FAR struct file *filep);
static int     mtdconfig_close(FAR struct file *filep);
static ssize_t mtdconfig_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int     mtdconfig_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);
static int     mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                              bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mtdconfig_fops =
{
  mtdconfig_open,  /* open */
  mtdconfig_close, /* close */
  mtdconfig_read,  /* read */
  NULL,            /* write */
  NULL,            /* seek */
  mtdconfig_ioctl, /* ioctl */
  mtdconfig_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdconfig_readbytes
 *
 *    Reads bytes from the contained MTD device.  This will either use
 *    the read function or if that is not available, the bread with a copy.
 *
 ****************************************************************************/

static int mtdconfig_readbytes(FAR struct mtdconfig_struct_s *dev,
                               int offset, FAR uint8_t *pdata,
                               int readlen)
{
  off_t  bytestoread = readlen;
  off_t  bytesthisblock;
  off_t  firstbyte;
  off_t  block;
  off_t  index;
  int    ret = OK;
  size_t bytes;

  /* Test if read interface supported.  If it is, use it directly. */

  if ((dev->mtd->read != NULL) && (readlen < dev->blocksize))
    {
      /* Read interface available.  Read directly to buffer */

      bytes = MTD_READ(dev->mtd, offset, readlen, pdata);
      if (bytes != readlen)
        {
          /* Error reading data! */

          ret = -EIO;
        }
    }
  else
    {
      /* Read interface not available, do a block read into our buffer */

      block = offset / dev->blocksize;
      firstbyte = offset - (block * dev->blocksize);
      bytesthisblock = dev->blocksize - firstbyte;
      if (bytesthisblock > readlen)
        {
          bytesthisblock = readlen;
        }

      index = 0;

      while (bytestoread > 0)
        {
          if (bytesthisblock < dev->blocksize ||
              bytestoread < dev->blocksize)
            {
              /* Copy to temp buffer first...don't need the whole block */

              bytes = MTD_BREAD(dev->mtd, block, 1, dev->buffer);
              if (bytes != 1)
                {
                  /* Error reading data!  */

                  ret = -EIO;
                  goto errout;
                }

              /* Copy data to the output buffer */

              memcpy(&pdata[index], &dev->buffer[firstbyte], bytesthisblock);
            }
          else
            {
              /* We are reading a whole block.  Read directly to buffer */

              bytes = MTD_BREAD(dev->mtd, block, 1, &pdata[index]);
              if (bytes != 1)
                {
                  /* Error reading data!  */

                  ret = -EIO;
                  goto errout;
                }
            }

          /* Update values for next block read */

          bytestoread -= bytesthisblock;
          index += bytesthisblock;
          bytesthisblock = dev->blocksize;
          if (bytesthisblock > bytestoread)
            {
              bytesthisblock = bytestoread;
            }

          firstbyte = 0;
          block++;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_writebytes
 *
 *    Writes bytes to the contained MTD device.  This will either use
 *    the byte write function or if that is not available, the bwrite.
 *
 ****************************************************************************/

static int mtdconfig_writebytes(FAR struct mtdconfig_struct_s *dev,
                                int offset, FAR const uint8_t *pdata,
                                int writelen)
{
  int ret = OK;

#ifdef CONFIG_MTD_BYTE_WRITE

  /* Test if this MTD device supports byte write */

  if (dev->mtd->write != NULL)
    {
      ret = MTD_WRITE(dev->mtd, offset, writelen, pdata);
    }
  else
#endif

  /* Perform the write using the block write method of the MTD */

    {
      uint16_t  block;
      uint16_t  index;
      off_t     bytes_this_block;
      off_t     bytes_written = 0;

      while (writelen > 0)
        {
          /* Read existing data from the block into the buffer */

          block = offset / dev->blocksize;
          ret = MTD_BREAD(dev->mtd, block, 1, dev->buffer);
          if (ret != 1)
            {
              ret = -EIO;
              goto errout;
            }

          index = offset - block * dev->blocksize;
          bytes_this_block = dev->blocksize - index;
          if (bytes_this_block > writelen)
            {
              bytes_this_block = writelen;
            }

          /* Now write data to the block */

          memcpy(&dev->buffer[index], pdata, bytes_this_block);
          ret = MTD_BWRITE(dev->mtd, block, 1, dev->buffer);
          if (ret != 1)
            {
              ret = -EIO;
              goto errout;
            }

          /* Update writelen, etc. */

          writelen      -= bytes_this_block;
          pdata         += bytes_this_block;
          offset        += bytes_this_block;
          bytes_written += bytes_this_block;
        }

      /* Return the number of bytes written */

      ret = bytes_written;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_findfirstentry
 *
 *    Locates the first config entry, even if it is empty.
 *
 * Returned Value:
 *     offset to the start of the entry.
 *
 ****************************************************************************/

static int  mtdconfig_findfirstentry(FAR struct mtdconfig_struct_s *dev,
                                     FAR struct mtdconfig_header_s *phdr)
{
  off_t     offset = CONFIGDATA_BLOCK_HDR_SIZE;
  uint8_t   sig[CONFIGDATA_BLOCK_HDR_SIZE];
  bool      found = false;
  int       ret;
  uint16_t  block;
  off_t     bytes_left_in_block;
  uint16_t  endblock;

  /* Read the signature bytes */

  ret = mtdconfig_readbytes(dev, 0, sig, sizeof(sig));
  if (ret != OK)
    {
      return 0;
    }

  if (sig[0] != 'C' || sig[1] != 'D' || sig[2] != CONFIGDATA_FORMAT_VERSION)
    {
      /* Config Data partition not formatted. */

      return 0;
    }

#ifdef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
  endblock = dev->neraseblocks;
#else
  if (dev->neraseblocks == 1)
    {
      endblock = 1;
    }
  else
    {
      endblock = dev->neraseblocks - 1;
    }
#endif

  /* Config is formatted.  Now loop until we find the first entry */

  while (!found)
    {
      /* Read headers until we find one that hasn't been released.
       */

      ret = mtdconfig_readbytes(dev, offset, (FAR uint8_t *)phdr,
              sizeof(struct mtdconfig_header_s));
      if (ret != OK)
        {
          return 0;
        }

      /* Test if this header has been released */

      if (phdr->flags != MTD_ERASED_FLAGS(dev))
        {
          /* This entry has been released.  Advance to next entry */

          offset += sizeof(struct mtdconfig_header_s) + phdr->len;
          block = offset / dev->erasesize;
          bytes_left_in_block = (block + 1) * dev->erasesize - offset;
          if (bytes_left_in_block <= sizeof(*phdr))
            {
              offset = (block + 1) * dev->erasesize +
                       CONFIGDATA_BLOCK_HDR_SIZE;
              if (block + 1 >= endblock)
                {
                  return 0;
                }
            }

          if (bytes_left_in_block == dev->erasesize)
            {
              /* Skip block header */

              offset += CONFIGDATA_BLOCK_HDR_SIZE;
            }
        }
      else
        {
          /* We found the first entry! */

          found = true;
        }
    }

  /* Return the offset of the first entry */

  return offset;
}

/****************************************************************************
 * Name: mtdconfig_findnextentry
 *
 *    Locates the next config entry starting at offset, even if it is empty.
 *
 * Returned Value:
 *     offset to the start of the next entry.
 *
 ****************************************************************************/

static int  mtdconfig_findnextentry(FAR struct mtdconfig_struct_s *dev,
                                    off_t offset,
                                    FAR struct mtdconfig_header_s *phdr,
                                    uint16_t size)
{
  bool      found = false;
  int       ret;
  uint16_t  block;
  uint16_t  bytes_left_in_block;
  uint16_t  endblock;

#ifdef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
  endblock = dev->neraseblocks;
#else
  if (dev->neraseblocks == 1)
    {
      endblock = 1;
    }
  else
    {
      endblock = dev->neraseblocks - 1;
    }
#endif

  /* Loop until next entry found */

  while (!found)
    {
      /* Calculate offset of the next entry */

      offset += sizeof(struct mtdconfig_header_s) + phdr->len;
      if (offset % dev->erasesize == 0)
        {
          offset += CONFIGDATA_BLOCK_HDR_SIZE;
        }

      if (offset >= endblock * dev->erasesize)
        {
          return 0;
        }

      /* Test if too close to the end of the erase block */

      block = offset / dev->erasesize;
      bytes_left_in_block = (block + 1) * dev->erasesize - offset;
      if (bytes_left_in_block <= sizeof(*phdr))
        {
          offset = (block + 1) * dev->erasesize + CONFIGDATA_BLOCK_HDR_SIZE;
          if (block + 1 >= endblock)
            {
              return 0;
            }
        }

      /* Read next header */

read_next:
      ret = mtdconfig_readbytes(dev, offset, (FAR uint8_t *)phdr,
                                sizeof(*phdr));
      if (ret != OK)
        {
          return 0;
        }

      /* Test if this header has is still active */

      if (phdr->flags == MTD_ERASED_FLAGS(dev))
        {
#ifdef CONFIG_MTD_CONFIG_NAMED
          if (phdr->name[0] == MTD_ERASED_ID(dev))
#else
          if (phdr->id == MTD_ERASED_ID(dev))
#endif
            {
              /* If we are searching for free space, then check
               * if this space is large enough.  If it is, then
               * we are done.
               */

              block = offset / dev->erasesize;
              bytes_left_in_block = (block + 1) * dev->erasesize - offset;
              if (size > 0 && bytes_left_in_block >= size + sizeof(*phdr))
                {
                  /* Free entry of large enough size found */

                  found = true;
                  break;
                }

              /* Advance to next erase block and continue search */

              offset = (block + 1) * dev->erasesize +
                       CONFIGDATA_BLOCK_HDR_SIZE;

              if (block + 1 >= endblock)
                {
                  return 0;
                }

              /* Don't return to top of loop since we manually adjusted
               * the offset, and don't want to go to the next entry.
               */

              goto read_next;
            }
          else
            {
              /* If we are searching for existing, active entries
               * only, then we are done ... we found one.
               */

              if (size == 0)
                {
                  found = true;
                }
            }
        }
    }

  return offset;
}

/****************************************************************************
 * Name: mtdconfig_ramconsolidate
 *
 *    Attempts to perform a RAM based consolidation of released
 *    items.  This requires a large enough free RAM block.  This
 *    method of consolidation is used when only a single erase
 *    block is available in the partition.
 *
 * Returned Value:
 *     Offset to the next available entry (after consolidation).
 *
 ****************************************************************************/

static off_t mtdconfig_ramconsolidate(FAR struct mtdconfig_struct_s *dev)
{
  FAR uint8_t *pbuf;
  FAR struct  mtdconfig_header_s *phdr;
  struct      mtdconfig_header_s  hdr;
  uint16_t    src_block = 0;
  uint16_t    dst_block = 0;
  uint16_t    blkper;
  off_t       dst_offset = CONFIGDATA_BLOCK_HDR_SIZE;
  off_t       src_offset = CONFIGDATA_BLOCK_HDR_SIZE;
  off_t       bytes_left_in_block;
  uint8_t     sig[CONFIGDATA_BLOCK_HDR_SIZE];
  int         ret;

  /* Allocate a consolidation buffer */

  pbuf = (FAR uint8_t *)kmm_malloc(dev->erasesize);
  if (pbuf == NULL)
    {
      /* Unable to allocate buffer, can't consolidate! */

      return 0;
    }

  /* Loop for all blocks and consolidate them */

  blkper = dev->erasesize / dev->blocksize;
  while (src_block < dev->neraseblocks)
    {
      /* Point to beginning of pbuf and read the next erase block */

      ret = MTD_BREAD(dev->mtd, src_block * blkper, blkper, pbuf);
      if (ret < 0)
        {
          /* Error doing block read */

          goto errout;
        }

      /* Now erase the block */

      ret = MTD_ERASE(dev->mtd, src_block, 1);
      if (ret < 0)
        {
          /* Error erasing the block */

          goto errout;
        }

      /* If this is block zero, then write a format signature */

      if (src_block == 0)
        {
          sig[0] = 'C';
          sig[1] = 'D';
          sig[2] = CONFIGDATA_FORMAT_VERSION;

          ret = mtdconfig_writebytes(dev, 0, sig, sizeof(sig));
          if (ret != sizeof(sig))
            {
              /* Cannot write even the signature. */

              ret = -EIO;
              goto errout;
            }
        }

      /* Copy active items back to the MTD device. */

      while (src_offset < dev->erasesize)
        {
          phdr = (FAR struct mtdconfig_header_s *) &pbuf[src_offset];
#ifdef CONFIG_MTD_CONFIG_NAMED
          if (phdr->name[0] == MTD_ERASED_ID(dev))
#else
          if (phdr->id == MTD_ERASED_ID(dev))
#endif
            {
              /* No more data in this erase block. */

              src_offset = dev->erasesize;
              continue;
            }

          if (phdr->flags == MTD_ERASED_FLAGS(dev))
            {
              /* This is an active entry.  Copy it.  Check if it
               * fits in the current destination block.
               */

              bytes_left_in_block = (dst_block + 1) * dev->erasesize -
                dst_offset;
              if (bytes_left_in_block < sizeof(*phdr) + phdr->len)
                {
                  /* Item won't fit in the destination block.  Move to
                   * the next block.
                   */

                  dst_block++;
                  dst_offset = dst_block * dev->erasesize +
                               CONFIGDATA_BLOCK_HDR_SIZE;

                  /* Test for program bug.  We shouldn't ever overflow
                   * even if no entries were inactive.
                   */

                  DEBUGASSERT(dst_block != dev->neraseblocks);
                }

              /* Now write the item to the current dst_offset location. */

              ret = mtdconfig_writebytes(dev, dst_offset,
                                         (FAR uint8_t *)phdr, sizeof(hdr));
              if (ret != sizeof(hdr))
                {
                  /* I/O Error! */

                  ret = -EIO;
                  goto errout;
                }

              dst_offset += sizeof(hdr);
              ret = mtdconfig_writebytes(dev, dst_offset,
                                         &pbuf[src_offset + sizeof(hdr)],
                                         phdr->len);
              if (ret != phdr->len)
                {
                  /* I/O Error! */

                  ret = -EIO;
                  goto errout;
                }

              dst_offset += phdr->len;

              /* Test if enough space in dst block for another header */

              if ((dst_offset + sizeof(hdr) >= (dst_block + 1) *
                  dev->erasesize) || (dst_offset == (dst_block + 1) *
                  dev->erasesize))
                {
                  dst_block++;
                  dst_offset = dst_block * dev->erasesize +
                               CONFIGDATA_BLOCK_HDR_SIZE;
                }
            }

          /* Increment past the current source item */

          src_offset += sizeof(hdr) + phdr->len;
          if (src_offset + sizeof(hdr) > dev->erasesize)
            {
              src_offset = dev->erasesize;
            }

          DEBUGASSERT(src_offset <= dev->erasesize);
        }

      /* Increment to next source block */

      src_block++;
      src_offset = CONFIGDATA_BLOCK_HDR_SIZE;
    }

  kmm_free(pbuf);
  return dst_offset;

errout:
  kmm_free(pbuf);
  ferr("ERROR: fail ram consolidate: %d\n", ret);
  return 0;
}

/****************************************************************************
 * Name: mtdconfig_consolidate
 *
 *    Performs consolidation by writing erase block zero to the
 *    reserved block at the end, erasing block zero, then
 *    walking through each item and copying them to the newly
 *    erased block.  It erases all blocks to the end of the
 *    partition as it goes.
 *
 * Returned Value:
 *     Offset to the next available entry (after consolidation).
 *
 ****************************************************************************/

#ifndef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
static off_t  mtdconfig_consolidate(FAR struct mtdconfig_struct_s *dev)
{
  off_t       src_block;
  off_t       dst_block;
  off_t       src_offset;
  off_t       dst_offset;
  uint16_t    blkper;
  uint16_t    x;
  uint16_t    bytes;
  uint16_t    bytes_left_in_block;
  struct mtdconfig_header_s hdr;
  int         ret;
  uint8_t     sig[CONFIGDATA_BLOCK_HDR_SIZE];
  FAR uint8_t *pbuf;

  /* Prepare to copy block 0 to the last block (erase blocks) */

  src_block = 0;
  dst_block = dev->neraseblocks - 1;

  /* Ensure the last block is erased */

  MTD_ERASE(dev->mtd, dst_block, 1);
  blkper = dev->erasesize / dev->blocksize;
  dst_block *= blkper;            /* Convert to read/write blocks */

  /* Allocate a small buffer for moving data */

  pbuf = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (pbuf == NULL)
    {
      return 0;
    }

  /* Now copy block zero to last block */

  for (x = 0; x < blkper; x++)
    {
      ret = MTD_BREAD(dev->mtd, src_block++, 1, dev->buffer);
      if (ret < 0)
        {
          /* I/O Error! */

          goto errout;
        }

      ret = MTD_BWRITE(dev->mtd, dst_block++, 1, dev->buffer);
      if (ret < 0)
        {
          /* I/O Error! */

          goto errout;
        }
    }

  /* Erase block zero and write a format signature. */

  MTD_ERASE(dev->mtd, 0, 1);
  sig[0] = 'C';
  sig[1] = 'D';
  sig[2] = CONFIGDATA_FORMAT_VERSION;

  ret = mtdconfig_writebytes(dev, 0, sig, sizeof(sig));
  if (ret != sizeof(sig))
    {
      /* Cannot write even the signature. */

      ret = -EIO;
      goto errout;
    }

  /* Now consolidate entries. */

  src_block = 1;
  dst_block = 0;
  src_offset = src_block * dev->erasesize + CONFIGDATA_BLOCK_HDR_SIZE;
  dst_offset = CONFIGDATA_BLOCK_HDR_SIZE;

  while (src_block < dev->neraseblocks)
    {
      /* Scan all headers and move them to the src_offset */

retry_relocate:
      bytes = MTD_READ(dev->mtd, src_offset,
                       sizeof(hdr), (FAR uint8_t *)&hdr);
      if (bytes != sizeof(hdr))
        {
          /* I/O Error! */

          ret = -EIO;
          goto errout;
        }

      if (hdr.flags == MTD_ERASED_FLAGS(dev))
        {
          /* Test if the source entry is active or if we are at the end
           * of data for this erase block.
           */

#ifdef CONFIG_MTD_CONFIG_NAMED
          if (hdr.name[0] == MTD_ERASED_ID(dev))
#else
          if (hdr.id == MTD_ERASED_ID(dev))
#endif
            {
              /* No more data in this erase block.  Advance to the
               * next one.
               */

              src_offset = (src_block + 1) * dev->erasesize +
                           CONFIGDATA_BLOCK_HDR_SIZE;
            }
          else
            {
              /* Test if this entry will fit in the current destination
               * block.
               */

              bytes_left_in_block = (dst_block + 1) * dev->erasesize -
                                    dst_offset;
              if (hdr.len + sizeof(hdr) > bytes_left_in_block)
                {
                  /* Item doesn't fit in the block. Advance to the next
                   * one.
                   */

                  /* Update control variables */

                  dst_block++;
                  dst_offset = dst_block * dev->erasesize +
                               CONFIGDATA_BLOCK_HDR_SIZE;

                  DEBUGASSERT(dst_block != src_block);

                  /* Retry the relocate */

                  goto retry_relocate;
                }

              /* Copy this entry to the destination */

              ret = mtdconfig_writebytes(dev, dst_offset,
                                         (FAR uint8_t *)&hdr, sizeof(hdr));
              if (ret != sizeof(hdr))
                {
                  /* I/O Error! */

                  ret = -EIO;
                  goto errout;
                }

              src_offset += sizeof(hdr);
              dst_offset += sizeof(hdr);

              /* Now copy the data */

              while (hdr.len)
                {
                  bytes = hdr.len;
                  if (bytes > dev->blocksize)
                    {
                      bytes = dev->blocksize;
                    }

                  /* Move the data. */

                  ret = mtdconfig_readbytes(dev, src_offset, pbuf, bytes);
                  if (ret != OK)
                    {
                      /* I/O Error! */

                      ret = -EIO;
                      goto errout;
                    }

                  ret = mtdconfig_writebytes(dev, dst_offset, pbuf, bytes);
                  if (ret != bytes)
                    {
                      /* I/O Error! */

                      ret = -EIO;
                      goto errout;
                    }

                  /* Update control variables */

                  hdr.len -= bytes;
                  src_offset += bytes;
                  dst_offset += bytes;
                }
            }
        }
      else
        {
          /* This item has been released.  Skip it! */

          src_offset += sizeof(hdr) + hdr.len;
          if (src_offset + sizeof(hdr) >= (src_block + 1) * dev->erasesize ||
              src_offset == (src_block + 1) * dev->erasesize)
            {
              /* No room left at end of source block */

              src_offset = (src_block + 1) * dev->erasesize +
                           CONFIGDATA_BLOCK_HDR_SIZE;
            }
        }

      /* Test if we are out of space in the src block */

      if (src_offset + sizeof(hdr) >= (src_block + 1) * dev->erasesize)
        {
          /* No room at end of src block for another header.  Go to next
           * source block.
           */

          src_offset = (src_block + 1) * dev->erasesize +
                       CONFIGDATA_BLOCK_HDR_SIZE;
        }

      /* Test if we advanced to the next block.  If we did, then erase the
       * old block.
       */

      if (src_block != src_offset / dev->erasesize)
        {
          /* Erase the block ... we have emptied it */

          MTD_ERASE(dev->mtd, src_block, 1);
          src_block++;
        }

      /* Test if we are out of space in the dst block */

      if (dst_offset + sizeof(hdr) >= (dst_block + 1) * dev->erasesize)
        {
          /* No room at end of dst block for another header.
           * Go to next block.
           */

          dst_block++;
          dst_offset = dst_block * dev->erasesize +
                       CONFIGDATA_BLOCK_HDR_SIZE;
          DEBUGASSERT(dst_block != src_block);
        }
    }

  kmm_free(pbuf);
  return dst_offset;

errout:
  kmm_free(pbuf);
  ferr("ERROR: fail consolidate: %d\n", ret);
  return 0;
}

#endif /* CONFIG_MTD_CONFIG_RAM_CONSOLIDATE */

/****************************************************************************
 * Name: mtdconfig_open
 ****************************************************************************/

static int  mtdconfig_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;
  int ret;

  /* Get exclusive access to the device */

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      ferr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout;
    }

  dev->readoff = 0;

errout:
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_close
 ****************************************************************************/

static int  mtdconfig_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;

  /* Release exclusive access to the device */

  nxsem_post(&dev->exclsem);
  return OK;
}

/****************************************************************************
 * Name: mtdconfig_read
 ****************************************************************************/

static ssize_t mtdconfig_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;
  size_t bytes;

  if (dev->readoff >= dev->neraseblocks * dev->erasesize)
    {
      return 0;
    }

  /* Read data from the file */

  bytes = MTD_READ(dev->mtd, dev->readoff, len, (FAR uint8_t *)buffer);
  if (bytes != len)
    {
      return -EIO;
    }

  dev->readoff += bytes;
  return bytes;
}

/****************************************************************************
 * Name: mtdconfig_findentry
 ****************************************************************************/

static int mtdconfig_findentry(FAR struct mtdconfig_struct_s *dev,
                               off_t offset,
                               FAR struct config_data_s *pdata,
                               FAR struct mtdconfig_header_s *phdr)
{
  uint16_t  endblock;
  int       ret;

#ifdef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
  endblock = dev->neraseblocks;
#else
  if (dev->neraseblocks == 1)
    {
      endblock = 1;
    }
  else
    {
      endblock = dev->neraseblocks - 1;
    }
#endif

#ifdef CONFIG_MTD_CONFIG_NAMED
  while (offset > 0 && strcmp(pdata->name, phdr->name) != 0)
#else
  while (offset > 0 && (pdata->id != phdr->id ||
         pdata->instance != phdr->instance))
#endif
    {
#ifdef CONFIG_MTD_CONFIG_NAMED
      if (phdr->name[0] == MTD_ERASED_ID(dev))
#else
      if (phdr->id == MTD_ERASED_ID(dev))
#endif
        {
          /* Advance to the next block and continue the search */

          offset = (offset + dev->erasesize) / dev->erasesize;
          offset = offset * dev->erasesize + CONFIGDATA_BLOCK_HDR_SIZE;
          if (offset >= endblock * dev->erasesize)
            {
              /* Entry doesn't exist on the device */

              offset = 0;
              break;
            }

          /* Read the 1st header from the next block */

          ret = mtdconfig_readbytes(dev, offset, (FAR uint8_t *)phdr,
                                    sizeof(*phdr));
          if (ret != OK)
            {
              /* Error reading the data */

              offset = 0;
              break;
            }

          if (phdr->flags == MTD_ERASED_FLAGS(dev))
            {
              continue;
            }
        }

      /* Nope, not the last header.  Get the next one */

      offset = mtdconfig_findnextentry(dev, offset, phdr, 0);
    }

  return offset;
}

/****************************************************************************
 * Name: mtdconfig_setconfig
 ****************************************************************************/

static int mtdconfig_setconfig(FAR struct mtdconfig_struct_s *dev,
                               FAR struct config_data_s *pdata)
{
  uint8_t   sig[CONFIGDATA_BLOCK_HDR_SIZE];       /* Format signature bytes ("CD") */
  char      retrycount = 0;
  int       ret = -ENOSYS;
  off_t     offset;
  off_t     bytes_left_in_block;
  off_t     bytes;
  uint16_t  block;
  struct mtdconfig_header_s hdr;
  uint8_t   ram_consolidate;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  /* Read and validate the signature bytes */

retry:
  offset = mtdconfig_findfirstentry(dev, &hdr);
  if (offset == 0)
    {
      /* Config Data partition not formatted. */

      if (retrycount)
        {
          ret = -ENOSYS;
          goto errout;
        }

      /* Try to format the config partition */

      ret = MTD_IOCTL(dev->mtd, MTDIOC_BULKERASE, 0);
      if (ret < 0)
        {
          goto errout;
        }

      /* Write a format signature */

      sig[0] = 'C';
      sig[1] = 'D';
      sig[2] = CONFIGDATA_FORMAT_VERSION;

      ret = mtdconfig_writebytes(dev, 0, sig, sizeof(sig));
      if (ret != sizeof(sig))
        {
          /* Cannot write even the signature. */

          ret = -EIO;
          goto errout;
        }

      /* Now go try to read the signature again (as verification) */

      retrycount++;
      goto retry;
    }

  /* Okay, the Config Data partition is formatted.  Check if the
   * config item being written is already in the database.  If it
   * is, we must mark it as obsolete before creating a new entry.
   */

  offset = mtdconfig_findentry(dev, offset, pdata, &hdr);

  /* Test if the header was found. */

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (offset > 0 && strcmp(pdata->name, hdr.name) == 0)
#else
  if (offset > 0 && pdata->id == hdr.id && pdata->instance ==
      hdr.instance)
#endif
    {
      /* Mark this entry as released */

      hdr.flags = (uint8_t)~MTD_ERASED_FLAGS(dev);
      mtdconfig_writebytes(dev, offset, &hdr.flags, sizeof(hdr.flags));
    }

  /* Test if the new length is zero.  If it is, then we are
   * deleting the entry.
   */

  if (pdata->len == 0)
    {
      ret = OK;
      goto errout;
    }

  /* Now find a new entry for this config data */

  retrycount = 0;

retry_find:
  offset = mtdconfig_findfirstentry(dev, &hdr);

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (offset > 0 && hdr.name[0] == MTD_ERASED_ID(dev))
#else
  if (offset > 0 && hdr.id == MTD_ERASED_ID(dev))
#endif
    {
      block = offset / dev->erasesize;
      bytes_left_in_block = (block + 1) * dev->erasesize - offset;
      if (bytes_left_in_block < sizeof(hdr) + pdata->len)
        {
          /* Simulate an active block to search for the next one
           * in the code below.
           */

#ifdef CONFIG_MTD_CONFIG_NAMED
          hdr.name[0] = 1;
#else
          hdr.id = 1;
#endif
        }
    }

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (hdr.name[0] != MTD_ERASED_ID(dev))
#else
  if (hdr.id != MTD_ERASED_ID(dev))
#endif
    {
      /* Read the next entry */

      offset = mtdconfig_findnextentry(dev, offset, &hdr, pdata->len);
      if (offset == 0)
        {
          /* No free entries left on device! */

#ifdef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
          ram_consolidate = 1;
#else
          ram_consolidate = dev->neraseblocks == 1;
#endif
          if (ram_consolidate)
            {
              /* If we only have 1 erase block, then we must do a RAM
               * assisted consolidation of released entries.
               */

              if (retrycount)
                {
                  /* Out of space! */

                  ret = -ENOMEM;
                  goto errout;
                }

              mtdconfig_ramconsolidate(dev);
              retrycount++;
              goto retry_find;
            }
#ifndef CONFIG_MTD_CONFIG_RAM_CONSOLIDATE
          else
            {
              if (retrycount)
                {
                  /* Out of space! */

                  ret = -ENOMEM;
                  goto errout;
                }

              mtdconfig_consolidate(dev);
              retrycount++;
              goto retry_find;
            }
#endif
        }
    }

  /* Test if a new entry was found */

  if (offset > 0)
    {
      /* Save the data at this entry */

#ifdef CONFIG_MTD_CONFIG_NAMED
      strcpy(hdr.name, pdata->name);
#else
      hdr.id = pdata->id;
      hdr.instance = pdata->instance;
#endif
      hdr.len = pdata->len;
      hdr.flags = MTD_ERASED_FLAGS(dev);

      ret = mtdconfig_writebytes(dev, offset,
                                 (FAR uint8_t *)&hdr, sizeof(hdr));
      if (ret != sizeof(hdr))
        {
          /* Cannot write even header! */

          ret = -EIO;
          goto errout;
        }

      bytes = mtdconfig_writebytes(dev, offset + sizeof(hdr),
                                   pdata->configdata, pdata->len);
      if (bytes != pdata->len)
        {
          /* Error writing data! */

          hdr.flags = MTD_ERASED_FLAGS(dev);
          mtdconfig_writebytes(dev, offset, (FAR uint8_t *)&hdr,
                               sizeof(hdr.flags));
          ret = -EIO;
          goto errout;
        }

      ret = OK;
    }

errout:

  /* Free the buffer */

  kmm_free(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_getconfig
 ****************************************************************************/

static int mtdconfig_getconfig(FAR struct mtdconfig_struct_s *dev,
                               FAR struct config_data_s *pdata)
{
  int    ret = -ENOSYS;
  off_t  offset;
  off_t  bytes_to_read;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  /* Get the offset of the first entry.  This will also check
   * the format signature bytes.
   */

  offset = mtdconfig_findfirstentry(dev, &hdr);
  offset = mtdconfig_findentry(dev, offset, pdata, &hdr);

  /* Test if the header was found. */

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (offset > 0 && strcmp(pdata->name, hdr.name) == 0)
#else
  if (offset > 0 && (pdata->id == hdr.id && pdata->instance == hdr.instance))
#endif
    {
      /* Entry found.  Read the data */

      bytes_to_read = hdr.len;
      if (bytes_to_read > pdata->len)
        {
          bytes_to_read = pdata->len;
        }

      /* Perform the read */

      ret = mtdconfig_readbytes(dev, offset + sizeof(hdr), pdata->configdata,
                                bytes_to_read);
      if (ret != OK)
        {
          /* Error reading the data */

          ret = -EIO;
          goto errout;
        }

      /* Set return data length to match the config item length */

      pdata->len = hdr.len;
      ret = OK;
    }

errout:

  /* Free the buffer */

  kmm_free(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_deleteconfig
 ****************************************************************************/

static int mtdconfig_deleteconfig(FAR struct mtdconfig_struct_s *dev,
                                  FAR struct config_data_s *pdata)
{
  int    ret = -ENOENT;
  off_t  offset;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  /* Get the offset of the first entry.  This will also check
   * the format signature bytes.
   */

  offset = mtdconfig_findfirstentry(dev, &hdr);
  offset = mtdconfig_findentry(dev, offset, pdata, &hdr);

  /* Test if the header was found. */

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (offset > 0 && strcmp(pdata->name, hdr.name) == 0)
#else
  if (offset > 0 && (pdata->id == hdr.id && pdata->instance == hdr.instance))
#endif
    {
      /* Entry found.  Mark this entry as released */

      hdr.flags = (uint8_t)~MTD_ERASED_FLAGS(dev);
      mtdconfig_writebytes(dev, offset, &hdr.flags, sizeof(hdr.flags));

      ret = OK;
    }

  /* Free the buffer */

  kmm_free(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_firstconfig
 ****************************************************************************/

static int mtdconfig_firstconfig(FAR struct mtdconfig_struct_s *dev,
                                 FAR struct config_data_s *pdata)
{
  int    ret = -ENOENT;
  off_t bytes_to_read;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  dev->readoff = mtdconfig_findfirstentry(dev, &hdr);

  /* Test if the config item is valid */

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (dev->readoff != 0 &&
      hdr.name[0] != MTD_ERASED_ID(dev))
#else
  if (dev->readoff != 0 && hdr.id != MTD_ERASED_ID(dev))
#endif
    {
      /* Perform the read */

      bytes_to_read = hdr.len;
      if (bytes_to_read > pdata->len)
        {
          bytes_to_read = pdata->len;
        }

      ret = mtdconfig_readbytes(dev, dev->readoff + sizeof(hdr),
                                pdata->configdata, bytes_to_read);
      if (ret < 0)
        {
          goto errout;
        }

      /* Set other return data items */

#ifdef CONFIG_MTD_CONFIG_NAMED
      strcpy(pdata->name, hdr.name);
#else
      pdata->id = hdr.id;
      pdata->instance = hdr.instance;
#endif
      pdata->len = bytes_to_read;
    }
  else
    {
      ret = -ENOENT;
    }

errout:

  /* Free the buffer */

  kmm_free(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_nextconfig
 ****************************************************************************/

static int mtdconfig_nextconfig(FAR struct mtdconfig_struct_s *dev,
                                FAR struct config_data_s *pdata)
{
  int    ret = -ENOENT;
  off_t bytes_to_read;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *)kmm_malloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  ret = mtdconfig_readbytes(dev, dev->readoff, (FAR uint8_t *)&hdr,
                            sizeof(hdr));
  if (ret < 0)
    {
      goto errout;
    }

  dev->readoff = mtdconfig_findnextentry(dev, dev->readoff, &hdr, 0);

  /* Test if the config item is valid */

#ifdef CONFIG_MTD_CONFIG_NAMED
  if (dev->readoff != 0 &&
      hdr.name[0] != MTD_ERASED_ID(dev))
#else
  if (dev->readoff != 0 && hdr.id != MTD_ERASED_ID(dev))
#endif
    {
      /* Test if this is an empty slot */

      bytes_to_read = hdr.len;
      if (bytes_to_read > pdata->len)
        {
          bytes_to_read = pdata->len;
        }

      /* Read the config item data */

      ret = mtdconfig_readbytes(dev, dev->readoff + sizeof(hdr),
                                pdata->configdata, bytes_to_read);
      if (ret < 0)
        {
          goto errout;
        }

#ifdef CONFIG_MTD_CONFIG_NAMED
      strcpy(pdata->name, hdr.name);
#else
      pdata->id = hdr.id;
      pdata->instance = hdr.instance;
#endif
      pdata->len = bytes_to_read;
    }
  else
    {
      ret = -ENOENT;
    }

errout:

  /* Free the buffer */

  kmm_free(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_ioctl
 ****************************************************************************/

static int mtdconfig_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;
  FAR struct config_data_s *pdata;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case CFGDIOC_SETCONFIG:

        /* Set the config item */

        pdata = (FAR struct config_data_s *)arg;
        ret = mtdconfig_setconfig(dev, pdata);
        break;

      case CFGDIOC_GETCONFIG:

        /* Get the config item */

        pdata = (FAR struct config_data_s *)arg;
        ret = mtdconfig_getconfig(dev, pdata);
        break;

      case CFGDIOC_DELCONFIG:

        /* Set the config item */

        pdata = (FAR struct config_data_s *)arg;
        ret = mtdconfig_deleteconfig(dev, pdata);
        break;

      case CFGDIOC_FIRSTCONFIG:

        /* Get the the first config item */

        pdata = (FAR struct config_data_s *)arg;
        ret = mtdconfig_firstconfig(dev, pdata);
        break;

      case CFGDIOC_NEXTCONFIG:

        /* Get the next config item */

        pdata = (FAR struct config_data_s *)arg;
        ret = mtdconfig_nextconfig(dev, pdata);
        break;

      case MTDIOC_BULKERASE:

        /* Call the MTD's ioctl for this */

        ret = MTD_IOCTL(dev->mtd, cmd, arg);

        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mtdconfig_poll
 ****************************************************************************/

static int mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN | POLLOUT));
      if (fds->revents != 0)
        {
          nxsem_post(fds->sem);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdconfig_register
 *
 * Description:
 *   Register a /dev/config device backed by an MTD
 *
 ****************************************************************************/

int mtdconfig_register(FAR struct mtd_dev_s *mtd)
{
  int ret = -ENOMEM;
  struct mtdconfig_struct_s *dev;
  struct mtd_geometry_s geo;      /* Device geometry */

  dev = (struct mtdconfig_struct_s *)
    kmm_malloc(sizeof(struct mtdconfig_struct_s));
  if (dev != NULL)
    {
      /* Initialize the mtdconfig device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&geo));
      if (ret < 0)
        {
          ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kmm_free(dev);
          goto errout;
        }

      dev->blocksize = geo.blocksize;
      dev->neraseblocks = geo.neraseblocks;
      dev->erasesize = geo.erasesize;
      dev->nblocks = geo.neraseblocks * geo.erasesize / geo.blocksize;

      /* And query the erase state */

      ret = MTD_IOCTL(mtd, MTDIOC_ERASESTATE,
                      (unsigned long)((uintptr_t)&dev->erasestate));
      if (ret < 0)
        {
          ferr("ERROR: MTD ioctl(MTDIOC_ERASESTATE) failed: %d\n", ret);
          kmm_free(dev);
          goto errout;
        }

      nxsem_init(&dev->exclsem, 0, 1);
      register_driver("/dev/config", &mtdconfig_fops, 0666, dev);
    }

errout:
  return ret;
}
#endif /* CONFIG_MTD_CONFIG */
