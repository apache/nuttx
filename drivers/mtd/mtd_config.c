/****************************************************************************
 * drivers/mtd/mtd_config.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#include <errno.h>
#include <debug.h>
#include <nuttx/fs/fs.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd.h>
#include <nuttx/configdata.h>

#ifdef CONFIG_MTD_CONFIG

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mtdconfig_struct_s
{
  FAR struct mtd_dev_s *mtd;      /* Contained MTD interface */
  sem_t             exclsem;      /* Supports mutual exclusion */
  uint16_t          blocksize;    /* Size of blocks in contained MTD */
  uint16_t          erasesize;    /* Size of erase block  in contained MTD */
  size_t            nblocks;      /* Number of blocks available */
  size_t            neraseblocks; /* Number of erase blocks available */
  off_t             readoff;      /* Read offset (for hexdump) */
  FAR uint8_t       *buffer;      /* Temp block read buffer */
  uint8_t           erasedvalue;  /* Value of and erased byte on the MTD */
};

struct mtdconfig_header_s
{
  uint8_t     flags;        /* Entry control flags */
  uint16_t    id;           /* ID of the config data item */
  int         instance;     /* Instance of the item */
  size_t      len;          /* Length of the data block */
} packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mtdconfig_open(FAR struct file *filep);
static int     mtdconfig_close(FAR struct file *filep);
static ssize_t mtdconfig_read(FAR struct file *, FAR char *, size_t);
static ssize_t mtdconfig_ioctl(FAR struct file *, int, unsigned long);
#ifndef CONFIG_DISABLE_POLL
static int     mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations mtdconfig_fops =
{
  mtdconfig_open,  /* open */
  mtdconfig_close, /* close */
  mtdconfig_read,  /* read */
  0,               /* write */
  0,               /* seek */
  mtdconfig_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , mtdconfig_poll /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdconfig_readbytes
 *
 *    Reads bytes from the contained MTD device.  This will either usee
 *    the read function or if that is not available, the bread with a copy.
 *
 ****************************************************************************/

static int  mtdconfig_readbytes(FAR struct mtdconfig_struct_s *dev, int offset,
                                FAR uint8_t *pdata, int readlen)
{
  off_t  bytestoread = readlen;
  off_t  bytesthisblock, firstbyte;
  off_t  block, index;
  int    ret = OK;
  size_t bytes;

  /* Test if read interface supported.  If it is, use it directly */

  if ((dev->mtd->read == NULL) && (readlen < dev->blocksize))
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
          if (bytesthisblock < dev->blocksize || bytestoread < dev->blocksize)
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
 *    Writes bytes to the contained MTD device.  This will either usee
 *    the byte write function or if that is not available, the bwrite.
 *
 ****************************************************************************/

static int  mtdconfig_writebytes(FAR struct mtdconfig_struct_s *dev, int offset,
                                 FAR const uint8_t *pdata, int writelen)
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
      uint16_t  block, index;
      off_t     bytes_this_block;

      while (writelen)
        {
          /* Read existing data from the the block into the buffer */

          block = offset / dev->blocksize;
          ret = MTD_BREAD(dev->mtd, block, 1, dev->buffer);
          if (ret != 1)
            {
              ret = -EIO;
              goto errout;
            }

          bytes_this_block = offset - block * dev->blocksize;
          index = bytes_this_block;
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

          writelen -= bytes_this_block;
          pdata += bytes_this_block;
          offset += bytes_this_block;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_findfirstentry
 *
 *    Locates the first config entry, even if it is empty.
 *
 * Returns:
 *     offset to the start of the entry.
 *
 ****************************************************************************/

static int  mtdconfig_findfirstentry(FAR struct mtdconfig_struct_s *dev,
                                     FAR struct mtdconfig_header_s *phdr)
{
  off_t   offset = 2;
  uint8_t sig[2];
  bool    found = false;
  int     ret;

  mtdconfig_readbytes(dev, 0, sig, sizeof(sig));  /* Read the signature bytes */
  if (sig[0] != 'C' || sig[1] != 'D')
    {
      /* Config Data partition not formatted. */

      return 0;
    }

  /* Config is formatted.  Now loop until we find the first entry */

  while (!found)
    {
      /* Read headers until we find one that hasn't been released.
       */

      ret = mtdconfig_readbytes(dev, offset, (uint8_t *) phdr,
              sizeof(struct mtdconfig_header_s));
      if (ret != OK)
        {
          return 0;
        }

      /* Test if this header has been released */

      if (phdr->flags != dev->erasedvalue)
        {
          /* This entry has been released.  Advance to next entry */

          offset += sizeof(struct mtdconfig_header_s) + phdr->len;
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
 * Returns:
 *     offset to the start of the next entry.
 *
 ****************************************************************************/

static int  mtdconfig_findnextentry(FAR struct mtdconfig_struct_s *dev,
                                    off_t offset,
                                    FAR struct mtdconfig_header_s *phdr)
{
  bool found = false;
  int  ret;

  /* Loop until next entry found */

  while (!found)
    {
      /* Calculate offset of the next entry */

      offset += sizeof(struct mtdconfig_header_s) + phdr->len;

      /* Read next header */

      ret = mtdconfig_readbytes(dev, offset, (uint8_t *) phdr, sizeof(*phdr));
      if (ret != OK)
        {
          return 0;
        }

      /* Test if this header has is still active */

      if (phdr->flags == dev->erasedvalue)
        {
          found = true;
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
 * Returns:
 *     offset to the next available entry (after consolidation)..
 *
 ****************************************************************************/

static off_t  mtdconfig_ramconsolidate(FAR struct mtdconfig_struct_s *dev)
{
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
 * Returns:
 *     offset to the next available entry (after consolidation)..
 *
 ****************************************************************************/

static off_t  mtdconfig_consolidate(FAR struct mtdconfig_struct_s *dev)
{
  off_t       src_block, dst_block;
  off_t       src_offset, dst_offset;
  uint16_t    blkper, x, bytes, bytes_left_in_block;
  struct mtdconfig_header_s hdr;
  int         ret;
  uint8_t     sig[2];

  /* Prepare to copy block 0 to the last block (erase blocks) */

  src_block = 0;
  dst_block = dev->neraseblocks - 1;

  /* Ensure the last block is erased */

  MTD_ERASE(dev->mtd, dst_block, 1);
  blkper = dev->erasesize / dev->blocksize;
  dst_block *= blkper;            /* Convert to read/write blocks */

  /* Now copy block zero to last block */

  for (x = 0; x < blkper; x++)
    {
      ret = MTD_BREAD(dev->mtd, src_block, 1, dev->buffer);
      if (ret < 0)
        {
          /* I/O Error! */

          return 0;
        }

      ret = MTD_BWRITE(dev->mtd, dst_block, 1, dev->buffer);
      if (ret < 0)
        {
          /* I/O Error! */

          return 0;
        }
    }

  /* Erase block zero and write a format signature. */

  MTD_ERASE(dev->mtd, 0, 1);
  sig[0] = 'C';
  sig[1] = 'D';
  mtdconfig_writebytes(dev, 0, sig, sizeof(sig));

  /* Now consolidate entries */

  src_block = 1;
  dst_block = 0;
  src_offset = src_block * dev->erasesize;
  dst_offset = 2;

  while (src_block < dev->neraseblocks)
    {
      /* Scan all headers and move them to the src_offset */

retry_relocate:
      MTD_READ(dev->mtd, src_offset, sizeof(hdr), (uint8_t *) &hdr);
      if (hdr.flags == dev->erasedvalue)
        {
          /* Test if this entry will fit in the current destination block */

          bytes_left_in_block = dst_offset - dst_block * dev->erasesize;
          if (hdr.len + sizeof(hdr) > bytes_left_in_block)
            {
              /* We need to release the rest of the bytes in this block */

              hdr.flags = ~dev->erasedvalue;
              hdr.len = bytes_left_in_block - sizeof(hdr);
              MTD_WRITE(dev->mtd, dst_offset, sizeof(hdr), (uint8_t *) &hdr);

              /* Update control variables */

              dst_offset += bytes_left_in_block;
              dst_block++;
              DEBUGASSERT(dst_block != src_block);
              DEBUGASSERT(dst_offset == dst_block * dev->erasesize);

              /* Retry the relocate */

              goto retry_relocate;
            }

          /* Copy this entry to the destination */

          MTD_WRITE(dev->mtd, dst_offset, sizeof(hdr), (uint8_t *) &hdr);
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

              MTD_READ(dev->mtd, src_offset, bytes, dev->buffer);
              MTD_WRITE(dev->mtd, dst_offset, bytes, dev->buffer);

              /* Update control variables */

              hdr.len -= bytes;
              src_offset += bytes;
              dst_offset += bytes;
            }
        }
      else
        {
          /* This item has been released.  Skip it! */

          src_offset += sizeof(hdr) + hdr.len;
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
    }

  return 0;
}

/****************************************************************************
 * Name: mtdconfig_open
 ****************************************************************************/

static int  mtdconfig_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;
  int        ret = OK;

  /* Get exclusive access to the device */

  ret = sem_wait(&dev->exclsem);
  if (ret < 0)
    {
      ret = -errno;
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

  sem_post(&dev->exclsem);
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
  size_t    bytes;

  if (dev->readoff >= dev->neraseblocks * dev->erasesize)
    {
      return 0;
    }

  /* Read data from the file */

  bytes = MTD_READ(dev->mtd, dev->readoff, len, (uint8_t *) buffer);
  dev->readoff += bytes;
  return bytes;
}

/****************************************************************************
 * Name: mtdconfig_setconfig
 ****************************************************************************/

static int mtdconfig_setconfig(FAR struct mtdconfig_struct_s *dev,
                               FAR struct config_data_s *pdata)
{
  uint8_t   sig[2];       /* Format signature bytes ("CD") */
  char      retrycount = 0;
  int       ret = -ENOSYS;
  off_t     offset, bytes_left_in_block, bytes;
  uint16_t  block;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *) kmalloc(dev->blocksize);

  /* Read and vaidate the signature bytes */

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
      mtdconfig_writebytes(dev, 0, sig, sizeof(sig));

      /* Now go try to read the signature again (as verification) */

      retrycount++;
      goto retry;
    }

  /* Okay, the Config Data partition is formatted.  Check if the
   * config item being written is already in the database.  If it
   * is, we must mark it as obsolete before creating a new entry.
   */

  while (offset > 0 && (pdata->id != hdr.id || pdata->instance != hdr.instance))
    {
      /* This header doesn't match.  Test if it is the last hdr */

      if (hdr.id == ((dev->erasedvalue << 8) | dev->erasedvalue))
        {
          break;
        }

      /* Nope, not the last header.  Get the next one */

      offset = mtdconfig_findnextentry(dev, offset, &hdr);
    }

  /* Test if the header was found. */

  if (pdata->id == hdr.id && pdata->instance == hdr.instance)
    {
      /* Mark this entry as released */

      hdr.flags = ~dev->erasedvalue;
      mtdconfig_writebytes(dev, offset, &hdr.flags, sizeof(hdr.flags));
    }

  /* Now find a new entry for this config data */

  while (hdr.id != ((dev->erasedvalue << 8) | dev->erasedvalue))
    {
      /* Read the next entry */

      offset = mtdconfig_findnextentry(dev, offset, &hdr);
    }

  /* Test if a new entry was found */

  retrycount = 0;
  if (offset > 0)
    {
      /* Try to write the data to this entry.  We have to be
       * sure it will fit in the erase block because we don't
       * support data spanning erase blocks.
       */

retry_fit:
      block = offset / dev->erasesize;
      bytes_left_in_block = block * dev->erasesize - offset;
      if (bytes_left_in_block < pdata->len)
        {
          /* Data doesn't fit in the block!  Our only recourse is
           * to release the rest of the data in this erase block.
           */

          hdr.id = 0;
          hdr.len = bytes_left_in_block - sizeof(hdr);
          hdr.flags = ~dev->erasedvalue;
          mtdconfig_writebytes(dev, offset, (uint8_t *) &hdr, sizeof(hdr));

          /* Now we need to advance to the next erase block */

          offset += bytes_left_in_block;
          block++;
          if (dev->neraseblocks == 1)
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

              offset = mtdconfig_ramconsolidate(dev);
              retrycount++;
              goto retry_fit;
            }

          /* If we have 2 or more erase blocks, then we reserve one
           * block to perform non-RAM assited consolidation.
           */

          else if (block + 1 == dev->neraseblocks)
            {
              if (retrycount)
                {
                  /* Out of space! */

                  ret = -ENOMEM;
                  goto errout;
                }

              offset = mtdconfig_consolidate(dev);
              retrycount++;
              goto retry_fit;
            }
        }

      /* Validate that we have a non-zero offset.  We may have done a
       * consolidation above that resulted
       */

      if (offset)
        {
          /* Save the data at this entry */

          hdr.id = pdata->id;
          hdr.instance = pdata->instance;
          hdr.len = pdata->len;
          hdr.flags = dev->erasedvalue;
          mtdconfig_writebytes(dev, offset, (uint8_t *) &hdr, sizeof(hdr));
          bytes = mtdconfig_writebytes(dev, offset + sizeof(hdr), pdata->configdata,
                                       pdata->len);
          if (bytes != pdata->len)
            {
              /* Error writing data! */

              hdr.flags = ~dev->erasedvalue;
              mtdconfig_writebytes(dev, offset, (uint8_t *) &hdr, sizeof(hdr.flags));
              ret = -EIO;
              goto errout;
            }

          ret = OK;
        }
    }

errout:

  /* Free the buffer */

  kfree(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_getconfig
 ****************************************************************************/

static int mtdconfig_getconfig(FAR struct mtdconfig_struct_s *dev,
              FAR struct config_data_s *pdata)
{
  int    ret = -ENOSYS;
  off_t  offset, bytes_to_read;
  struct mtdconfig_header_s hdr;

  /* Allocate a temp block buffer */

  dev->buffer = (FAR uint8_t *) kmalloc(dev->blocksize);
  if (dev->buffer == NULL)
    {
      return -ENOMEM;
    }

  /* Get the offset of the first entry.  This will also check
   * the format signature bytes.
   */

  offset = mtdconfig_findfirstentry(dev, &hdr);
  while (offset > 0 && (pdata->id != hdr.id || pdata->instance != hdr.instance))
    {
      /* This header doesn't match.  Test if it is the last hdr */

      if (hdr.id == ((dev->erasedvalue << 8) | dev->erasedvalue))
        {
          break;
        }

      /* Nope, not the last header.  Get the next one */

      offset = mtdconfig_findnextentry(dev, offset, &hdr);
    }

  /* Test if the header was found. */

  if (pdata->id == hdr.id && pdata->instance == hdr.instance)
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

      ret = OK;
    }

errout:
  /* Free the buffer */

  kfree(dev->buffer);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_ioctl
 ****************************************************************************/

static int  mtdconfig_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdconfig_struct_s *dev = inode->i_private;
  FAR struct config_data_s *pdata;
  int   ret = -ENOSYS;

  switch (cmd)
    {
      case CFGDIOC_SETCONFIG:

        /* Set the config item */

        pdata = (FAR struct config_data_s *) arg;
        ret = mtdconfig_setconfig(dev, pdata);
        break;

      case CFGDIOC_GETCONFIG:

        /* Get the config item */

        pdata = (FAR struct config_data_s *) arg;
        ret = mtdconfig_getconfig(dev, pdata);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: mtdconfig_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  if (setup)
    {
      fds->revents |= (fds->events & (POLLIN|POLLOUT));
      if (fds->revents != 0)
        {
          sem_post(fds->sem);
        }
    }

  return OK;
}
#endif

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
  int ret = OK;
  struct mtdconfig_struct_s *dev;
  struct mtd_geometry_s geo;      /* Device geometry */

  dev = (struct mtdconfig_struct_s *)kmalloc(sizeof(struct mtdconfig_struct_s));
  if (dev)
    {
      /* Initialize the mtdconfig device structure */

      dev->mtd = mtd;
      sem_init(&dev->exclsem, 0, 1);

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
      if (ret < 0)
        {
          fdbg("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kfree(dev);
          goto errout;
        }

      dev->blocksize = geo.blocksize;
      dev->neraseblocks = geo.neraseblocks;
      dev->erasesize = geo.erasesize;
      dev->nblocks = geo.neraseblocks * geo.erasesize / geo.blocksize;
      dev->erasedvalue = 0xFF;  /* TODO:  fix this */

      (void)register_driver("/dev/config", &mtdconfig_fops, 0666, dev);
    }

errout:
  return ret;
}
#endif /* CONFIG_MTD_CONFIG */
