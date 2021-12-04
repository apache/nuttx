/****************************************************************************
 * drivers/mtd/filemtd.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_FILEMTD_BLOCKSIZE
#  define CONFIG_FILEMTD_BLOCKSIZE 512
#endif

#ifndef CONFIG_FILEMTD_ERASESIZE
#  define CONFIG_FILEMTD_ERASESIZE 4096
#endif

#ifndef CONFIG_FILEMTD_ERASESTATE
#  define CONFIG_FILEMTD_ERASESTATE 0xff
#endif

#if CONFIG_FILEMTD_ERASESTATE != 0xff && CONFIG_FILEMTD_ERASESTATE != 0x00
#  error "Unsupported value for CONFIG_FILEMTD_ERASESTATE"
#endif

#if CONFIG_FILEMTD_BLOCKSIZE > CONFIG_FILEMTD_ERASESIZE
#  error "Must have CONFIG_FILEMTD_BLOCKSIZE <= CONFIG_FILEMTD_ERASESIZE"
#endif

#undef  FILEMTD_BLKPER
#define FILEMTD_BLKPER (CONFIG_FILEMTD_ERASESIZE/CONFIG_FILEMTD_BLOCKSIZE)

#if FILEMTD_BLKPER*CONFIG_FILEMTD_BLOCKSIZE != CONFIG_FILEMTD_ERASESIZE
#  error "CONFIG_FILEMTD_ERASESIZE must be an even multiple of CONFIG_FILEMTD_BLOCKSIZE"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct file_dev_s.
 */

struct file_dev_s
{
  struct mtd_dev_s mtd;        /* MTD device */
  struct file      mtdfile;
  size_t           nblocks;    /* Number of erase blocks */
  size_t           offset;     /* Offset from start of file */
  size_t           erasesize;  /* Offset from start of file */
  size_t           blocksize;  /* Offset from start of file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t filemtd_read(FAR struct file_dev_s *priv,
                 FAR unsigned char *buffer, size_t offsetbytes,
                 unsigned int nbytes);
static ssize_t filemtd_write(FAR struct file_dev_s *priv, size_t offset,
                 FAR const void *src, size_t len);

/* MTD driver methods */

static int     filemtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t filemtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR uint8_t *buf);
static ssize_t filemtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR const uint8_t *buf);
static ssize_t filemtd_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t file_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR const uint8_t *buf);
#endif
static int     filemtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: filemtd_write
 ****************************************************************************/

static ssize_t filemtd_write(FAR struct file_dev_s *priv, size_t offset,
                             FAR const void *src, size_t len)
{
  FAR const uint8_t *pin  = (FAR const uint8_t *)src;
  FAR uint8_t       *pout;
  char               buf[128];
  int                buflen = 0;
  uint8_t            oldvalue;
  uint8_t            srcvalue;
  uint8_t            newvalue;
  size_t             seekpos;

  /* Set the starting location in the file */

  seekpos = priv->offset + offset;

  while (len-- > 0)
    {
      if (buflen == 0)
        {
          /* Read more data from the file */

          file_seek(&priv->mtdfile, seekpos, SEEK_SET);
          buflen = file_read(&priv->mtdfile, buf, sizeof(buf));
          pout   = (FAR uint8_t *) buf;
        }

      /* Get the source and destination values */

      oldvalue = *pout;
      srcvalue = *pin++;

      /* Get the new destination value, accounting for bits that cannot be
       * changes because they are not in the erased state.
       */

#if CONFIG_FILEMTD_ERASESTATE == 0xff
      newvalue = oldvalue & srcvalue; /* We can only clear bits */
#else /* CONFIG_FILEMTD_ERASESTATE == 0x00 */
      newvalue = oldvalue | srcvalue; /* We can only set bits */
#endif

      /* Report any attempt to change the value of bits that are not in the
       * erased state.
       */

#ifdef CONFIG_DEBUG_FEATURES
      if (newvalue != srcvalue)
        {
          ferr("ERROR: Bad write: source=%02x dest=%02x result=%02x\n",
              srcvalue, oldvalue, newvalue);
        }
#endif

      /* Write the modified value to simulated FLASH */

      *pout++ = newvalue;
      buflen--;

      /* If our buffer is full, then seek back to beginning of
       * the file and write the buffer contents
       */

      if (buflen == 0)
        {
          file_seek(&priv->mtdfile, seekpos, SEEK_SET);
          file_write(&priv->mtdfile, buf, sizeof(buf));
          seekpos += sizeof(buf);
        }
    }

  /* Write remaining bytes */

  if (buflen != 0)
    {
      file_seek(&priv->mtdfile, seekpos, SEEK_SET);
      file_write(&priv->mtdfile, buf, sizeof(buf));
    }

  return len;
}

/****************************************************************************
 * Name: filemtd_read
 ****************************************************************************/

static ssize_t filemtd_read(FAR struct file_dev_s *priv,
                            FAR unsigned char *buffer, size_t offsetbytes,
                            unsigned int nbytes)
{
  /* Set the starting location in the file */

  file_seek(&priv->mtdfile, priv->offset + offsetbytes, SEEK_SET);

  return file_read(&priv->mtdfile, buffer, nbytes);
}

/****************************************************************************
 * Name: filemtd_erase
 ****************************************************************************/

static int filemtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  size_t    nbytes;
  size_t    offset;
  char      buffer[128];

  DEBUGASSERT(dev);

  /* Don't let the erase exceed the original size of the file */

  if (startblock >= priv->nblocks)
    {
      return 0;
    }

  if (startblock + nblocks > priv->nblocks)
    {
      nblocks = priv->nblocks - startblock;
    }

  /* Convert the erase block to a logical block and the number of blocks
   * in logical block numbers
   */

  startblock *= (priv->erasesize / priv->blocksize);
  nblocks    *= (priv->erasesize / priv->blocksize);

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then erase the data in the file */

  file_seek(&priv->mtdfile, priv->offset + offset, SEEK_SET);
  memset(buffer, CONFIG_FILEMTD_ERASESTATE, sizeof(buffer));
  while (nbytes)
    {
      file_write(&priv->mtdfile, buffer, sizeof(buffer));
      nbytes -= sizeof(buffer);
    }

  return OK;
}

/****************************************************************************
 * Name: filemtd_bread
 ****************************************************************************/

static ssize_t filemtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the original size of the file */

  maxblock = priv->nblocks * (priv->erasesize / priv->blocksize);
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then read the data from the file */

  filemtd_read(priv, buf, offset, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: filemtd_bwrite
 ****************************************************************************/

static ssize_t filemtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t offset;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the original size of the file */

  maxblock = priv->nblocks * (priv->erasesize / priv->blocksize);
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the offset corresponding to the first block and the size
   * corresponding to the number of blocks.
   */

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  /* Then write the data to the file */

  filemtd_write(priv, offset, buf, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: filemtd_byteread
 ****************************************************************************/

static ssize_t filemtd_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, FAR uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t maxoffset;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the original size of the file */

  maxoffset = priv->nblocks * priv->erasesize;
  if (offset >= maxoffset)
    {
      return 0;
    }

  if (offset + nbytes > maxoffset)
    {
      nbytes = maxoffset - offset;
    }

  filemtd_read(priv, buf, offset, nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: file_bytewrite
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t file_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, FAR const uint8_t *buf)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  off_t maxoffset;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the original size of the file */

  maxoffset = priv->nblocks * priv->erasesize;
  if (offset + nbytes > maxoffset)
    {
      return 0;
    }

  if (offset + nbytes > maxoffset)
    {
      nbytes = maxoffset - offset;
    }

  /* Then write the data to the file */

  filemtd_write(priv, offset, buf, nbytes);
  return nbytes;
}
#endif

/****************************************************************************
 * Name: filemtd_ioctl
 ****************************************************************************/

static int filemtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->erasesize;
              geo->neraseblocks = priv->nblocks;
              ret               = OK;
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->nblocks *
                                  priv->erasesize / priv->blocksize;
              info->sectorsize  = priv->blocksize;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = filemtd_erase(dev, 0, priv->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = CONFIG_FILEMTD_ERASESTATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: blockmtd_initialize
 *
 * Description:
 *   Create and initialize a BLOCK MTD device instance.
 *
 * Input Parameters:
 *   path - Path name of the block device backing the MTD device
 *
 ****************************************************************************/

FAR struct mtd_dev_s *blockmtd_initialize(FAR const char *path,
                                          size_t offset, size_t mtdlen,
                                          int16_t sectsize,
                                          int32_t erasesize)
{
  FAR struct file_dev_s *priv;
  size_t nblocks;
  int mode;
  int ret;

  /* Create an instance of the FILE MTD device state structure */

  priv = (FAR struct file_dev_s *)kmm_zalloc(sizeof(struct file_dev_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate the FILE MTD state structure\n");
      return NULL;
    }

  /* Set the file open mode. */

  mode = O_RDOK | O_WROK;

  /* Try to open the file.  NOTE that block devices will use a character
   * driver proxy.
   */

  ret = file_open(&priv->mtdfile, path, mode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open the FILE MTD file %s: %d\n", path, ret);
      kmm_free(priv);
      return NULL;
    }

  /* Set the block size based on the provided sectsize parameter */

  if (sectsize <= 0)
    {
      priv->blocksize = CONFIG_FILEMTD_BLOCKSIZE;
    }
  else
    {
      priv->blocksize = sectsize;
    }

  /* Set the erase size based on the provided erasesize parameter */

  if (erasesize <= 0)
    {
      priv->erasesize = CONFIG_FILEMTD_ERASESIZE;
    }
  else
    {
      priv->erasesize = erasesize;
    }

  if ((priv->erasesize / priv->blocksize) * priv->blocksize
      != priv->erasesize)
    {
      ferr("ERROR: erasesize must be an even multiple of sectsize\n");
      file_close(&priv->mtdfile);
      kmm_free(priv);
      return NULL;
    }

  /* Force the size to be an even number of the erase block size */

  nblocks = mtdlen / priv->erasesize;
  if (nblocks < 3)
    {
      ferr("ERROR: Need to provide at least three full erase block\n");
      file_close(&priv->mtdfile);
      kmm_free(priv);
      return NULL;
    }

  /* Perform initialization as necessary. (unsupported methods were
   * nullified by kmm_zalloc).
   */

  priv->mtd.erase  = filemtd_erase;
  priv->mtd.bread  = filemtd_bread;
  priv->mtd.bwrite = filemtd_bwrite;
  priv->mtd.read   = filemtd_byteread;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = file_bytewrite;
#endif
  priv->mtd.ioctl  = filemtd_ioctl;
  priv->mtd.name   = "filemtd";
  priv->offset     = offset;
  priv->nblocks    = nblocks;

  return &priv->mtd;
}

/****************************************************************************
 * Name: blockmtd_teardown
 *
 * Description:
 *   Teardown a previously created blockmtd device.
 *
 * Input Parameters:
 *   dev - Pointer to the mtd driver instance.
 *
 ****************************************************************************/

void blockmtd_teardown(FAR struct mtd_dev_s *dev)
{
  FAR struct file_dev_s *priv;

  /* Close the enclosed file */

  priv = (FAR struct file_dev_s *) dev;
  file_close(&priv->mtdfile);

#ifdef CONFIG_MTD_REGISTRATION
  /* Un-register the MTD with the procfs system if enabled */

  mtd_unregister(&priv->mtd);
#endif

  /* Free the memory */

  kmm_free(priv);
}

/****************************************************************************
 * Name: filemtd_initialize
 *
 * Description:
 *   Create and initialize a FILE MTD device instance.
 *
 * Input Parameters:
 *   path - Path name of the file backing the MTD device
 *
 ****************************************************************************/

FAR struct mtd_dev_s *filemtd_initialize(FAR const char *path, size_t offset,
                                         int16_t sectsize, int32_t erasesize)
{
  size_t filelen;
  struct stat sb;
  int ret;

  /* Stat the file */

  ret = nx_stat(path, &sb, 1);
  if (ret < 0)
    {
      ferr("ERROR: Failed to stat %s: %d\n", path, ret);
      return NULL;
    }

  filelen = sb.st_size;

  if (offset > filelen)
    {
      ferr("ERROR: Offset beyond end of file\n");
      return NULL;
    }

  return blockmtd_initialize(path, offset, filelen - offset, sectsize,
                             erasesize);
}

/****************************************************************************
 * Name: filemtd_teardown
 *
 * Description:
 *   Teardown a previously created filemtd device.
 *
 * Input Parameters:
 *   dev - Pointer to the mtd driver instance.
 *
 ****************************************************************************/

void filemtd_teardown(FAR struct mtd_dev_s *dev)
{
  blockmtd_teardown(dev);
}

/****************************************************************************
 * Name: filemtd_isfilemtd
 *
 * Description:
 *   Tests if the provided mtd is a filemtd or blockmtd device.
 *
 * Input Parameters:
 *   mtd - Pointer to the mtd.
 *
 ****************************************************************************/

bool filemtd_isfilemtd(FAR struct mtd_dev_s *dev)
{
  FAR struct file_dev_s *priv = (FAR struct file_dev_s *) dev;

  return (priv->mtd.erase == filemtd_erase);
}
