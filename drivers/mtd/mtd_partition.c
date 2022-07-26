/****************************************************************************
 * drivers/mtd/mtd_partition.c
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
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>

#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#ifdef CONFIG_FS_PROCFS
#include <nuttx/fs/procfs.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PART_NAME_MAX           15

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct mtd_partition_s.
 */

struct mtd_partition_s
{
  /* This structure must reside at the beginning so that we can simply cast
   * from struct mtd_dev_s * to struct mtd_partition_s *
   */

  struct mtd_dev_s child;       /* The "child" MTD vtable that manages the
                                 * sub-region */

  /* Other implementation specific data may follow here */

  FAR struct mtd_dev_s *parent; /* The "parent" MTD driver that manages the
                                 * entire FLASH */
  off_t firstblock;             /* Offset to the first block of the managed
                                 * sub-region */
  off_t neraseblocks;           /* The number of erase blocks in the managed
                                 * sub-region */
  off_t blocksize;              /* The size of one read/write block */
  uint16_t blkpererase;         /* Number of R/W blocks in one erase block */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)
  struct mtd_partition_s  *pnext; /* Pointer to next partition struct */
#endif
#ifdef CONFIG_MTD_PARTITION_NAMES
  char name[PART_NAME_MAX + 1]; /* Name of the partition */
#endif
};

/* This structure describes one open "file" */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)
struct part_procfs_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */
  struct mtd_partition_s *nextpart;
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int     part_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t part_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR uint8_t *buf);
static ssize_t part_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR const uint8_t *buf);
static ssize_t part_read(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t part_write(FAR struct mtd_dev_s *dev, off_t offset,
                  size_t nbytes, FAR const uint8_t *buffer);
#endif
static int     part_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                  unsigned long arg);

/* File system methods */

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)
static int     part_procfs_open(FAR struct file *filep,
                 FAR const char *relpath, int oflags, mode_t mode);
static int     part_procfs_close(FAR struct file *filep);
static ssize_t part_procfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     part_procfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

#if 0 /* Not implemented */
static int     part_procfs_opendir(const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     part_procfs_closedir(FAR struct fs_dirent_s *dir);
static int     part_procfs_readdir(FAR struct fs_dirent_s *dir,
                                   FAR struct dirent *entry);
static int     part_procfs_rewinddir(FAR struct fs_dirent_s *dir);
#endif

static int     part_procfs_stat(FAR const char *relpath,
                 FAR struct stat *buf);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)
static struct mtd_partition_s *g_pfirstpartition = NULL;

const struct procfs_operations part_procfsoperations =
{
  part_procfs_open,       /* open */
  part_procfs_close,      /* close */
  part_procfs_read,       /* read */
  NULL,                   /* write */

  part_procfs_dup,        /* dup */

  NULL,                   /* opendir */
  NULL,                   /* closedir */
  NULL,                   /* readdir */
  NULL,                   /* rewinddir */

  part_procfs_stat        /* stat */
};
#endif

/****************************************************************************
 * Name: part_blockcheck
 *
 * Description:
 *   Check if the provided block offset lies within the partition
 *
 ****************************************************************************/

static bool part_blockcheck(FAR struct mtd_partition_s *priv, off_t block)
{
  off_t partsize;

  partsize = priv->neraseblocks * priv->blkpererase;
  return block < partsize;
}

/****************************************************************************
 * Name: part_bytecheck
 *
 * Description:
 *   Check if the provided byte offset lies within the partition
 *
 ****************************************************************************/

static bool part_bytecheck(FAR struct mtd_partition_s *priv, off_t byoff)
{
  off_t erasesize;
  off_t readend;

  erasesize = priv->blocksize * priv->blkpererase;
  readend   = (byoff + erasesize - 1) / erasesize;
  return readend <= priv->neraseblocks;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: part_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int part_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t eoffset;

  DEBUGASSERT(priv);

  /* Make sure that erase would not extend past the end of the partition */

  if (!part_blockcheck(priv, (startblock + nblocks - 1) * priv->blkpererase))
    {
      ferr("ERROR: Erase beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the erase.
   *
   * NOTE: the offset here is in units of erase blocks.
   */

  eoffset = priv->firstblock / priv->blkpererase;
  DEBUGASSERT(eoffset * priv->blkpererase == priv->firstblock);

  return priv->parent->erase(priv->parent, startblock + eoffset, nblocks);
}

/****************************************************************************
 * Name: part_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;

  DEBUGASSERT(priv && (buf || nblocks == 0));

  /* Make sure that read would not extend past the end of the partition */

  if (!part_blockcheck(priv, startblock + nblocks - 1))
    {
      ferr("ERROR: Read beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the read.
   */

  return priv->parent->bread(priv->parent, startblock + priv->firstblock,
                             nblocks, buf);
}

/****************************************************************************
 * Name: part_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;

  DEBUGASSERT(priv && (buf || nblocks == 0));

  /* Make sure that write would not extend past the end of the partition */

  if (!part_blockcheck(priv, startblock + nblocks - 1))
    {
      ferr("ERROR: Write beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the write.
   */

  return priv->parent->bwrite(priv->parent, startblock + priv->firstblock,
                              nblocks, buf);
}

/****************************************************************************
 * Name: part_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_read(FAR struct mtd_dev_s *dev, off_t offset,
                         size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t newoffset;

  DEBUGASSERT(priv && (buffer || nbytes == 0));

  /* Does the underlying MTD device support the read method? */

  if (priv->parent->read)
    {
      /* Make sure that read would not extend past the end of the partition */

      if (!part_bytecheck(priv, offset + nbytes - 1))
        {
          ferr("ERROR: Read beyond the end of the partition\n");
          return -ENXIO;
        }

      /* Just add the partition offset to the requested block and let the
       * underlying MTD driver perform the read.
       */

      newoffset = offset + priv->firstblock * priv->blocksize;
      return priv->parent->read(priv->parent, newoffset, nbytes, buffer);
    }

  /* The underlying MTD driver does not support the read() method */

  return -ENOSYS;
}

/****************************************************************************
 * Name: part_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t part_write(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t newoffset;

  DEBUGASSERT(priv && (buffer || nbytes == 0));

  /* Does the underlying MTD device support the write method? */

  if (priv->parent->write)
    {
      /* Make sure that write would not extend past the end of the
       * partition
       */

      if (!part_bytecheck(priv, offset + nbytes - 1))
        {
          ferr("ERROR: Write beyond the end of the partition\n");
          return -ENXIO;
        }

      /* Just add the partition offset to the requested block and let the
       * underlying MTD driver perform the write.
       */

      newoffset = offset + priv->firstblock * priv->blocksize;
      return priv->parent->write(priv->parent, newoffset, nbytes, buffer);
    }

  /* The underlying MTD driver does not support the write() method */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: part_ioctl
 ****************************************************************************/

static int part_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  DEBUGASSERT(priv);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to
               * know the capacity and how to access the device.
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->blocksize * priv->blkpererase;
              geo->neraseblocks = priv->neraseblocks;
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
              info->numsectors  = priv->neraseblocks * priv->blkpererase;
              info->sectorsize  = priv->blocksize;
              info->startsector = priv->firstblock;

              strncpy(info->parent, priv->parent->name, NAME_MAX);

              ret = OK;
          }
        }
        break;

      case BIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;
          unsigned long base;

          if (ppv)
            {
              /* Get the XIP base of the entire FLASH */

              ret = priv->parent->ioctl(priv->parent, BIOC_XIPBASE,
                                        (unsigned long)((uintptr_t)&base));
              if (ret == OK)
                {
                  /* Add the offset of this partition to the XIP base and
                   * return the sum to the caller.
                   */

                  *ppv = (FAR void *)(uintptr_t)
                            (base + priv->firstblock * priv->blocksize);
                }
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire partition */

          ret = priv->parent->erase(priv->parent,
                                    priv->firstblock / priv->blkpererase,
                                    priv->neraseblocks);
        }
        break;

      default:
        {
          /* Pass any unhandled ioctl() calls to the underlying driver */

          ret = priv->parent->ioctl(priv->parent, cmd, arg);
        }
        break;
    }

  return ret;
}

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)

/****************************************************************************
 * Name: part_procfs_open
 ****************************************************************************/

static int part_procfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct part_procfs_file_s *attr;

  finfo("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate a container to hold the task and attribute selection */

  attr = (FAR struct part_procfs_file_s *)
         kmm_zalloc(sizeof(struct part_procfs_file_s));
  if (!attr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* Initialize the file attributes */

  attr->nextpart = g_pfirstpartition;

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)attr;
  return OK;
}

/****************************************************************************
 * Name: part_procfs_close
 ****************************************************************************/

static int part_procfs_close(FAR struct file *filep)
{
  FAR struct part_procfs_file_s *attr;

  /* Recover our private data from the struct file instance */

  attr = (FAR struct part_procfs_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* Release the file attributes structure */

  kmm_free(attr);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: part_procfs_read
 ****************************************************************************/

static ssize_t part_procfs_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct part_procfs_file_s *attr;
  FAR struct mtd_geometry_s geo;
  ssize_t total = 0;
  ssize_t blkpererase;
  ssize_t ret;
#ifdef CONFIG_MTD_PARTITION_NAMES
  char partname[PART_NAME_MAX + 1];
  FAR const char *ptr;
  uint8_t x;
#endif

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  attr = (FAR struct part_procfs_file_s *)filep->f_priv;
  DEBUGASSERT(attr);

  /* If we are at the end of the list, then return 0 signifying the
   * end-of-file.  This also handles the special case when there are
   * no registered MTD partitions.
   */

  if (attr->nextpart)
    {
      /* Output a header before the first entry */

      if (attr->nextpart == g_pfirstpartition)
        {
#ifdef CONFIG_MTD_PARTITION_NAMES
          total = snprintf(buffer, buflen, "%-*s  Start    Size   MTD\n",
                           PART_NAME_MAX, "Name");
#else
          total = snprintf(buffer, buflen, "  Start    Size   MTD\n");
#endif
        }

      /* Provide the requested data */

      do
        {
          /* Get the geometry of the FLASH device */

          ret = attr->nextpart->parent->ioctl(attr->nextpart->parent,
                  MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
          if (ret < 0)
            {
              ferr("ERROR: mtd->ioctl failed: %zd\n", ret);
              return 0;
            }

          /* Get the number of blocks per erase.  There must be an even
           * number of blocks in one erase blocks.
           */

          blkpererase = geo.erasesize / geo.blocksize;

          /* Copy data from the next known partition */

#ifdef CONFIG_MTD_PARTITION_NAMES
          ptr = attr->nextpart->name;
          for (x = 0; x < sizeof(partname) - 1; x++)
            {
              /* Test for end of partition name */

              if (*ptr == ',' || *ptr == '\0')
                {
                  /* Perform space fill for alignment */

                  partname[x] = ' ';
                }
              else
                {
                  /* Copy next byte of partition name */

                  partname[x] = *ptr++;
                }
            }

          partname[x] = '\0';

          /* Terminate the partition name and add to output buffer */

          ret = snprintf(&buffer[total], buflen - total,
                         "%s%7ju %7ju   %s\n",
                         partname,
                         (uintmax_t)attr->nextpart->firstblock / blkpererase,
                         (uintmax_t)attr->nextpart->neraseblocks,
                         attr->nextpart->parent->name);
#else
          ret = snprintf(&buffer[total], buflen - total, "%7ju %7ju   %s\n",
                         (uintmax_t)attr->nextpart->firstblock / blkpererase,
                         (uintmax_t)attr->nextpart->neraseblocks,
                         attr->nextpart->parent->name);
#endif

          if (ret + total < buflen)
            {
              /* It fit in the buffer totally.  Advance total and move to
               * next partition.
               */

              total += ret;
              attr->nextpart = attr->nextpart->pnext;
            }
          else
            {
              /* This one didn't fit completely.  Truncate the partial
               * entry and break the loop.
               */

              buffer[total] = '\0';
              break;
            }
        }
      while (attr->nextpart);
    }

  /* Update the file offset */

  if (total > 0)
    {
      filep->f_pos += total;
    }

  return total;
}

/****************************************************************************
 * Name: part_procfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int part_procfs_dup(FAR const struct file *oldp,
                           FAR struct file *newp)
{
  FAR struct part_procfs_file_s *oldattr;
  FAR struct part_procfs_file_s *newattr;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldattr = (FAR struct part_procfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldattr);

  /* Allocate a new container to hold the task and attribute selection */

  newattr = (FAR struct part_procfs_file_s *)
            kmm_zalloc(sizeof(struct part_procfs_file_s));
  if (!newattr)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribute from the old attributes to the new */

  memcpy(newattr, oldattr, sizeof(struct part_procfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newattr;
  return OK;
}

/****************************************************************************
 * Name: part_procfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int part_procfs_stat(const char *relpath, struct stat *buf)
{
  /* File/directory size, access block size */

  buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_partition
 *
 * Description:
 *   Give an instance of an MTD driver, create a flash partition, ie.,
 *   another MTD driver instance that only operates with a sub-region of
 *   FLASH media.  That sub-region is defined by a sector offset and a
 *   sector count (where the size of a sector is provided the by parent MTD
 *   driver).
 *
 *   NOTE: Since there may be a number of MTD partition drivers operating on
 *   the same, underlying FLASH driver, that FLASH driver must be capable
 *   of enforcing mutually exclusive access to the FLASH device.  Without
 *   partitions, that mutual exclusion would be provided by the file system
 *   above the FLASH driver.
 *
 * Input Parameters:
 *   mtd        - The MTD device to be partitioned
 *   firstblock - The offset in bytes to the first block
 *   nblocks    - The number of blocks in the partition
 *
 * Returned Value:
 *   On success, another MTD device representing the partition is returned.
 *   A NULL value is returned on a failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd,
                                    off_t firstblock,
                                    off_t nblocks)
{
  FAR struct mtd_partition_s *part;
  FAR struct mtd_geometry_s geo;
  unsigned int blkpererase;
  off_t erasestart;
  off_t eraseend;
  int ret;

  DEBUGASSERT(mtd);

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      ferr("ERROR: mtd->ioctl failed: %d\n", ret);
      return NULL;
    }

  /* Get the number of blocks per erase.  There must be an even number of
   * blocks in one erase blocks.
   */

  blkpererase = geo.erasesize / geo.blocksize;
  DEBUGASSERT(blkpererase * geo.blocksize == geo.erasesize);

  /* Adjust the offset and size if necessary so that they are multiples of
   * the erase block size (making sure that we do not go outside of the
   * requested sub-region).  NOTE that 'eraseend' is the first erase block
   * beyond the sub-region.
   */

  erasestart = (firstblock + blkpererase - 1) / blkpererase;
  eraseend   = (firstblock + nblocks) / blkpererase;

  if (erasestart >= eraseend)
    {
      ferr("ERROR: sub-region too small\n");
      return NULL;
    }

  /* Verify that the sub-region is valid for this geometry */

  if (eraseend > geo.neraseblocks)
    {
      ferr("ERROR: sub-region too big\n");
      return NULL;
    }

  /* Allocate a partition device structure */

  part = (FAR struct mtd_partition_s *)
         kmm_zalloc(sizeof(struct mtd_partition_s));
  if (!part)
    {
      ferr("ERROR: Failed to allocate memory for the partition device\n");
      return NULL;
    }

  /* Initialize the partition device structure. (unsupported methods were
   * nullified by kmm_zalloc).
   */

  part->child.erase  = part_erase;
  part->child.bread  = part_bread;
  part->child.bwrite = part_bwrite;
  part->child.read   = mtd->read ? part_read : NULL;
  part->child.ioctl  = part_ioctl;
#ifdef CONFIG_MTD_BYTE_WRITE
  part->child.write  = mtd->write ? part_write : NULL;
#endif
  part->child.name   = "part";

  part->parent       = mtd;
  part->firstblock   = erasestart * blkpererase;
  part->neraseblocks = eraseend - erasestart;
  part->blocksize    = geo.blocksize;
  part->blkpererase  = blkpererase;

#ifdef CONFIG_MTD_PARTITION_NAMES
  strcpy(part->name, "(noname)");
#endif

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_PROCFS_EXCLUDE_PARTITIONS)
  /* Add this partition to the list of known partitions */

  if (g_pfirstpartition == NULL)
    {
      g_pfirstpartition = part;
    }
  else
    {
      struct mtd_partition_s *plast;

      /* Add the partition to the end of the list */

      part->pnext = NULL;

      plast = g_pfirstpartition;
      while (plast->pnext != NULL)
        {
          /* Get pointer to next partition */

          plast = plast->pnext;
        }

      plast->pnext = part;
    }
#endif

  /* Return the implementation-specific state structure as the MTD device */

  return &part->child;
}

/****************************************************************************
 * Name: mtd_setpartitionname
 *
 * Description:
 *   Sets the name of the specified partition.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_PARTITION_NAMES
int mtd_setpartitionname(FAR struct mtd_dev_s *mtd, FAR const char *name)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)mtd;

  if (priv == NULL || name == NULL)
    {
      return -EINVAL;
    }

  /* Allocate space for the name */

  strlcpy(priv->name, name, sizeof(priv->name));
  return OK;
}
#endif
