/****************************************************************************
 * fs/driver/fs_blockmerge.c
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
#include <sys/mount.h>
#include <sys/param.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <stdarg.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct merge_part_s
{
  FAR struct inode *inode;
  struct geometry geo;
  char path[PATH_MAX];
};

struct merge_priv_s
{
  size_t npart;
  struct merge_part_s part[0];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     merge_open(FAR struct inode *inode);
static int     merge_close(FAR struct inode *inode);
static ssize_t merge_read(FAR struct inode *inode, FAR unsigned char *buffer,
                          blkcnt_t startsector, unsigned int nsectors);
static ssize_t merge_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t startsector, unsigned int nsectors);
static int     merge_geometry(FAR struct inode *inode,
                              FAR struct geometry *geometry);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_merge_bops =
{
  merge_open,     /* open     */
  merge_close,    /* close    */
  merge_read,     /* read     */
  merge_write,    /* write    */
  merge_geometry, /* geometry */
  NULL            /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int merge_open(FAR struct inode *inode)
{
  FAR struct merge_priv_s *priv;
  int ret = 0;
  size_t i;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  for (i = 0; i < priv->npart; i++)
    {
      ret = open_blockdriver(priv->part[i].path, 0, &inode);
      if (ret < 0)
        {
          ferr("ERROR: Failed to open block driver for %s\n",
               priv->part[i].path);
          goto err_with_open;
        }

      priv->part[i].inode = inode;

      if (inode->u.i_bops->geometry == NULL ||
          inode->u.i_bops->geometry(inode, &priv->part[i].geo) < 0 ||
          priv->part[i].geo.geo_sectorsize <= 0 ||
          priv->part[i].geo.geo_nsectors <= 0)
        {
          ferr("ERROR: Failed to get geometry for %s\n", priv->part[i].path);
          ret = -EINVAL;
          goto err_with_inode;
        }

      if (priv->part[0].geo.geo_sectorsize !=
          priv->part[i].geo.geo_sectorsize)
        {
          ferr("ERROR: Inconsistent sectorsize!\n");
          ret = -EINVAL;
          goto err_with_inode;
        }

      finfo("[%s] nsectors: %" PRIuOFF " sectorsize:%u\n",
            priv->part[i].path, priv->part[i].geo.geo_nsectors,
            priv->part[i].geo.geo_sectorsize);
    }

  return ret;

err_with_inode:
  close_blockdriver(priv->part[i].inode);
err_with_open:
  while (i--)
    {
      close_blockdriver(priv->part[i].inode);
    }

  return ret;
}

static int merge_close(FAR struct inode *inode)
{
  FAR struct merge_priv_s *priv;
  size_t i;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  for (i = 0; i < priv->npart; i++)
    {
      int ret = close_blockdriver(priv->part[i].inode);
      if (ret < 0)
        {
          ferr("ERROR: Failed to close block driver for %s\n",
               priv->part[i].path);
          return ret;
        }
    }

  return OK;
}

static ssize_t merge_read(FAR struct inode *inode, FAR unsigned char *buffer,
                          blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct merge_priv_s *priv;
  ssize_t nread = 0;
  size_t ntotal = 0;
  size_t i;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  for (i = 0; nsectors > 0 && i < priv->npart; i++)
    {
      if ((startsector >= priv->part[i].geo.geo_nsectors))
        {
          startsector -= priv->part[i].geo.geo_nsectors;
          continue;
        }

      inode = priv->part[i].inode;
      nread = MIN(priv->part[i].geo.geo_nsectors - startsector, nsectors);
      nread = inode->u.i_bops->read(inode, buffer, startsector, nread);
      if (nread < 0)
        {
          break;
        }

      buffer += nread * priv->part[i].geo.geo_sectorsize;
      nsectors -= nread;
      ntotal += nread;
      startsector = 0;
    }

  /* On success, return the number of blocks read */

  return ntotal ? ntotal : nread;
}

static ssize_t merge_write(FAR struct inode *inode,
                           FAR const unsigned char *buffer,
                           blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct merge_priv_s *priv;
  ssize_t nwritten = 0;
  size_t ntotal = 0;
  size_t i;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  for (i = 0; nsectors > 0 && i < priv->npart; i++)
    {
      if (startsector >= priv->part[i].geo.geo_nsectors)
        {
          startsector -= priv->part[i].geo.geo_nsectors;
          continue;
        }

      inode = priv->part[i].inode;
      nwritten = MIN(priv->part[i].geo.geo_nsectors - startsector, nsectors);
      nwritten = inode->u.i_bops->write(inode, buffer, startsector,
                                        nwritten);
      if (nwritten < 0)
        {
          break;
        }

      buffer += nwritten * priv->part[i].geo.geo_sectorsize;
      nsectors -= nwritten;
      ntotal += nwritten;
      startsector = 0;
    }

  /* On success, return the number of blocks written */

  return ntotal ? ntotal : nwritten;
}

static int merge_geometry(FAR struct inode *inode,
                          FAR struct geometry *geometry)
{
  DEBUGASSERT(inode && inode->i_private);

  if (geometry)
    {
      FAR struct merge_priv_s *priv = inode->i_private;
      size_t i;

      memcpy(geometry, &priv->part[0].geo, sizeof(*geometry));
      for (i = 1; i < priv->npart; i++)
        {
          geometry->geo_nsectors += priv->part[i].geo.geo_nsectors;
        }

      finfo("nsectors: %" PRIuOFF " sectorsize:%u\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int register_merge_blockdriver(FAR const char *merge, ...)
{
  FAR struct merge_priv_s *priv;
  FAR const char *name;
  va_list args;
  size_t i;
  int ret;

  va_start(args, merge);
  for (i = 0; ; i++)
    {
      name = va_arg(args, FAR const char *);
      if (name == NULL)
        {
          break;
        }
    }

  va_end(args);

  if (i < 2)
    {
      ferr("ERROR: invalid number of partitions\n");
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct merge_priv_s) + i *
                    sizeof(struct merge_part_s));
  if (priv == NULL)
    {
      ferr("ERROR: out of memory\n");
      return -ENOMEM;
    }

  priv->npart = i;

  va_start(args, merge);
  for (i = 0; i < priv->npart; i++)
    {
      name = va_arg(args, FAR const char *);
      strlcpy(priv->part[i].path, name, PATH_MAX);
    }

  ret = register_blockdriver(merge, &g_merge_bops, 0, priv);
  if (ret < 0)
    {
      ferr("ERROR: register blockdriver failed: %d\n", ret);
      goto err;
    }

  va_end(args);
  return ret;

err:
  va_end(args);
  kmm_free(priv);
  return ret;
}
