/****************************************************************************
 * drivers/bch/bchlib_setup.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/drivers/drivers.h>

#include "bch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchlib_setup
 *
 * Description:
 *   Setup so that the block driver referenced by 'blkdev' can be accessed
 *   similar to a character device.
 *
 ****************************************************************************/

int bchlib_setup(const char *blkdev, bool readonly, FAR void **handle)
{
  FAR struct bchlib_s *bch;
  struct geometry geo;
  int ret;

  DEBUGASSERT(blkdev);

  /* Allocate the BCH state structure */

  bch = (FAR struct bchlib_s *)kmm_zalloc(sizeof(struct bchlib_s));
  if (!bch)
    {
      ferr("ERROR: Failed to allocate BCH structure\n");
      return -ENOMEM;
    }

  /* Open the block driver */

  ret = open_blockdriver(blkdev, readonly ? MS_RDONLY : 0, &bch->inode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open driver %s: %d\n", blkdev, -ret);
      goto errout_with_bch;
    }

  DEBUGASSERT(bch->inode && bch->inode->u.i_bops &&
              bch->inode->u.i_bops->geometry);

  ret = bch->inode->u.i_bops->geometry(bch->inode, &geo);
  if (ret < 0)
    {
      ferr("ERROR: geometry failed: %d\n", -ret);
      goto errout_with_bch;
    }

  if (!geo.geo_available)
    {
      ferr("ERROR: geometry failed: %d\n", -ret);
      ret = -ENODEV;
      goto errout_with_bch;
    }

  if (!readonly && (!bch->inode->u.i_bops->write || !geo.geo_writeenabled))
    {
      ferr("ERROR: write access not supported\n");
      ret = -EACCES;
      goto errout_with_bch;
    }

  /* Save the geometry info and complete initialization of the structure */

  nxmutex_init(&bch->lock);
  bch->nsectors = geo.geo_nsectors;
  bch->sectsize = geo.geo_sectorsize;
  bch->sector   = (size_t)-1;
  bch->readonly = readonly;

  /* Allocate the sector I/O buffer */

#if CONFIG_BCH_BUFFER_ALIGNMENT != 0
  bch->buffer = kmm_memalign(CONFIG_BCH_BUFFER_ALIGNMENT, bch->sectsize);
#else
  bch->buffer = kmm_malloc(bch->sectsize);
#endif
  if (!bch->buffer)
    {
      ferr("ERROR: Failed to allocate sector buffer\n");
      ret = -ENOMEM;
      goto errout_with_bch;
    }

  *handle = bch;
  return OK;

errout_with_bch:
  kmm_free(bch);
  return ret;
}
