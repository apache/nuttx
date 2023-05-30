/****************************************************************************
 * drivers/bch/bchlib_teardown.c
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
 * Name: bchlib_teardown
 *
 * Description:
 *   Setup so that the block driver referenced by 'blkdev' can be accessed
 *   similar to a character device.
 *
 ****************************************************************************/

int bchlib_teardown(FAR void *handle)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;

  DEBUGASSERT(handle);

  /* Check that there are not outstanding reference counts on the object */

  if (bch->refs > 0)
    {
      return -EBUSY;
    }

  /* Flush any pending data to the block driver */

  bchlib_flushsector(bch);

  /* Close the block driver */

  close_blockdriver(bch->inode);

  /* Free the BCH state structure */

  if (bch->buffer)
    {
      kmm_free(bch->buffer);
    }

  nxmutex_destroy(&bch->lock);
  kmm_free(bch);
  return OK;
}
