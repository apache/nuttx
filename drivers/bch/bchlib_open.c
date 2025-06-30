/****************************************************************************
 * drivers/bch/bchlib_open.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 #include <stdint.h>
 #include <stdbool.h>
 #include <string.h>
 #include <errno.h>
 #include <assert.h>
 #include <debug.h>

 #include <nuttx/fs/fs.h>
 #include <nuttx/drivers/drivers.h>

 #include "bch.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchlib_open
 *
 * Description:
 *   Opens a block device and increments the reference count for tracking
 *   the number of active users. Ensures no more than the maximum allowed
 *   open count (`MAX_OPENCNT`) is exceeded.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code such as:
 *     -EMFILE: Maximum open count exceeded.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

int bchlib_open(FAR void *handle)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  int ret = OK;

  /* Increment the reference count */

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the maximum open count is reached */

  if (bch->refs == MAX_OPENCNT)
    {
      ret = -EMFILE;
    }
  else
    {
      bch->refs++;
    }

  nxmutex_unlock(&bch->lock);
  return ret;
}
