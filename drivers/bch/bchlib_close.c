/****************************************************************************
 * drivers/bch/bchlib_close.c
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
 * Name: bchlib_close
 *
 * Description:
 *   Closes the block device by decrementing the reference count and
 *   performing cleanup operations like flushing dirty pages in the cache.
 *   If all references are released and the device is marked as unlinked,
 *   it tears down the BCH device.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code such as:
 *     -EIO: Reference count is already zero.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

int bchlib_close(FAR void *handle)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  int ret = OK;

  /* Get exclusive access */

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush any dirty pages remaining in the cache */

  bchlib_flushsector(bch, false);

  /* Decrement the reference count (I don't use bchlib_decref() because I
   * want the entire close operation to be atomic wrt other driver
   * operations.
   */

  if (bch->refs == 0)
    {
      ret = -EIO;
    }
  else
    {
      bch->refs--;

      /* If the reference count decremented to zero AND if the character
       * driver has been unlinked, then teardown the BCH device now.
       */

      if (bch->refs == 0 && bch->unlinked)
        {
          /* Tear the driver down now. */

          ret = bchlib_teardown((FAR void *)bch);

          /* bchlib_teardown() would only fail if there are outstanding
           * references on the device.  Since we know that is not true, it
           * should not fail at all.
           */

          DEBUGASSERT(ret >= 0);
          if (ret >= 0)
            {
              /* Return without releasing the stale semaphore */

              return OK;
            }
        }
    }

  nxmutex_unlock(&bch->lock);
  return ret;
}
