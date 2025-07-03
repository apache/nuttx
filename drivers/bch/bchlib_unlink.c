/****************************************************************************
 * drivers/bch/bchlib_unlink.c
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
 * Name: bchlib_unlink
 *
 * Description:
 *   Marks the block device as unlinked (no longer actively accessible). If
 *   there are no remaining open references, tears down the BCH device.
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *
 * Return:
 *   On success, returns `OK`.
 *   On failure, returns a negative error code indicating issues like mutex
 *   lock errors.
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS

int bchlib_unlink(FAR void *handle)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  int ret = OK;

  /* Get exclusive access to the BCH device */

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Indicate that the driver has been unlinked */

  bch->unlinked = true;

  /* If there are no open references to the driver then teardown the BCH
   * device now.
   */

  if (bch->refs == 0)
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

  nxmutex_unlock(&bch->lock);
  return ret;
}
#endif
