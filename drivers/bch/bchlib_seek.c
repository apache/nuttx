/****************************************************************************
 * drivers/bch/bchlib_seek.c
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
 * Name: bchlib_seek
 *
 * Description:
 *   Adjusts the current position within the block device based on the
 *   provided offset and the specified seek mode (whence).
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *   offset - The offset value used to calculate the new position.
 *   whence - Specifies the base position for seeking. It can take one of
 *            the following values:
 *            - SEEK_SET: Seek from the start of the device.
 *            - SEEK_CUR: Seek from the current position.
 *            - SEEK_END: Seek from the end of the device.
 *   pos    - Pointer to the position value that will be updated to reflect
 *            the new position after seeking.
 *
 * Return:
 *   On success, returns the new position within the block device.
 *   On failure, returns a negative error code, such as:
 *     -EINVAL: Invalid value for `whence` or if the resulting position is
 *              negative.
 *     Other error codes may reflect mutex lock issues.
 ****************************************************************************/

off_t bchlib_seek(FAR void *handle, off_t offset, int whence, off_t *pos)
{
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;
  off_t newpos;
  off_t ret;

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = *pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = (off_t)bch->sectsize * bch->nsectors + offset;
      break;

    default:

      /* Return EINVAL if the whence argument is invalid */

      nxmutex_unlock(&bch->lock);
      return -EINVAL;
    }

  /* Opengroup.org:
   *
   *  "The lseek() function shall allow the file offset to be set beyond the
   *   end of the existing data in the file. If data is later written at this
   *   point, subsequent reads of data in the gap shall return bytes with the
   *   value 0 until data is actually written into the gap."
   *
   * We can conform to the first part, but not the second. But return -EINVAL
   * if:
   *
   *  "...the resulting file offset would be negative for a regular file,
   *  block special file, or directory."
   */

  if (newpos >= 0)
    {
      *pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&bch->lock);
  return ret;
}
