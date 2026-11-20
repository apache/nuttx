/****************************************************************************
 * fs/driver/fs_blockproxy.c
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

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/lib/lib.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>

#include "driver.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: block_proxy
 *
 * Description:
 *   Create a temporary char driver using drivers/bch to mediate character
 *   oriented accessed to the block driver.
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *   blkdev - The path to the block driver
 *   oflags - Character driver open flags
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int block_proxy(FAR struct file *filep, FAR const char *blkdev, int oflags)
{
  char chardev[16];
  struct file temp;
  int ret;

  DEBUGASSERT(blkdev);

  /* Create a unique temporary file name for the character device */

  ret = unique_devname('c', chardev, sizeof(chardev));
  if (ret != OK)
    {
      ferr("ERROR: Failed to create temporary device name\n");
      return ret;
    }

  /* Wrap the block driver with an instance of the BCH driver */

  ret = bchdev_register(blkdev, chardev, oflags);
  if (ret < 0)
    {
      ferr("ERROR: bchdev_register(%s, %s) failed: %d\n",
           blkdev, chardev, ret);
      return ret;
    }

  /* Open the newly created character driver */

  oflags &= ~(O_CREAT | O_EXCL | O_APPEND | O_TRUNC);
  ret = file_open(&temp, chardev, oflags);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", chardev, ret);
      nx_unlink(chardev);
      return ret;
    }

  ret = file_dup2(&temp, filep);
  file_close(&temp);
  if (ret < 0)
    {
      ferr("ERROR: Failed to dup2%s: %d\n", chardev, ret);
      nx_unlink(chardev);
      return ret;
    }

  /* Unlink the character device name.  The driver instance will persist,
   * provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise, we have
   * a problem here!)
   */

  ret = nx_unlink(chardev);
  if (ret < 0)
    {
      ferr("ERROR: Failed to unlink %s: %d\n", chardev, ret);
    }

  return ret;
}

#endif /* !CONFIG_DISABLE_MOUNTPOINT && !CONFIG_DISABLE_PSEUDOFS_OPERATIONS */
