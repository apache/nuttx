/****************************************************************************
 * drivers/bch/bchlib_ioctl.c
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
 * Name: bchlib_ioctl
 *
 * Description:
 *   Handles I/O control requests (IOCTLs) for the block device. Implements
 *   device-specific commands like retrieving private data, flushing the
 *   cache, or setting encryption keys (if encryption is enabled).
 *
 * Parameters:
 *   handle - Pointer to the BCH handle (device context).
 *   cmd    - Command indicating the requested operation.
 *   arg    - Argument for the command.
 *
 * Return:
 *   On success, returns `OK` or a command-specific result value.
 *   On failure, returns a negative error code:
 *     -ENOTTY: Command not supported.
 *     -EINVAL: Invalid argument.
 *     Other error codes may reflect underlying implementation issues.
 ****************************************************************************/

int bchlib_ioctl(FAR void *handle, int cmd, unsigned long arg)
{
  int ret = -ENOTTY;
  FAR struct bchlib_s *bch = (FAR struct bchlib_s *)handle;

  /* Process the call according to the command */

  switch (cmd)
    {
      /* This isa request to get the private data structure */

      case DIOC_GETPRIV:
        {
          FAR struct bchlib_s **bchr =
            (FAR struct bchlib_s **)((uintptr_t)arg);

          ret = nxmutex_lock(&bch->lock);
          if (ret < 0)
            {
              return ret;
            }

          if (!bchr || bch->refs == MAX_OPENCNT)
            {
              ret   = -EINVAL;
            }
          else
            {
              bch->refs++;
              *bchr = bch;
              ret   = OK;
            }

          nxmutex_unlock(&bch->lock);
        }
        break;

      /* This is a required to return the geometry of the underlying block
       * driver.
       */

      case BIOC_GEOMETRY:
        {
          FAR struct geometry *geo = (FAR struct geometry *)((uintptr_t)arg);

          DEBUGASSERT(geo != NULL && bch->inode && bch->inode->u.i_bops &&
                      bch->inode->u.i_bops->geometry);

          ret = bch->inode->u.i_bops->geometry(bch->inode, geo);
          if (ret < 0)
            {
              ferr("ERROR: geometry failed: %d\n", -ret);
            }
          else if (!geo->geo_available)
            {
              ferr("ERROR: geometry failed: %d\n", -ret);
              ret = -ENODEV;
            }
        }
        break;

#ifdef CONFIG_BCH_ENCRYPTION
      /* This is a request to set the encryption key? */

      case DIOC_SETKEY:
        {
          memcpy(bch->key, (FAR void *)arg, CONFIG_BCH_ENCRYPTION_KEY_SIZE);
          ret = OK;
        }
        break;
#endif

      case BIOC_DISCARD:
        {
          /* Invalidate the sector so next read is from the device- */

          bch->sector = (size_t)-1;
          goto ioctl_default;
        }

      case BIOC_FLUSH:
        {
          /* Flush any dirty pages remaining in the cache */

          ret = bchlib_flushsector(bch, false);
          if (ret < 0)
            {
              break;
            }

          /* Go through */
        }

      /* Pass the IOCTL command on to the contained block driver. */

ioctl_default:
      default:
        {
          FAR struct inode *bchinode = bch->inode;

          /* Does the block driver support the ioctl method? */

          if (bchinode->u.i_bops->ioctl != NULL)
            {
              ret = bchinode->u.i_bops->ioctl(bchinode, cmd, arg);

              /* Drivers may not support command BIOC_FLUSH */

              if (ret == -ENOTTY && (cmd == BIOC_FLUSH ||
                  cmd == BIOC_DISCARD))
                {
                  ret = 0;
                }
            }
        }
        break;
    }

  return ret;
}
