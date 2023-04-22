/****************************************************************************
 * drivers/bch/bchdev_driver.c
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
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <poll.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/drivers/drivers.h>

#include "bch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     bch_open(FAR struct file *filep);
static int     bch_close(FAR struct file *filep);
static off_t   bch_seek(FAR struct file *filep, off_t offset, int whence);
static ssize_t bch_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t bch_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     bch_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     bch_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     bch_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct file_operations g_bch_fops =
{
  bch_open,    /* open */
  bch_close,   /* close */
  bch_read,    /* read */
  bch_write,   /* write */
  bch_seek,    /* seek */
  bch_ioctl,   /* ioctl */
  NULL,        /* mmap */
  NULL,        /* truncate */
  bch_poll     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , bch_unlink /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bch_poll
 ****************************************************************************/

static int bch_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  if (setup)
    {
      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

/****************************************************************************
 * Name: bch_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int bch_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  /* Increment the reference count */

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

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

/****************************************************************************
 * Name: bch_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int bch_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  /* Get exclusive access */

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush any dirty pages remaining in the cache */

  bchlib_flushsector(bch);

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

/****************************************************************************
 * Name: bch_seek
 ****************************************************************************/

static off_t bch_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  off_t newpos;
  off_t ret;

  DEBUGASSERT(inode && inode->i_private);

  bch = (FAR struct bchlib_s *)inode->i_private;
  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Determine the new, requested file position */

  switch (whence)
    {
    case SEEK_CUR:
      newpos = filep->f_pos + offset;
      break;

    case SEEK_SET:
      newpos = offset;
      break;

    case SEEK_END:
      newpos = bch->sectsize * bch->nsectors + offset;
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
      filep->f_pos = newpos;
      ret = newpos;
    }
  else
    {
      ret = -EINVAL;
    }

  nxmutex_unlock(&bch->lock);
  return ret;
}

/****************************************************************************
 * Name: bch_read
 ****************************************************************************/

static ssize_t bch_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  ssize_t ret;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  ret = nxmutex_lock(&bch->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = bchlib_read(bch, buffer, filep->f_pos, len);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&bch->lock);
  return ret;
}

/****************************************************************************
 * Name: bch_write
 ****************************************************************************/

static ssize_t bch_write(FAR struct file *filep, FAR const char *buffer,
                         size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  ssize_t ret = -EACCES;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

  if (!bch->readonly)
    {
      ret = nxmutex_lock(&bch->lock);
      if (ret < 0)
        {
          return ret;
        }

      ret = bchlib_write(bch, buffer, filep->f_pos, len);
      if (ret > 0)
        {
          filep->f_pos += ret;
        }

      nxmutex_unlock(&bch->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: bch_ioctl
 *
 * Description:
 *   Handle IOCTL commands
 *
 ****************************************************************************/

static int bch_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bchlib_s *bch;
  int ret = -ENOTTY;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

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

      case BIOC_FLUSH:
        {
          /* Flush any dirty pages remaining in the cache */

          ret = bchlib_flushsector(bch);
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

      /* Otherwise, pass the IOCTL command on to the contained block
       * driver.
       */

      default:
        {
          FAR struct inode *bchinode = bch->inode;

          /* Does the block driver support the ioctl method? */

          if (bchinode->u.i_bops->ioctl != NULL)
            {
              ret = bchinode->u.i_bops->ioctl(bchinode, cmd, arg);
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: bch_unlink
 *
 * Handle unlinking of the BCH device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int bch_unlink(FAR struct inode *inode)
{
  FAR struct bchlib_s *bch;
  int ret = OK;

  DEBUGASSERT(inode && inode->i_private);
  bch = (FAR struct bchlib_s *)inode->i_private;

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/
