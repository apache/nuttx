/****************************************************************************
 * fs/vfs/fs_open.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <stdarg.h>

#include <nuttx/cancelpt.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "driver/driver.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_vopen
 ****************************************************************************/

static int file_vopen(FAR struct file *filep,
                      FAR const char *path, int oflags, va_list ap)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
#if defined(CONFIG_FILE_MODE) || !defined(CONFIG_DISABLE_MOUNTPOINT)
  mode_t mode = 0666;
#endif
  int ret;

  if (path == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_FILE_MODE
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "File creation not implemented"
#  endif

  /* If the file is opened for creation, then get the mode bits */

  if ((oflags & (O_WRONLY | O_CREAT)) != 0)
    {
      mode = va_arg(ap, mode_t);
    }
#endif

  /* Get an inode for this file */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* "O_CREAT is not set and the named file does not exist.  Or, a
       * directory component in pathname does not exist or is a dangling
       * symbolic link."
       */

      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && \
    !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
  /* If the inode is block driver, then we may return a character driver
   * proxy for the block driver.  block_proxy() will instantiate a BCH
   * character driver wrapper around the block driver, open(), then
   * unlink() the character driver.
   *
   * NOTE: This will recurse to open the character driver proxy.
   */

  if (INODE_IS_BLOCK(inode) || INODE_IS_MTD(inode))
    {
      /* Release the inode reference */

      inode_release(inode);
      RELEASE_SEARCH(&desc);

      /* Get the file structure of the opened character driver proxy */

      return block_proxy(filep, path, oflags);
    }
  else
#endif

  /* Verify that the inode is either a "normal" character driver or a
   * mountpoint.  We specifically "special" inodes (semaphores, message
   * queues, shared memory).
   */

#ifndef CONFIG_DISABLE_MOUNTPOINT
  if ((!INODE_IS_DRIVER(inode) && !INODE_IS_MOUNTPT(inode)) ||
      !inode->u.i_ops)
#else
  if (!INODE_IS_DRIVER(inode) || !inode->u.i_ops)
#endif
    {
      ret = -ENXIO;
      goto errout_with_inode;
    }

  /* Make sure that the inode supports the requested access */

  ret = inode_checkflags(inode, oflags);
  if (ret < 0)
    {
      goto errout_with_inode;
    }

  /* Associate the inode with a file structure */

  filep->f_oflags = oflags;
  filep->f_pos    = 0;
  filep->f_inode  = inode;
  filep->f_priv   = NULL;

  /* Perform the driver open operation.  NOTE that the open method may be
   * called many times.  The driver/mountpoint logic should handled this
   * because it may also be closed that many times.
   */

  ret = OK;
  if (inode->u.i_ops->open)
    {
#ifndef CONFIG_DISABLE_MOUNTPOINT
      if (INODE_IS_MOUNTPT(inode))
        {
          ret = inode->u.i_mops->open(filep, desc.relpath, oflags, mode);
        }
      else
#endif
        {
          ret = inode->u.i_ops->open(filep);
        }
    }

  if (ret < 0)
    {
      goto errout_with_inode;
    }

  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  filep->f_inode = NULL;
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: nx_vopen
 ****************************************************************************/

static int nx_vopen(FAR const char *path, int oflags, va_list ap)
{
  struct file filep;
  int ret;
  int fd;

  /* Let file_vopen() do all of the work */

  ret = file_vopen(&filep, path, oflags, ap);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a new file descriptor for the inode */

  fd = files_allocate(filep.f_inode, filep.f_oflags,
                      filep.f_pos, filep.f_priv, 0);
  if (fd < 0)
    {
      file_close(&filep);
      return fd;
    }

  return fd;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_checkflags
 *
 * Description:
 *   Check if the access described by 'oflags' is supported on 'inode'
 *
 ****************************************************************************/

int inode_checkflags(FAR struct inode *inode, int oflags)
{
  if (((oflags & O_RDOK) != 0 && !inode->u.i_ops->read) ||
      ((oflags & O_WROK) != 0 && !inode->u.i_ops->write))
    {
      return -EACCES;
    }
  else
    {
      return OK;
    }
}

/****************************************************************************
 * Name: file_open
 *
 * Description:
 *   file_open() is similar to the standard 'open' interface except that it
 *   returns an instance of 'struct file' rather than a file descriptor.  It
 *   also is not a cancellation point and does not modify the errno variable.
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *   path   - The full path to the file to be open.
 *   oflags - open flags
 *   ...    - Variable number of arguments, may include 'mode_t mode'
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int file_open(FAR struct file *filep, FAR const char *path, int oflags, ...)
{
  va_list ap;
  int ret;

  va_start(ap, oflags);
  ret = file_vopen(filep, path, oflags, ap);
  va_end(ap);

  return ret;
}

/****************************************************************************
 * Name: nx_open
 *
 * Description:
 *   nx_open() is similar to the standard 'open' interface except that is is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_open() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_open(FAR const char *path, int oflags, ...)
{
  va_list ap;
  int fd;

  /* Let nx_vopen() do all of the work */

  va_start(ap, oflags);
  fd = nx_vopen(path, oflags, ap);
  va_end(ap);

  return fd;
}

/****************************************************************************
 * Name: open
 *
 * Description:
 *   Standard 'open' interface
 *
 * Returned Value:
 *   The new file descriptor is returned on success; -1 (ERROR) is returned
 *   on any failure the errno value set appropriately.
 *
 ****************************************************************************/

int open(FAR const char *path, int oflags, ...)
{
  va_list ap;
  int fd;

  /* open() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_vopen() do most of the work */

  va_start(ap, oflags);
  fd = nx_vopen(path, oflags, ap);
  va_end(ap);

  /* Set the errno value if any errors were reported by nx_open() */

  if (fd < 0)
    {
      set_errno(-fd);
      fd = ERROR;
    }

  leave_cancellation_point();
  return fd;
}
