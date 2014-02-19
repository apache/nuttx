/****************************************************************************
 * fs/fs_mkdir.c
 *
 *   Copyright (C) 2007, 2008, 2014 Gregory Nutt. All rights reserved.
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
#include <sys/stat.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include "fs_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkdir
 *
 * Description:  Create a directory
 *
 ****************************************************************************/

int mkdir(const char *pathname, mode_t mode)
{
  FAR struct inode *inode;
  const char       *relpath = NULL;
  int               errcode;
  int               ret;

  /* Find the inode that includes this path */

  inode = inode_find(pathname, &relpath);
  if (inode)
    {
      /* An inode was found that includes this path and possibly refers to a
       * mountpoint.
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Check if the inode is a valid mountpoint. */

      if (!INODE_IS_MOUNTPT(inode) || !inode->u.i_mops)
        {
          /* The inode is not a mountpoint */

          errcode = ENXIO;
          goto errout_with_inode;
        }

      /* Perform the mkdir operation using the relative path
       * at the mountpoint.
       */

      if (inode->u.i_mops->mkdir)
        {
          ret = inode->u.i_mops->mkdir(inode, relpath, mode);
          if (ret < 0)
            {
              errcode = -ret;
              goto errout_with_inode;
            }
        }
      else
        { 
          errcode = ENOSYS;
          goto errout_with_inode;
        }

      /* Release our reference on the inode */

      inode_release(inode);
#else
      /* But mountpoints are not supported in this configuration */

      errocode = EEXIST;
      goto errout_with_inode;     
#endif
    }

  /* No inode exists that contains this path.  Create a new inode in the
   * pseudo-filesystem at this location.
   */

  else
    {
      /* Create an inode in the pseudo-filesystem at this path */

      inode_semtake();
      ret = inode_reserve(pathname, &inode);
      inode_semgive();

      if (ret < 0)
        {
          errcode = -ret;
          goto errout;
        }
    }

  /* Directory successfully created */

  return OK;

 errout_with_inode:
  inode_release(inode);
 errout:
  set_errno(errcode);
  return ERROR;
}
