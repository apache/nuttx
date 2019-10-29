/****************************************************************************
 * fs/driver/fs_findmtddriver.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in pathname and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of pathname code must retain the above copyright
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
#include <sys/mount.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "driver/driver.h"

#ifdef CONFIG_MTD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_mtddriver
 *
 * Description:
 *   Return the inode of the named MTD driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname   - the full path to the named MTD driver to be located
 *   ppinode    - address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   ENOENT  - No MTD driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not an MTD driver
 *
 ****************************************************************************/

int find_mtddriver(FAR const char *pathname, FAR struct inode **ppinode)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret = 0; /* Assume success */

  DEBUGASSERT(pathname != NULL || ppinode != NULL);

  /* Find the inode registered with this pathname */

  SETUP_SEARCH(&desc, pathname, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s\n", pathname);
      ret = -ENOENT;
      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that the inode is a block driver. */

  if (!INODE_IS_MTD(inode))
    {
      ferr("ERROR: %s is not a named MTD driver\n", pathname);
      ret = -ENOTBLK;
      goto errout_with_inode;
    }

  /* Return the MTD inode reference */

  DEBUGASSERT(inode->u.i_mtd != NULL);

  *ppinode = inode;
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

#else

int find_mtddriver(FAR const char *pathname, FAR struct inode **ppinode)
{
  return -ENODEV;
}

#endif /* CONFIG_MTD */
