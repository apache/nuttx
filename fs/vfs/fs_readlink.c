/****************************************************************************
 * fs/vfs/fs_readlink.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT will THE
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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkdir
 *
 * Description:
 *   The readlink() function will place the contents of the symbolic link
 *   referred to by 'path' in the buffer 'buf' which has size 'bufsize'. If
 *   the number of bytes in the symbolic link is less than bufsize, the
 *   contents of the remainder of 'buf' are unspecified. If the buf argument
 *   is not large enough to contain the link content, the first bufsize bytes
 *   will be placed in buf.  If the value of bufsize is greater than
 *   {SSIZE_MAX}, the result is implementation-defined. *
 *
 * Input Parameters:
 *   path   - The full path to the symbolic link
 *   buf    - The user-provided buffer in which to return the path to the
 *            link target.
 *   bufixe - The size of 'buf'
 *
 * Returned Value:
 *   Upon successful completion, readlink() will return the count of bytes
 *   placed in the buffer. Otherwise, it will return a value of -1, leave the
 *   buffer unchanged, and set errno to indicate the error.
 *
 ****************************************************************************/

ssize_t readlink(FAR const char *path, FAR char *buf, size_t bufsize)
{
  struct inode_search_s desc;
  FAR struct inode *node;
  int errcode;
  int ret;

  DEBUGASSERT(path != NULL && buf != NULL && bufsize > 0);

  /* Find the inode that includes this path (without derefencing the final)
   * symbolic link node.
   */

  SETUP_SEARCH(&desc, path, true);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_search;
    }

  /* Get the search results */

  node = desc.node;
  DEBUGASSERT(node != NULL);

  /* An inode was found that includes this path and possibly refers to a
   * symbolic link.
   *
   * Check if the inode is a valid symbolic link.
   */

  if (!INODE_IS_SOFTLINK(node))
    {
      errcode = EINVAL;
      goto errout_with_inode;
    }

  /* Copy the link target pathto the user-provided buffer. */

  *buf = '\0';
  (void)strncpy(buf, node->u.i_link, bufsize);

  /* Release our reference on the inode and return the length */

  inode_release(node);
  RELEASE_SEARCH(&desc);
  return strlen(buf);

errout_with_inode:
  inode_release(node);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  return ERROR;
}

#endif /* CONFIG_PSEUDOFS_SOFTLINKS */
