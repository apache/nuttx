/****************************************************************************
 * fs/vfs/fs_readlink.c
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

  /* Find the inode that includes this path (without dereferencing the final)
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

  strlcpy(buf, node->u.i_link, bufsize);

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
