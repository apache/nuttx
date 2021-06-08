/****************************************************************************
 * fs/vfs/fs_chown.c
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

#include <errno.h>
#include <unistd.h>
#include <pwd.h>
#include <grp.h>
#include <assert.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chown
 *
 * Description:
 *   changes the ownership of the file specified by pathname, which is
 *   dereferenced if it is a symbolic link.
 *
 * NOTE: Due to the characteristics of fs, this function is incomplete.s
 *
 * Input Parameters:
 *   pathname: path of file.
 *   owner   : User id.
 *   group   : Group id.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; -1 is returned on fail, and errno is
 *   set appropriately.
 *
 ****************************************************************************/

int chown(FAR const char *pathname, uid_t owner, gid_t group)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret;

  if (pathname == NULL || (owner <= -1 && group <= -1))
    {
      set_errno(-EINVAL);
      return ERROR;
    }
  /* Get an inode for this file */

  SETUP_SEARCH(&desc, pathname, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* "O_CREAT is not set and the named file does not exist.  Or, a
       * directory component in pathname does not exist or is a dangling
       * symbolic link."
       */

      RELEASE_SEARCH(&desc);
      set_errno(ret);
      return ERROR;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Set uid and gid */

  inode->i_uid = owner > -1 ? owner : inode->i_uid;
  inode->i_gid = group > -1 ? group : inode->i_gid;

  RELEASE_SEARCH(&desc);
  inode_release(inode);
  return OK;
}
