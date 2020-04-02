/****************************************************************************
 * fs/inode/fs_inodefind.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_find
 *
 * Description:
 *   This is called from the open() logic to get a reference to the inode
 *   associated with a path.  This is accomplished by calling inode_search().
 *   inode_find() is a simple wrapper around inode_search().  The primary
 *   difference between inode_find() and inode_search is that inode_find()
 *   will lock the inode tree and increment the reference count on the inode.
 *
 ****************************************************************************/

int inode_find(FAR struct inode_search_s *desc)
{
  int ret;

  /* Find the node matching the path.  If found, increment the count of
   * references on the node.
   */

  ret = inode_semtake();
  if (ret < 0)
    {
      return ret;
    }

  ret = inode_search(desc);
  if (ret >= 0)
    {
      /* Found it */

      FAR struct inode *node = desc->node;
      DEBUGASSERT(node != NULL);

      /* Increment the reference count on the inode */

      node->i_crefs++;
    }

  inode_semgive();
  return ret;
}
