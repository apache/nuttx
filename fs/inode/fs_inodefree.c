/****************************************************************************
 * fs/inode/fs_inodefree.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "fs_heap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_free
 *
 * Description:
 *   Free resources used by an inode
 *
 ****************************************************************************/

void inode_free(FAR struct inode *inode)
{
  /* Verify that we were passed valid pointer to an inode */

  if (inode != NULL)
    {
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      /* Symbol links should never have peers or children */

      DEBUGASSERT(!INODE_IS_SOFTLINK(inode) ||
                  (inode->i_peer == NULL && inode->i_child == NULL));
#endif

      /* Free all peers and children of this i_node */

      inode_free(inode->i_peer);
      inode_free(inode->i_child);

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      /* If the inode is a symbolic link, the free the path to the linked
       * entity.
       */

      if (INODE_IS_SOFTLINK(inode) && inode->u.i_link != NULL)
        {
          fs_heap_free(inode->u.i_link);
        }
#endif

      fs_heap_free(inode);
    }
}
