/****************************************************************************
 * fs/vfs/fs_close.c
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

#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_close
 *
 * Description:
 *   Close a file that was previously opened with file_open().
 *
 * Input Parameters:
 *   filep - A pointer to a user provided memory location containing the
 *           open file data returned by file_open().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_close(FAR struct file *filep)
{
  struct inode *inode;
  int ret = OK;

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  /* Check if the struct file is open (i.e., assigned an inode) */

  if (inode)
    {
      /* Close the file, driver, or mountpoint. */

      if (inode->u.i_ops && inode->u.i_ops->close)
        {
          /* Perform the close operation */

          ret = inode->u.i_ops->close(filep);
        }

      /* And release the inode */

      inode_release(inode);

      /* Reset the user file struct instance so that it cannot be reused. */

      filep->f_oflags = 0;
      filep->f_pos    = 0;
      filep->f_inode  = NULL;
      filep->f_priv   = NULL;
    }

  return ret;
}
