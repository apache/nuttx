/****************************************************************************
 * fs/driver/fs_registerpseudodir.c
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

#include <sys/types.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "vfs/vfs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_pseudodir
 *
 * Description:
 *   Create an empty directory in the pseudo file system, for a driver or a
 *   file system that creates its nodes below it later (e.g. /dev/pts/N).
 *
 * Input Parameters:
 *   path - The path of the directory
 *   mode - The permissions of the directory
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure, e.g. -EEXIST if
 *   'path' exists already.
 *
 ****************************************************************************/

int register_pseudodir(FAR const char *path, mode_t mode)
{
  FAR struct inode *node;
  int               ret;

  /* An inode with no operations is a pseudo directory */

  inode_lock();
  ret = inode_reserve(path, mode, &node);
  inode_unlock();

#ifdef CONFIG_FS_NOTIFY
  if (ret >= 0)
    {
      notify_mkdir(path);
    }
#endif

  return ret;
}
