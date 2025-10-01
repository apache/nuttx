/****************************************************************************
 * fs/driver/fs_finddriver.c
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
#include <debug.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_driver
 *
 * Description:
 *   Returns the pointer of a registered driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname - the full path to the driver's device node in file system
 *
 * Returned Value:
 *   Pointer to driver's registered private pointer or NULL if not found.
 *
 ****************************************************************************/

FAR void *find_driver(FAR const char *pathname)
{
  struct inode_search_s desc;
  FAR void *drvr = NULL;

  DEBUGASSERT(pathname != NULL);

  /* Find the inode registered with this pathname */

  SETUP_SEARCH(&desc, pathname, false);

  /* Get the search results */

  inode_lock();
  if (inode_find(&desc) < 0)
    {
      ferr("ERROR: Failed to find %s\n", pathname);
    }
  else
    {
      drvr = desc.node->i_private;
      inode_release(desc.node);
    }

  inode_unlock();
  RELEASE_SEARCH(&desc);

  return drvr;
}
