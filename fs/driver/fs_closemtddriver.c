/****************************************************************************
 * fs/driver/fs_closemtddriver.c
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

#include <errno.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: close_mtddriver
 *
 * Description:
 *   Release the inode got by function find_mtddriver()
 *
 * Input Parameters:
 *   pinode    - pointer to the inode
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - inode is NULL
 *
 ****************************************************************************/

#ifdef CONFIG_MTD
int close_mtddriver(FAR struct inode *pinode)
{
  /* Sanity checks */

  if (pinode == NULL)
    {
      return -EINVAL;
    }

  inode_release(pinode);

  return OK;
}
#else
int close_mtddriver(FAR struct inode *pinode)
{
  return -ENODEV;
}
#endif /* CONFIG_MTD */
