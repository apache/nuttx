/****************************************************************************
 * fs/vfs/fs_link.c
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

#include <errno.h>
#include <unistd.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: link
 *
 * Description:
 *  The link function provides a wrapper to symlink when pseudo filesystem
 *  softlinks are enabled. Otherwise, hard links are unsupported.
 *
 * Input Parameters:
 *   path1 - Points to a pathname naming an existing file.
 *   path2 - Points to a pathname naming the new directory entry to be
 *           created.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  Otherwise, -1 (ERROR) is returned
 *   the errno variable is set appropriately.
 *
 ****************************************************************************/

int link(FAR const char *path1, FAR const char *path2)
{
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  return symlink(path1, path2);
#else
  (void)path1;
  (void)path2;

  set_errno(ENOSYS);
  return ERROR;
#endif
}
