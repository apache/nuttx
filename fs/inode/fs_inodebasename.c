/****************************************************************************
 * fs/inode/fs_inodebasename.c
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

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_basename
 *
 * Description:
 *   Given a path with node names separated by '/', return name of last
 *   segment in the path.  ""
 *
 ****************************************************************************/

FAR const char *inode_basename(FAR const char *name)
{
  FAR const char *basename = NULL;

  for (; ; )
    {
      /* Get the name for the next path segment */

      name = inode_nextname(name);

      /* When the final segment is terminated by the NUL character, then
       * previous name that we saved is the basename.
       */

      if (name == NULL || *name == '\0')
        {
          /* Break out of the loop with basename pointer to the final
           * segment of the path.
           */

          break;
        }

      /* Set basename to point to the remainder of the path */

      basename = name;
    }

  return basename;
}
