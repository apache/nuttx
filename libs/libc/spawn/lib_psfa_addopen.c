/****************************************************************************
 * libs/libc/spawn/lib_psfa_addopen.c
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

#include <stdlib.h>
#include <string.h>
#include <spawn.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spawn.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_addopen
 *
 * Description:
 *   The posix_spawn_file_actions_addopen() function adds an open operation
 *   to the list of operations associated with the object referenced by
 *   file_actions, for subsequent use in a call to posix_spawn() or
 *   posix_spawnp().  The descriptor referred to by fd is opened using
 *   the path, oflag, and mode arguments as if open() had been called on it
 *   prior to the new child process starting execution.  The string path is
 *   copied by the posix_spawn_file_actions_addopen() function during this
 *   process, so storage need not be persistent in the caller.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to append the action.
 *   fd - The file descriptor to be opened.
 *   path - The path to be opened.
 *   oflags - Open flags
 *   mode - File creation mode
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_addopen(
                                FAR posix_spawn_file_actions_t *file_actions,
                                int fd, FAR const char *path, int oflags,
                                mode_t mode)
{
  FAR struct spawn_open_file_action_s *entry;
  size_t len;
  size_t alloc;

  DEBUGASSERT(file_actions && path && fd >= 0);

  /* Get the size of the action including storage for the path plus its NUL
   * terminating character.
   */

  len   = strlen(path);
  alloc = SIZEOF_OPEN_FILE_ACTION_S(len);

  /* Allocate the action list entry of this size */

  entry = (FAR struct spawn_open_file_action_s *)lib_zalloc(alloc);
  if (!entry)
    {
      return ENOMEM;
    }

  /* Initialize the file action entry */

  entry->action = SPAWN_FILE_ACTION_OPEN;
  entry->fd     = fd;
  entry->oflags = oflags;
  entry->mode   = mode;
  strlcpy(entry->path, path, len + 1);

  /* And add it to the file action list */

  add_file_action(file_actions,
                  (FAR struct spawn_general_file_action_s *)entry);
  return OK;
}
