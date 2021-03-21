/****************************************************************************
 * libs/libc/spawn/lib_psfa_addclose.c
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
#include <spawn.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/spawn.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_addclose
 *
 * Description:
 *   The posix_spawn_file_actions_addclose() function adds a close operation
 *   to the list of operations associated with the object referenced by
 *   file_actions, for subsequent use in a call to posix_spawn() or
 *   posix_spawnp().  The descriptor referred to by fd is closed as if
 *   close() had been called on it prior to the new child process starting
 *   execution.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to append the action.
 *   fd - The file descriptor to be closed.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_addclose(
                      FAR posix_spawn_file_actions_t *file_actions,
                      int fd)
{
  FAR struct spawn_close_file_action_s *entry;

  DEBUGASSERT(file_actions && fd >= 0);

  /* Allocate the action list entry */

  entry = (FAR struct spawn_close_file_action_s *)
    lib_zalloc(sizeof(struct spawn_close_file_action_s));

  if (!entry)
    {
      return ENOMEM;
    }

  /* Initialize the file action entry */

  entry->action = SPAWN_FILE_ACTION_CLOSE;
  entry->fd     = fd;

  /* And add it to the file action list */

  add_file_action(file_actions,
                 (FAR struct spawn_general_file_action_s *)entry);
  return OK;
}
