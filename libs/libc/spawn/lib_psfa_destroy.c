/****************************************************************************
 * libs/libc/spawn/lib_psfa_destroy.c
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

#include <nuttx/spawn.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_destroy
 *
 * Description:
 *   The posix_spawn_file_actions_destroy() function destroys the object
 *   referenced by file_actions which was previously initializeed by
 *   posix_spawn_file_actions_init(), returning any resources obtained at the
 *   time of initialization to the system for subsequent reuse.  A
 *   posix_spawn_file_actions_t may be reinitialized after having been
 *   destroyed, but must not be reused after destruction, unless it has been
 *   reinitialized.
 *
 * Input Parameters:
 *   file_actions - The posix_spawn_file_actions_t to be destroyed.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_destroy(
                                FAR posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *curr;
  FAR struct spawn_general_file_action_s *next;

  DEBUGASSERT(file_actions);

  /* Destroy each file action, one at a time */

  for (curr = (FAR struct spawn_general_file_action_s *)*file_actions;
       curr;
       curr = next)
    {
      /* Get the pointer to the next element before destroying the current
       * one
       */

      next = curr->flink;
      lib_free(curr);
    }

  /* Mark the list empty */

  *file_actions = NULL;
  return OK;
}
