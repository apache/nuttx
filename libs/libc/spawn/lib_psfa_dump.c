/****************************************************************************
 * libs/libc/spawn/lib_psfa_dump.c
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

/* Output debug info even if debug output is not selected. */

#undef  CONFIG_DEBUG_ERROR
#undef  CONFIG_DEBUG_WARN
#undef  CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_ERROR 1
#define CONFIG_DEBUG_WARN 1
#define CONFIG_DEBUG_INFO 1

#include <spawn.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spawn.h>

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_dump
 *
 * Description:
 *   Show the entryent file actions.
 *
 * Input Parameters:
 *   file_actions - The address of the file_actions to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void posix_spawn_file_actions_dump(
                            FAR posix_spawn_file_actions_t *file_actions)
{
  FAR struct spawn_general_file_action_s *entry;

  DEBUGASSERT(file_actions);

  _err("File Actions[%p->%p]:\n", file_actions, *file_actions);
  if (!*file_actions)
    {
      _err("  NONE\n");
      return;
    }

  /* Destroy each file action, one at a time */

  for (entry = (FAR struct spawn_general_file_action_s *)*file_actions;
       entry;
       entry = entry->flink)
    {
      switch (entry->action)
        {
        case SPAWN_FILE_ACTION_CLOSE:
          {
            FAR struct spawn_close_file_action_s *action =
              (FAR struct spawn_close_file_action_s *)entry;

            _err("  CLOSE: fd=%d\n", action->fd);
          }
          break;

        case SPAWN_FILE_ACTION_DUP2:
          {
            FAR struct spawn_dup2_file_action_s *action =
              (FAR struct spawn_dup2_file_action_s *)entry;

            _err("  DUP2: %d->%d\n", action->fd1, action->fd2);
          }
          break;

        case SPAWN_FILE_ACTION_OPEN:
          {
            FAR struct spawn_open_file_action_s *action =
              (FAR struct spawn_open_file_action_s *)entry;

            _err("  OPEN: path=%s oflags=%04x mode=%04x fd=%d\n",
                action->path, action->oflags, action->mode, action->fd);
          }
          break;

        case SPAWN_FILE_ACTION_NONE:
        default:
          _err("  ERROR: Unknown action: %d\n", entry->action);
          break;
        }
    }
}

#endif /* CONFIG_DEBUG_FEATURES */
