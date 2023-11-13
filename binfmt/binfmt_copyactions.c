/****************************************************************************
 * binfmt/binfmt_copyactions.c
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

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL) && !defined(CONFIG_BINFMT_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROUNDUP(x, y)     (((x) + (y) - 1) / (y) * (y))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_copyactions
 *
 * Description:
 *   In the kernel build, the file actions will likely lie in the caller's
 *   address environment and, hence, be inaccessible when we switch to the
 *   address environment of the new process address environment.  So we
 *   do not have any real option other than to copy the callers action list.
 *
 * Input Parameters:
 *   copy     - Pointer of file actions
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

int binfmt_copyactions(FAR const posix_spawn_file_actions_t **copy,
                       FAR const posix_spawn_file_actions_t *actions)
{
  FAR struct spawn_general_file_action_s *entry;
  FAR struct spawn_general_file_action_s *prev;
  FAR struct spawn_close_file_action_s *close;
  FAR struct spawn_open_file_action_s *open;
  FAR struct spawn_open_file_action_s *tmp;
  FAR struct spawn_dup2_file_action_s *dup2;
  FAR void *buffer;
  int size = 0;

  if (actions == NULL)
    {
      *copy = NULL;
      return OK;
    }

  for (entry = (FAR struct spawn_general_file_action_s *)actions;
       entry != NULL;
       entry = entry->flink)
    {
      switch (entry->action)
        {
          case SPAWN_FILE_ACTION_CLOSE:
            size += sizeof(struct spawn_close_file_action_s);
            break;

          case SPAWN_FILE_ACTION_DUP2:
            size += sizeof(struct spawn_dup2_file_action_s);
            break;

          case SPAWN_FILE_ACTION_OPEN:
            open = (FAR struct spawn_open_file_action_s *)entry;
            size += ROUNDUP(SIZEOF_OPEN_FILE_ACTION_S(strlen(open->path)),
                            sizeof(FAR void *));
            break;

          default:
            return -EINVAL;
        }
    }

  *copy = buffer = kmm_malloc(size);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }

  for (entry = (FAR struct spawn_general_file_action_s *)actions,
       prev = NULL; entry != NULL; prev = entry, entry = entry->flink)
    {
      switch (entry->action)
        {
          case SPAWN_FILE_ACTION_CLOSE:
            close = buffer;
            memcpy(close, entry, sizeof(struct spawn_close_file_action_s));
            close->flink = NULL;
            if (prev)
              {
                prev->flink = (FAR void *)close;
              }

            buffer = close + 1;
            break;

          case SPAWN_FILE_ACTION_DUP2:
            dup2 = buffer;
            memcpy(dup2, entry, sizeof(struct spawn_dup2_file_action_s));
            dup2->flink = NULL;
            if (prev)
              {
                prev->flink = (FAR void *)dup2;
              }

            buffer = dup2 + 1;
            break;

          case SPAWN_FILE_ACTION_OPEN:
            tmp = (FAR struct spawn_open_file_action_s *)entry;
            open = buffer;
            memcpy(open, entry, sizeof(struct spawn_open_file_action_s));
            open->flink = NULL;
            if (prev)
              {
                prev->flink = (FAR void *)open;
              }

            strcpy(open->path, tmp->path);

            buffer = (FAR char *)buffer +
                     ROUNDUP(SIZEOF_OPEN_FILE_ACTION_S(strlen(tmp->path)),
                             sizeof(FAR void *));
            break;

          default:
            break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: binfmt_freeactions
 *
 * Description:
 *   Release the copied file action list.
 *
 * Input Parameters:
 *   copy     - Pointer of file actions
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void binfmt_freeactions(FAR const posix_spawn_file_actions_t *copy)
{
  /* Is there an allocated argument buffer */

  if (copy != NULL)
    {
      /* Free the argument buffer */

      kmm_free((FAR void *)copy);
    }
}

#endif
