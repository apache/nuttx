/****************************************************************************
 * binfmt/binfmt_copyactions.c
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

#include <string.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL) && !defined(CONFIG_BINFMT_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_FILE_ACTIONS 256

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
 *   copy     - Pointer of the copied output file actions
 *   actions  - Pointer of file actions to be copy
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

int binfmt_copyactions(FAR const posix_spawn_file_actions_t **copy,
                       FAR const posix_spawn_file_actions_t *actions)
{
  FAR struct spawn_general_file_action_s *entry;
  FAR struct spawn_general_file_action_s *prev = NULL;
  FAR struct spawn_open_file_action_s *open;
  FAR struct spawn_open_file_action_s *src;
  enum spawn_file_actions_e action;
  FAR char *buffer;
  FAR char *end;
  size_t size = 0;
  size_t len;
  int count = 0;
  int i;

  *copy = NULL;
  if (actions == NULL)
    {
      return OK;
    }

  for (entry = (FAR struct spawn_general_file_action_s *)actions;
       entry != NULL;
       entry = entry->flink)
    {
      if (++count > MAX_FILE_ACTIONS)
        {
          return -EFAULT;
        }

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
            size += ALIGN_UP(SIZEOF_OPEN_FILE_ACTION_S(strlen(open->path)),
                             sizeof(FAR void *));
            break;

          default:
            return -EINVAL;
        }
    }

  buffer = kmm_malloc(size);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }

  *copy = (FAR const posix_spawn_file_actions_t *)buffer;
  end   = buffer + size;
  entry = (FAR struct spawn_general_file_action_s *)actions;

  for (i = 0; i < count; i++, entry = entry->flink)
    {
      if (entry == NULL)
        {
          goto errout;
        }

      action = entry->action;
      switch (action)
        {
          case SPAWN_FILE_ACTION_CLOSE:
            len = sizeof(struct spawn_close_file_action_s);
            break;

          case SPAWN_FILE_ACTION_DUP2:
            len = sizeof(struct spawn_dup2_file_action_s);
            break;

          case SPAWN_FILE_ACTION_OPEN:
            len = sizeof(struct spawn_open_file_action_s);
            break;

          default:
            goto errout;
        }

      if (len > end - buffer)
        {
          goto errout;
        }

      memcpy(buffer, entry, len);
      if (action == SPAWN_FILE_ACTION_OPEN)
        {
          open = (FAR struct spawn_open_file_action_s *)buffer;
          src  = (FAR struct spawn_open_file_action_s *)entry;
          len  = strnlen(src->path, end - buffer - len);
          memcpy(open->path, src->path, len);
          open->path[len] = '\0';
          len = ALIGN_UP(SIZEOF_OPEN_FILE_ACTION_S(len), sizeof(FAR void *));
        }

      ((FAR struct spawn_general_file_action_s *)buffer)->flink  = NULL;
      ((FAR struct spawn_general_file_action_s *)buffer)->action = action;
      if (prev)
        {
          prev->flink = (FAR struct spawn_general_file_action_s *)buffer;
        }

      prev    = (FAR struct spawn_general_file_action_s *)buffer;
      buffer += len;
    }

  return OK;

errout:
  kmm_free((FAR void *)*copy);
  *copy = NULL;
  return -EFAULT;
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
