/****************************************************************************
 * libs/libc/spawn/lib_psfa_addaction.c
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

#include <spawn.h>

#include <nuttx/spawn.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: add_file_action
 *
 * Description:
 *   Add the file action to the end for the file action list.
 *
 * Input Parameters:
 *   file_actions - The head of the file action list.
 *   entry - The file action to be added
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void add_file_action(FAR posix_spawn_file_actions_t *file_actions,
                     FAR struct spawn_general_file_action_s *entry)
{
  FAR struct spawn_general_file_action_s *prev;
  FAR struct spawn_general_file_action_s *next;

  /* Find the end of the list */

  for (prev = NULL,
       next = (FAR struct spawn_general_file_action_s *)*file_actions;
       next;
       prev = next, next = next->flink);

  /* Here next is NULL and prev points to the last entry in the list (or
   * is NULL if the list is empty).
   */

  if (prev)
    {
      prev->flink = entry;
    }
  else
    {
      *file_actions = (posix_spawn_file_actions_t)entry;
    }

  entry->flink = NULL;
}
