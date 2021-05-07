/****************************************************************************
 * libs/libc/spawn/lib_psfa_init.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: posix_spawn_file_actions_init
 *
 * Description:
 *   The posix_spawn_file_actions_init() function initializes the object
 *   referenced by file_actions to an empty set of file actions for
 *   subsequent use in a call to posix_spawn() or posix_spawnp().
 *
 * Input Parameters:
 *   file_actions - The address of the posix_spawn_file_actions_t to be
 *   initialized.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int posix_spawn_file_actions_init(
                FAR posix_spawn_file_actions_t *file_actions)
{
  *file_actions = NULL;
  return OK;
}
