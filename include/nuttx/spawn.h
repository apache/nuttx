/****************************************************************************
 * include/nuttx/spawn.h
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

#ifndef __INCLUDE_NUTTX_SPAWN_H
#define __INCLUDE_NUTTX_SPAWN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <spawn.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* This enumerator identifies a file action */

enum spawn_file_actions_e
{
  SPAWN_FILE_ACTION_NONE = 0,
  SPAWN_FILE_ACTION_CLOSE,
  SPAWN_FILE_ACTION_DUP2,
  SPAWN_FILE_ACTION_OPEN
};

/* posix_spawn_file_actions_addclose(), posix_spawn_file_actions_adddup2(),
 * and posix_spawn_file_actions_addopen() will allocate memory and append
 * a new file action to an instance of posix_spawn_file_actions_t.  The
 * internal representation of these structures are defined below:
 */

struct spawn_general_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* A member of enum spawn_file_actions_e */
};

struct spawn_close_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_CLOSE */
  int fd;                                         /* The file descriptor to close */
};

struct spawn_dup2_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_DUP2 */
  int fd1;                                        /* The first file descriptor for dup2() */
  int fd2;                                        /* The second file descriptor for dup2() */
};

struct spawn_open_file_action_s
{
  FAR struct spawn_general_file_action_s *flink;  /* Supports a singly linked list */
  enum spawn_file_actions_e action;               /* SPAWN_FILE_ACTION_OPEN */
  int fd;                                         /* The file descriptor after opening */
  int oflags;                                     /* Open flags */
  mode_t mode;                                    /* File creation mode */
  char path[1];                                   /* Start of the path to be
                                                   * opened */
};

#define SIZEOF_OPEN_FILE_ACTION_S(n) \
  (sizeof(struct spawn_open_file_action_s) + (n))

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

void add_file_action(FAR posix_spawn_file_actions_t *file_action,
                     FAR struct spawn_general_file_action_s *entry);

int spawn_file_actions(FAR struct tcb_s *tcb,
                       FAR const posix_spawn_file_actions_t *actions);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SPAWN_H */
