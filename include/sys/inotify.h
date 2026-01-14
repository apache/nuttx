/****************************************************************************
 * include/sys/inotify.h
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

#ifndef __INCLUDE_SYS_INOTIFY_H
#define __INCLUDE_SYS_INOTIFY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IN_ACCESS        0x00000001      /* File was accessed */
#define IN_MODIFY        0x00000002      /* File was modified */
#define IN_ATTRIB        0x00000004      /* Metadata changed */
#define IN_CLOSE_WRITE   0x00000008      /* Writtable file was closed */
#define IN_CLOSE_NOWRITE 0x00000010      /* Unwrittable file closed */
#define IN_OPEN          0x00000020      /* File was opened */
#define IN_MOVED_FROM    0x00000040      /* File was moved from X */
#define IN_MOVED_TO      0x00000080      /* File was moved to Y */
#define IN_CREATE        0x00000100      /* Subfile was created */
#define IN_DELETE        0x00000200      /* Subfile was deleted */
#define IN_DELETE_SELF   0x00000400      /* Self was deleted */
#define IN_MOVE_SELF     0x00000800      /* Self was moved */

#define IN_UNMOUNT       0x00002000      /* Backing fs was unmounted */
#define IN_Q_OVERFLOW    0x00004000      /* Event queued overflowed */
#define IN_IGNORED       0x00008000      /* File was ignored */

#define IN_ONLYDIR       0x01000000      /* Only watch the path if it is a directory.  */
#define IN_DONT_FOLLOW   0x02000000      /* Do not follow a sym link.  */
#define IN_EXCL_UNLINK   0x04000000      /* Exclude events on unlinked objects.  */
#define IN_MASK_CREATE   0x10000000      /* Only create watches.  */

#define IN_MASK_ADD      0x20000000      /* Add to the mask of an already existing watch */
#define IN_ISDIR         0x40000000      /* Event occurred against dir */
#define IN_ONESHOT       0x80000000      /* Only send event once */

#define IN_CLOSE        (IN_CLOSE_WRITE | IN_CLOSE_NOWRITE)  /* Close */
#define IN_MOVE         (IN_MOVED_FROM | IN_MOVED_TO)        /* Moves */

#define IN_ALL_EVENTS   (IN_ACCESS | IN_MODIFY | IN_ATTRIB | IN_CLOSE_WRITE | \
                         IN_CLOSE_NOWRITE | IN_OPEN | IN_MOVED_FROM | IN_MOVED_TO | \
                         IN_CREATE | IN_DELETE | IN_DELETE_SELF | IN_MOVE_SELF)

#define IN_CLOEXEC       O_CLOEXEC   /* Set close_on_exec for the inotify file descriptor */
#define IN_NONBLOCK      O_NONBLOCK  /* Set O_NONBLOCK for the inotify file descriptor */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct inotify_event
{
  int      wd;              /* Watch descriptor */
  uint32_t mask;            /* Mask describing event */
  uint32_t cookie;          /* Unique cookie associating related events (for rename(2)) */
  uint32_t len;             /* Size of name field */
  char     name[0];         /* Stub for possible name */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: inotify_init
 *
 * Description:
 *  Initializes a new inotify instance and returns a file descriptor
 *  associated with a new inotify event queue.
 *
 * Returned Value:
 *  On success, these system calls return a new file descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_init(void);

/****************************************************************************
 * Name: inotify_init1
 *
 * Description:
 *  Initializes a new inotify instance and returns a file descriptor
 *  associated with a new inotify event queue.
 *
 * Input Parameters:
 *  flags - The following values are recognized in flags:
 *    IN_NONBLOCK - Set the O_NONBLOCK file status flag on the new open file
 *      description. Using this flag saves extra calls to fcntl(2) to achieve
 *      the same result.
 *    IN_CLOEXEC - Set the close-on-exec (FD_CLOEXEC) flag on the new file
 *      descriptor. See the description of the O_CLOEXEC flag in open(2) for
 *      reasons why this may be useful.
 *
 * Returned Value:
 *  On success, these system calls return a new file descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_init1(int flags);

/****************************************************************************
 * Name: inotify_add_watch
 *
 * Description:
 *  Adds a new watch, or modifies an existing watch, for the file whose
 *  location is specified in pathname; the caller must have read permission
 *  for this file.  The fd argument is a file descriptor referring to the
 *  inotify instance whose watch list is to be modified.  The events to be
 *  monitored for pathname are specified in the mask bit-mask argument.
 *
 * Input Parameters:
 *  fd - The file descriptor associated with an instance of inotify.
 *  pathname - The path to the file to be monitored.
 *  mask - The bit mask of events to be monitored.
 *
 * Returned Value:
 *  On success, inotify_add_watch() returns a nonnegative watch descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_add_watch(int fd, FAR const char *pathname, uint32_t mask);

/****************************************************************************
 * Name: inotify_rm_watch
 *
 * Description:
 *  Removes the watch associated with the watch descriptor wd from the
 *  inotify instance associated with the file descriptor fd.
 *
 * Input Parameters:
 *  fd - The file descriptor associated with an instance of inotify.
 *  wd - The watch descriptor to be removed.
 *
 * Returned Value:
 *  On success, inotify_rm_watch() returns zero.  On error, -1 is returned
 *  and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_rm_watch(int fd, int wd);

#endif
