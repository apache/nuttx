/****************************************************************************
 * fs/hostfs/hostfs.h
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

#ifndef __FS_HOSTFS_HOSTFS_H
#define __FS_HOSTFS_HOSTFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOSTFS_MAX_PATH     256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the state of one open file.  This structure
 * is protected by the volume semaphore.
 */

struct hostfs_ofile_s
{
  struct hostfs_ofile_s    *fnext;      /* Supports a singly linked list */
  int16_t                   crefs;      /* Reference count */
  mode_t                    oflags;     /* Open mode */
  int                       fd;
};

/* This structure represents the overall mountpoint state.  An instance of
 * this structure is retained as inode private data on each mountpoint that
 * is mounted with a hostfs filesystem.
 */

struct hostfs_mountpt_s
{
  sem_t                      *fs_sem;       /* Used to assure thread-safe access */
  FAR struct hostfs_ofile_s  *fs_head;      /* A singly-linked list of open files */
  char                        fs_root[HOSTFS_MAX_PATH];
};

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/* Semaphore access for internal use */

int  hostfs_semtake(struct hostfs_mountpt_s *fs);
void hostfs_semgive(struct hostfs_mountpt_s *fs);

/* Forward references for utility functions */

struct hostfs_mountpt_s;

struct file;        /* Forward references */
struct inode;
struct fs_dirent_s;
struct statfs;
struct stat;

#endif /* __FS_HOSTFS_HOSTFS_H */
