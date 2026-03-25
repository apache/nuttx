/****************************************************************************
 * fs/vfs/vfs.h
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

#ifndef __FS_VFS_VFS_H
#define __FS_VFS_VFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/fs/fs.h>
#include <fcntl.h>

#ifdef CONFIG_FS_PROFILER
#include <nuttx/clock.h>
#include <nuttx/atomic.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_FS_LOCK_BUCKET_SIZE == 0
#  define file_initlk()
#  define file_closelk(filep)
#  define file_getlk(filep, flock) ((void)flock, -ENOSYS)
#  define file_setlk(filep, flock, nonblock) ((void)flock, -ENOSYS)
#else

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: file_initlk
 *
 * Description:
 *   Initializing file locks
 *
 ****************************************************************************/

void file_initlk(void);

/****************************************************************************
 * Name: file_closelk
 *
 * Description:
 *   Remove all locks associated with the filep when call close is applied.
 *
 * Input Parameters:
 *   filep - The filep that corresponds to the shutdown.
 *
 ****************************************************************************/

void file_closelk(FAR struct file *filep);

/****************************************************************************
 * Name: file_getlk
 *
 * Description:
 *   Attempts to lock the region (not a real lock), and if there is a
 *   conflict then returns information about the conflicting locks
 *
 * Input Parameters:
 *   filep - File structure instance
 *   flock - Lock types to be converted
 *
 * Returned Value:
 *   The resulting 0 on success. A errno value is returned on any failure.
 *
 ****************************************************************************/

int file_getlk(FAR struct file *filep, FAR struct flock *flock);

/****************************************************************************
 * Name: file_setlk
 *
 * Description:
 *   Actual execution of locking and unlocking behaviors
 *
 * Input Parameters:
 *   filep    - File structure instance
 *   flock    - Lock types to be converted
 *   nonblock - Waiting for lock
 *
 * Returned Value:
 *   The resulting 0 on success. A errno value is returned on any failure.
 *
 ****************************************************************************/

int file_setlk(FAR struct file *filep, FAR struct flock *flock,
               bool nonblock);

#endif /* CONFIG_FS_LOCK_BUCKET_SIZE */

#ifdef CONFIG_FS_NOTIFY
void notify_open(FAR const char *path, int oflags);
void notify_close(FAR const char *path, int oflags);
void notify_close2(FAR struct inode *inode);
void notify_read(FAR struct file *filep);
void notify_write(FAR struct file *filep);
void notify_chstat(FAR struct file *filep);
void notify_unlink(FAR const char *path);
void notify_unmount(FAR const char *path);
void notify_mkdir(FAR const char *path);
void notify_create(FAR const char *path);
void notify_rename(FAR const char *oldpath, bool oldisdir,
                   FAR const char *newpath, bool newisdir);
void notify_initialize(void);
#endif /* CONFIG_FS_NOTIFY */

#ifdef CONFIG_FS_PROFILER

struct fs_profile_s
{
  atomic_t   reads;
  atomic_t   writes;
  atomic_t   opens;
  atomic_t   closes;
  atomic64_t total_read_time;
  atomic64_t total_write_time;
  atomic64_t total_open_time;
  atomic64_t total_close_time;
};

extern struct fs_profile_s g_fs_profile;

void fs_profile_start(FAR clock_t *start);
void fs_profile_stop(FAR clock_t *start, FAR atomic64_t *total,
                      FAR atomic_t *count);

#define FS_PROFILE_START(start_time) fs_profile_start(&start_time)
#define FS_PROFILE_STOP(start_time, total_time, count) \
  fs_profile_stop(&start_time, &total_time, &count)

#else

#define FS_PROFILE_START(start_time) ((void)(start_time))
#define FS_PROFILE_STOP(start_time, total_time, count) ((void)(start_time))

#endif /* CONFIG_FS_PROFILER */

#endif /* __FS_VFS_VFS_H */
