/****************************************************************************
 * fs/rpmsgfs/rpmsgfs.h
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

#ifndef __FS_RPMSGFS_RPMSGFS_H
#define __FS_RPMSGFS_RPMSGFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSGFS_NAME_PREFIX     "rpmsgfs-"

#define RPMSGFS_OPEN            1
#define RPMSGFS_CLOSE           2
#define RPMSGFS_READ            3
#define RPMSGFS_WRITE           4
#define RPMSGFS_LSEEK           5
#define RPMSGFS_IOCTL           6
#define RPMSGFS_SYNC            7
#define RPMSGFS_DUP             8
#define RPMSGFS_FSTAT           9
#define RPMSGFS_FTRUNCATE       10
#define RPMSGFS_OPENDIR         11
#define RPMSGFS_READDIR         12
#define RPMSGFS_REWINDDIR       13
#define RPMSGFS_CLOSEDIR        14
#define RPMSGFS_STATFS          15
#define RPMSGFS_UNLINK          16
#define RPMSGFS_MKDIR           17
#define RPMSGFS_RMDIR           18
#define RPMSGFS_RENAME          19
#define RPMSGFS_STAT            20
#define RPMSGFS_FCHSTAT         21
#define RPMSGFS_CHSTAT          22

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct rpmsgfs_header_s
{
  uint32_t                command;
  int32_t                 result;
  uint64_t                cookie;
} end_packed_struct;

begin_packed_struct struct rpmsgfs_open_s
{
  struct rpmsgfs_header_s header;
  int32_t                 flags;
  int32_t                 mode;
  char                    pathname[0];
} end_packed_struct;

begin_packed_struct struct rpmsgfs_close_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
} end_packed_struct;

begin_packed_struct struct rpmsgfs_read_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
  uint32_t                count;
  char                    buf[0];
} end_packed_struct;

#define rpmsgfs_write_s rpmsgfs_read_s

begin_packed_struct struct rpmsgfs_lseek_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
  int32_t                 whence;
  int32_t                 offset;
} end_packed_struct;

begin_packed_struct struct rpmsgfs_ioctl_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
  int32_t                 request;
  uint64_t                arg;
  uint32_t                arglen;
  char                    buf[0];
} end_packed_struct;

#define rpmsgfs_sync_s rpmsgfs_close_s
#define rpmsgfs_dup_s  rpmsgfs_close_s

begin_packed_struct struct rpmsgfs_stat_priv_s
{
  uint32_t dev;       /* Device ID of device containing file */
  uint32_t mode;      /* File type, attributes, and access mode bits */
  uint32_t rdev;      /* Device ID (if file is character or block special) */
  uint16_t ino;       /* File serial number */
  uint16_t nlink;     /* Number of hard links to the file */
  int64_t  size;      /* Size of file/directory, in bytes */
  int64_t  atim_sec;  /* Time of last access, seconds */
  int64_t  atim_nsec; /* Time of last access, nanoseconds */
  int64_t  mtim_sec;  /* Time of last modification, seconds */
  int64_t  mtim_nsec; /* Time of last modification, nanoseconds */
  int64_t  ctim_sec;  /* Time of last status change, seconds */
  int64_t  ctim_nsec; /* Time of last status change, nanoseconds */
  uint64_t blocks;    /* Number of blocks allocated */
  int16_t  uid;       /* User ID of file */
  int16_t  gid;       /* Group ID of file */
  int16_t  blksize;   /* Block size used for filesystem I/O */
  uint16_t reserved;  /* Reserved space */
} end_packed_struct;

begin_packed_struct struct rpmsgfs_fstat_s
{
  struct rpmsgfs_header_s    header;
  struct rpmsgfs_stat_priv_s buf;
  union
  {
    int32_t                  fd;
    char                     pathname[0];
  };
} end_packed_struct;

begin_packed_struct struct rpmsgfs_ftruncate_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
  int32_t                 length;
} end_packed_struct;

begin_packed_struct struct rpmsgfs_opendir_s
{
  struct rpmsgfs_header_s header;
  char                    pathname[0];
} end_packed_struct;

begin_packed_struct struct rpmsgfs_readdir_s
{
  struct rpmsgfs_header_s header;
  int32_t                 fd;
  uint32_t                type;
  char                    name[0];
} end_packed_struct;

#define rpmsgfs_rewinddir_s rpmsgfs_close_s
#define rpmsgfs_closedir_s rpmsgfs_close_s

begin_packed_struct struct rpmsgfs_statfs_s
{
  struct rpmsgfs_header_s header;
  uint32_t                type;     /* Type of filesystem */
  uint32_t                reserved; /* Reserved space */
  uint64_t                namelen;  /* Maximum length of filenames */
  uint64_t                bsize;    /* Optimal block size for transfers */
  uint64_t                blocks;   /* Total data blocks in the file system of this size */
  uint64_t                bfree;    /* Free blocks in the file system */
  uint64_t                bavail;   /* Free blocks avail to non-superuser */
  uint64_t                files;    /* Total file nodes in the file system */
  uint64_t                ffree;    /* Free file nodes in the file system */
  char                    pathname[0];
} end_packed_struct;

#define rpmsgfs_unlink_s rpmsgfs_opendir_s

begin_packed_struct struct rpmsgfs_mkdir_s
{
  struct rpmsgfs_header_s header;
  int32_t                 mode;
  uint32_t                reserved;
  char                    pathname[0];
} end_packed_struct;

#define rpmsgfs_rmdir_s rpmsgfs_opendir_s
#define rpmsgfs_rename_s rpmsgfs_opendir_s
#define rpmsgfs_stat_s rpmsgfs_fstat_s

begin_packed_struct struct rpmsgfs_fchstat_s
{
  struct rpmsgfs_header_s    header;
  struct rpmsgfs_stat_priv_s buf;
  int32_t                    flags;
  union
  {
    int32_t                  fd;
    char                     pathname[0];
  };
} end_packed_struct;

#define rpmsgfs_chstat_s rpmsgfs_fchstat_s

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

int       rpmsgfs_client_open(FAR void *handle,
                              FAR const char *pathname, int flags, int mode);
int       rpmsgfs_client_close(FAR void *handle, int fd);
ssize_t   rpmsgfs_client_read(FAR void *handle, int fd,
                              FAR void *buf, size_t count);
ssize_t   rpmsgfs_client_write(FAR void *handle, int fd,
                               FAR const void *buf, size_t count);
off_t     rpmsgfs_client_lseek(FAR void *handle, int fd,
                               off_t offset, int whence);
int       rpmsgfs_client_ioctl(FAR void *handle, int fd,
                               int request, unsigned long arg);
void      rpmsgfs_client_sync(FAR void *handle, int fd);
int       rpmsgfs_client_dup(FAR void *handle, int fd);
int       rpmsgfs_client_fstat(FAR void *handle, int fd,
                               FAR struct stat *buf);
int       rpmsgfs_client_fchstat(FAR void *handle, int fd,
                                 FAR const struct stat *buf, int flags);
int       rpmsgfs_client_ftruncate(FAR void *handle, int fd, off_t length);
FAR void *rpmsgfs_client_opendir(FAR void *handle, FAR const char *name);
int       rpmsgfs_client_readdir(FAR void *handle, FAR void *dirp,
                                 FAR struct dirent *entry);
void      rpmsgfs_client_rewinddir(FAR void *handle, FAR void *dirp);
int       rpmsgfs_client_bind(FAR void **handle, FAR const char *cpuname);
int       rpmsgfs_client_unbind(FAR void *handle);
int       rpmsgfs_client_closedir(FAR void *handle, FAR void *dirp);
int       rpmsgfs_client_statfs(FAR void *handle, FAR const char *path,
                                FAR struct statfs *buf);
int       rpmsgfs_client_unlink(FAR void *handle, FAR const char *pathname);
int       rpmsgfs_client_mkdir(FAR void *handle, FAR const char *pathname,
                               mode_t mode);
int       rpmsgfs_client_rmdir(FAR void *handle, FAR const char *pathname);
int       rpmsgfs_client_rename(FAR void *handle, FAR const char *oldpath,
                                FAR const char *newpath);
int       rpmsgfs_client_stat(FAR void *handle, FAR const char *path,
                              FAR struct stat *buf);
int       rpmsgfs_client_chstat(FAR void *handle, FAR const char *path,
                                FAR const struct stat *buf, int flags);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int       rpmsgfs_server_init(void);

#endif /* __FS_RPMSGFS_RPMSGFS_H */
