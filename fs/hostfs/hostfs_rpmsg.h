/****************************************************************************
 * fs/hostfs/hostfs_rpmsg.h
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

#ifndef __FS_HOSTFS_HOSTFS_RPMSG_H
#define __FS_HOSTFS_HOSTFS_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/stat.h>
#include <sys/statfs.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)             (sizeof(x) / sizeof((x)[0]))
#endif

#define HOSTFS_RPMSG_EPT_NAME       "rpmsg-hostfs"

#define HOSTFS_RPMSG_OPEN           1
#define HOSTFS_RPMSG_CLOSE          2
#define HOSTFS_RPMSG_READ           3
#define HOSTFS_RPMSG_WRITE          4
#define HOSTFS_RPMSG_LSEEK          5
#define HOSTFS_RPMSG_IOCTL          6
#define HOSTFS_RPMSG_SYNC           7
#define HOSTFS_RPMSG_DUP            8
#define HOSTFS_RPMSG_FSTAT          9
#define HOSTFS_RPMSG_FTRUNCATE      10
#define HOSTFS_RPMSG_OPENDIR        11
#define HOSTFS_RPMSG_READDIR        12
#define HOSTFS_RPMSG_REWINDDIR      13
#define HOSTFS_RPMSG_CLOSEDIR       14
#define HOSTFS_RPMSG_STATFS         15
#define HOSTFS_RPMSG_UNLINK         16
#define HOSTFS_RPMSG_MKDIR          17
#define HOSTFS_RPMSG_RMDIR          18
#define HOSTFS_RPMSG_RENAME         19
#define HOSTFS_RPMSG_STAT           20

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct hostfs_rpmsg_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t cookie;
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_open_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      flags;
  int32_t                      mode;
  char                         pathname[0];
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_close_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_read_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  uint32_t                     count;
  char                         buf[0];
} end_packed_struct;

#define hostfs_rpmsg_write_s hostfs_rpmsg_read_s

begin_packed_struct struct hostfs_rpmsg_lseek_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  int32_t                      whence;
  int32_t                      offset;
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_ioctl_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  int32_t                      request;
  int32_t                      arg;
} end_packed_struct;

#define hostfs_rpmsg_sync_s hostfs_rpmsg_close_s
#define hostfs_rpmsg_dup_s  hostfs_rpmsg_close_s

begin_packed_struct struct hostfs_rpmsg_fstat_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  uint32_t                     reserved;
  struct stat                  buf;
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_ftruncate_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  int32_t                      length;
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_opendir_s
{
  struct hostfs_rpmsg_header_s header;
  char                         pathname[0];
} end_packed_struct;

begin_packed_struct struct hostfs_rpmsg_readdir_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      fd;
  uint32_t                     type;
  char                         name[0];
} end_packed_struct;

#define hostfs_rpmsg_rewinddir_s hostfs_rpmsg_close_s
#define hostfs_rpmsg_closedir_s hostfs_rpmsg_close_s

begin_packed_struct struct hostfs_rpmsg_statfs_s
{
  struct hostfs_rpmsg_header_s header;
  union
  {
    struct statfs              buf;
    uint32_t                   reserved[16];
  };

  char                         pathname[0];
} end_packed_struct;

#define hostfs_rpmsg_unlink_s hostfs_rpmsg_opendir_s

begin_packed_struct struct hostfs_rpmsg_mkdir_s
{
  struct hostfs_rpmsg_header_s header;
  int32_t                      mode;
  uint32_t                     reserved;
  char                         pathname[0];
} end_packed_struct;

#define hostfs_rpmsg_rmdir_s hostfs_rpmsg_opendir_s
#define hostfs_rpmsg_rename_s hostfs_rpmsg_opendir_s

begin_packed_struct struct hostfs_rpmsg_stat_s
{
  struct hostfs_rpmsg_header_s header;
  union
  {
    struct stat                buf;
    uint32_t                   reserved[16];
  };

  char                         pathname[0];
} end_packed_struct;

#endif /* __FS_HOSTFS_HOSTFS_RPMSG_H */
