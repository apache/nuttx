/****************************************************************************
 * fs/hostfs/hostfs_rpmsg.h
 * Hostfs rpmsg driver header file
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
