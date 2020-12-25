/****************************************************************************
 * libs/libc/unistd/lib_fstatvfs.c
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

#include <sys/statfs.h>
#include <sys/statvfs.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fstatvfs
 *
 * Description:
 *   The fstatvfs() function shall obtain information about the file system
 *   containing the file referenced by fd.
 *
 *   the buf argument is a pointer to a statvfs structure that shall be
 *   filled. Read, write, or execute permission of the named file is not
 *   required.
 *
 * Returned Value:
 *   Upon successful completion, statvfs() shall return 0. Otherwise, it
 *   shall return -1 and set errno to indicate the error.
 *
 ****************************************************************************/

int fstatvfs(int fd, FAR struct statvfs *buf)
{
  struct statfs tmp;
  int ret;

  ret = fstatfs(fd, &tmp);
  if (ret < 0)
    {
      return ret;
    }

  buf->f_bsize   = tmp.f_bsize;
  buf->f_frsize  = tmp.f_bsize;
  buf->f_blocks  = tmp.f_blocks;
  buf->f_bfree   = tmp.f_bfree;
  buf->f_bavail  = tmp.f_bavail;
  buf->f_files   = tmp.f_files;
  buf->f_ffree   = tmp.f_ffree;
  buf->f_favail  = tmp.f_ffree;
  buf->f_fsid    = 0;
  buf->f_flag    = 0;
  buf->f_namemax = tmp.f_namelen;

  return ret;
}
