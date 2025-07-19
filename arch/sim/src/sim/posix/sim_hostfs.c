/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostfs.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/ioctl.h>

#include <dirent.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include "hostfs.h"
#include "sim_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_stat_convert
 ****************************************************************************/

static void host_stat_convert(struct stat *hostbuf, struct nuttx_stat_s *buf)
{
  /* Map the return values */

  buf->st_mode = hostbuf->st_mode & 07777;

  if (S_ISDIR(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFDIR;
    }
  else if (S_ISREG(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFREG;
    }
  else if (S_ISCHR(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFCHR;
    }
  else if (S_ISBLK(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFBLK;
    }
  else if (S_ISLNK(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFLNK;
    }
  else if (S_ISFIFO(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFIFO;
    }
  else if (S_ISSOCK(hostbuf->st_mode))
    {
      buf->st_mode |= NUTTX_S_IFSOCK;
    }
  else if (S_TYPEISSEM(hostbuf))
    {
      buf->st_mode |= NUTTX_S_IFSEM;
    }
  else if (S_TYPEISMQ(hostbuf))
    {
      buf->st_mode |= NUTTX_S_IFMQ;
    }
  else if (S_TYPEISSHM(hostbuf))
    {
      buf->st_mode |= NUTTX_S_IFSHM;
    }

  buf->st_dev          = hostbuf->st_dev;
  buf->st_ino          = hostbuf->st_ino;
  buf->st_nlink        = hostbuf->st_nlink;
  buf->st_uid          = hostbuf->st_uid;
  buf->st_gid          = hostbuf->st_gid;
  buf->st_rdev         = hostbuf->st_rdev;
  buf->st_size         = hostbuf->st_size;
#if defined(__APPLE__)
  buf->st_atim.tv_sec  = hostbuf->st_atimespec.tv_sec;
  buf->st_atim.tv_nsec = hostbuf->st_atimespec.tv_nsec;
  buf->st_mtim.tv_sec  = hostbuf->st_mtimespec.tv_sec;
  buf->st_mtim.tv_nsec = hostbuf->st_mtimespec.tv_nsec;
  buf->st_ctim.tv_sec  = hostbuf->st_ctimespec.tv_sec;
  buf->st_ctim.tv_nsec = hostbuf->st_ctimespec.tv_nsec;
#else
  buf->st_atim.tv_sec  = hostbuf->st_atim.tv_sec;
  buf->st_atim.tv_nsec = hostbuf->st_atim.tv_nsec;
  buf->st_mtim.tv_sec  = hostbuf->st_mtim.tv_sec;
  buf->st_mtim.tv_nsec = hostbuf->st_mtim.tv_nsec;
  buf->st_ctim.tv_sec  = hostbuf->st_ctim.tv_sec;
  buf->st_ctim.tv_nsec = hostbuf->st_ctim.tv_nsec;
#endif
  buf->st_blksize      = hostbuf->st_blksize;
  buf->st_blocks       = hostbuf->st_blocks;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_open
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  int mapflags = 0;

  /* Perform flag mapping */

  if ((flags & NUTTX_O_RDWR) == NUTTX_O_RDWR)
    {
      mapflags = O_RDWR;
    }
  else if (flags & NUTTX_O_RDONLY)
    {
      mapflags = O_RDONLY;
    }
  else if (flags & NUTTX_O_WRONLY)
    {
      mapflags = O_WRONLY;
    }

  if (flags & NUTTX_O_APPEND)
    {
      mapflags |= O_APPEND;
    }

  if (flags & NUTTX_O_CREAT)
    {
      mapflags |= O_CREAT;
    }

  if (flags & NUTTX_O_EXCL)
    {
      mapflags |= O_EXCL;
    }

  if (flags & NUTTX_O_TRUNC)
    {
      mapflags |= O_TRUNC;
    }

  if (flags & NUTTX_O_NONBLOCK)
    {
      mapflags |= O_NONBLOCK;
    }

  if (flags & NUTTX_O_SYNC)
    {
      mapflags |= O_SYNC;
    }

#ifdef O_DIRECT
  if (flags & NUTTX_O_DIRECT)
    {
      mapflags |= O_DIRECT;
    }
#endif

  if (flags & NUTTX_O_CLOEXEC)
    {
      mapflags |= O_CLOEXEC;
    }

  if (flags & NUTTX_O_DIRECTORY)
    {
      mapflags |= O_DIRECTORY;
    }

  return host_uninterruptible_errno(open, pathname, mapflags, mode);
}

/****************************************************************************
 * Name: host_close
 ****************************************************************************/

int host_close(int fd)
{
  /* Just call the close routine */

  return host_uninterruptible_errno(close, fd);
}

/****************************************************************************
 * Name: host_read
 ****************************************************************************/

nuttx_ssize_t host_read(int fd, void *buf, nuttx_size_t count)
{
  /* Just call the read routine */

  return host_uninterruptible_errno(read, fd, buf, count);
}

/****************************************************************************
 * Name: host_write
 ****************************************************************************/

nuttx_ssize_t host_write(int fd, const void *buf, nuttx_size_t count)
{
  /* Just call the write routine */

  return host_uninterruptible_errno(write, fd, buf, count);
}

/****************************************************************************
 * Name: host_lseek
 ****************************************************************************/

nuttx_off_t host_lseek(int fd, nuttx_off_t pos, nuttx_off_t offset,
                       int whence)
{
  /* Just call the lseek routine */

  return host_uninterruptible_errno(lseek, fd, offset, whence);
}

/****************************************************************************
 * Name: host_ioctl
 ****************************************************************************/

int host_ioctl(int fd, int request, unsigned long arg)
{
  /* Just call the ioctl routine */

  return host_uninterruptible_errno(ioctl, fd, request, arg);
}

/****************************************************************************
 * Name: host_sync
 ****************************************************************************/

void host_sync(int fd)
{
  /* Just call the sync routine */

  fsync(fd);
}

/****************************************************************************
 * Name: host_dup
 ****************************************************************************/

int host_dup(int fd)
{
  return host_uninterruptible_errno(dup, fd);
}

/****************************************************************************
 * Name: host_fstat
 ****************************************************************************/

int host_fstat(int fd, struct nuttx_stat_s *buf)
{
  struct stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = host_uninterruptible_errno(fstat, fd, &hostbuf);

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}

/****************************************************************************
 * Name: host_fchstat
 ****************************************************************************/

int host_fchstat(int fd, const struct nuttx_stat_s *buf, int flags)
{
  struct timespec times[2];
  int ret;

  if (flags & NUTTX_CH_STAT_MODE)
    {
      ret = host_uninterruptible_errno(fchmod, fd, buf->st_mode);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (flags & (NUTTX_CH_STAT_UID | NUTTX_CH_STAT_GID))
    {
      ret = host_uninterruptible_errno(fchown, fd, buf->st_uid, buf->st_gid);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (flags & (NUTTX_CH_STAT_ATIME | NUTTX_CH_STAT_MTIME))
    {
      if (flags & NUTTX_CH_STAT_ATIME)
        {
          times[0].tv_sec  = buf->st_atim.tv_sec;
          times[0].tv_nsec = buf->st_atim.tv_nsec;
        }
      else
        {
          times[0].tv_sec = 0;
          times[0].tv_nsec = UTIME_OMIT;
        }

      if (flags & NUTTX_CH_STAT_MTIME)
        {
          times[1].tv_sec  = buf->st_mtim.tv_sec;
          times[1].tv_nsec = buf->st_mtim.tv_nsec;
        }
      else
        {
          times[1].tv_sec = 0;
          times[1].tv_nsec = UTIME_OMIT;
        }

      ret = host_uninterruptible_errno(futimens, fd, times);
      if (ret < 0)
        {
          return ret;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: host_truncate
 ****************************************************************************/

int host_ftruncate(int fd, nuttx_off_t length)
{
  return host_uninterruptible_errno(ftruncate, fd, length);
}

/****************************************************************************
 * Name: host_opendir
 ****************************************************************************/

void *host_opendir(const char *name)
{
  /* Return the host DIR pointer */

  return (void *)opendir(name);
}

/****************************************************************************
 * Name: host_readdir
 ****************************************************************************/

int host_readdir(void *dirp, struct nuttx_dirent_s *entry)
{
  struct dirent *ent;

  /* Call the host's readdir routine */

  ent = readdir(dirp);
  if (ent != NULL)
    {
      /* Copy the entry name */

      strncpy(entry->d_name, ent->d_name, sizeof(entry->d_name) - 1);
      entry->d_name[sizeof(entry->d_name) - 1] = 0;

      /* Map the type */

      if (ent->d_type == DT_REG)
        {
          entry->d_type = NUTTX_DTYPE_FILE;
        }
      else if (ent->d_type == DT_FIFO)
        {
          entry->d_type = NUTTX_DTYPE_FIFO;
        }
      else if (ent->d_type == DT_CHR)
        {
          entry->d_type = NUTTX_DTYPE_CHR;
        }
      else if (ent->d_type == DT_BLK)
        {
          entry->d_type = NUTTX_DTYPE_BLK;
        }
      else if (ent->d_type == DT_DIR)
        {
          entry->d_type = NUTTX_DTYPE_DIRECTORY;
        }
      else if (ent->d_type == DT_LNK)
        {
          entry->d_type = NUTTX_DTYPE_LINK;
        }
      else if (ent->d_type == DT_SOCK)
        {
          entry->d_type = NUTTX_DTYPE_SOCK;
        }
      else
        {
          entry->d_type = NUTTX_DTYPE_UNKNOWN;
        }

      return 0;
    }

  return -ENOENT;
}

/****************************************************************************
 * Name: host_rewinddir
 ****************************************************************************/

void host_rewinddir(void *dirp)
{
  /* Just call the rewinddir routine */

  rewinddir(dirp);
}

/****************************************************************************
 * Name: host_closedir
 ****************************************************************************/

int host_closedir(void *dirp)
{
  return host_uninterruptible_errno(closedir, dirp);
}

/****************************************************************************
 * Name: host_statfs
 ****************************************************************************/

int host_statfs(const char *path, struct nuttx_statfs_s *buf)
{
  int            ret;
  struct statvfs hostbuf;

  /* Call the host's statfs routine */

  ret = host_uninterruptible_errno(statvfs, path, &hostbuf);

  /* Map the struct statfs value */

  buf->f_type    = 0; /* hostfs overwrites f_type anyway */
  buf->f_namelen = hostbuf.f_namemax;
  buf->f_bsize   = hostbuf.f_bsize;
  buf->f_blocks  = hostbuf.f_blocks;
  buf->f_bfree   = hostbuf.f_bfree;
  buf->f_bavail  = hostbuf.f_bavail;
  buf->f_files   = hostbuf.f_files;
  buf->f_ffree   = hostbuf.f_ffree;

  return ret;
}

/****************************************************************************
 * Name: host_unlink
 ****************************************************************************/

int host_unlink(const char *pathname)
{
  return host_uninterruptible_errno(unlink, pathname);
}

/****************************************************************************
 * Name: host_mkdir
 ****************************************************************************/

int host_mkdir(const char *pathname, int mode)
{
  /* Just call the host's mkdir routine */

  return host_uninterruptible_errno(mkdir, pathname, mode);
}

/****************************************************************************
 * Name: host_rmdir
 ****************************************************************************/

int host_rmdir(const char *pathname)
{
  return host_uninterruptible_errno(rmdir, pathname);
}

/****************************************************************************
 * Name: host_rename
 ****************************************************************************/

int host_rename(const char *oldpath, const char *newpath)
{
  return host_uninterruptible_errno(rename, oldpath, newpath);
}

/****************************************************************************
 * Name: host_stat
 ****************************************************************************/

int host_stat(const char *path, struct nuttx_stat_s *buf)
{
  struct stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = host_uninterruptible_errno(stat, path, &hostbuf);

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}

/****************************************************************************
 * Name: host_chstat
 ****************************************************************************/

int host_chstat(const char *path, const struct nuttx_stat_s *buf, int flags)
{
  struct timespec times[2];
  int ret;

  if (flags & NUTTX_CH_STAT_MODE)
    {
      ret = host_uninterruptible_errno(chmod, path, buf->st_mode);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (flags & (NUTTX_CH_STAT_UID | NUTTX_CH_STAT_GID))
    {
      ret = host_uninterruptible_errno(chown, path,
                                       buf->st_uid, buf->st_gid);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (flags & (NUTTX_CH_STAT_ATIME | NUTTX_CH_STAT_MTIME))
    {
      if (flags & NUTTX_CH_STAT_ATIME)
        {
          times[0].tv_sec  = buf->st_atim.tv_sec;
          times[0].tv_nsec = buf->st_atim.tv_nsec;
        }
      else
        {
          times[0].tv_sec = 0;
          times[0].tv_nsec = UTIME_OMIT;
        }

      if (flags & NUTTX_CH_STAT_MTIME)
        {
          times[1].tv_sec  = buf->st_mtim.tv_sec;
          times[1].tv_nsec = buf->st_mtim.tv_nsec;
        }
      else
        {
          times[1].tv_sec = 0;
          times[1].tv_nsec = UTIME_OMIT;
        }

      ret = host_uninterruptible_errno(utimensat, AT_FDCWD, path, times, 0);
      if (ret < 0)
        {
          return ret;
        }
    }

  return 0;
}
