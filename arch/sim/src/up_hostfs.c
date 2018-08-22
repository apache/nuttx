/****************************************************************************
 * arch/sim/src/up_hostfs.c
 *
 *   Copyright (C) 2015 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#define _BSD_SOURCE

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <sys/ioctl.h>

#include <dirent.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#define __SIM__ 1
#include "hostfs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_stat_convert
 ****************************************************************************/

static void host_stat_convert(struct stat *hostbuf, struct nuttx_stat_s *buf)
{
  /* Map the return values */

  buf->st_mode = hostbuf->st_mode & 0777;

  if (hostbuf->st_mode & S_IFDIR)
    {
      buf->st_mode |= NUTTX_S_IFDIR;
    }
  else if (hostbuf->st_mode & S_IFREG)
    {
      buf->st_mode |= NUTTX_S_IFREG;
    }
  else if (hostbuf->st_mode & S_IFCHR)
    {
      buf->st_mode |= NUTTX_S_IFCHR;
    }
  else if (hostbuf->st_mode & S_IFBLK)
    {
      buf->st_mode |= NUTTX_S_IFBLK;
    }
  else if (hostbuf->st_mode & S_IFLNK)
    {
      buf->st_mode |= NUTTX_S_IFLNK;
    }
  else /* if (hostbuf->st_mode & S_IFIFO) */
    {
      buf->st_mode |= NUTTX_S_IFIFO;
    }

  buf->st_size    = hostbuf->st_size;
  buf->st_blksize = hostbuf->st_blksize;
  buf->st_blocks  = hostbuf->st_blocks;
  buf->st_atim    = hostbuf->st_atime;
  buf->st_mtim    = hostbuf->st_mtime;
  buf->st_ctim    = hostbuf->st_ctime;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_open
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  int mapflags;

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

  return open(pathname, mapflags, mode);
}

/****************************************************************************
 * Name: host_close
 ****************************************************************************/

int host_close(int fd)
{
  /* Just call the close routine */

  return close(fd);
}

/****************************************************************************
 * Name: host_read
 ****************************************************************************/

ssize_t host_read(int fd, void* buf, size_t count)
{
  /* Just call the read routine */

  return read(fd, buf, count);
}

/****************************************************************************
 * Name: host_write
 ****************************************************************************/

ssize_t host_write(int fd, const void *buf, size_t count)
{
  /* Just call the write routine */

  return write(fd, buf, count);
}

/****************************************************************************
 * Name: host_lseek
 ****************************************************************************/

off_t host_lseek(int fd, off_t offset, int whence)
{
  /* Just call the lseek routine */

  return lseek(fd, offset, whence);
}

/****************************************************************************
 * Name: host_ioctl
 ****************************************************************************/

int host_ioctl(int fd, int request, unsigned long arg)
{
  /* Just call the ioctl routine */

  return ioctl(fd, request, arg);
}

/****************************************************************************
 * Name: host_sync
 ****************************************************************************/

void host_sync(int fd)
{
  /* Just call the sync routine */

  sync();
}

/****************************************************************************
 * Name: host_dup
 ****************************************************************************/

int host_dup(int fd)
{
  return dup(fd);
}

/****************************************************************************
 * Name: host_fstat
 ****************************************************************************/

int host_fstat(int fd, struct nuttx_stat_s *buf)
{
  struct stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = fstat(fd, &hostbuf);

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}

/****************************************************************************
 * Name: host_truncate
 ****************************************************************************/

int host_ftruncate(int fd, off_t length)
{
  return ftruncate(fd, length);
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

int host_readdir(void* dirp, struct nuttx_dirent_s* entry)
{
  struct dirent *ent;

  /* Call the host's readdir routine */

  ent = readdir(dirp);
  if (ent != NULL)
    {
      /* Copy the entry name */

      strncpy(entry->d_name, ent->d_name, sizeof(entry->d_name));

      /* Map the type */

      entry->d_type = 0;
      if (ent->d_type == DT_REG)
        {
          entry->d_type = NUTTX_DTYPE_FILE;
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
  return closedir(dirp);
}

/****************************************************************************
 * Name: host_statfs
 ****************************************************************************/

int host_statfs(const char *path, struct nuttx_statfs_s *buf)
{
  int           ret;
  struct statfs hostbuf;

  /* Call the host's statfs routine */

  ret = statfs(path, &hostbuf);

  /* Map the struct statfs value */

  buf->f_type    = hostbuf.f_type;
  buf->f_namelen = hostbuf.f_namelen;
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
  return unlink(pathname);
}

/****************************************************************************
 * Name: host_mkdir
 ****************************************************************************/

int host_mkdir(const char *pathname, mode_t mode)
{
  /* Just call the host's mkdir routine */

  return mkdir(pathname, mode);
}

/****************************************************************************
 * Name: host_rmdir
 ****************************************************************************/

int host_rmdir(const char *pathname)
{
  return rmdir(pathname);
}

/****************************************************************************
 * Name: host_rename
 ****************************************************************************/

int host_rename(const char *oldpath, const char *newpath)
{
  return rename(oldpath, newpath);
}

/****************************************************************************
 * Name: host_stat
 ****************************************************************************/

int host_stat(const char *path, struct nuttx_stat_s *buf)
{
  struct stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = stat(path, &hostbuf);

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}
