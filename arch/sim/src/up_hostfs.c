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

#include "hostfs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  int mapflags;

  /* Perform flag mapping */

  if ((flags & (HOSTFS_FLAG_RDOK | HOSTFS_FLAG_WROK)) == 
        (HOSTFS_FLAG_RDOK | HOSTFS_FLAG_WROK))
    {
      mapflags = O_RDWR;
    }
  else if (flags & HOSTFS_FLAG_RDOK)
    {
      mapflags = O_RDONLY;
    }
  else if (flags & HOSTFS_FLAG_WROK)
    {
      mapflags = O_WRONLY;
    }

  if (flags & HOSTFS_FLAG_APPEND)
    {
      mapflags |= O_APPEND;
    }

  if (flags & HOSTFS_FLAG_CREAT)
    {
      mapflags |= O_CREAT;
    }

  if (flags & HOSTFS_FLAG_EXCL)
    {
      mapflags |= O_EXCL;
    }

  if (flags & HOSTFS_FLAG_TRUNC)
    {
      mapflags |= O_TRUNC;
    }

  return open(pathname, mapflags, mode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_close(int fd)
{
  /* Just call the close routine */

  return close(fd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

ssize_t host_read(int fd, void* buf, size_t count)
{
  /* Just call the read routine */

  return read(fd, buf, count);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

ssize_t host_write(int fd, const void *buf, size_t count)
{
  /* Just call the write routine */

  return write(fd, buf, count);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

off_t host_lseek(int fd, off_t offset, int whence)
{
  /* Just call the lseek routine */

  return lseek(fd, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_ioctl(int fd, int request, unsigned long arg)
{
  /* Just call the ioctl routine */

  return ioctl(fd, request, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void host_sync(int fd)
{
  /* Just call the sync routine */

  sync();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_dup(int fd)
{
  return dup(fd);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void *host_opendir(const char *name)
{
  return (void *) opendir(name);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_readdir(void* dirp, struct host_dirent_s* entry)
{
    struct dirent  *ent;

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
            entry->d_type = HOSTFS_DTYPE_FILE;
          }
        else if (ent->d_type == DT_CHR)
          {
            entry->d_type = HOSTFS_DTYPE_CHR;
          }
        else if (ent->d_type == DT_BLK)
          {
            entry->d_type = HOSTFS_DTYPE_BLK;
          }
        else if (ent->d_type == DT_DIR)
          {
            entry->d_type = HOSTFS_DTYPE_DIRECTORY;
          }
      }

    if (ent)
      {
        return 0;
      }

    return -ENOENT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void host_rewinddir(void* dirp)
{
  /* Just call the rewinddir routine */

  rewinddir(dirp);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_closedir(void* dirp)
{
  return closedir(dirp);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_statfs(const char *path, struct host_statfs_s *buf)
{
  int           ret;
  struct statfs host_buf; 

  /* Call the host's statfs routine */

  ret = statfs(path, &host_buf);

  /* Map the return values */

  buf->f_type    = host_buf.f_type;
  buf->f_bsize   = host_buf.f_bsize;
  buf->f_blocks  = host_buf.f_blocks;
  buf->f_bfree   = host_buf.f_bfree;
  buf->f_bavail  = host_buf.f_bavail;
  buf->f_files   = host_buf.f_files;
  buf->f_ffree   = host_buf.f_ffree;
  buf->f_fsid    = 0;
  buf->f_namelen = host_buf.f_namelen;
  buf->f_frsize  = host_buf.f_frsize;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_unlink(const char *pathname)
{
  return unlink(pathname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_mkdir(const char *pathname, mode_t mode)
{
  /* Just call the host's mkdir routine */

  return mkdir(pathname, mode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_rmdir(const char *pathname)
{
  return rmdir(pathname);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_rename(const char *oldpath, const char *newpath)
{
  return rename(oldpath, newpath);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_stat(const char *path, struct host_stat_s *buf)
{
  struct stat   host_buf;
  int           ret;

  /* Call the host's stat routine */

  ret = stat(path, &host_buf);

  /* Now map the return values to the common struct */

  buf->st_dev     = host_buf.st_dev;      /* ID of the device containing file */
  buf->st_ino     = host_buf.st_ino;;     /* inode number */
  buf->st_nlink   = host_buf.st_nlink;    /* number of hard links */
  buf->st_uid     = host_buf.st_uid;      /* user ID of owner */
  buf->st_gid     = host_buf.st_gid;      /* group ID of owner */
  buf->st_rdev    = host_buf.st_rdev;     /* device ID */
  buf->st_size    = host_buf.st_size;     /* total size, in bytes */
  buf->st_blksize = host_buf.st_blksize;  /* blocksize for file system I/O */
  buf->st_blocks  = host_buf.st_blocks;   /* number of 512B blocks allocated */
  buf->st_atim    = host_buf.st_atime;    /* time of last access */
  buf->st_mtim    = host_buf.st_mtime;    /* time of last modification */
  buf->st_ctim    = host_buf.st_ctime;    /* time of last status change */

  /* Map the mode bits */

  buf->st_mode = host_buf.st_mode & 0xFFF;
  if (S_ISREG(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_REG;
    }

  if (S_ISDIR(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_DIR;
    }

  if (S_ISCHR(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_CHR;
    }

  if (S_ISBLK(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_BLK;
    }

  if (S_ISFIFO(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_PIPE;
    }

  if (S_ISLNK(host_buf.st_mode))
    {
      buf->st_mode |= HOST_ST_MODE_LINK;
    }

  return ret;
}
