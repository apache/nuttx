/****************************************************************************
 * arch/sim/src/sim/win/sim_hostfs.c
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <io.h>
#include <windows.h>

#include "hostfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_stat_convert
 ****************************************************************************/

static void host_stat_convert(struct _stat *hostbuf,
                              struct nuttx_stat_s *buf)
{
  /* Map the return values */

  buf->st_mode = hostbuf->st_mode & 07777;

  if (hostbuf->st_mode & _S_IFDIR)
    {
      buf->st_mode |= NUTTX_S_IFDIR;
    }
  else if (hostbuf->st_mode & _S_IFREG)
    {
      buf->st_mode |= NUTTX_S_IFREG;
    }
  else if (hostbuf->st_mode & _S_IFCHR)
    {
      buf->st_mode |= NUTTX_S_IFCHR;
    }

  buf->st_dev          = hostbuf->st_dev;
  buf->st_ino          = hostbuf->st_ino;
  buf->st_nlink        = hostbuf->st_nlink;
  buf->st_uid          = hostbuf->st_uid;
  buf->st_gid          = hostbuf->st_gid;
  buf->st_rdev         = hostbuf->st_rdev;
  buf->st_size         = hostbuf->st_size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_open
 ****************************************************************************/

int host_open(const char *pathname, int flags, nuttx_mode_t mode)
{
  int mapflags = 0;
  int ret;

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

  ret = _open(pathname, mapflags, mode);
  if (ret == -1)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_close
 ****************************************************************************/

int host_close(int fd)
{
  /* Just call the close routine */

  int ret = _close(fd);
  if (ret == -1)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_read
 ****************************************************************************/

nuttx_ssize_t host_read(int fd, void *buf, nuttx_size_t count)
{
  /* Just call the read routine */

  nuttx_ssize_t ret = _read(fd, buf, count);
  if (ret == -1)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_write
 ****************************************************************************/

nuttx_ssize_t host_write(int fd, const void *buf, nuttx_size_t count)
{
  /* Just call the write routine */

  nuttx_ssize_t ret = _write(fd, buf, count);
  if (ret == -1)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_lseek
 ****************************************************************************/

nuttx_off_t host_lseek(int fd, nuttx_off_t offset, int whence)
{
  /* Just call the lseek routine */

  nuttx_off_t ret = _lseek(fd, offset, whence);
  if (ret == (nuttx_off_t)-1)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_ioctl
 ****************************************************************************/

int host_ioctl(int fd, int request, unsigned long arg)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_sync
 ****************************************************************************/

void host_sync(int fd)
{
}

/****************************************************************************
 * Name: host_dup
 ****************************************************************************/

int host_dup(int fd)
{
  int ret = _dup(fd);
  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_fstat
 ****************************************************************************/

int host_fstat(int fd, struct nuttx_stat_s *buf)
{
  struct _stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = _fstat(fd, &hostbuf);
  if (ret < 0)
    {
      return -errno;
    }

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}

/****************************************************************************
 * Name: host_fchstat
 ****************************************************************************/

int host_fchstat(int fd, const struct nuttx_stat_s *buf, int flags)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_truncate
 ****************************************************************************/

int host_ftruncate(int fd, nuttx_off_t length)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_opendir
 ****************************************************************************/

void *host_opendir(const char *name)
{
  char namebuf[CONFIG_PATH_MAX + 4];
  WIN32_FIND_DATA data;

  snprintf(namebuf, sizeof(namebuf), "%s/*", name);

  return FindFirstFile(namebuf, &data);
}

/****************************************************************************
 * Name: host_readdir
 ****************************************************************************/

int host_readdir(void *dirp, struct nuttx_dirent_s *entry)
{
  WIN32_FIND_DATA FileData;
  BOOL ret;

  ret = FindNextFile(dirp, &FileData);
  if (ret == FALSE)
    {
      return GetLastError() == ERROR_NO_MORE_FILES ?
             -ENOENT : HRESULT_FROM_WIN32(GetLastError());
    }

  strncpy(entry->d_name, FileData.cFileName,
          sizeof(entry->d_name) - 1);

  if (FileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
    {
      entry->d_type = NUTTX_DTYPE_DIRECTORY;
    }
  else
    {
      entry->d_type = NUTTX_DTYPE_FILE;
    }

  return 0;
}

/****************************************************************************
 * Name: host_rewinddir
 ****************************************************************************/

void host_rewinddir(void *dirp)
{
}

/****************************************************************************
 * Name: host_closedir
 ****************************************************************************/

int host_closedir(void *dirp)
{
  if (FindClose(dirp))
    {
      return 0;
    }

  return HRESULT_FROM_WIN32(GetLastError());
}

/****************************************************************************
 * Name: host_statfs
 ****************************************************************************/

int host_statfs(const char *path, struct nuttx_statfs_s *buf)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: host_unlink
 ****************************************************************************/

int host_unlink(const char *pathname)
{
  int ret = _unlink(pathname);
  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_mkdir
 ****************************************************************************/

int host_mkdir(const char *pathname, nuttx_mode_t mode)
{
  /* Just call the host's mkdir routine */

  int ret = _mkdir(pathname, mode);
  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_rmdir
 ****************************************************************************/

int host_rmdir(const char *pathname)
{
  int ret = _rmdir(pathname);
  if (ret < 0)
    {
      ret = -errno;
    }

  return ret;
}

/****************************************************************************
 * Name: host_rename
 ****************************************************************************/

int host_rename(const char *oldpath, const char *newpath)
{
  if (MoveFile(oldpath, newpath))
    {
      return 0;
    }

  return HRESULT_FROM_WIN32(GetLastError());
}

/****************************************************************************
 * Name: host_stat
 ****************************************************************************/

int host_stat(const char *path, struct nuttx_stat_s *buf)
{
  struct _stat hostbuf;
  int ret;

  /* Call the host's stat routine */

  ret = _stat(path, &hostbuf);
  if (ret < 0)
    {
      ret = -errno;
    }

  /* Map the return values */

  host_stat_convert(&hostbuf, buf);
  return ret;
}

/****************************************************************************
 * Name: host_chstat
 ****************************************************************************/

int host_chstat(const char *path, const struct nuttx_stat_s *buf, int flags)
{
  return -ENOSYS;
}
