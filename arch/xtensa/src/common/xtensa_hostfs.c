/****************************************************************************
 * arch/xtensa/src/common/xtensa_hostfs.c
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

#include <nuttx/config.h>
#include <nuttx/cache.h>
#include <nuttx/fs/hostfs.h>

#include <arch/simcall.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <syscall.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int host_call(int nr, int param1, int param2, int param3)
{
  int ret;
  int err;

  ret = simcall(nr, param1, param2, param3, &err);
  if (ret < 0)
    {
      ret = -err;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  int simcall_flags = 0;

  switch ((flags & O_ACCMODE))
    {
      case O_RDONLY:
        simcall_flags = SIMCALL_O_RDONLY;
        break;
      case O_WRONLY:
        simcall_flags = SIMCALL_O_WRONLY;
        break;
      case O_RDWR:
        simcall_flags = SIMCALL_O_RDWR;
        break;
    }

  if ((flags & O_APPEND) != 0)
    {
      simcall_flags |= SIMCALL_O_APPEND;
    }

  if ((flags & O_TRUNC) != 0)
    {
      simcall_flags |= SIMCALL_O_TRUNC;
    }

  if ((flags & O_CREAT) != 0)
    {
      simcall_flags |= SIMCALL_O_CREAT;
    }

  if ((flags & O_EXCL) != 0)
    {
      simcall_flags |= SIMCALL_O_EXCL;
    }

#ifdef CONFIG_XTENSA_SEMIHOSTING_HOSTFS_CACHE_COHERENCE
  up_clean_dcache(pathname, pathname + strlen(pathname) + 1);
#endif
  return host_call(SIMCALL_SYS_OPEN, (int)pathname, simcall_flags, mode);
}

int host_close(int fd)
{
  return host_call(SIMCALL_SYS_CLOSE, fd, 0, 0);
}

ssize_t host_read(int fd, void *buf, size_t count)
{
#ifdef CONFIG_XTENSA_SEMIHOSTING_HOSTFS_CACHE_COHERENCE
  up_invalidate_dcache(buf, buf + count);
#endif

  return host_call(SIMCALL_SYS_READ, fd, (int)buf, count);
}

ssize_t host_write(int fd, const void *buf, size_t count)
{
#ifdef CONFIG_XTENSA_SEMIHOSTING_HOSTFS_CACHE_COHERENCE
  up_clean_dcache(buf, buf + count);
#endif

  return host_call(SIMCALL_SYS_WRITE, fd, (int)buf, count);
}

off_t host_lseek(int fd, off_t offset, int whence)
{
  return host_call(SIMCALL_SYS_LSEEK, fd, offset, whence);
}

int host_ioctl(int fd, int request, unsigned long arg)
{
  return -ENOSYS;
}

void host_sync(int fd)
{
}

int host_dup(int fd)
{
  return -ENOSYS;
}

int host_fstat(int fd, struct stat *buf)
{
  /* Determine the size using lseek.
   *
   * Assumptions:
   *  - host_lseek never fails
   *  - It's ok to change the file offset temporarily as
   *    hostfs_lock provides enough serialization.
   */

  off_t saved_off = host_lseek(fd, 0, SEEK_CUR);
  off_t size = host_lseek(fd, 0, SEEK_END);
  host_lseek(fd, saved_off, SEEK_SET);

  memset(buf, 0, sizeof(*buf));
  buf->st_mode = S_IFREG | 0777;
  buf->st_size = size;
  return 0;
}

int host_fchstat(int fd, const struct stat *buf, int flags)
{
  return -ENOSYS;
}

int host_ftruncate(int fd, off_t length)
{
  return -ENOSYS;
}

void *host_opendir(const char *name)
{
  return NULL;
}

int host_readdir(void *dirp, struct dirent *entry)
{
  return -ENOSYS;
}

void host_rewinddir(void *dirp)
{
}

int host_closedir(void *dirp)
{
  return -ENOSYS;
}

int host_statfs(const char *path, struct statfs *buf)
{
  return 0;
}

int host_unlink(const char *pathname)
{
  return -ENOSYS;
}

int host_mkdir(const char *pathname, int mode)
{
  return -ENOSYS;
}

int host_rmdir(const char *pathname)
{
  return host_unlink(pathname);
}

int host_rename(const char *oldpath, const char *newpath)
{
  return -ENOSYS;
}

int host_stat(const char *path, struct stat *buf)
{
  int ret = host_open(path, O_RDONLY, 0);
  if (ret >= 0)
    {
      int fd = ret;
      ret = host_fstat(fd, buf);
      host_close(fd);
    }

  if (ret < 0)
    {
      /* Since semihosting doesn't support directory yet,
       * we have to assume it's a directory here.
       */

      ret = 0;
      memset(buf, 0, sizeof(*buf));
      buf->st_mode = S_IFDIR | 0777;
    }

  return ret;
}

int host_chstat(const char *path, const struct stat *buf, int flags)
{
  return -ENOSYS;
}
