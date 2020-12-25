/****************************************************************************
 * arch/arm/src/common/arm_hostfs.c
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
#include <nuttx/fs/hostfs.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <syscall.h>
#include <unistd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOST_OPEN           0x01
#define HOST_CLOSE          0x02
#define HOST_WRITE          0x05
#define HOST_READ           0x06
#define HOST_SEEK           0x0a
#define HOST_FLEN           0x0c
#define HOST_REMOVE         0x0e
#define HOST_RENAME         0x0f
#define HOST_ERROR          0x13

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static long host_call(unsigned int nbr, void *parm)
{
  long ret = smh_call(nbr, parm);
  if (ret < 0)
    {
      long err = smh_call(HOST_ERROR, NULL);
      if (err > 0)
        {
          ret = -err;
        }
    }

  return ret;
}

static ssize_t host_flen(long fd)
{
  return host_call(HOST_FLEN, &fd);
}

static int host_flags_to_mode(int flags)
{
  static const int modeflags[] =
  {
    O_RDONLY,
    O_RDONLY | O_BINARY,
    O_RDWR,
    O_RDWR | O_BINARY,
    O_WRONLY | O_CREAT | O_TRUNC,
    O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
    O_RDWR | O_CREAT | O_TRUNC,
    O_RDWR | O_CREAT | O_TRUNC | O_BINARY,
    O_WRONLY | O_CREAT | O_APPEND,
    O_WRONLY | O_CREAT | O_APPEND | O_BINARY,
    O_RDWR | O_CREAT | O_APPEND,
    O_RDWR | O_CREAT | O_APPEND | O_BINARY,
    0,
  };

  int i;
  for (i = 0; modeflags[i] != 0; i++)
    {
      if (modeflags[i] == flags)
        {
          return i;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_open(const char *pathname, int flags, int mode)
{
  struct
  {
    const char *pathname;
    long mode;
    size_t len;
  } open =
  {
    .pathname = pathname,
    .mode = host_flags_to_mode(flags),
    .len = strlen(pathname),
  };

  return host_call(HOST_OPEN, &open);
}

int host_close(int fd_)
{
  long fd = fd_;
  return host_call(HOST_CLOSE, &fd);
}

ssize_t host_read(int fd, void *buf, size_t count)
{
  struct
  {
    long fd;
    void *buf;
    size_t count;
  } read =
  {
    .fd = fd,
    .buf = buf,
    .count = count,
  };

  ssize_t ret = host_call(HOST_READ, &read);
  return ret < 0 ? ret : count - ret;
}

ssize_t host_write(int fd, const void *buf, size_t count)
{
  struct
  {
    long fd;
    const void *buf;
    size_t count;
  } write =
  {
    .fd = fd,
    .buf = buf,
    .count = count,
  };

  ssize_t ret = host_call(HOST_WRITE, &write);
  return ret < 0 ? ret : count - ret;
}

off_t host_lseek(int fd, off_t offset, int whence)
{
  off_t ret = -ENOSYS;

  if (whence == SEEK_END)
    {
      ret = host_flen(fd);
      if (ret >= 0)
        {
          offset += ret;
          whence = SEEK_SET;
        }
    }

  if (whence == SEEK_SET)
    {
      struct
      {
        long fd;
        size_t pos;
      } seek =
      {
        .fd = fd,
        .pos = offset,
      };

      ret = host_call(HOST_SEEK, &seek);
      if (ret >= 0)
        {
            ret = offset;
        }
    }

  return ret;
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
  memset(buf, 0, sizeof(*buf));
  buf->st_mode = S_IFREG | 0777;
  buf->st_size = host_flen(fd);
  return buf->st_size < 0 ? buf->st_size : 0;
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
  struct
  {
    const char *pathname;
    size_t pathname_len;
  } remove =
  {
    .pathname = pathname,
    .pathname_len = strlen(pathname),
  };

  return host_call(HOST_REMOVE, &remove);
}

int host_mkdir(const char *pathname, mode_t mode)
{
  return -ENOSYS;
}

int host_rmdir(const char *pathname)
{
  return host_unlink(pathname);
}

int host_rename(const char *oldpath, const char *newpath)
{
  struct
  {
    const char *oldpath;
    size_t oldpath_len;
    const char *newpath;
    size_t newpath_len;
  } rename =
  {
    .oldpath = oldpath,
    .oldpath_len = strlen(oldpath),
    .newpath = newpath,
    .newpath_len = strlen(newpath),
  };

  return host_call(HOST_RENAME, &rename);
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
      /* Since semihosting doesn't support directory yet, */

      ret = 0; /* we have to assume it's a directory here. */
      memset(buf, 0, sizeof(*buf));
      buf->st_mode = S_IFDIR | 0777;
    }

  return ret;
}
