/****************************************************************************
 * fs/exfat/exfat_io.c
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

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "exfat.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct exfat_dev
{
  struct file file;
  enum exfat_mode mode;
  off_t size; /* in bytes */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct exfat_dev *exfat_open(FAR const char *spec, enum exfat_mode mode)
{
  FAR struct exfat_dev *dev;
  struct stat stbuf;
  int ret;

  dev = kmm_malloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  ret = file_open(&dev->file, spec, (mode != EXFAT_MODE_RO) ?
                  O_RDWR : O_RDONLY);
  if (ret < 0)
    {
      if (mode == EXFAT_MODE_ANY)
        {
          mode = EXFAT_MODE_RO;
          ret = file_open(&dev->file, spec, O_RDONLY);
        }

      if (ret < 0)
        {
          return NULL;
        }
    }

  dev->mode = mode;

  ret = file_fstat(&dev->file, &stbuf);
  if (ret < 0)
    {
      goto errout;
    }

  if (!S_ISBLK(stbuf.st_mode) && !S_ISCHR(stbuf.st_mode) &&
      !S_ISREG(stbuf.st_mode))
    {
      goto errout;
    }

  dev->size = stbuf.st_size;
  return dev;

errout:
  file_close(&dev->file);
  kmm_free(dev);
  return NULL;
}

int exfat_close(FAR struct exfat_dev *dev)
{
  int ret;

  ret = file_close(&dev->file);
  kmm_free(dev);
  return ret;
}

int exfat_fsync(FAR struct exfat_dev *dev)
{
  int ret;

  ret = file_ioctl(&dev->file, BIOC_FLUSH, 0);
  return ret == -ENOTTY ? OK : ret;
}

enum exfat_mode exfat_get_mode(FAR const struct exfat_dev *dev)
{
  return dev->mode;
}

off_t exfat_get_size(FAR const struct exfat_dev *dev)
{
  return dev->size;
}

off_t exfat_seek(FAR struct exfat_dev *dev, off_t offset, int whence)
{
  return file_seek(&dev->file, offset, whence);
}

ssize_t exfat_read(FAR struct exfat_dev *dev, FAR void *buffer, size_t size)
{
  return file_read(&dev->file, buffer, size);
}

ssize_t exfat_write(FAR struct exfat_dev *dev, FAR const void *buffer,
                    size_t size)
{
  ssize_t ret;

  ret = file_write(&dev->file, buffer, size);
  if (ret >= 0)
    {
      exfat_fsync(dev);
    }

  return ret;
}

ssize_t exfat_pread(FAR struct exfat_dev *dev, FAR void *buffer, size_t size,
                    off_t offset)
{
  return file_pread(&dev->file, buffer, size, offset);
}

ssize_t exfat_pwrite(FAR struct exfat_dev *dev, FAR const void *buffer,
                     size_t size, off_t offset)
{
  ssize_t ret;

  ret = file_pwrite(&dev->file, buffer, size, offset);
  if (ret >= 0)
    {
      exfat_fsync(dev);
    }

  return ret;
}
