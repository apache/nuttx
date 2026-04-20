/****************************************************************************
 * arch/sim/src/sim/posix/sim_host_v4l2.c
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

#include <errno.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include <syslog.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "sim_hostvideo.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_REQBUFS 3
#define WARN(fmt, ...) \
        syslog(LOG_WARNING, "sim_host_video: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct host_video_dev_s
{
  int fd;
  char path[PATH_MAX];
  void *addrs[MAX_REQBUFS];
  size_t buflen[MAX_REQBUFS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int host_video_ioctl(int fd, int request, void *arg)
{
  int r;

  do
    {
      r = ioctl(fd, request, arg);
    }
  while (-1 == r && EINTR == errno);

  return r;
}

static int host_video_parse_device_index(const char *name, int *index)
{
  long value;
  char *endptr;

  if (strncmp(name, "video", 5) != 0)
    {
      return -EINVAL;
    }

  name += 5;
  if (*name == '\0')
    {
      return -EINVAL;
    }

  value = strtol(name, &endptr, 10);
  if (*endptr != '\0' || value < 0 || value > INT_MAX)
    {
      return -EINVAL;
    }

  *index = value;
  return 0;
}

static int host_video_get_next_device_path(int current_index,
                                           char *devpath,
                                           size_t devpathlen,
                                           int *next_index)
{
  DIR *dir;
  struct dirent *entry;
  int candidate;
  int found = INT_MAX;

  dir = opendir("/dev");
  if (dir == NULL)
    {
      return -errno;
    }

  while ((entry = readdir(dir)) != NULL)
    {
      if (host_video_parse_device_index(entry->d_name, &candidate) < 0 ||
          candidate <= current_index || candidate >= found)
        {
          continue;
        }

      found = candidate;
    }

  closedir(dir);

  if (found == INT_MAX)
    {
      return -ENODEV;
    }

  if (snprintf(devpath, devpathlen, "/dev/video%d", found) >=
      (int)devpathlen)
    {
      return -ENAMETOOLONG;
    }

  *next_index = found;
  return 0;
}

static bool host_video_is_capture_device(const char *host_video_dev_path)
{
  struct v4l2_capability cap;
  int fd;
  bool available = false;

  fd = open(host_video_dev_path, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      fd = open(host_video_dev_path, O_RDONLY | O_NONBLOCK);
      if (fd < 0)
        {
          return false;
        }
    }

  memset(&cap, 0, sizeof(cap));
  if (host_video_ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0)
    {
      uint32_t capabilities = cap.device_caps != 0 ? cap.device_caps :
                              cap.capabilities;

      if ((capabilities & V4L2_CAP_VIDEO_CAPTURE) != 0)
        {
          available = true;
        }
    }

  close(fd);
  return available;
}

static int host_video_get_device_path_by_index(int index,
                                               char *devpath,
                                               size_t devpathlen)
{
  int count = 0;
  int current_index = -1;
  char path[PATH_MAX];

  while ((host_video_get_next_device_path(current_index, path,
                                          sizeof(path),
                                          &current_index)) == 0)
    {
      if (!host_video_is_capture_device(path))
        {
          continue;
        }

      if (count == index)
        {
          if (snprintf(devpath, devpathlen, "%s", path) >=
              (int)devpathlen)
            {
              return -ENAMETOOLONG;
            }

          return 0;
        }

      count++;
    }

  return -ENODEV;
}

static int host_video_resolve_device_path(const char *host_video_dev_path,
                                          char *resolved_path,
                                          size_t resolved_path_len)
{
  const char *name;
  char *endptr;
  long index;
  int ret;

  name = strrchr(host_video_dev_path, '/');
  name = name != NULL ? name + 1 : host_video_dev_path;

  if (strncmp(name, "video", 5) != 0)
    {
      if (snprintf(resolved_path, resolved_path_len, "%s",
                   host_video_dev_path) >= (int)resolved_path_len)
        {
          return -ENAMETOOLONG;
        }

      return 0;
    }

  index = strtol(name + 5, &endptr, 10);
  if (endptr == name + 5 || *endptr != '\0' || index < 0 ||
      index > INT_MAX)
    {
      return -EINVAL;
    }

  ret = host_video_get_device_path_by_index(index, resolved_path,
                                            resolved_path_len);

  if (ret < 0)
    {
      WARN("failed to resolve %s to host capture device: %d",
           host_video_dev_path, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_video_get_device_count(void)
{
  int count = 0;
  char devpath[PATH_MAX];

  while (host_video_get_device_path_by_index(count, devpath,
                                             sizeof(devpath)) == 0)
    {
      count++;
    }

  return count > 0 ? count : -ENODEV;
}

bool host_video_is_available(const char *host_video_dev_path)
{
  char resolved_path[PATH_MAX];
  int ret;

  ret = host_video_resolve_device_path(host_video_dev_path, resolved_path,
                                       sizeof(resolved_path));
  if (ret < 0)
    {
      return false;
    }

  return host_video_is_capture_device(resolved_path);
}

struct host_video_dev_s *host_video_init(const char *host_video_dev_path)
{
  int fd;
  int ret;
  char resolved_path[PATH_MAX];
  struct host_video_dev_s *vdev;

  ret = host_video_resolve_device_path(host_video_dev_path, resolved_path,
                                       sizeof(resolved_path));
  if (ret < 0)
    {
      errno = -ret;
      perror(host_video_dev_path);
      return NULL;
    }

  fd = open(resolved_path, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      perror(resolved_path);
      return NULL;
    }

  vdev = calloc(1, sizeof(*vdev));
  if (vdev == NULL)
    {
      perror("host_video_init failed");
      close(fd);
      return NULL;
    }

  vdev->fd = fd;
  snprintf(vdev->path, sizeof(vdev->path), "%s", resolved_path);
  return vdev;
}

int host_video_dqbuf(struct host_video_dev_s *vdev, uint8_t *addr,
                     uint32_t size)
{
  struct v4l2_buffer buf;

  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  /* Dequeue a buffer */

  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_DQBUF, &buf))
    {
      switch (errno)
        {
          case EAGAIN:

            /* No buffer in the outgoing queue */

            return 0;

          default:
            perror("VIDIOC_DQBUF");
            return -errno;
        }
    }

  if (size > buf.bytesused)
    {
      size = buf.bytesused;
    }

  memcpy(addr, vdev->addrs[buf.index], size);
  if (-1 == ioctl(vdev->fd, VIDIOC_QBUF, &buf))
    {
      perror("VIDIOC_QBUF");
      return -errno;
    }

  return size;
}

int host_video_uninit(struct host_video_dev_s *vdev)
{
  if (vdev != NULL)
    {
      close(vdev->fd);
      free(vdev);
    }

  return 0;
}

int host_video_start_capture(struct host_video_dev_s *vdev)
{
  struct v4l2_buffer buf;
  struct v4l2_requestbuffers reqbuf;
  enum v4l2_buf_type type;
  int i;

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  memset(&reqbuf, 0, sizeof(reqbuf));
  reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  reqbuf.memory = V4L2_MEMORY_MMAP;
  reqbuf.count  = MAX_REQBUFS;

  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_REQBUFS, &reqbuf))
    {
      perror("VIDIOC_REQBUFS");
      return -errno;
    }

  if (reqbuf.count < 2)
    {
      errno = ENOMEM;
      perror("Not enough buffers");
      return -ENOMEM;
    }

  for (i = 0; i < reqbuf.count; i++)
    {
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (-1 == host_video_ioctl(vdev->fd, VIDIOC_QUERYBUF, &buf))
        {
          perror("VIDIOC_QUERYBUF");
          goto err_out;
        }

      vdev->addrs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                            MAP_SHARED, vdev->fd, buf.m.offset);
      if (vdev->addrs[i] == MAP_FAILED)
        {
          perror("Mmap failed");
          goto err_out;
        }

      vdev->buflen[i] = buf.length;
      if (-1 == host_video_ioctl(vdev->fd, VIDIOC_QBUF, &buf))
        {
          perror("VIDIOC_QBUF");
          goto err_out;
        }
    }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_STREAMON, &type))
    {
      perror("VIDIOC_STREAMON");
      goto err_out;
    }

  return 0;

err_out:
  while (i--)
    {
      munmap(vdev->addrs[i], vdev->buflen[i]);
      vdev->addrs[i] = NULL;
      vdev->buflen[i] = 0;
    }

  return -errno;
}

int host_video_stop_capture(struct host_video_dev_s *vdev)
{
  enum v4l2_buf_type type;
  int i;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_STREAMOFF, &type))
    {
      perror("VIDIOC_STREAMOFF");
      return -errno;
    }

  for (i = 0; i < MAX_REQBUFS; i++)
    {
      if (vdev->buflen[i] == 0)
        {
          break;
        }

        munmap(vdev->addrs[i], vdev->buflen[i]);
        vdev->addrs[i] = NULL;
        vdev->buflen[i] = 0;
    }

  return 0;
}

int host_video_set_fmt(struct host_video_dev_s *vdev,
                       uint16_t width, uint16_t height, uint32_t fmt,
                       uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt;
  struct v4l2_streamparm streamparm;

  memset(&v4l2_fmt, 0, sizeof(v4l2_fmt));
  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_S_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_S_FMT");
      return -errno;
    }

  if (v4l2_fmt.fmt.pix.pixelformat != fmt)
    {
      WARN("%s fallback pixel format from %08x to %08x",
           vdev->path, fmt, v4l2_fmt.fmt.pix.pixelformat);
      return -EINVAL;
    }

  if (v4l2_fmt.fmt.pix.width != width || v4l2_fmt.fmt.pix.height != height)
    {
      WARN("%s fallback frame size from %ux%u to %ux%u",
           vdev->path, width, height,
           v4l2_fmt.fmt.pix.width, v4l2_fmt.fmt.pix.height);
      return -EINVAL;
    }

  if (denom == 0 || numer == 0)
    {
      return 0; /* Keep default frame interval */
    }

  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_G_PARM, &streamparm))
    {
      if (errno == EINVAL)
        {
          WARN("%s does not support VIDIOC_G_PARM, keep default "
               "frame interval", vdev->path);
          return 0; /* Keep default frame interval */
        }

      perror("VIDIOC_G_PARM");
      return -errno;
    }

  if ((streamparm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) == 0)
    {
      WARN("%s does not support programmable frame interval",
           vdev->path);
      return 0; /* Keep default frame interval */
    }

  streamparm.parm.capture.timeperframe.numerator = numer;
  streamparm.parm.capture.timeperframe.denominator = denom;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_S_PARM, &streamparm))
    {
      if (errno == EINVAL)
        {
          WARN("%s rejected frame interval %u/%u, keep default",
               vdev->path, numer, denom);
          return 0; /* Keep default frame interval */
        }

      perror("VIDIOC_S_PARM");
      return -errno;
    }

  return 0;
}

int host_video_try_fmt(struct host_video_dev_s *vdev,
                       uint16_t width, uint16_t height, uint32_t fmt,
                       uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt;
  struct v4l2_frmivalenum v4l2_frmival;

  memset(&v4l2_fmt, 0, sizeof(v4l2_fmt));
  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_TRY_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_TRY_FMT");
      return -errno;
    }

  if (v4l2_fmt.fmt.pix.pixelformat != fmt)
    {
      WARN("%s does not support pixel format %08x, fallback to %08x",
           vdev->path, fmt, v4l2_fmt.fmt.pix.pixelformat);
      return -EINVAL;
    }

  if (v4l2_fmt.fmt.pix.width != width || v4l2_fmt.fmt.pix.height != height)
    {
      WARN("%s does not support frame size %ux%u, fallback to %ux%u",
           vdev->path, width, height,
           v4l2_fmt.fmt.pix.width, v4l2_fmt.fmt.pix.height);
      return -EINVAL;
    }

  /* Need not check frame interval for STILL type */

  if (denom == 0 || numer == 0)
    {
      return 0;
    }

  memset(&v4l2_frmival, 0, sizeof(v4l2_frmival));
  v4l2_frmival.width = width;
  v4l2_frmival.height = height;
  v4l2_frmival.pixel_format = fmt;
  while (host_video_ioctl(vdev->fd, VIDIOC_ENUM_FRAMEINTERVALS,
                          &v4l2_frmival) == 0)
    {
      if (v4l2_frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE &&
          v4l2_frmival.discrete.denominator == denom &&
          v4l2_frmival.discrete.numerator == numer)
        {
          return 0;
        }

      v4l2_frmival.index++;
    }

  if (errno != EINVAL)
    {
      WARN("%s failed to enumerate frame intervals: %d",
           vdev->path, errno);
    }
  else
    {
      WARN("%s does not expose frame interval %u/%u, fallback to default",
           vdev->path, numer, denom);
    }

  return 0;
}
