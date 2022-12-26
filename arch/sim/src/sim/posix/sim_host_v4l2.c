/****************************************************************************
 * arch/sim/src/sim/posix/sim_host_v4l2.c
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
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool host_video_is_available(const char *host_video_dev_path)
{
  return access(host_video_dev_path, F_OK) == 0;
}

struct host_video_dev_s *host_video_init(const char *host_video_dev_path)
{
  int fd;
  struct host_video_dev_s *vdev;

  fd = open(host_video_dev_path, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      perror(host_video_dev_path);
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

  memset(&streamparm, 0, sizeof(streamparm));
  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_G_PARM, &streamparm))
    {
      perror("VIDIOC_G_PARM");
      return -errno;
    }

  streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
  streamparm.parm.capture.timeperframe.numerator = numer;
  streamparm.parm.capture.timeperframe.denominator = denom;
  if (-1 == host_video_ioctl(vdev->fd, VIDIOC_S_PARM, &streamparm))
    {
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
      WARN("Pixel format not supported");
      return -EINVAL;
    }

  /* Need not check frame interval for STILL type */

  if (!denom)
    {
      memset(&v4l2_frmival, 0, sizeof(v4l2_frmival));
      v4l2_frmival.width = width;
      v4l2_frmival.height = height;
      v4l2_frmival.pixel_format = fmt;
      while (host_video_ioctl(vdev->fd, VIDIOC_ENUM_FRAMEINTERVALS,
                              &v4l2_frmival) == 0)
        {
          if (v4l2_frmival.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
              v4l2_frmival.discrete.denominator == denom &&
              v4l2_frmival.discrete.numerator == numer)
            {
              return 0;
            }

          v4l2_frmival.index++;
        }

        WARN("Invalid frame interval, fallback to default");
    }

  return 0;
}
