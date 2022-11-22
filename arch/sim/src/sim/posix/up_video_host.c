/****************************************************************************
 * arch/sim/src/sim/posix/up_video_host.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WARN(fmt, ...) \
        syslog(LOG_WARNING, "up_video_host: " fmt "\n", ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  int fd;
  int index;
  unsigned int reqbuf_count;
  struct v4l2_requestbuffers reqbuf;
} video_host_dev_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static video_host_dev_t priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xioctl(int fd, int request, void *arg)
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

bool video_host_is_available(const char *video_host_dev_path)
{
  if (access(video_host_dev_path, F_OK) == 0)
    {
      return true;
    }

  return false;
}

int video_host_init(const char *video_host_dev_path)
{
  int fd = open(video_host_dev_path, O_RDWR);
  if (fd < 0)
    {
      perror(video_host_dev_path);
      return -errno;
    }

  priv.fd = fd;
  priv.index = 0;
  return 0;
}

int video_host_enq_buf(uint8_t *addr, uint32_t size)
{
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  buf.m.userptr = (uintptr_t)addr;
  buf.length = size;
  buf.index = priv.index;
  priv.index = (priv.index + 1) % priv.reqbuf.count;
  int ret = ioctl(priv.fd, VIDIOC_QBUF, (unsigned long)&buf);
  if (ret < 0)
    {
      perror("VIDIOC_QBUF");
      return ret;
    }

  return 0;
}

int video_host_set_buf(uint8_t *addr, uint32_t size)
{
  return 0;
}

int video_host_dq_buf(uint8_t **addr)
{
  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_USERPTR;
  *addr = NULL;

  /* Dequeue a buffer */

  if (-1 == xioctl(priv.fd, VIDIOC_DQBUF, &buf))
    {
      switch (errno)
        {
          case EAGAIN:

            /* No buffer in the outgoing queue */

            return 0;
          case EIO:

            /* fall through */

          default:
            perror("VIDIOC_DQBUF");
            return -errno;
        }
    }

  *addr = (uint8_t *)(uintptr_t)buf.m.userptr;

  return 0;
}

int video_host_uninit(void)
{
  close(priv.fd);
  return 0;
}

int video_host_data_init(void)
{
  return 0;
}

int video_host_start_capture(int reqbuf_count)
{
  int ret;

  /* VIDIOC_REQBUFS initiate user pointer I/O */

  priv.reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  priv.reqbuf.memory = V4L2_MEMORY_USERPTR;
  priv.reqbuf.count  = reqbuf_count;

  ret = ioctl(priv.fd, VIDIOC_REQBUFS,
    (unsigned long)&priv.reqbuf);
  if (ret < 0)
    {
      perror("VIDIOC_REQBUFS");
      return ret;
    }

  if (priv.reqbuf.count < reqbuf_count)
    {
      errno = ENOMEM;
      perror("Not enough buffers");
      close(priv.fd);
      return -ENOMEM;
    }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(priv.fd, VIDIOC_STREAMON, &type))
    {
      perror("VIDIOC_STREAMON");
      return -errno;
    }

  return 0;
}

int video_host_stop_capture(void)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(priv.fd, VIDIOC_STREAMOFF, &type))
    {
      perror("VIDIOC_STREAMOFF");
      return -errno;
    }

  return 0;
}

int video_host_set_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt =
    {
      0
    };

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(priv.fd, VIDIOC_S_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_S_FMT");
      close(priv.fd);
      return -errno;
    }

  struct v4l2_streamparm streamparm =
    {
      0
    };

  streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (-1 == xioctl(priv.fd, VIDIOC_G_PARM, &streamparm))
    {
      perror("VIDIOC_G_PARM");
      close(priv.fd);
      return -errno;
    }

  streamparm.parm.capture.capturemode |= V4L2_CAP_TIMEPERFRAME;
  streamparm.parm.capture.timeperframe.numerator = numer;
  streamparm.parm.capture.timeperframe.denominator = denom;
  if (-1 == xioctl(priv.fd, VIDIOC_S_PARM, &streamparm))
    {
      perror("VIDIOC_S_PARM");
      close(priv.fd);
      return -errno;
    }

  return 0;
}

int video_host_try_fmt(uint16_t width, uint16_t height, uint32_t fmt,
    uint32_t denom, uint32_t numer)
{
  struct v4l2_format v4l2_fmt =
    {
      0
    };

  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = fmt;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (-1 == xioctl(priv.fd, VIDIOC_TRY_FMT, &v4l2_fmt))
    {
      perror("VIDIOC_TRY_FMT");
      return -errno;
    }

  struct v4l2_frmivalenum v4l2_frmival =
    {
      0
    };

  v4l2_frmival.width = width;
  v4l2_frmival.height = height;
  v4l2_frmival.pixel_format = fmt;

  /* Need not check frame interval for STILL type */

  if (!denom)
    {
      while (xioctl(priv.fd, VIDIOC_ENUM_FRAMEINTERVALS,
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
