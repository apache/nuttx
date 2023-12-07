/****************************************************************************
 * drivers/video/goldfish_camera.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/kthread.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/video.h>
#include <nuttx/video/v4l2_cap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GOLDFISH_CAMERA_MAX_NUMBER      8
#define GOLDFISH_CAMERA_SIZE_MAX_NUMBER 32

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  uint16_t width;
  uint16_t height;
} goldfish_camera_size_t;

typedef struct
{
  char                   name[32];
  uint32_t               channel;
  uint32_t               pix;
  char                   dir[32];
  goldfish_camera_size_t size[GOLDFISH_CAMERA_SIZE_MAX_NUMBER];
} goldfish_camera_info_t;

typedef struct
{
  struct imgdata_s       data;
  struct imgsensor_s     sensor;
  imgdata_capture_t      capture_cb;
  FAR void               *capture_arg;
  uint32_t               buf_size;
  FAR uint8_t            *next_buf;
  struct file            file;
  sem_t                  run;
  goldfish_camera_info_t info;
  bool                   streaming;
  pid_t                  pid;
} goldfish_camera_priv_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Video image sensor operations */

static bool goldfish_camera_is_available(FAR struct imgsensor_s *sensor);
static int goldfish_camera_init(FAR struct imgsensor_s *sensor);
static int goldfish_camera_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
goldfish_camera_get_driver_name(FAR struct imgsensor_s *sensor);
static int
goldfish_camera_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_datafmt,
                                       FAR imgsensor_format_t *datafmts,
                                       FAR imgsensor_interval_t *interval);
static int goldfish_camera_start_capture(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_datafmt,
                                         FAR imgsensor_format_t *datafmts,
                                         FAR imgsensor_interval_t *interval);
static int goldfish_camera_stop_capture(FAR struct imgsensor_s *sensor,
                                        imgsensor_stream_type_t type);

/* Video image data operations */

static int goldfish_camera_data_init(FAR struct imgdata_s *data);
static int goldfish_camera_data_uninit(FAR struct imgdata_s *data);
static int
goldfish_camera_data_validate_frame_setting(FAR struct imgdata_s *data,
                                           uint8_t nr_datafmt,
                                           FAR imgdata_format_t *datafmt,
                                           FAR imgdata_interval_t *interval);
static int
goldfish_camera_data_start_capture(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmt,
                                   FAR imgdata_format_t *datafmt,
                                   FAR imgdata_interval_t *interval,
                                   imgdata_capture_t callback,
                                   FAR void *arg);
static int goldfish_camera_data_stop_capture(FAR struct imgdata_s *data);
static int goldfish_camera_data_set_buf(FAR struct imgdata_s *data,
                                        uint8_t nr_datafmts,
                                        FAR imgdata_format_t *datafmts,
                                        FAR uint8_t *addr,
                                        uint32_t size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Communication service for qemu-pipe */

static const char g_qemudservice[] = "qemud";

/* Camera service. */

static const char g_cameraservice[] = "camera";

static const struct imgsensor_ops_s g_goldfish_camera_ops =
{
  .is_available           = goldfish_camera_is_available,
  .init                   = goldfish_camera_init,
  .uninit                 = goldfish_camera_uninit,
  .get_driver_name        = goldfish_camera_get_driver_name,
  .validate_frame_setting = goldfish_camera_validate_frame_setting,
  .start_capture          = goldfish_camera_start_capture,
  .stop_capture           = goldfish_camera_stop_capture,
};

static const struct imgdata_ops_s g_goldfish_camera_data_ops =
{
  .init                   = goldfish_camera_data_init,
  .uninit                 = goldfish_camera_data_uninit,
  .set_buf                = goldfish_camera_data_set_buf,
  .validate_frame_setting = goldfish_camera_data_validate_frame_setting,
  .start_capture          = goldfish_camera_data_start_capture,
  .stop_capture           = goldfish_camera_data_stop_capture,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int goldfish_camera_read_fully(FAR struct file *file,
                                      FAR void *buffer,
                                      size_t size)
{
  FAR char *p = (FAR char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_read(file, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

static int goldfish_camera_write_fully(FAR struct file *file,
                                       FAR const void *buffer,
                                       size_t size)
{
  FAR const char *p = (FAR const char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_write(file, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

static int goldfish_camera_send(FAR struct file *file,
                                FAR const char *format,
                                ...)
{
  char buf[256];
  va_list ap;
  int len;

  va_start(ap, format);
  len = vsnprintf(buf, sizeof(buf), format, ap);
  va_end(ap);

  if (len < 0)
    {
      return len;
    }

  return goldfish_camera_write_fully(file, buf, len + 1);
}

static ssize_t goldfish_camera_recv(FAR struct file *file,
                                    FAR void **data)
{
  char buf[9];
  size_t payload_size;
  bool empty = !*data;
  int ret;

  ret = goldfish_camera_read_fully(file, buf, 8);
  if (ret < 0)
    {
      return ret;
    }

  buf[8] = '\0';
  payload_size = strtoul(buf, NULL, 16);

  if (payload_size < 3)
    {
      return -EINVAL;
    }

  ret = goldfish_camera_read_fully(file, buf, 3);
  if (ret < 0)
    {
      return ret;
    }

  if (memcmp(buf, "ok", 2) != 0)
    {
      return -EINVAL;
    }

  if (buf[2] == '\0')
    {
      return 0;
    }

  payload_size -= 3;
  if (payload_size == 0)
    {
      return 0;
    }

  if (empty)
    {
      *data = kmm_malloc(payload_size);
      if (*data == NULL)
        {
          return -ENOMEM;
        }
    }

  ret = goldfish_camera_read_fully(file, *data, payload_size);
  if (ret < 0)
    {
      if (empty)
        {
          kmm_free(*data);
        }

      return ret;
    }

  return payload_size;
}

static ssize_t goldfish_camera_get_list(FAR goldfish_camera_priv_t **priv,
                                        size_t size)
{
  FAR void *cameradata = NULL;
  struct file file;
  FAR char *data;
  size_t count = 0;
  int ret;

  /* Queries list of cameras connected to the host. */

  ret = file_open(&file,
                  CONFIG_GOLDFISH_CAMERA_PIPE_PATH,
                  O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      verr("Failed to open: %s: %d\n",
           CONFIG_GOLDFISH_CAMERA_PIPE_PATH, ret);
      return ret;
    }

  ret = goldfish_camera_send(&file,
                             "pipe:%s:%s",
                             g_qemudservice,
                             g_cameraservice);
  if (ret < 0)
    {
      goto out;
    }

  ret = goldfish_camera_send(&file, "list");
  if (ret < 0)
    {
      goto out;
    }

  ret = goldfish_camera_recv(&file, &cameradata);
  if (ret < 0)
    {
      goto out;
    }

  /* Parse string
   *
   * For example:
   * 'name=virtualscene channel=0 pix=876758866 dir=back framedims=640x480'
   *
   * Use the strstr function to parse name/channel/pix/dir/framedims
   * of multiple devices by "name";
   *
   * Use the strchr function to parse multiple size by ',';
   */

  data = (FAR char *)cameradata;

  while ((data = strstr(data, "name")) != NULL)
    {
      char sizelist[256];
      FAR char *sizedata = sizelist;
      size_t sizecount = 0;

      priv[count] = kmm_zalloc(sizeof(goldfish_camera_priv_t));
      if (priv[count] == NULL)
        {
          verr("Failed to allocate instance\n");
          ret = -ENOMEM;
          goto out;
        }

      sscanf(data,
             "name=%s channel=%"SCNu32" pix=%"SCNu32" dir=%s framedims=%s",
             priv[count]->info.name,
             &priv[count]->info.channel,
             &priv[count]->info.pix,
             priv[count]->info.dir,
             sizelist);

      sscanf(sizedata,
             "%"SCNu16"x%"SCNu16"",
             &priv[count]->info.size[sizecount].width,
             &priv[count]->info.size[sizecount].height);

      while ((sizedata = strchr(sizedata, ',')) != NULL)
        {
          if (++sizecount >= GOLDFISH_CAMERA_SIZE_MAX_NUMBER)
            {
              ret = -E2BIG;
              goto err;
            }

          sizedata++;
          sscanf(sizedata,
                 "%"SCNu16"x%"SCNu16"",
                 &priv[count]->info.size[sizecount].width,
                 &priv[count]->info.size[sizecount].height);
        }

      if (++count >= size)
        {
          ret = -E2BIG;
          goto err;
        }

      data++;
    }

  ret = count;
  goto out;

err:
  do
    {
      if (priv[count])
        {
          kmm_free(priv[count]);
        }
    }
  while (count--);

out:
  file_close(&file);

  if (cameradata)
    {
      kmm_free(cameradata);
    }

  return ret;
}

static int goldfish_camera_thread(int argc, FAR char *argv[])
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)
                                     ((uintptr_t)strtoul(argv[1], NULL, 16));
  int ret;

  while (1)
    {
      if (nxsem_wait(&priv->run) < 0)
        {
          continue;
        }

      if (!priv->streaming)
        {
          return 0;
        }

      ret = goldfish_camera_send(&priv->file,
                                 "frame video=%"PRIu32" preview=0 "
                                 "whiteb=1,1,1 expcomp=1 time=0",
                                 priv->buf_size);

      if (ret < 0)
        {
          return ret;
        }

reload:
      ret = goldfish_camera_recv(&priv->file,
                                 (FAR void **)&priv->next_buf);
      if (ret < 0)
        {
          return ret;
        }
      else if (ret == 0)
        {
          goto reload;
        }
      else
        {
          struct timespec ts;
          struct timeval tv;

          DEBUGASSERT(ret == priv->buf_size);

          if (priv->capture_cb == NULL)
            {
              return 0;
            }

          clock_systime_timespec(&ts);
          TIMESPEC_TO_TIMEVAL(&tv, &ts);
          priv->capture_cb(0, priv->buf_size, &tv, priv->capture_arg);
        }
    }
}

/* Helper functions */

static uint32_t imgdata_fmt_to_v4l2(uint32_t pixelformat)
{
  switch (pixelformat)
    {
      case IMGDATA_PIX_FMT_YUV420P:
        return V4L2_PIX_FMT_YUV420;

      case IMGDATA_PIX_FMT_YUYV:
        return V4L2_PIX_FMT_YUYV;

      case IMGDATA_PIX_FMT_JPEG_WITH_SUBIMG:
        return V4L2_PIX_FMT_JPEG;

      case IMGDATA_PIX_FMT_JPEG:
        return V4L2_PIX_FMT_JPEG;

      case IMGDATA_PIX_FMT_RGB565:
        return V4L2_PIX_FMT_RGB565;

      case IMGDATA_PIX_FMT_UYVY:
        return V4L2_PIX_FMT_UYVY;

      default:

      /* Unsupported format */

        return 0;
    }
}

/* Sensor op functions are mostly dummy */

static bool goldfish_camera_is_available(FAR struct imgsensor_s *sensor)
{
  return access(CONFIG_GOLDFISH_CAMERA_PIPE_PATH, F_OK) == 0;
}

static int goldfish_camera_init(FAR struct imgsensor_s *sensor)
{
  return 0;
}

static int goldfish_camera_uninit(FAR struct imgsensor_s *sensor)
{
  return 0;
}

static FAR const char *
goldfish_camera_get_driver_name(FAR struct imgsensor_s *sensor)
{
  FAR goldfish_camera_priv_t *priv = container_of(sensor,
                                                  goldfish_camera_priv_t,
                                                  sensor);
  return priv->info.name;
}

static int
goldfish_camera_validate_frame_setting(FAR struct imgsensor_s *sensor,
                                       imgsensor_stream_type_t type,
                                       uint8_t nr_fmt,
                                       FAR imgsensor_format_t *fmt,
                                       FAR imgsensor_interval_t *interval)
{
  return 0;
}

static int goldfish_camera_start_capture(FAR struct imgsensor_s *sensor,
                                         imgsensor_stream_type_t type,
                                         uint8_t nr_fmt,
                                         FAR imgsensor_format_t *fmt,
                                         FAR imgsensor_interval_t *interval)
{
  return 0;
}

static int goldfish_camera_stop_capture(FAR struct imgsensor_s *sensor,
                                        imgsensor_stream_type_t type)
{
  return 0;
}

/* Data op functions do all the real work */

static int goldfish_camera_data_init(FAR struct imgdata_s *data)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  ret = file_open(&priv->file,
                  CONFIG_GOLDFISH_CAMERA_PIPE_PATH,
                  O_RDWR | O_CLOEXEC);
  if (ret < 0)
    {
      verr("Failed to open: %s: %d\n",
           CONFIG_GOLDFISH_CAMERA_PIPE_PATH, ret);
      return ret;
    }

  nxsem_init(&priv->run, 0, 0);
  priv->streaming = true;

  ret = goldfish_camera_send(&priv->file,
                             "pipe:%s:%s:name=%s",
                             g_qemudservice,
                             g_cameraservice,
                             priv->info.name);
  if (ret < 0)
    {
      goto err;
    }

  ret = goldfish_camera_send(&priv->file, "connect");
  if (ret < 0)
    {
      goto err;
    }

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("goldfish_camera_thread",
                        SCHED_PRIORITY_DEFAULT,
                        CONFIG_DEFAULT_TASK_STACKSIZE,
                        goldfish_camera_thread, argv);
  if (ret < 0)
    {
      goto err;
    }

  priv->pid = ret;
  return 0;

err:
  nxsem_destroy(&priv->run);
  priv->streaming = false;
  file_close(&priv->file);
  return ret;
}

static int goldfish_camera_data_uninit(FAR struct imgdata_s *data)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  int ret;

  ret = goldfish_camera_send(&priv->file, "disconnect");
  if (ret < 0)
    {
      return ret;
    }

  priv->streaming = false;
  nxsem_post(&priv->run);
  nxsched_waitpid(priv->pid, NULL, 0);

  nxsem_destroy(&priv->run);
  file_close(&priv->file);

  return 0;
}

static int goldfish_camera_data_validate_buf(FAR uint8_t *addr,
                                             uint32_t size)
{
  if (!addr || ((uintptr_t)addr & 0x1f))
    {
      return -EINVAL;
    }

  return 0;
}

static int goldfish_camera_data_set_buf(FAR struct imgdata_s *data,
                                        uint8_t nr_datafmts,
                                        FAR imgdata_format_t *datafmts,
                                        FAR uint8_t *addr,
                                        uint32_t size)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  int ret;

  ret = goldfish_camera_data_validate_buf(addr, size);
  if (ret < 0)
    {
      return ret;
    }

  priv->next_buf = addr;
  priv->buf_size = size;

  nxsem_post(&priv->run);

  return 0;
}

static int
goldfish_camera_data_validate_frame_setting(FAR struct imgdata_s *data,
                                            uint8_t nr_datafmt,
                                            FAR imgdata_format_t *datafmt,
                                            FAR imgdata_interval_t *interval)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  int i;

  for (i = 0; i < GOLDFISH_CAMERA_SIZE_MAX_NUMBER; i++)
    {
      if (datafmt[IMGDATA_FMT_MAIN].width == priv->info.size[i].width &&
          datafmt[IMGDATA_FMT_MAIN].height == priv->info.size[i].height &&
          imgdata_fmt_to_v4l2(datafmt[IMGDATA_FMT_MAIN].pixelformat)
                                                        == priv->info.pix)
        {
          return 0;
        }
    }

  return -ENOTSUP;
}

static int
goldfish_camera_data_start_capture(FAR struct imgdata_s *data,
                                   uint8_t nr_datafmt,
                                   FAR imgdata_format_t *datafmt,
                                   FAR imgdata_interval_t *interval,
                                   imgdata_capture_t callback,
                                   FAR void *arg)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  int ret;

  ret = goldfish_camera_send(&priv->file,
                             "start dim=%dx%d pix=%d",
                             datafmt[IMGDATA_FMT_MAIN].width,
                             datafmt[IMGDATA_FMT_MAIN].height,
                             imgdata_fmt_to_v4l2(
                                 datafmt[IMGDATA_FMT_MAIN].pixelformat));
  if (ret < 0)
    {
      return ret;
    }

  priv->capture_cb = callback;
  priv->capture_arg = arg;

  return 0;
}

static int goldfish_camera_data_stop_capture(FAR struct imgdata_s *data)
{
  FAR goldfish_camera_priv_t *priv = (FAR goldfish_camera_priv_t *)data;
  int ret;

  ret = goldfish_camera_send(&priv->file, "stop");
  if (ret < 0)
    {
      return ret;
    }

  priv->capture_cb = NULL;
  priv->capture_arg = NULL;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int goldfish_camera_initialize(void)
{
  FAR goldfish_camera_priv_t *priv[GOLDFISH_CAMERA_MAX_NUMBER];
  FAR struct imgsensor_s *sensor;
  char devpath[32];
  ssize_t number;
  ssize_t i;

  number = goldfish_camera_get_list(priv, GOLDFISH_CAMERA_MAX_NUMBER);
  if (number < 0)
    {
      return number;
    }

  for (i = 0; i < number; i++)
    {
      priv[i]->sensor.ops = &g_goldfish_camera_ops;
      priv[i]->data.ops   = &g_goldfish_camera_data_ops;

      sensor = &priv[i]->sensor;

      if (i == 0)
        {
          snprintf(devpath, sizeof(devpath), "/dev/video");
        }
      else
        {
          snprintf(devpath, sizeof(devpath), "/dev/video%zd", i);
        }

      capture_register(devpath, &priv[i]->data, &sensor, 1);
    }

  return 0;
}
