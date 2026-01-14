/****************************************************************************
 * drivers/sensors/goldfish_sensor_uorb.c
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

#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/sensors/goldfish_sensor.h>
#include <nuttx/sensors/sensor.h>
#include <sys/param.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GOLDFISH_LIST_SENSOR_CMD "list-sensors"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct goldfish_lowerhalf_s
{
  struct sensor_lowerhalf_s lower;
  FAR const struct sensor_device_info_s *info;
  intptr_t offset;
};

struct goldfish_sensor_s
{
  int64_t time_bias_ns;
  struct file pipe;
  struct goldfish_lowerhalf_s lower_accel;
  struct goldfish_lowerhalf_s lower_gyro;
  struct goldfish_lowerhalf_s lower_mag;
  struct goldfish_lowerhalf_s lower_orient;
  struct goldfish_lowerhalf_s lower_temp;
  struct goldfish_lowerhalf_s lower_prox;
  struct goldfish_lowerhalf_s lower_light;
  struct goldfish_lowerhalf_s lower_baro;
  struct goldfish_lowerhalf_s lower_humi;
  struct goldfish_lowerhalf_s lower_mag_uncalibrated;
  struct goldfish_lowerhalf_s lower_gyro_uncalibrated;
  struct goldfish_lowerhalf_s lower_hinge_angle0;
  struct goldfish_lowerhalf_s lower_hinge_angle1;
  struct goldfish_lowerhalf_s lower_hinge_angle2;
  struct goldfish_lowerhalf_s lower_hrate;
  struct goldfish_lowerhalf_s lower_wrist_tilt;
  struct goldfish_lowerhalf_s lower_accel_uncalibrated;
  uint32_t interval;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_sensor_activate(FAR struct sensor_lowerhalf_s *lower_,
                                    FAR struct file *filep, bool enabled);
static int
goldfish_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *period_us);
static int goldfish_sensor_get_info(FAR struct sensor_lowerhalf_s *lower_,
                                    FAR struct file *filep,
                                    FAR struct sensor_device_info_s *info);
static int goldfish_sensor_thread(int argc, FAR char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_goldfish_sensor_ops =
{
  .activate = goldfish_sensor_activate,
  .set_interval = goldfish_sensor_set_interval,
  .get_info = goldfish_sensor_get_info,
};

static const struct sensor_device_info_s g_goldfish_sensor_accel =
{
    .version                    = 1,
    .power                      = 3.0f,
    .max_range                  = 2.8f,
    .resolution                 = 1.0f / 4032.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "acceleration",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_gyro =
{
    .version                    = 1,
    .power                      = 3.0f,
    .max_range                  = 11.1111111,
    .resolution                 = 1.0f / 1000.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "gyroscope",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_mag =
{
    .version                    = 1,
    .power                      = 6.7f,
    .max_range                  = 2000.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "magnetic-field",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_orient =
{
    .version                    = 1,
    .power                      = 9.7f,
    .max_range                  = 360.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "orientation",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_temp =
{
    .version                    = 1,
    .power                      = 0.0f,
    .max_range                  = 80.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "temperature",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_prox =
{
    .version                    = 1,
    .power                      = 20.0f,
    .max_range                  = 1.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "proximity",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_light =
{
    .version                    = 1,
    .power                      = 20.0f,
    .max_range                  = 40000.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "light",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_baro =
{
    .version                    = 1,
    .power                      = 20.0f,
    .max_range                  = 800.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "pressure",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_humi =
{
    .version                    = 1,
    .power                      = 20.0f,
    .max_range                  = 100.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .fifo_reserved_event_count  = 0,
    .fifo_max_event_count       = 0,
    .name                       = "humidity",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_mag_uncal =
{
    .version                    = 1,
    .power                      = 6.7f,
    .max_range                  = 2000.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "magnetic-field-uncalibrated",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_gyro_uncal =
{
    .version                    = 1,
    .power                      = 3.0,
    .max_range                  = 16.46,
    .resolution                 = 1.0 / 1000.0,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "gyroscope-uncalibrated",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_hinge_angle0 =
{
    .version                    = 1,
    .power                      = 3.0,
    .max_range                  = 360,
    .resolution                 = 1.0,
    .min_delay                  = 0,
    .max_delay                  = 0,
    .name                       = "hinge-angle0",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_hinge_angle1 =
{
    .version                    = 1,
    .power                      = 3.0,
    .max_range                  = 360,
    .resolution                 = 1.0,
    .min_delay                  = 0,
    .max_delay                  = 0,
    .name                       = "hinge-angle1",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_hinge_angle2 =
{
    .version                    = 1,
    .power                      = 3.0,
    .max_range                  = 360,
    .resolution                 = 1.0,
    .min_delay                  = 0,
    .max_delay                  = 0,
    .name                       = "hinge-angle2",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_hrate =
{
    .version                    = 1,
    .power                      = 20.0,
    .max_range                  = 500,
    .resolution                 = 1.0,
    .min_delay                  = 0,
    .max_delay                  = 500 * 1000,
    .name                       = "heart-rate",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_wrist_tilt =
{
    .version                    = 1,
    .power                      = 20.0,
    .max_range                  = 1.0,
    .resolution                 = 1.0,
    .min_delay                  = 0,
    .max_delay                  = 500 * 1000,
    .name                       = "wrist-tilt",
    .vendor                     = "The Android Open Source Project",
};

static const struct sensor_device_info_s g_goldfish_sensor_accel_uncal =
{
    .version                    = 1,
    .power                      = 3.0,
    .max_range                  = 39.3,
    .resolution                 = 1.0 / 4032.0,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "acceleration-uncalibrated",
    .vendor                     = "The Android Open Source Project",
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int goldfish_sensor_read_pipe(FAR struct file *pipe,
                                            FAR void *buffer,
                                            size_t size)
{
  FAR char *p = (FAR char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_read(pipe, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

static inline int goldfish_sensor_write_pipe(FAR struct file *pipe,
                                             FAR const void *buffer,
                                             size_t size)
{
  FAR const char *p = (const char *)buffer;

  while (size > 0)
    {
      ssize_t n = file_write(pipe, p, size);
      if (n < 0)
        {
          return n;
        }

      p += n;
      size -= n;
    }

  return 0;
}

static inline int goldfish_sensor_send(FAR struct file *pipe,
                                       FAR const void *msg,
                                       size_t size)
{
  char header[5];
  int ret = 0;

  if (size == 0)
    {
      size = strlen(msg);
    }

  snprintf(header, sizeof(header), "%04zx", size);

  ret = goldfish_sensor_write_pipe(pipe, header, 4);
  if (ret < 0)
    {
      return ret;
    }

  return goldfish_sensor_write_pipe(pipe, msg, size);
}

static inline ssize_t goldfish_sensor_recv(FAR struct file *pipe,
                                           FAR void *msg,
                                           size_t maxsize)
{
  char header[5];
  size_t size;
  int ret;

  ret = goldfish_sensor_read_pipe(pipe, header, 4);
  if (ret < 0)
    {
      return ret;
    }

  header[4] = 0;

  if (sscanf(header, "%04zx", &size) != 1)
    {
      return -EINVAL;
    }

  if (size > maxsize)
    {
      return -E2BIG;
    }

  ret = goldfish_sensor_read_pipe(pipe, msg, size);
  if (ret < 0)
    {
      return ret;
    }

  return size;
}

static inline int goldfish_sensor_open_pipe(FAR struct file *filep,
                                            FAR const char *ns,
                                            FAR const char *pipe_name,
                                            int flags)
{
  char buf[256];
  int len;
  int ret;

  ret = file_open(filep, "/dev/goldfish_pipe", flags);
  if (ret < 0)
    {
      snerr("Could not open /dev/goldfish_pipe : %d", ret);
      return ret;
    }

  if (ns)
    {
      len = snprintf(buf, sizeof(buf), "pipe:%s:%s", ns, pipe_name);
    }
  else
    {
      len = snprintf(buf, sizeof(buf), "pipe:%s", pipe_name);
    }

  ret = goldfish_sensor_write_pipe(filep, buf, len + 1);
  if (ret < 0)
    {
      snerr("Could not connect to the '%s' service: %d", buf, ret);
      file_close(filep);
    }

  return ret;
}

static FAR const char *
goldfish_sensor_match(FAR const char *s, FAR const char *p)
{
  size_t l = strlen(p);
  return strncmp(s, p, l) ? NULL : s + l;
}

static int64_t
goldfish_sensor_weigthed_average(int64_t a, int64_t aw,
                                 int64_t b, int64_t bw)
{
  return (a * aw + b * bw) / (aw + bw);
}

static void goldfish_sensor_parse_event(FAR struct goldfish_sensor_s *sensor)
{
  FAR const char *value;
  char buf[256];
  ssize_t len;
  uint64_t now_ns;

  len = goldfish_sensor_recv(&sensor->pipe, buf, sizeof(buf) - 1);
  if (len < 0)
    {
      snerr("goldfish_sensor_recv failed\n");
      return;
    }

  now_ns = sensor_get_timestamp();
  buf[len] = 0;

  if ((value = goldfish_sensor_match(buf, "acceleration:")) != NULL)
    {
      struct sensor_accel accel;
      if (sscanf(value, "%f:%f:%f",
                 &accel.x, &accel.y, &accel.z) == 3)
        {
          accel.temperature = NAN;
          accel.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_accel.lower.push_event(
                  sensor->lower_accel.lower.priv,
                  &accel, sizeof(struct sensor_accel));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "gyroscope:")) != NULL)
    {
      struct sensor_gyro gyro;
      if (sscanf(value, "%f:%f:%f",
                 &gyro.x, &gyro.y, &gyro.z) == 3)
        {
          gyro.temperature = NAN;
          gyro.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_gyro.lower.push_event(
                  sensor->lower_gyro.lower.priv,
                  &gyro, sizeof(struct sensor_gyro));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "magnetic:")) != NULL)
    {
      struct sensor_mag mag;
      if (sscanf(value, "%f:%f:%f",
                 &mag.x, &mag.y, &mag.z) == 3)
        {
          mag.temperature = NAN;
          mag.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_mag.lower.push_event(
                  sensor->lower_mag.lower.priv,
                  &mag, sizeof(struct sensor_mag));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "orientation:")) != NULL)
    {
      struct sensor_orientation orient;
      if (sscanf(value, "%f:%f:%f",
                 &orient.x, &orient.y, &orient.z) == 3)
        {
          orient.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_orient.lower.push_event(
                  sensor->lower_orient.lower.priv,
                  &orient, sizeof(struct sensor_orientation));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "temperature:")) != NULL)
    {
      struct sensor_temp temp;
      if (sscanf(value, "%f", &temp.temperature) == 1)
        {
          temp.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_temp.lower.push_event(
                  sensor->lower_temp.lower.priv,
                  &temp, sizeof(struct sensor_temp));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "proximity:")) != NULL)
    {
      struct sensor_prox prox;
      if (sscanf(value, "%f", &prox.proximity) == 1)
        {
          prox.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_prox.lower.push_event(
                  sensor->lower_prox.lower.priv,
                  &prox, sizeof(struct sensor_prox));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "light:")) != NULL)
    {
      struct sensor_light light;
      if (sscanf(value, "%f", &light.light) == 1)
        {
          light.timestamp = now_ns + sensor->time_bias_ns;
          light.ir = NAN;
          sensor->lower_light.lower.push_event(
                  sensor->lower_light.lower.priv,
                  &light, sizeof(struct sensor_light));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "pressure:")) != NULL)
    {
      struct sensor_baro baro;
      if (sscanf(value, "%f", &baro.pressure) == 1)
        {
          baro.timestamp = now_ns + sensor->time_bias_ns;
          baro.temperature = NAN;
          sensor->lower_baro.lower.push_event(
                  sensor->lower_baro.lower.priv,
                  &baro, sizeof(struct sensor_baro));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "humidity:")) != NULL)
    {
      struct sensor_humi humi;
      if (sscanf(value, "%f", &humi.humidity) == 1)
        {
          humi.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_humi.lower.push_event(
                  sensor->lower_humi.lower.priv,
                  &humi, sizeof(struct sensor_humi));
        }
    }
  else if ((value = goldfish_sensor_match(
                    buf, "magnetic-uncalibrated:")) != NULL)
    {
      struct sensor_mag mag;
      if (sscanf(value, "%f:%f:%f",
                 &mag.x, &mag.y, &mag.z) == 3)
        {
          mag.temperature = NAN;
          mag.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_mag_uncalibrated.lower.push_event(
                  sensor->lower_mag_uncalibrated.lower.priv,
                  &mag, sizeof(struct sensor_mag));
        }
    }
  else if ((value = goldfish_sensor_match(
                    buf, "gyroscope-uncalibrated:")) != NULL)
    {
      struct sensor_gyro gyro;
      if (sscanf(value, "%f:%f:%f",
                 &gyro.x, &gyro.y, &gyro.z) == 3)
        {
          gyro.temperature = NAN;
          gyro.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_gyro_uncalibrated.lower.push_event(
                  sensor->lower_gyro_uncalibrated.lower.priv,
                  &gyro, sizeof(struct sensor_gyro));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "hinge-angle0")) != NULL)
    {
      struct sensor_angle angle;
      if (sscanf(value, "%f:", &angle.angle) == 1)
        {
          angle.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_hinge_angle0.lower.push_event(
                  sensor->lower_hinge_angle0.lower.priv,
                  &angle, sizeof(struct sensor_angle));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "hinge-angle1")) != NULL)
    {
      struct sensor_angle angle;
      if (sscanf(value, "%f:", &angle.angle) == 1)
        {
          angle.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_hinge_angle1.lower.push_event(
                  sensor->lower_hinge_angle1.lower.priv,
                  &angle, sizeof(struct sensor_angle));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "hinge-angle2")) != NULL)
    {
      struct sensor_angle angle;
      if (sscanf(value, "%f:", &angle.angle) == 1)
        {
          angle.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_hinge_angle2.lower.push_event(
                  sensor->lower_hinge_angle2.lower.priv,
                  &angle, sizeof(struct sensor_angle));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "heart-rate:")) != NULL)
    {
      struct sensor_hrate hrate;
      if (sscanf(value, "%f", &hrate.bpm) == 1)
        {
          hrate.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_hrate.lower.push_event(
                  sensor->lower_hrate.lower.priv,
                  &hrate, sizeof(struct sensor_hrate));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "wrist-tilt")) != NULL)
    {
      struct sensor_event wrist;
      if (sscanf(value, "%" PRIu32, &wrist.event) == 1)
        {
          wrist.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_wrist_tilt.lower.push_event(
                  sensor->lower_wrist_tilt.lower.priv,
                  &wrist, sizeof(struct sensor_event));
        }
    }
  else if ((value = goldfish_sensor_match(
                    buf, "acceleration-uncalibrated:")) != NULL)
    {
      struct sensor_accel accel;
      if (sscanf(value, "%f:%f:%f",
                 &accel.x, &accel.y, &accel.z) == 3)
        {
          accel.temperature = NAN;
          accel.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_accel_uncalibrated.lower.push_event(
                  sensor->lower_accel_uncalibrated.lower.priv,
                  &accel, sizeof(struct sensor_accel));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "guest-sync:")) != NULL)
    {
      int64_t guest_ms;
      if ((sscanf(value, "%" PRId64, &guest_ms) == 1) && (guest_ms >= 0))
        {
          int64_t time_bias_ns = 1000 * guest_ms - now_ns;
          sensor->time_bias_ns =
            MIN(0, goldfish_sensor_weigthed_average(sensor->time_bias_ns,
                                                    3, time_bias_ns, 1));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "sync:")) != NULL)
    {
    }
  else
    {
      snerr("don't know how to parse '%s'\n", buf);
    }
}

static FAR struct goldfish_sensor_s *
goldfish_get_priv(FAR struct sensor_lowerhalf_s *lower_)
{
  FAR struct goldfish_lowerhalf_s *lower =
    container_of(lower_, struct goldfish_lowerhalf_s, lower);
  intptr_t offset = lower->offset;

  return (FAR struct goldfish_sensor_s *)((FAR char *)lower - offset);
}

static int goldfish_sensor_activate(FAR struct sensor_lowerhalf_s *lower_,
                                    FAR struct file *filep, bool enabled)
{
  FAR struct goldfish_lowerhalf_s *lower =
    container_of(lower_, struct goldfish_lowerhalf_s, lower);
  FAR struct goldfish_sensor_s *priv =  goldfish_get_priv(lower_);
  char buffer[64];
  int len;

  len = snprintf(buffer, sizeof(buffer),
                 "set:%s:%d",
                 lower->info->name,
                 enabled);
  return goldfish_sensor_send(&priv->pipe, buffer, len);
}

static int
goldfish_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR uint32_t *period_us)
{
  FAR struct goldfish_sensor_s *priv = goldfish_get_priv(lower);
  char buffer[64];
  int len;
  int ret;

  len = snprintf(buffer, sizeof(buffer), "set-delay: %d",
                 (int)(*period_us / 1000));
  ret = goldfish_sensor_send(&priv->pipe, buffer, len);
  if (ret < 0)
    {
      return ret;
    }

  priv->interval = *period_us;
  return OK;
}

static int goldfish_sensor_get_info(FAR struct sensor_lowerhalf_s *lower_,
                                    FAR struct file *filep,
                                    FAR struct sensor_device_info_s *info)
{
  FAR struct goldfish_lowerhalf_s *lower =
    container_of(lower_, struct goldfish_lowerhalf_s, lower);
  memcpy(info, lower->info, sizeof(struct sensor_device_info_s));
  return OK;
}

static int goldfish_sensor_thread(int argc, FAR char** argv)
{
  FAR struct goldfish_sensor_s *priv =
    (FAR struct goldfish_sensor_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      goldfish_sensor_parse_event(priv);
    }

  return OK;
}

static int
goldfish_register_sensor(FAR struct goldfish_lowerhalf_s *lower,
                         FAR const struct sensor_device_info_s *info,
                         int type, int devno, uint32_t batch_number,
                         bool uncalibrated, intptr_t offset)
{
  lower->lower.uncalibrated = uncalibrated;
  lower->lower.type = type;
  lower->lower.ops = &g_goldfish_sensor_ops;
  lower->lower.nbuffer = batch_number;
  lower->info = info;
  lower->offset = offset;
  return sensor_register(&lower->lower, devno);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: goldfish_sensor_init
 *
 * Description:
 *   Goldfish Multi-Sensors driver entrypoint.
 *
 * Input Parameters:
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch.
 *
 * Returned Value:
 *   Zero (OK) or positive on success; a negated errno value on failure.
 *
 ****************************************************************************/

int goldfish_sensor_init(int devno, uint32_t batch_number)
{
  FAR struct goldfish_sensor_s *sensor;
  uint32_t sensors_mask;
  FAR char *argv[2];
  char arg1[32];
  char buffer[64];
  int ret;
  int len;

  /* Alloc memory for sensor */

  sensor = kmm_zalloc(sizeof(struct goldfish_sensor_s));
  if (!sensor)
    {
      snerr("Memory cannot be allocated for goldfish_sensor\n");
      return -ENOMEM;
    }

  ret = goldfish_sensor_open_pipe(&sensor->pipe, "qemud", "sensors", O_RDWR);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ret;
    }

  len = snprintf(buffer, sizeof(buffer),
                 "time:%" PRId64, sensor_get_timestamp());

  ret = goldfish_sensor_send(&sensor->pipe, buffer, len);
  if (ret < 0)
    {
      snerr("goldfish_sensor_send failed\n");
      return ret;
    }

  ret = goldfish_sensor_send(&sensor->pipe,
                             GOLDFISH_LIST_SENSOR_CMD,
                             strlen(GOLDFISH_LIST_SENSOR_CMD));
  if (ret < 0)
    {
      snerr("goldfish_sensor_send failed\n");
      return ret;
    }

  len = goldfish_sensor_recv(&sensor->pipe, buffer, sizeof(buffer) - 1);
  if (len < 0)
    {
      snerr("goldfish_sensor_recv failed\n");
      return len;
    }

  buffer[len] = 0;
  if (sscanf(buffer, "%" SCNu32, &sensors_mask) != 1)
    {
      snerr("Can't parse qemud response\n");
      return -EINVAL;
    }

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", sensor);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("goldfish_sensor_thread",
                       SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       goldfish_sensor_thread, argv);
  if (ret < 0)
    {
      file_close(&sensor->pipe);
      kmm_free(sensor);
      return ret;
    }

  /*  Register sensor */

  return goldfish_register_sensor(
            &sensor->lower_accel,
            &g_goldfish_sensor_accel,
            SENSOR_TYPE_ACCELEROMETER,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_accel)) |
         goldfish_register_sensor(
            &sensor->lower_gyro,
            &g_goldfish_sensor_gyro,
            SENSOR_TYPE_GYROSCOPE,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_gyro)) |
         goldfish_register_sensor(
            &sensor->lower_mag,
            &g_goldfish_sensor_mag,
            SENSOR_TYPE_MAGNETIC_FIELD,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_mag)) |
          goldfish_register_sensor(
            &sensor->lower_orient,
            &g_goldfish_sensor_orient,
            SENSOR_TYPE_ORIENTATION,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_orient)) |
          goldfish_register_sensor(
            &sensor->lower_temp,
            &g_goldfish_sensor_temp,
            SENSOR_TYPE_AMBIENT_TEMPERATURE,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_temp)) |
          goldfish_register_sensor(
            &sensor->lower_prox,
            &g_goldfish_sensor_prox,
            SENSOR_TYPE_PROXIMITY,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_prox)) |
          goldfish_register_sensor(
            &sensor->lower_light,
            &g_goldfish_sensor_light,
            SENSOR_TYPE_LIGHT,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_light)) |
          goldfish_register_sensor(
            &sensor->lower_baro,
            &g_goldfish_sensor_baro,
            SENSOR_TYPE_BAROMETER,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_baro)) |
          goldfish_register_sensor(
            &sensor->lower_humi,
            &g_goldfish_sensor_humi,
            SENSOR_TYPE_RELATIVE_HUMIDITY,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_humi)) |
          goldfish_register_sensor(
            &sensor->lower_mag_uncalibrated,
            &g_goldfish_sensor_mag_uncal,
            SENSOR_TYPE_MAGNETIC_FIELD,
            devno, batch_number, true,
            offsetof(struct goldfish_sensor_s, lower_mag_uncalibrated)) |
          goldfish_register_sensor(
            &sensor->lower_gyro_uncalibrated,
            &g_goldfish_sensor_gyro_uncal,
            SENSOR_TYPE_GYROSCOPE,
            devno, batch_number, true,
            offsetof(struct goldfish_sensor_s, lower_gyro_uncalibrated)) |
          goldfish_register_sensor(
            &sensor->lower_hinge_angle0,
            &g_goldfish_sensor_hinge_angle0,
            SENSOR_TYPE_HINGE_ANGLE,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_hinge_angle0)) |
          goldfish_register_sensor(
            &sensor->lower_hinge_angle1,
            &g_goldfish_sensor_hinge_angle1,
            SENSOR_TYPE_HINGE_ANGLE,
            devno + 1, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_hinge_angle1)) |
          goldfish_register_sensor(
            &sensor->lower_hinge_angle2,
            &g_goldfish_sensor_hinge_angle2,
            SENSOR_TYPE_HINGE_ANGLE,
            devno + 2, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_hinge_angle2)) |
          goldfish_register_sensor(
            &sensor->lower_hrate,
            &g_goldfish_sensor_hrate,
            SENSOR_TYPE_HEART_RATE,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_hrate)) |
          goldfish_register_sensor(
            &sensor->lower_wrist_tilt,
            &g_goldfish_sensor_wrist_tilt,
            SENSOR_TYPE_WRIST_TILT_GESTURE,
            devno, batch_number, false,
            offsetof(struct goldfish_sensor_s, lower_wrist_tilt)) |
          goldfish_register_sensor(
            &sensor->lower_accel_uncalibrated,
            &g_goldfish_sensor_accel_uncal,
            SENSOR_TYPE_ACCELEROMETER,
            devno, batch_number, true,
            offsetof(struct goldfish_sensor_s, lower_accel_uncalibrated));
}
