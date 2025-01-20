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
#include <math.h>
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

#define GOLDFISH_ACCELERATION 0
#define GOLDFISH_GYROSCOPE 1
#define GOLDFISH_MAGNETIC_FIELD 2
#define GOLDFISH_ORIENTATION 3
#define GOLDFISH_AMBIENT_TEMPERATURE 4
#define GOLDFISH_PROXIMITY 5
#define GOLDFISH_LIGHT 6
#define GOLDFISH_PRESSURE 7
#define GOLDFISH_RELATIVE_HUMIDITY 8
#define GOLDFISH_MAGNETIC_FIELD_UNCALIBRATED 9
#define GOLDFISH_GYROSCOPE_UNCALIBRATED 10
#define GOLDFISH_HINGE_ANGLE0 11
#define GOLDFISH_HINGE_ANGLE1 12
#define GOLDFISH_HINGE_ANGLE2 13
#define GOLDFISH_HEART_RATE 14
#define GOLDFISH_RGBC_LIGHT 15
#define GOLDFISH_WRIST_TILT 16
#define GOLDFISH_ACCELERATION_UNCALIBRATED 17

#define GOLDFISH_LIST_SENSOR_CMD "list-sensors"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct goldfish_sensor_s
{
  int64_t time_bias_ns;
  struct file pipe;
  struct sensor_lowerhalf_s lower_accel;
  struct sensor_lowerhalf_s lower_mag;
  struct sensor_lowerhalf_s lower_gyro;
  struct sensor_lowerhalf_s lower_accel_uncalibrated;
  struct sensor_lowerhalf_s lower_mag_uncalibrated;
  struct sensor_lowerhalf_s lower_gyro_uncalibrated;
  struct sensor_lowerhalf_s lower_prox;
  struct sensor_lowerhalf_s lower_light;
  struct sensor_lowerhalf_s lower_baro;
  struct sensor_lowerhalf_s lower_humi;
  struct sensor_lowerhalf_s lower_temp;
  struct sensor_lowerhalf_s lower_hrate;
  uint32_t interval;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int goldfish_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep, bool enabled);
static int goldfish_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                        FAR struct file *filep,
                                        FAR uint32_t *period_us);
static int goldfish_sensor_get_info(FAR struct sensor_lowerhalf_s *lower,
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

FAR static const char *const g_goldfish_sensor_name[] =
{
  "acceleration",
  "gyroscope",
  "magnetic-field",
  "orientation",
  "temperature",
  "proximity",
  "light",
  "pressure",
  "humidity",
  "magnetic-field-uncalibrated",
  "gyroscope-uncalibrated",
  "hinge-angle0",
  "hinge-angle1",
  "hinge-angle2",
  "heart-rate",
  "rgbc-light",
  "wrist-tilt",
  "acceleration-uncalibrated",
};

static struct sensor_device_info_s g_goldfish_sensor_info[] =
{
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
  },
  {
    .version                    = 1,
    .power                      = 3.0f,
    .max_range                  = 11.1111111,
    .resolution                 = 1.0f / 1000.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "gyroscope",
    .vendor                     = "The Android Open Source Project",
  },
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
  },
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
  },
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
  },
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
  },
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
  },
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
  },
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
  },
  {
    .version                    = 1,
    .power                      = 6.7f,
    .max_range                  = 2000.0f,
    .resolution                 = 1.0f,
    .min_delay                  = 10000,
    .max_delay                  = 500 * 1000,
    .name                       = "magnetic-field-uncalibrated",
    .vendor                     = "The Android Open Source Project",
  },
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

static inline FAR const char *goldfish_sensor_get_name(int h)
{
  return g_goldfish_sensor_name[h];
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

static int goldfish_sensor_do_activate(FAR struct file *pipe,
                                       int handle,
                                       bool enabled)
{
  char buffer[64];
  int len;

  len = snprintf(buffer, sizeof(buffer),
                 "set:%s:%d",
                 goldfish_sensor_get_name(handle),
                 enabled);

  return goldfish_sensor_send(pipe, buffer, len);
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
          sensor->lower_accel.push_event(sensor->lower_accel.priv,
                                         &accel,
                                         sizeof(struct sensor_accel));
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
          sensor->lower_gyro.push_event(sensor->lower_gyro.priv,
                                        &gyro,
                                        sizeof(struct sensor_gyro));
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
          sensor->lower_mag.push_event(sensor->lower_mag.priv,
                                       &mag,
                                       sizeof(struct sensor_mag));
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
          sensor->lower_gyro_uncalibrated.push_event(
                  sensor->lower_gyro_uncalibrated.priv,
                  &gyro,
                  sizeof(struct sensor_gyro));
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
          sensor->lower_accel_uncalibrated.push_event(
                  sensor->lower_accel_uncalibrated.priv,
                  &accel,
                  sizeof(struct sensor_accel));
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
          sensor->lower_mag_uncalibrated.push_event(
                  sensor->lower_mag_uncalibrated.priv,
                  &mag,
                  sizeof(struct sensor_mag));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "temperature:")) != NULL)
    {
      struct sensor_temp temp;
      if (sscanf(value, "%f", &temp.temperature) == 1)
        {
          temp.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_temp.push_event(sensor->lower_temp.priv,
                                        &temp,
                                        sizeof(struct sensor_temp));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "proximity:")) != NULL)
    {
      struct sensor_prox prox;
      if (sscanf(value, "%f", &prox.proximity) == 1)
        {
          prox.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_prox.push_event(sensor->lower_prox.priv,
                                        &prox,
                                        sizeof(struct sensor_prox));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "light:")) != NULL)
    {
      struct sensor_light light;
      if (sscanf(value, "%f", &light.light) == 1)
        {
          light.timestamp = now_ns + sensor->time_bias_ns;
          light.ir = NAN;
          sensor->lower_light.push_event(sensor->lower_light.priv,
                                         &light,
                                         sizeof(struct sensor_light));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "pressure:")) != NULL)
    {
      struct sensor_baro baro;
      if (sscanf(value, "%f", &baro.pressure) == 1)
        {
          baro.timestamp = now_ns + sensor->time_bias_ns;
          baro.temperature = NAN;
          sensor->lower_baro.push_event(sensor->lower_baro.priv,
                                        &baro,
                                        sizeof(struct sensor_baro));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "humidity:")) != NULL)
    {
      struct sensor_humi humi;
      if (sscanf(value, "%f", &humi.humidity) == 1)
        {
          humi.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_humi.push_event(sensor->lower_humi.priv,
                                        &humi,
                                        sizeof(struct sensor_humi));
        }
    }
  else if ((value = goldfish_sensor_match(buf, "heart-rate:")) != NULL)
    {
      struct sensor_hrate hrate;
      if (sscanf(value, "%f", &hrate.bpm) == 1)
        {
          hrate.timestamp = now_ns + sensor->time_bias_ns;
          sensor->lower_hrate.push_event(sensor->lower_hrate.priv,
                                         &hrate,
                                         sizeof(struct sensor_hrate));
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

static int goldfish_get_priv(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct goldfish_sensor_s **priv)
{
  switch (lower->type)
  {
  case SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED:
    priv = container_of(lower, struct goldfish_sensor_s,
                        lower_accel_uncalibrated);
    return GOLDFISH_ACCELERATION_UNCALIBRATED;
  case SENSOR_TYPE_ACCELEROMETER:
    priv = container_of(lower, struct goldfish_sensor_s, lower_accel);
    return GOLDFISH_ACCELERATION;
  case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
    priv = container_of(lower, struct goldfish_sensor_s,
                        lower_mag_uncalibrated);
    return GOLDFISH_MAGNETIC_FIELD_UNCALIBRATED;
  case SENSOR_TYPE_MAGNETIC_FIELD:
    priv = container_of(lower, struct goldfish_sensor_s, lower_mag);
    return GOLDFISH_MAGNETIC_FIELD;
  case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
    priv = container_of(lower, struct goldfish_sensor_s,
                        lower_gyro_uncalibrated);
    return GOLDFISH_GYROSCOPE_UNCALIBRATED;
  case SENSOR_TYPE_GYROSCOPE:
    priv = container_of(lower, struct goldfish_sensor_s, lower_gyro);
    return GOLDFISH_GYROSCOPE;
  case SENSOR_TYPE_PROXIMITY:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_prox);
    return GOLDFISH_PROXIMITY;
  case SENSOR_TYPE_LIGHT:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_light);
    return GOLDFISH_LIGHT;
  case SENSOR_TYPE_BAROMETER:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_baro);
    return GOLDFISH_PRESSURE;
  case SENSOR_TYPE_RELATIVE_HUMIDITY:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_humi);
    return GOLDFISH_RELATIVE_HUMIDITY;
  case SENSOR_TYPE_AMBIENT_TEMPERATURE:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_temp);
    return GOLDFISH_AMBIENT_TEMPERATURE;
  case SENSOR_TYPE_HEART_RATE:
    *priv = container_of(lower, struct goldfish_sensor_s, lower_hrate);
    return GOLDFISH_HEART_RATE;
  default:
    return -EINVAL;
  }
}

static int goldfish_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep, bool enabled)
{
  FAR struct goldfish_sensor_s *priv;
  int handle = goldfish_get_priv(lower, &priv);

  if (handle < 0)
    {
      return handle;
    }

  return goldfish_sensor_do_activate(&priv->pipe, handle, enabled);
}

static int goldfish_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                        FAR struct file *filep,
                                        FAR uint32_t *period_us)
{
  struct FAR goldfish_sensor_s *priv;
  char buffer[64];
  int handle;
  int len;

  handle = goldfish_get_priv(lower, &priv);
  if (handle < 0)
    {
      return handle;
    }

  len = snprintf(buffer, sizeof(buffer), "set-delay: %d",
                 (int)(*period_us / 1000));
  goldfish_sensor_send(&priv->pipe, buffer, len);
  priv->interval = *period_us;
  return OK;
}

static int goldfish_sensor_get_info(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    FAR struct sensor_device_info_s *info)
{
  FAR struct goldfish_sensor_s *priv;
  int handle;
  int i;

  handle = goldfish_get_priv(lower, &priv);
  if (handle < 0)
    {
      return -handle;
    }

  for (i = 0; i < sizeof(g_goldfish_sensor_info); i++)
    {
      if (!strncmp(goldfish_sensor_get_name(handle),
                   g_goldfish_sensor_info[i].name,
                   strlen(g_goldfish_sensor_info[i].name)))
        {
          memcpy(info, &g_goldfish_sensor_info[i],
                 sizeof(struct sensor_device_info_s));
          return OK;
        }
    }

  return -EINVAL;
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

  sensor->lower_accel.type = SENSOR_TYPE_ACCELEROMETER;
  sensor->lower_accel.ops = &g_goldfish_sensor_ops;
  sensor->lower_accel.nbuffer = batch_number;

  sensor->lower_mag.type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->lower_mag.ops = &g_goldfish_sensor_ops;
  sensor->lower_mag.nbuffer = batch_number;

  sensor->lower_gyro.type = SENSOR_TYPE_GYROSCOPE;
  sensor->lower_gyro.ops = &g_goldfish_sensor_ops;
  sensor->lower_gyro.nbuffer = batch_number;

  sensor->lower_accel_uncalibrated.type =
                         SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED;
  sensor->lower_accel_uncalibrated.ops = &g_goldfish_sensor_ops;
  sensor->lower_accel_uncalibrated.nbuffer = batch_number;

  sensor->lower_mag_uncalibrated.type =
                      SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
  sensor->lower_mag_uncalibrated.ops = &g_goldfish_sensor_ops;
  sensor->lower_mag_uncalibrated.nbuffer = batch_number;

  sensor->lower_gyro_uncalibrated.type =
                            SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
  sensor->lower_gyro_uncalibrated.ops = &g_goldfish_sensor_ops;
  sensor->lower_gyro_uncalibrated.nbuffer = batch_number;

  sensor->lower_prox.type = SENSOR_TYPE_PROXIMITY;
  sensor->lower_prox.ops = &g_goldfish_sensor_ops;
  sensor->lower_prox.nbuffer = batch_number;

  sensor->lower_light.type = SENSOR_TYPE_LIGHT;
  sensor->lower_light.ops = &g_goldfish_sensor_ops;
  sensor->lower_light.nbuffer = batch_number;

  sensor->lower_baro.type = SENSOR_TYPE_BAROMETER;
  sensor->lower_baro.ops = &g_goldfish_sensor_ops;
  sensor->lower_baro.nbuffer = batch_number;

  sensor->lower_humi.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sensor->lower_humi.ops = &g_goldfish_sensor_ops;
  sensor->lower_humi.nbuffer = batch_number;

  sensor->lower_temp.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  sensor->lower_temp.ops = &g_goldfish_sensor_ops;
  sensor->lower_temp.nbuffer = batch_number;

  sensor->lower_hrate.type = SENSOR_TYPE_HEART_RATE;
  sensor->lower_hrate.ops = &g_goldfish_sensor_ops;
  sensor->lower_hrate.nbuffer = batch_number;

  return sensor_register(&sensor->lower_accel, devno) |
         sensor_register(&sensor->lower_mag, devno) |
         sensor_register(&sensor->lower_gyro, devno) |
         sensor_register(&sensor->lower_accel_uncalibrated, devno) |
         sensor_register(&sensor->lower_mag_uncalibrated, devno) |
         sensor_register(&sensor->lower_gyro_uncalibrated, devno) |
         sensor_register(&sensor->lower_prox, devno) |
         sensor_register(&sensor->lower_light, devno) |
         sensor_register(&sensor->lower_baro, devno) |
         sensor_register(&sensor->lower_humi, devno) |
         sensor_register(&sensor->lower_temp, devno) |
         sensor_register(&sensor->lower_hrate, devno);
}
