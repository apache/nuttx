/****************************************************************************
 * drivers/sensors/fakesensor_uorb.c
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

#include <nuttx/config.h>

#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/gnss.h>
#include <nuttx/signal.h>
#include <debug.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fakesensor_s
{
  union
    {
      struct sensor_lowerhalf_s lower;
      struct gnss_lowerhalf_s gnss;
    };

  int type;
  struct file data;
  uint32_t interval;
  uint32_t batch;
  int raw_start;
  FAR const char *file_path;
  sem_t wakeup;
  volatile bool running;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fakesensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable);
static int fakesensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us);
static int fakesensor_batch(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR uint32_t *latency_us);
static int fakegnss_activate(FAR struct gnss_lowerhalf_s *lower,
                             FAR struct file *filep, bool sw);
static int fakegnss_set_interval(FAR struct gnss_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us);
static void fakesensor_push_event(FAR struct fakesensor_s *sensor,
                                  uint64_t event_timestamp);
static int fakesensor_thread(int argc, char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_ops_s g_fakesensor_ops =
{
  .activate = fakesensor_activate,
  .set_interval = fakesensor_set_interval,
  .batch = fakesensor_batch,
};

static struct gnss_ops_s g_fakegnss_ops =
{
  .activate = fakegnss_activate,
  .set_interval = fakegnss_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int fakesensor_read_csv_line(FAR struct file *file,
                                    FAR char *buffer, int len, int start)
{
  int i;

  len = file_read(file, buffer, len);
  if (len == 0)
    {
      /* Loop reads */

      file_seek(file, start, SEEK_SET);
      len = file_read(file, buffer, len);
    }

  for (i = 0; i < len; i++)
    {
      if (buffer[i] == '\n')
        {
          file_seek(file, i - len + 1, SEEK_CUR);
          buffer[i + 1] = '\0';
          break;
        }
    }

  return i + 1;
}

static int fakesensor_read_csv_header(FAR struct fakesensor_s *sensor)
{
  char buffer[40];

  /* Set interval */

  sensor->raw_start =
      fakesensor_read_csv_line(&sensor->data, buffer, sizeof(buffer), 0);
  if (sensor->interval == 0)
    {
      sscanf(buffer, "interval:%"PRIu32"\n", &sensor->interval);
      sensor->interval *= 1000;
    }

  /*  Skip the CSV header */

  sensor->raw_start +=
      fakesensor_read_csv_line(&sensor->data, buffer, sizeof(buffer), 0);
  return OK;
}

static inline void fakesensor_read_accel(FAR struct fakesensor_s *sensor,
                                         uint64_t event_timestamp)
{
  struct sensor_accel accel;
  char raw[50];
  fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
  sscanf(raw, "%f,%f,%f\n", &accel.x, &accel.y, &accel.z);
  accel.temperature = NAN;
  accel.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &accel,
                    sizeof(struct sensor_accel));
}

static inline void fakesensor_read_mag(FAR struct fakesensor_s *sensor,
                                       uint64_t event_timestamp)
{
  struct sensor_mag mag;
  char raw[50];
  fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
  sscanf(raw, "%f,%f,%f\n", &mag.x, &mag.y, &mag.z);
  mag.temperature = NAN;
  mag.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &mag,
                           sizeof(struct sensor_mag));
}

static inline void fakesensor_read_gyro(FAR struct fakesensor_s *sensor,
                                        uint64_t event_timestamp)
{
  struct sensor_gyro gyro;
  char raw[50];
  fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
  sscanf(raw, "%f,%f,%f\n", &gyro.x, &gyro.y, &gyro.z);
  gyro.temperature = NAN;
  gyro.timestamp = event_timestamp;
  sensor->lower.push_event(sensor->lower.priv, &gyro,
                    sizeof(struct sensor_gyro));
}

static inline void fakesensor_read_gnss(FAR struct fakesensor_s *sensor)
{
  char raw[150];

  while (1)
    {
      fakesensor_read_csv_line(&sensor->data, raw,
                               sizeof(raw), sensor->raw_start);
      sensor->gnss.push_data(sensor->gnss.priv, raw,
                             strlen(raw), true);
      if (strstr(raw, "GGA") != NULL)
        {
          break;
        }
    }
}

static int fakesensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, bool enable)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);
  if (enable)
    {
      sensor->running = true;

      /* Wake up the thread */

      nxsem_post(&sensor->wakeup);
    }
  else
    {
      sensor->running = false;
    }

  return OK;
}

static int fakegnss_activate(FAR struct gnss_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  return fakesensor_activate((FAR void *)lower, filep, enable);
}

static int fakesensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   FAR uint32_t *period_us)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);
  sensor->interval = *period_us;
  return OK;
}

static int fakegnss_set_interval(FAR struct gnss_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  return fakesensor_set_interval((FAR void *)lower, filep, period_us);
}

static int fakesensor_batch(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR uint32_t *latency_us)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);
  uint32_t max_latency = sensor->lower.nbuffer * sensor->interval;
  if (*latency_us > max_latency)
    {
      *latency_us = max_latency;
    }
  else if (*latency_us < sensor->interval && *latency_us > 0)
    {
      *latency_us = sensor->interval;
    }

  sensor->batch = *latency_us;
  return OK;
}

void fakesensor_push_event(FAR struct fakesensor_s *sensor,
                           uint64_t event_timestamp)
{
  switch (sensor->type)
  {
    case SENSOR_TYPE_ACCELEROMETER:
      fakesensor_read_accel(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_MAGNETIC_FIELD:
      fakesensor_read_mag(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_GYROSCOPE:
      fakesensor_read_gyro(sensor, event_timestamp);
      break;

    case SENSOR_TYPE_GNSS:
    case SENSOR_TYPE_GNSS_SATELLITE:
      fakesensor_read_gnss(sensor);
      break;

    default:
      snerr("fakesensor: unsupported type sensor type\n");
      break;
  }
}

static int fakesensor_thread(int argc, char** argv)
{
  FAR struct fakesensor_s *sensor = (FAR struct fakesensor_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 16));
  int ret;

  while (true)
    {
      /* Waiting to be woken up */

      nxsem_wait_uninterruptible(&sensor->wakeup);

      /* Open csv file and init file handle */

      ret = file_open(&sensor->data, sensor->file_path,
                      O_RDONLY | O_CLOEXEC);
      if (ret < 0)
        {
          snerr("Failed to open file:%s, err:%d", sensor->file_path, ret);
          return ret;
        }

      fakesensor_read_csv_header(sensor);

      while (sensor->running)
        {
          /* Sleeping thread for interval */

          nxsig_usleep(sensor->batch ? sensor->batch : sensor->interval);

          /* Notify upper */

          if (sensor->batch)
            {
              uint32_t batch_num = sensor->batch / sensor->interval;
              uint64_t event_timestamp =
                  sensor_get_timestamp() - sensor->interval * batch_num;
              int i;

              for (i = 0; i < batch_num; i++)
                {
                  fakesensor_push_event(sensor, event_timestamp);
                  event_timestamp += sensor->interval;
                }
            }
          else
            {
              fakesensor_push_event(sensor, sensor_get_timestamp());
            }
        }

      /* Close csv file handle when running change true to false */

      ret = file_close(&sensor->data);
      if (ret < 0)
        {
          snerr("Failed to close file:%s, err:%d", sensor->file_path, ret);
          return ret;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fakesensor_init
 *
 * Description:
 *   This function generates a sensor node under /dev/uorb/. And then
 *   report the data from csv file.
 *
 * Input Parameters:
 *   type        - The type of sensor and defined in <nuttx/sensors/sensor.h>
 *   file_name   - The name of csv name and the file structure is as follows:
 *                    First row : set interval, unit millisecond
 *                    Second row: csv file header
 *                    third row : data
 *                    (Each line should not exceed 50 characters)
 *                    For example:
 *                    interval:12
 *                    x,y,z
 *                    2.1234,3.23443,2.23456
 *                    ...
 *   devno       - The user specifies which device of this type, from 0.
 *   batch_number- The maximum number of batch
 ****************************************************************************/

int fakesensor_init(int type, FAR const char *file_name,
                    int devno, uint32_t batch_number)
{
  FAR struct fakesensor_s *sensor;
  FAR char *argv[2];
  char arg1[32];
  int ret;

  /* Alloc memory for sensor */

  sensor = kmm_zalloc(sizeof(struct fakesensor_s));
  if (!sensor)
    {
      snerr("Memory cannot be allocated for fakesensor\n");
      return -ENOMEM;
    }

  sensor->file_path = file_name;
  sensor->type = type;

  nxsem_init(&sensor->wakeup, 0, 0);

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", sensor);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("fakesensor_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       fakesensor_thread, argv);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ERROR;
    }

  /*  Register sensor */

  if (type == SENSOR_TYPE_GNSS || type == SENSOR_TYPE_GNSS_SATELLITE)
    {
      sensor->gnss.ops = &g_fakegnss_ops;
      gnss_register(&sensor->gnss, devno, batch_number);
    }
  else
    {
      sensor->lower.type = type;
      sensor->lower.ops = &g_fakesensor_ops;
      sensor->lower.nbuffer = batch_number;
      sensor_register(&sensor->lower, devno);
    }

  return OK;
}

