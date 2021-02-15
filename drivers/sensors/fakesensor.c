/****************************************************************************
 * drivers/sensors/fakesensor.c
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

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fakesensor_s
{
  struct sensor_lowerhalf_s lower;
  struct file data;
  unsigned int interval;
  int raw_start;
  FAR const char *file_path;
  sem_t run;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int fakesensor_activate(FAR struct sensor_lowerhalf_s *lower,
                               bool sw);
static int fakesensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR unsigned int *period_us);
static int fakesensor_fetch(FAR struct sensor_lowerhalf_s *lower,
                            FAR char *buffer, size_t buflen);
static int fakesensor_thread(int argc, char** argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sensor_ops_s g_fakesensor_ops =
{
  .activate = fakesensor_activate,
  .set_interval = fakesensor_set_interval,
  .fetch = fakesensor_fetch,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int fakesensor_read_csv_line(FAR struct file *file,
                                    char *buffer, int len, int start)
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
          break;
        }
    }

  return i + 1;
}

static int fakesensor_read_csv_header(struct fakesensor_s *sensor)
{
  char buffer[40];

  /* Set interval */

  sensor->raw_start =
      fakesensor_read_csv_line(&sensor->data, buffer, sizeof(buffer), 0);
  sscanf(buffer, "interval:%d\n", &sensor->interval);
  sensor->interval *= 1000;

  /*  Skip the CSV header */

  sensor->raw_start +=
      fakesensor_read_csv_line(&sensor->data, buffer, sizeof(buffer), 0);
  return OK;
}

static int fakesensor_activate(FAR struct sensor_lowerhalf_s *lower, bool sw)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);
  int ret;

  if (sw)
    {
      ret = file_open(&sensor->data, sensor->file_path, O_RDONLY);
      if (ret < 0)
        {
          snerr("Failed to open file:%s, err:%d", sensor->file_path, ret);
          return ret;
        }

      fakesensor_read_csv_header(sensor);

      /* Wake up the thread */

      nxsem_post(&sensor->run);
    }
  else
    {
      ret = file_close(&sensor->data);
      if (ret < 0)
        {
          snerr("Failed to close file:%s, err:%d", sensor->file_path, ret);
          return ret;
        }
    }

  return OK;
}

static int fakesensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                   FAR unsigned int *period_us)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);
  sensor->interval = *period_us;
  return OK;
}

static int fakesensor_fetch(FAR struct sensor_lowerhalf_s *lower,
                            FAR char *buffer, size_t buflen)
{
  FAR struct fakesensor_s *sensor = container_of(lower,
                                                 struct fakesensor_s, lower);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      struct sensor_event_accel accel;
      char raw[50];
      fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
      sscanf(raw, "%f,%f,%f\n", &accel.x, &accel.y, &accel.z);
      accel.temperature = NAN;
      accel.timestamp = sensor_get_timestamp();
      memcpy(buffer, &accel, buflen);
      return buflen;
    }

  if (lower->type == SENSOR_TYPE_MAGNETIC_FIELD)
    {
      struct sensor_event_accel mag;
      char raw[50];
      fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
      sscanf(raw, "%f,%f,%f\n", &mag.x, &mag.y, &mag.z);
      mag.temperature = NAN;
      mag.timestamp = sensor_get_timestamp();
      memcpy(buffer, &mag, buflen);
      return buflen;
    }

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      struct sensor_event_accel gyro;
      char raw[50];
      fakesensor_read_csv_line(
          &sensor->data, raw, sizeof(raw), sensor->raw_start);
      sscanf(raw, "%f,%f,%f\n", &gyro.x, &gyro.y, &gyro.z);
      gyro.temperature = NAN;
      gyro.timestamp = sensor_get_timestamp();
      memcpy(buffer, &gyro, buflen);
      return buflen;
    }

  return -ENOTSUP;
}

static int fakesensor_thread(int argc, char** argv)
{
  FAR struct fakesensor_s *sensor = (FAR struct fakesensor_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 0));

  while (true)
    {
      if (sensor->data.f_inode != NULL)
        {
          /* Notify upper */

          sensor->lower.notify_event(sensor->lower.priv);

          /* Sleeping thread for interval */

          usleep(sensor->interval);
        }
      else
        {
          /* Waiting to be woken up */

          nxsem_wait(&sensor->run);
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
 *   This function generates a sensor node under /dev/sensor/. And then
 *   report the data from csv file.
 *
 * Input Parameters:
 *   type      - The type of sensor and Defined in <nuttx/sensors/sensor.h>
 *   file_name - The name of csv name and the file structure is as follows:
 *               First row : set interval, unit millisecond
 *               Second row: csv file header
 *               third row : data
 *               (Each line should not exceed 50 characters)
 *               For example:
 *               interval:12
 *               x,y,z
 *               2.1234,3.23443,2.23456
 *               ...
 *   devno     - The user specifies which device of this type, from 0.
 ****************************************************************************/

int fakesensor_init(int type, FAR const char *file_name, int devno)
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

  sensor->lower.type = type;
  sensor->lower.ops = &g_fakesensor_ops;
  sensor->interval = 1;
  sensor->file_path = file_name;

  nxsem_init(&sensor->run, 0, 0);
  nxsem_set_protocol(&sensor->run, SEM_PRIO_NONE);

  /* Create thread for sensor */

  snprintf(arg1, 32, "%p", sensor);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("fakesensor_thread", SCHED_PRIORITY_DEFAULT,
                    CONFIG_DEFAULT_TASK_STACKSIZE, fakesensor_thread, argv);
  if (ret < 0)
    {
      kmm_free(sensor);
      return ERROR;
    }

  /*  Register sensor */

  sensor_register(&sensor->lower, devno);

  return OK;
}
