/****************************************************************************
 * drivers/sensors/wtgahrs2.c
 * Driver for the Wit-Motion WTGAHRS2 accelerometer, gyroscope, magnetic,
 * angle, barometer, temperature, gps sensors by serial interface with host
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

#include <nuttx/sensors/wtgahrs2.h>
#include <nuttx/kthread.h>
#include <nuttx/kmalloc.h>

#include <sys/param.h>

#include <termios.h>
#include <math.h>
#include <fcntl.h>
#include <stdio.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WTGAHRS2_ACCEL_IDX         0
#define WTGAHRS2_GYRO_IDX          1
#define WTGAHRS2_MAG_IDX           2
#define WTGAHRS2_BARO_IDX          3
#define WTGAHRS2_GPS_IDX           4
#define WTGAHRS2_MAX_IDX           5

#define WTGAHRS2_GPS0_MASK         (1 << 0) /* Time */
#define WTGAHRS2_GPS1_MASK         (1 << 1) /* Longitude, Latitude */
#define WTGAHRS2_GPS2_MASK         (1 << 2) /* Ground speed, Height, Yaw */
#define WTGAHRS2_GPS_MASK          (7 << 0)

#define WTGAHRS2_GPS0_INFO         0x50
#define WTGAHRS2_ACCEL_INFO        0x51
#define WTGAHRS2_GYRO_INFO         0x52
#define WTGAHRS2_MAG_INFO          0x54
#define WTGAHRS2_BARO_INFO         0x56
#define WTGAHRS2_GPS1_INFO         0x57
#define WTGAHRS2_GPS2_INFO         0x58

#define WTGAHRS2_RSP_HEADER        0x55
#define WTGAHRS2_RSP_LENGTH        11
#define WTGAHRS2_CMD_LENGTH        5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wtgahrs2_sensor_s
{
  struct sensor_lowerhalf_s lower;
  unsigned long             interval;
  uint64_t                  last_update;
  bool                      enable;
};

struct wtgahrs2_dev_s
{
  struct wtgahrs2_sensor_s  dev[WTGAHRS2_MAX_IDX];
  struct file               file;

  struct sensor_gps   gps;
  unsigned char             gps_mask;
};

/****************************************************************************
 * Private
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int wtgahrs2_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool sw);
static int wtgahrs2_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *interval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* in microseconds */

static const unsigned long g_wtgahrs2_interval[] =
{
  10000000,  /* 0.1 hz */
  2000000,   /* 0.5 hz */
  1000000,   /* 1   hz */
  500000,    /* 2   hz */
  200000,    /* 5   hz */
  100000,    /* 10  hz */
  50000,     /* 20  hz */
  20000,     /* 50  hz */
  10000,     /* 100 hz */
  5000,      /* 200 hz */
};

static const uint8_t g_wtgahrs2_unlock[] =
{
  0xff, 0xaa, 0x69, 0x88, 0xb5
};

static const uint8_t g_wtgahrs2_odr_200hz[] =
{
  0xff, 0xaa, 0x03, 0x0b, 0x00
};

static const uint8_t g_wtgahrs2_enable_sensor[] =
{
  0xff, 0xaa, 0x02, 0xd7, 0x05
};

static const struct sensor_ops_s g_wtgahrs2_ops =
{
  .activate           = wtgahrs2_activate,
  .set_interval       = wtgahrs2_set_interval,
  .batch              = NULL,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void wtgahrs2_sendcmd(FAR struct wtgahrs2_dev_s *rtdata,
                             const void *cmd)
{
  file_write(&rtdata->file, cmd, WTGAHRS2_CMD_LENGTH);
  usleep(10000);
}

static int wtgahrs2_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool sw)
{
  FAR struct wtgahrs2_sensor_s *dev = (FAR struct wtgahrs2_sensor_s *)lower;
  dev->enable = sw;

  return 0;
}

static int wtgahrs2_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR unsigned long *interval)
{
  FAR struct wtgahrs2_sensor_s *dev = (FAR struct wtgahrs2_sensor_s *)lower;
  int idx = 0;

  for (; idx < nitems(g_wtgahrs2_interval) - 1; idx++)
    {
      if (*interval >= g_wtgahrs2_interval[idx])
        {
          break;
        }
    }

  *interval = g_wtgahrs2_interval[idx];
  dev->interval = *interval;

  return 0;
}

static void wtgahrs2_accel_data(FAR struct wtgahrs2_dev_s *rtdata,
                                FAR unsigned char *buffer)
{
  FAR struct wtgahrs2_sensor_s *dev = &rtdata->dev[WTGAHRS2_ACCEL_IDX];
  FAR struct sensor_lowerhalf_s *lower = &dev->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_accel accel;

  if (!dev->enable || now - dev->last_update < dev->interval)
    {
      return;
    }

  dev->last_update = now;

  accel.timestamp = now;
  accel.x = (short)(buffer[1] << 8 | buffer[0]) * (16 * 9.8f / 32768);
  accel.y = (short)(buffer[3] << 8 | buffer[2]) * (16 * 9.8f / 32768);
  accel.z = (short)(buffer[5] << 8 | buffer[4]) * (16 * 9.8f / 32768);
  accel.temperature = (short)(buffer[7] << 8 | buffer[6]) / 100.0f;

  lower->push_event(lower->priv, &accel, sizeof(accel));
  sninfo("Accel: %.3fm/s^2 %.3fm/s^2 %.3fm/s^2, t:%.1f\n",
         accel.x, accel.y, accel.z, accel.temperature);
}

static void wtgahrs2_gyro_data(FAR struct wtgahrs2_dev_s *rtdata,
                               FAR unsigned char *buffer)
{
  FAR struct wtgahrs2_sensor_s *dev = &rtdata->dev[WTGAHRS2_GYRO_IDX];
  FAR struct sensor_lowerhalf_s *lower = &dev->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_gyro gyro;

  if (!dev->enable || now - dev->last_update < dev->interval)
    {
      return;
    }

  dev->last_update = now;

  gyro.timestamp = now;
  gyro.x = (short)(buffer[1] << 8 | buffer[0]) * (2000 * M_PI / 180 / 32768);
  gyro.y = (short)(buffer[3] << 8 | buffer[2]) * (2000 * M_PI / 180 / 32768);
  gyro.z = (short)(buffer[5] << 8 | buffer[4]) * (2000 * M_PI / 180 / 32768);
  gyro.temperature = (short)(buffer[7] << 8 | buffer[6]) / 100.0f;

  lower->push_event(lower->priv, &gyro, sizeof(gyro));
  sninfo("Gyro: %.3frad/s %.3frad/s %.3frad/s, t:%.1f\n",
          gyro.x, gyro.y, gyro.z, gyro.temperature);
}

static void wtgahrs2_mag_data(FAR struct wtgahrs2_dev_s *rtdata,
                              FAR unsigned char *buffer)
{
  FAR struct wtgahrs2_sensor_s *dev = &rtdata->dev[WTGAHRS2_MAG_IDX];
  FAR struct sensor_lowerhalf_s *lower = &dev->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_mag mag;

  if (!dev->enable || now - dev->last_update < dev->interval)
    {
      return;
    }

  dev->last_update = now;

  mag.timestamp = now;
  mag.x = (short)(buffer[1] << 8 | buffer[0]) * 0.16f;
  mag.y = (short)(buffer[3] << 8 | buffer[2]) * 0.16f;
  mag.z = (short)(buffer[5] << 8 | buffer[4]) * 0.16f;
  mag.temperature = (short)(buffer[7] << 8 | buffer[6]) / 100.0f;

  lower->push_event(lower->priv, &mag, sizeof(mag));
  sninfo("Mag: %.3fuT %.3fuT %.3fuT, t:%.1f\n",
         mag.x, mag.y, mag.z, mag.temperature);
}

static void wtgahrs2_baro_data(FAR struct wtgahrs2_dev_s *rtdata,
                               FAR unsigned char *buffer)
{
  FAR struct wtgahrs2_sensor_s *dev = &rtdata->dev[WTGAHRS2_BARO_IDX];
  FAR struct sensor_lowerhalf_s *lower = &dev->lower;
  uint64_t now = sensor_get_timestamp();
  struct sensor_baro baro;

  if (!dev->enable || now - dev->last_update < dev->interval)
    {
      return;
    }

  dev->last_update = now;

  baro.timestamp = now;
  baro.pressure = (long)(buffer[3] << 24 | buffer[2] << 16 |
                  buffer[1] << 8 | buffer[0]) / 100.0f;
  baro.temperature = NAN;

  lower->push_event(lower->priv, &baro, sizeof(baro));
  sninfo("Pressure : %.3fhPa\n", baro.pressure);
}

static void wtgahrs2_gps_data(FAR struct wtgahrs2_dev_s *rtdata,
                              FAR unsigned char *buffer, int info_type)
{
  FAR struct wtgahrs2_sensor_s *dev = &rtdata->dev[WTGAHRS2_GPS_IDX];
  FAR struct sensor_lowerhalf_s *lower = &dev->lower;
  uint64_t now = sensor_get_timestamp();

  if (!dev->enable || now - dev->last_update < dev->interval)
    {
      return;
    }

  if (rtdata->gps_mask == 0)
    {
      dev->last_update = now;
    }

  switch (info_type)
    {
      case WTGAHRS2_GPS0_INFO:
        rtdata->gps_mask |= WTGAHRS2_GPS0_MASK;
        break;

      case WTGAHRS2_GPS1_INFO:
        rtdata->gps_mask |= WTGAHRS2_GPS1_MASK;
        rtdata->gps.longitude = (buffer[3] << 8
                                | buffer[2] << 8
                                | buffer[1] << 8
                                | buffer[0]) / 10000000.0f;
        rtdata->gps.latitude = (buffer[7] << 8
                               | buffer[6] << 8
                               | buffer[5] << 8
                               | buffer[4]) / 10000000.0f;
        break;

      case WTGAHRS2_GPS2_INFO:
        rtdata->gps_mask |= WTGAHRS2_GPS2_MASK;
        rtdata->gps.altitude = (float)(buffer[1] << 8 | buffer[0]) / 10.0f;
        rtdata->gps.ground_speed = (float)(buffer[7] << 8 | buffer[6] << 8
                    | buffer[5] << 8 | buffer[4]) / 3600.0f;
        break;
    }

  if (rtdata->gps_mask == WTGAHRS2_GPS_MASK)
    {
      rtdata->gps_mask = 0;
      lower->push_event(lower->priv, &rtdata->gps, sizeof(rtdata->gps));
      sninfo("Time : %" PRIu64 " utc_time: %" PRIu64 "\n",
             rtdata->gps.timestamp, rtdata->gps.time_utc);
      sninfo("GPS longitude : %fdegree, latitude:%fdegree\n",
              rtdata->gps.longitude, rtdata->gps.latitude);
      sninfo("GPS speed: %fm/s, altitude: %fm\n",
              rtdata->gps.ground_speed, rtdata->gps.altitude);
    }
}

static bool wtgahrs2_process_data(FAR struct wtgahrs2_dev_s *rtdata,
                                  FAR unsigned char *buffer)
{
  unsigned char sum = 0;
  int i;

  /* calculate sum and verify checksum */

  for (i = 0; i < WTGAHRS2_RSP_LENGTH - 1; i++)
    {
      sum += buffer[i];
    }

  if (sum != buffer[WTGAHRS2_RSP_LENGTH - 1])
    {
      return false;
    }

  switch (buffer[1])
    {
      case WTGAHRS2_ACCEL_INFO:
        wtgahrs2_accel_data(rtdata, &buffer[2]);
        break;

      case WTGAHRS2_GYRO_INFO:
        wtgahrs2_gyro_data(rtdata, &buffer[2]);
        break;

      case WTGAHRS2_MAG_INFO:
        wtgahrs2_mag_data(rtdata, &buffer[2]);
        break;

      case WTGAHRS2_BARO_INFO:
        wtgahrs2_baro_data(rtdata, &buffer[2]);
        break;

      case WTGAHRS2_GPS0_INFO:
      case WTGAHRS2_GPS1_INFO:
      case WTGAHRS2_GPS2_INFO:
        wtgahrs2_gps_data(rtdata, &buffer[2], buffer[1]);
        break;
    }

  return true;
}

static int wtgahrs2_thread(int argc, FAR char *argv[])
{
  FAR struct wtgahrs2_dev_s *rtdata = (FAR struct wtgahrs2_dev_s *)
                                      ((uintptr_t)strtoul(argv[1], NULL, 0));
  unsigned char buffer[8 * WTGAHRS2_RSP_LENGTH];
  ssize_t count = 0;
  ssize_t pos;

  while (1)
    {
      count += file_read(&rtdata->file, buffer + count,
                         sizeof(buffer) - count);
      for (pos = 0; pos < count; pos++)
        {
          if (buffer[pos] != WTGAHRS2_RSP_HEADER)
            {
              continue;
            }

          if (count - pos < WTGAHRS2_RSP_LENGTH)
            {
              memmove(buffer, buffer + pos, count - pos);
              break;
            }

          if (wtgahrs2_process_data(rtdata, &buffer[pos]))
            {
              pos += WTGAHRS2_RSP_LENGTH - 1;
            }
        }

        count -= pos;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int wtgahrs2_initialize(FAR const char *path, int devno)
{
  FAR struct wtgahrs2_dev_s *rtdata;
  FAR struct wtgahrs2_sensor_s *tmp;
#ifdef CONFIG_SERIAL_TERMIOS
  struct termios opt;
#endif
  FAR char *argv[2];
  char arg1[16];
  int ret;

  if (!path)
    {
      snerr("Invalid path for serial interface\n");
      return -EINVAL;
    }

  rtdata = kmm_zalloc(sizeof(struct wtgahrs2_dev_s));
  if (!rtdata)
    {
      snerr("Memory cannot be allocated for wtgahrs2\n");
      return -ENOMEM;
    }

  /* Open serial tty port and set baud rate */

  ret = file_open(&rtdata->file, path, O_RDWR);
  if (ret < 0)
    {
      snerr("Failed to open wtgahrs2 serial:%s, err:%d", path, ret);
      goto open_err;
    }

#ifdef CONFIG_SERIAL_TERMIOS
  file_ioctl(&rtdata->file, TCGETS, &opt);
  cfmakeraw(&opt);
  cfsetispeed(&opt, B115200);
  cfsetospeed(&opt, B115200);
  file_ioctl(&rtdata->file, TCSETS, &opt);
#endif

  /* Accelerometer register */

  tmp = &rtdata->dev[WTGAHRS2_ACCEL_IDX];
  tmp->lower.ops = &g_wtgahrs2_ops;
  tmp->lower.type = SENSOR_TYPE_ACCELEROMETER;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto accel_err;
    }

  /* Gyroscope register */

  tmp = &rtdata->dev[WTGAHRS2_GYRO_IDX];
  tmp->lower.ops = &g_wtgahrs2_ops;
  tmp->lower.type = SENSOR_TYPE_GYROSCOPE;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto gyro_err;
    }

  /* Magnetic register */

  tmp = &rtdata->dev[WTGAHRS2_MAG_IDX];
  tmp->lower.ops = &g_wtgahrs2_ops;
  tmp->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto mag_err;
    }

  /* Barometer register */

  tmp = &rtdata->dev[WTGAHRS2_BARO_IDX];
  tmp->lower.ops = &g_wtgahrs2_ops;
  tmp->lower.type = SENSOR_TYPE_BAROMETER;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto baro_err;
    }

  /* GPS register */

  tmp = &rtdata->dev[WTGAHRS2_GPS_IDX];
  tmp->lower.ops = &g_wtgahrs2_ops;
  tmp->lower.type = SENSOR_TYPE_GPS;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto gps_err;
    }

  /* Set sensor default attributes and enter unlock mode */

  wtgahrs2_sendcmd(rtdata, g_wtgahrs2_unlock);

  /* Set sensor default odr 200hz */

  wtgahrs2_sendcmd(rtdata, g_wtgahrs2_odr_200hz);

  /* Enable all sensor */

  wtgahrs2_sendcmd(rtdata, g_wtgahrs2_enable_sensor);

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)rtdata);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("wtgahrs2_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_DEFAULT_TASK_STACKSIZE,
                       wtgahrs2_thread, argv);
  if (ret < 0)
    {
      goto thr_err;
    }

  return ret;

thr_err:
  sensor_unregister(&rtdata->dev[WTGAHRS2_GPS_IDX].lower, devno);
gps_err:
  sensor_unregister(&rtdata->dev[WTGAHRS2_BARO_IDX].lower, devno);
baro_err:
  sensor_unregister(&rtdata->dev[WTGAHRS2_MAG_IDX].lower, devno);
mag_err:
  sensor_unregister(&rtdata->dev[WTGAHRS2_GYRO_IDX].lower, devno);
gyro_err:
  sensor_unregister(&rtdata->dev[WTGAHRS2_ACCEL_IDX].lower, devno);
accel_err:
  file_close(&rtdata->file);
open_err:
  kmm_free(rtdata);
  return ret;
}
