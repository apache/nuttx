/****************************************************************************
 * drivers/sensors/sht4x_uorb.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
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
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/sht4x.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SHT4X_I2C_FREQUENCY
#define CONFIG_SHT4X_I2C_FREQUENCY 400000
#endif

#define SHT4X_CRC_INIT 0xFF /* Initial value of the calculated CRC. */
#define SHT4X_CRC_POLY 0x31 /* CRC calculation polynomial. */

#define SHT4X_SOFT_RESET 0x94     /* Soft reset command. */
#define SHT4X_READ_SERIAL 0x89    /* Read serial number command. */
#define SHT4X_READ_LOW_PREC 0xE0  /* Low precision read temp & humidity. */
#define SHT4X_READ_MED_PREC 0xF6  /* Med precision read temp & humidity. */
#define SHT4X_READ_HIGH_PREC 0xFD /* High precision read temp & humidity. */
#define SHT4X_HEAT_200_1 0x39     /* Activate heater with 200mW for 1s. */
#define SHT4X_HEAT_200_P1 0x32    /* Activate heater with 200mW for 0.1s. */
#define SHT4X_HEAT_110_1 0x2F     /* Activate heater with 110mW for 1s. */
#define SHT4X_HEAT_110_P1 0x24    /* Activate heater with 110mW for 0.1s. */
#define SHT4X_HEAT_20_1 0x1E      /* Activate heater with 20mW for 1s. */
#define SHT4X_HEAT_20_P1 0x15     /* Activate heater with 20mW for 0.1s. */

/****************************************************************************
 * Private
 ****************************************************************************/

/* Sensor information for the lowerhalf sensors.
 * Since the SHT4X has both a relative humidity and temperature sensor,
 * two lower halves are needed which will follow this structure.
 */

struct sht4x_sensor_s
{
  FAR struct sensor_lowerhalf_s sensor_lower; /* Lower-half driver */
  bool enabled;                               /* If this sensor is enabled */
  FAR struct sht4x_dev_s *dev;                /* Backward reference to
                                               * device */
};

/* Represents the main device, with two lower halves for both data types. */

struct sht4x_dev_s
{
  struct sht4x_sensor_s hum;        /* Humidity lower-half */
  struct sht4x_sensor_s temp;       /* Temperature lower-half */
  FAR struct i2c_master_s *i2c;     /* I2C interface. */
  uint8_t addr;                     /* I2C address. */
  struct timespec last_heat;        /* Last time heater was active. */
  enum sht4x_precision_e precision; /* The precision for read operations. */
  sem_t run;                        /* Lock for the polling measurement
                                     * cycle */
  mutex_t devlock;                  /* Exclusive device access */
  uint32_t interval;                /* Measurement interval for polling in
                                     * us, shared by both halves. */
};

/* Easy unpacking of serial number from I2C packet. */

union sht4x_serialno_t
{
  uint32_t full; /* Full serial number. */
  struct
  {
    uint16_t msb; /* Most significant 2 bytes. */
    uint16_t lsb; /* Least significant 2 bytes. */
  } halves;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Commands for reading at different precisions */

static const uint8_t g_precision_read[] =
{
  [SHT4X_PREC_LOW] = SHT4X_READ_LOW_PREC,
  [SHT4X_PREC_MED] = SHT4X_READ_MED_PREC,
  [SHT4X_PREC_HIGH] = SHT4X_READ_HIGH_PREC,
};

#ifdef CONFIG_SHT4X_CRC_LOOKUP

/* CRC lookup table. */

static const uint8_t g_crc_lookup[] =
    {
        0x0,  0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb,
        0xea, 0x7d, 0x4c, 0x1f, 0x2e, 0x43, 0x72, 0x21, 0x10, 0x87, 0xb6,
        0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0xf,  0x5c, 0x6d, 0x86,
        0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0xe,  0x5d, 0x6c,
        0xfb, 0xca, 0x99, 0xa8, 0xc5, 0xf4, 0xa7, 0x96, 0x1,  0x30, 0x63,
        0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 0x3d, 0xc,
        0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40,
        0x71, 0x22, 0x13, 0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9,
        0xc7, 0xf6, 0xa5, 0x94, 0x3,  0x32, 0x61, 0x50, 0xbb, 0x8a, 0xd9,
        0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x2,  0x33, 0x60, 0x51, 0xc6, 0xf7,
        0xa4, 0x95, 0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0xd,  0x5e, 0x6f, 0x41,
        0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6, 0x7a, 0x4b, 0x18, 0x29,
        0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x7,  0x36, 0x65,
        0x54, 0x39, 0x8,  0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1,
        0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17, 0xfc, 0xcd, 0x9e, 0xaf, 0x38,
        0x9,  0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2,
        0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x6,  0x37, 0x64,
        0x55, 0xc2, 0xf3, 0xa0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xb2,
        0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0xb,  0x58, 0x69, 0x4,
        0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee,
        0x79, 0x48, 0x1b, 0x2a, 0xc1, 0xf0, 0xa3, 0x92, 0x5,  0x34, 0x67,
        0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 0x82, 0xb3,
        0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0xa,  0x59, 0x68, 0xff,
        0xce, 0x9d, 0xac,
}
#endif

/* Measurement times for the various precisions, in microseconds. */

static const uint16_t g_measurement_times[] =
{
  [SHT4X_PREC_LOW] = 1600,
  [SHT4X_PREC_MED] = 4500,
  [SHT4X_PREC_HIGH] = 8300,
};

/* Commands for the various heating options. */

static const uint8_t g_heat_cmds[] =
{
  [SHT4X_HEATER_200MW_1] = SHT4X_HEAT_200_1,
  [SHT4X_HEATER_200MW_01] = SHT4X_HEAT_200_P1,
  [SHT4X_HEATER_110MW_1] = SHT4X_HEAT_110_1,
  [SHT4X_HEATER_110MW_01] = SHT4X_HEAT_110_P1,
  [SHT4X_HEATER_20MW_1] = SHT4X_HEAT_20_1,
  [SHT4X_HEATER_20MW_01] = SHT4X_HEAT_20_P1,
};

/* Timeouts for the various heating options in microseconds. */

static const uint32_t g_heat_times[] =
{
  [SHT4X_HEATER_200MW_1] = 1000000, [SHT4X_HEATER_200MW_01] = 100000,
  [SHT4X_HEATER_110MW_1] = 1000000, [SHT4X_HEATER_110MW_01] = 100000,
  [SHT4X_HEATER_20MW_1] = 1000000,  [SHT4X_HEATER_20MW_01] = 100000,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sht4x_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us);
static int sht4x_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable);
static int sht4x_get_info(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR struct sensor_device_info_s *info);
static int sht4x_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_SENSORS_SHT4X_POLL
static int sht4x_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
    .activate = sht4x_activate,
#ifdef CONFIG_SENSORS_SHT4X_POLL
    .fetch = NULL,
#else
    .fetch = sht4x_fetch,
#endif
    .set_interval = sht4x_set_interval,
    .get_info = sht4x_get_info,
    .control = sht4x_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SHT4X_CRC_LOOKUP

/****************************************************************************
 * Name: sht4x_crc_lookup
 *
 * Description:
 *   Perform a CRC calculation using the lookup table.
 *
 ****************************************************************************/

uint8_t sht4x_crc_lookup(const uint8_t *buf, uint8_t nbytes)
{
  uint8_t crc = SHT4X_CRC_INIT;
  for (uint8_t byte = 0; byte < nbytes; byte++)
    {
      crc = g_crc_lookup[crc ^ buf[byte]];
    }

  return crc;
}

#endif

/****************************************************************************
 * Name: sht4x_crc_bitwise
 *
 * Description:
 *   Perform a CRC calculation using the bitwise calculation.
 *
 ****************************************************************************/

uint8_t sht4x_crc_bitwise(const uint8_t *buf, uint8_t nbytes)
{
  uint8_t crc = SHT4X_CRC_INIT;
  for (uint8_t byte = 0; byte < nbytes; byte++)
    {
      crc ^= buf[byte];
      for (uint8_t bit = 0; bit < 8; bit++)
        {
          if (crc & 0x80)
            {
              /* Discard the highest bit (implicit XOR), then divide by the
               * polynomial.
               */

              crc <<= 1;
              crc ^= SHT4X_CRC_POLY;
            }
          else
            {
              /* Continue until the highest bit is set. */

              crc <<= 1;
            }
        }
    }

  return crc;
}

/****************************************************************************
 * Name: sht4x_crc
 *
 * Description:
 *   Perform a CRC calculation on the data to check for validity.
 *   The `buf` param must be a data buffer that ends with its own 8 bit CRC.
 *   The `nbytes` param is the length of the data including the 8 bit CRC.
 *
 ****************************************************************************/

static int sht4x_crc(const uint8_t *buf, uint8_t nbytes)
{
#ifdef CONFIG_SHT4X_CRC_LOOKUP
  if (sht4x_crc_lookup(buf, nbytes) != 0)
    {
      return -EBADMSG;
    }
#else
  if (sht4x_crc_bitwise(buf, nbytes) != 0)
    {
      return -EBADMSG;
    }

#endif
  return 0;
}

/****************************************************************************
 * Name: sht4x_cmd
 *
 * Description:
 *   I2C access helper for sending commands to the SHT4x which receive data
 *   in response.
 *
 ****************************************************************************/

static int sht4x_cmd(FAR struct sht4x_dev_s *priv, uint8_t command,
                     uint32_t timeout, uint16_t *data1, uint16_t *data2)
{
  struct i2c_msg_s cmd;
  struct i2c_msg_s read_data;
  int err;

  /* 2 byte data, 1 byte CRC, 2 byte data, 1 byte CRC. */

  uint8_t combined_data[6];

  cmd.frequency = CONFIG_SHT4X_I2C_FREQUENCY;
  cmd.addr = priv->addr;
  cmd.flags = 0;
  cmd.buffer = &command;
  cmd.length = sizeof(command);

  /* Trigger measurement. */

  err = I2C_TRANSFER(priv->i2c, &cmd, 1);
  if (err < 0)
    {
      return err;
    }

  /* Wait for measurement to complete. Serial number command does not require
   * wait time.
   */

  if (timeout > 0)
    {
      usleep(timeout);
    }

  read_data.frequency = CONFIG_SHT4X_I2C_FREQUENCY;
  read_data.addr = priv->addr;
  read_data.flags = I2C_M_READ;
  read_data.buffer = combined_data;
  read_data.length = sizeof(combined_data);

  err = I2C_TRANSFER(priv->i2c, &read_data, 1);
  if (err < 0)
    {
      return err;
    }

  /* Check CRC for first data packet */

  err = sht4x_crc(combined_data, 3);
  if (err < 0)
    {
      return err;
    }

  /* Check CRC for second data packet */

  err = sht4x_crc(combined_data + 3, 3);
  if (err < 0)
    {
      return err;
    }

  *data1 = (uint16_t)(combined_data[0] << 8) | combined_data[1];
  *data2 = (uint16_t)(combined_data[3] << 8) | combined_data[4];
  return err;
}

/****************************************************************************
 * Name: sht4x_reset
 *
 * Description:
 *   Perform a soft reset of the sensor. WARNING: Sensor is not available
 *   until 1ms after this command is sent.
 *
 ****************************************************************************/

static int sht4x_reset(FAR struct sht4x_dev_s *priv)
{
  struct i2c_msg_s msg[1];
  uint8_t reset_cmd = SHT4X_SOFT_RESET;

  msg[0].frequency = CONFIG_SHT4X_I2C_FREQUENCY;
  msg[0].addr = priv->addr;
  msg[0].flags = 0;
  msg[0].buffer = &reset_cmd;
  msg[0].length = sizeof(reset_cmd);

  return I2C_TRANSFER(priv->i2c, msg, 1);
}

/****************************************************************************
 * Name: sht4x_calc_temp
 *
 * Description:
 *   Calculate the temperature in degrees Celsius from the SHT4X raw data.
 *
 ****************************************************************************/

static float sht4x_calc_temp(uint16_t temp)
{
  return (float)(-45000 + 175 * ((temp * 1000) / 65535)) / 1000.0f;
}

/****************************************************************************
 * Name: sht4x_calc_hum
 *
 * Description:
 *   Calculate the humidity from the SHT4X raw data.
 *
 ****************************************************************************/

static float sht4x_calc_hum(uint16_t humidity)
{
  int16_t hum = -600 + 125 * ((humidity * 100) / 65535); /* 0.01 %RH */
#ifdef CONFIG_SHT4X_LIMIT_HUMIDITY

  /* Limit values of humidity, since values outside of 0-100 are invalid. */

  if (hum < 0)
    {
      hum = 0;
    }
  else if (hum > 10000)
    {
      hum = 10000;
    }

#endif
  return (float)(hum / 100.0f);
}

/****************************************************************************
 * Name: sht4x_read
 *
 * Description: Read temperature and humidity into UORB sensor formats.
 *
 * Return:
 *   Negated error code, or 0 on success.
 *
 ****************************************************************************/

static int sht4x_read(FAR struct sht4x_dev_s *priv,
                      FAR struct sensor_humi *humi,
                      FAR struct sensor_temp *temp)
{
  uint16_t raw_temp;
  uint16_t raw_hum;
  int err;

  /* Exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Get temp and humidity */

  err = sht4x_cmd(priv, g_precision_read[priv->precision],
                  g_measurement_times[priv->precision], &raw_temp, &raw_hum);
  if (err < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return err;
    }

  /* Release mutex */

  err = nxmutex_unlock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Store results */

  humi->timestamp = sensor_get_timestamp();
  humi->humidity = sht4x_calc_hum(raw_hum);
  temp->timestamp = humi->timestamp;
  temp->temperature = sht4x_calc_temp(raw_temp);

  return err;
}

/****************************************************************************
 * Name: has_time_passed
 *
 * Description:
 *   Return true if curr >= start + secs_since_start
 *
 ****************************************************************************/

static bool has_time_passed(struct timespec curr, struct timespec start,
                            unsigned int secs_since_start)
{
  if ((long)((start.tv_sec + secs_since_start) - curr.tv_sec) == 0)
    {
      return start.tv_nsec <= curr.tv_nsec;
    }

  return (long)((start.tv_sec + secs_since_start) - curr.tv_sec) <= 0;
}

/****************************************************************************
 * Name: sht4x_get_info
 ****************************************************************************/

static int sht4x_get_info(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          FAR struct sensor_device_info_s *info)
{
  /* Fill out information */

  info->version = 0;
  info->power = 0.0004f;  /* 0.4uA */
  info->min_delay = 1300; /* 1.3ms */
  info->max_delay = 8300; /* 8.3ms */
  memcpy(info->name, "SHT4x", sizeof("SHT4x"));
  memcpy(info->vendor, "Sensirion", sizeof("Sensirion"));

  /* Fill out information specific to a certain lower half */

  if (lower->type == SENSOR_TYPE_RELATIVE_HUMIDITY)
    {
      info->max_range = 100.0f;
      info->resolution = 0.01f;
    }
  else if (lower->type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      info->max_range = 125.0f;
      info->resolution = 0.01f;
    }

  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count = 0;
  return 0;
}

/****************************************************************************
 * Name: sht4x_set_interval
 *
 * Description:
 *     Sets the measurement interval for the SHT4X sensor in microseconds.
 *
 ****************************************************************************/

static int sht4x_set_interval(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR uint32_t *period_us)
{
  FAR struct sht4x_sensor_s *priv =
      container_of(lower, FAR struct sht4x_sensor_s, sensor_lower);
  FAR struct sht4x_dev_s *dev = priv->dev;
  dev->interval = *period_us;
  return 0;
}

/****************************************************************************
 * Name: sht4x_activate
 ****************************************************************************/

static int sht4x_activate(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  FAR struct sht4x_sensor_s *priv =
      container_of(lower, FAR struct sht4x_sensor_s, sensor_lower);
  FAR struct sht4x_dev_s *dev = priv->dev;

  if (enable)
    {
      if (!priv->enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the polling thread */

      nxsem_post(&dev->run);
    }

  return 0;
}

/****************************************************************************
 * Name: sht4x_control
 ****************************************************************************/

static int sht4x_control(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sht4x_sensor_s *priv =
      container_of(lower, FAR struct sht4x_sensor_s, sensor_lower);
  FAR struct sht4x_dev_s *dev = priv->dev;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
    case SNIOC_RESET:
      err = sht4x_reset(dev);
      break;

    case SNIOC_WHO_AM_I:
      {
        union sht4x_serialno_t serialno;
        err = sht4x_cmd(dev, SHT4X_READ_SERIAL, 10, &serialno.halves.msb,
                        &serialno.halves.lsb);
        *((FAR uint32_t *)(arg)) = serialno.full;
      }
      break;

    case SNIOC_HEAT:
      {
        struct timespec now;
        clock_systime_timespec(&now);

        /* Check if it has been one second since the last heat command. */

        if (!has_time_passed(now, dev->last_heat, 1))
          {
            err = -EAGAIN; /* Signal to try again in some time. */
            break;
          }

        /* Check for invalid heater command */

        if (0 < arg || arg >= (sizeof(g_heat_cmds) / sizeof(g_heat_cmds[0])))
          {
            return -EINVAL;
          }

        /* Heat for the desired period */

        uint16_t trash;
        err = sht4x_cmd(dev, g_heat_cmds[arg], g_heat_times[arg], &trash,
                        &trash);
        if (err)
          {
            break;
          }

        clock_systime_timespec(&dev->last_heat); /* Update last heat time. */
      }
      break;

    case SNIOC_CONFIGURE:

      /* Caller must pass precision option as argument. */

      dev->precision = arg;
      break;

    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: sht4x_fetch
 ****************************************************************************/

#ifndef CONFIG_SENSORS_SHT4X_POLL
static int sht4x_fetch(FAR struct sensor_lowerhalf_s *lower,
                       FAR struct file *filep, FAR char *buffer,
                       size_t buflen)
{
  FAR struct sht4x_sensor_s *priv =
      container_of(lower, FAR struct sht4x_sensor_s, sensor_lower);
  FAR struct sht4x_dev_s *dev = priv->dev;
  int err;

  err = sht4x_read(dev, &humi_data, &temp_data);
  if (err < 0)
    {
      return err;
    }

  if (dev->hum.enabled)
    {
      dev->hum.sensor_lower.push_event(dev->hum.sensor_lower.priv,
                                       &humi_data, sizeof(humi_data));
    }

  if (dev->temp.enabled)
    {
      dev->temp.sensor_lower.push_event(dev->temp.sensor_lower.priv,
                                        &temp_data, sizeof(temp_data));
    }

  return err;
}
#endif /* CONFIG_SENSORS_SHT4X_POLL */

/****************************************************************************
 * Name: sht4x_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number of arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int sht4x_thread(int argc, char **argv)
{
  FAR struct sht4x_dev_s *dev =
      (FAR struct sht4x_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  struct sensor_temp temp_data;
  struct sensor_humi humi_data;
  int err;

  while (true)
    {
      if (!dev->hum.enabled && !dev->temp.enabled)
        {
          /* Wait for one of the lower halves to be enabled and wake us up */

          err = nxsem_wait(&dev->run);
          if (err < 0)
            {
              continue;
            }
        }

      err = sht4x_read(dev, &humi_data, &temp_data);
      if (err < 0)
        {
          continue;
        }

      if (dev->hum.enabled)
        {
          dev->hum.sensor_lower.push_event(dev->hum.sensor_lower.priv,
                                           &humi_data, sizeof(humi_data));
        }

      if (dev->temp.enabled)
        {
          dev->temp.sensor_lower.push_event(dev->temp.sensor_lower.priv,
                                            &temp_data, sizeof(temp_data));
        }

      /* Sleep before next fetch */

      nxsig_usleep(dev->interval);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sht4x_register
 *
 * Description:
 *   Register the SHT4X character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the SHT4X
 *   addr    - The I2C address of the SHT4X. The I2C address is one of 0x44,
 *             0x45 and 0x46.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sht4x_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr)
{
  FAR struct sht4x_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x44 || addr == 0x45 || addr == 0x46);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct sht4x_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->precision = SHT4X_PREC_HIGH;
  priv->interval = 1000000; /* 1s interval */
  err = clock_systime_timespec(&priv->last_heat);

  if (err < 0)
    {
      snerr("ERROR: Failed to get timespec: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Allow heat immediately after registration since in theory the sensor has
   * never had its heater activated yet.
   */

  priv->last_heat.tv_sec -= 1;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("ERROR: Failed to register SHT4X driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register SHT4X driver: %d\n", err);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return err;
    }

  /* Register lower half for humidity */

  priv->hum.sensor_lower.ops = &g_sensor_ops;
  priv->hum.sensor_lower.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  priv->hum.enabled = false;
  priv->hum.dev = priv;

  /* Register UORB Sensor */

  err = sensor_register(&priv->hum.sensor_lower, devno);
  if (err < 0)
    {
      snerr("Failed to register SHT4X driver: %d\n", err);
      nxmutex_destroy(&priv->devlock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return err;
    }

  /* Register lower half for temperature */

  priv->temp.sensor_lower.ops = &g_sensor_ops;
  priv->temp.sensor_lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->temp.enabled = false;
  priv->temp.dev = priv;

  /* Register UORB Sensor */

  err = sensor_register(&priv->temp.sensor_lower, devno);
  if (err < 0)
    {
      snerr("Failed to register SHT4X driver: %d\n", err);
      sensor_unregister(&priv->hum.sensor_lower, devno);
      nxmutex_destroy(&priv->devlock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return err;
    }

  /* Polling thread */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  err = kthread_create("sht4x_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SHT4X_THREAD_STACKSIZE, sht4x_thread, argv);
  if (err < 0)
    {
      snerr("Failed to create the SHT4X notification kthread.\n");
      sensor_unregister(&priv->hum.sensor_lower, devno);
      sensor_unregister(&priv->temp.sensor_lower, devno);
      nxmutex_destroy(&priv->devlock);
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return err;
    }

  sninfo("Registered SHT4X driver.\n");
  return err;
}
