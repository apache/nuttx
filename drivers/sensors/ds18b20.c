/****************************************************************************
 * drivers/sensors/ds18b20.c
 * Character driver for DS18B20 Digital Humidity and Temperature Module.
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/1wire/1wire.h>
#include <nuttx/1wire/1wire_master.h>
#include <nuttx/1wire/1wire_crc.h>
#include <nuttx/sensors/ds18b20.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_1WIRE) && defined(CONFIG_SENSORS_DS18B20)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ENDIAN_BIG
#  define ds18b20_leuint64(x) (x)
#endif

/* Scratchpad data definition for reading */

#define DS18B20_SPAD_LSB_OFFSET       0
#define DS18B20_SPAD_MSB_OFFSET       1
#define DS18B20_SPAD_TH_OFFSET        2
#define DS18B20_SPAD_TL_OFFSET        3
#define DS18B20_SPAD_RES_OFFSET       4
#define DS18B20_SPAD_CRC_OFFSET       9
#define DS18B20_SPAD_SIZE             9

/* Scratchpad data definition for writing */

#define DS18B20_WSPAD_CMD_OFFSET       0
#define DS18B20_WSPAD_TH_OFFSET        0
#define DS18B20_WSPAD_TL_OFFSET        1
#define DS18B20_WSPAD_RES_OFFSET       2
#define DS18B20_WSPAD_SIZE             3

/* Measurement resolution bit definition */

#define DS18B20_RESMIN                 0
#define DS18B20_RESMAX                 3
#define DS18B20_NRES                   DS18B20_RESMAX - DS18B20_RESMIN + 1

#define DS18B20_RES_VAL(x)             (((x) >> 5) & 0x3)
#define DS18B20_RES_CONV(x)            (((x) & 0x3) << 5)

/* Measurement timneout offset */

#define DS18B20_TIMEOUT_OFFSET(x)      (DS18B20_RESMAX - (x))

/* Default alarm temperature */

#define DS18B20_TALARM_MIN             -55
#define DS18B20_TALARM_MAX             125

/* Helper for getting data from scratchpad memory */

#define DS18B20_TEMP_PREDEC(lsb, msb)  (int8_t)(((msb) << 4) | ((lsb) >> 4))
#define DS18B20_TEMP_RES(lsb, res)     (DS18B20_RESMAX - DS18B20_RES_VAL(res))
#define DS18B20_TEMP_RESMASK(lsb, res) (0x0f << DS18B20_TEMP_RES(lsb, res))
#define DS18B20_TEMP_DEC(lsb, res)     (((lsb) & 0x0f) & \
                                       DS18B20_TEMP_RESMASK(lsb, res))
#define DS18B20_TEMP(lsb, msb, res)    (DS18B20_TEMP_PREDEC(lsb, msb) + \
                                        DS18B20_TEMP_DEC(lsb, res) / 16.0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Internal sensor data */

struct ds18b20_sensor_data_s
{
  uint64_t timestamp;
  uint8_t  spad[DS18B20_SPAD_SIZE];
};

/* User configuration register */

struct ds18b20_config_s
{
  uint8_t res;                           /* Sensor resolution */
  struct ds18b20_alarm_s alarm;          /* Sensor temperature alarm */
};

/* Callback handling for alarm detection */

struct ds18b20_cb_alarm_s
{
  bool     isalarm;
  uint64_t romcode;
};

/* Lower sensor */

struct ds18b20_sensor_s
{
  struct sensor_lowerhalf_s lower;       /* Common lower interface */
#ifdef CONFIG_SENSORS_DS18B20_POLL
  bool                      enabled;     /* Sensor activated */
#endif
};

/* Internal device */

struct ds18b20_dev_s
{
  struct ds18b20_sensor_s     sensor;         /* Sensor  */
  FAR struct onewire_master_s *master;        /* 1wire master interface */
  struct onewire_config_s     config;         /* 1wire device configuration */
  struct ds18b20_config_s     reg;            /* Sensor resolution */
#ifdef CONFIG_SENSORS_DS18B20_POLL
  unsigned long               interval;       /* Polling interval */
  sem_t                       run;            /* Locks sensor thread */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor functions */

static int ds18b20_active(FAR struct sensor_lowerhalf_s *lower,
                          bool enabled);

static int ds18b20_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR char *buffer, size_t buflen);

static int ds18b20_control(FAR struct sensor_lowerhalf_s *lower,
                           int cmd, unsigned long arg);

#ifdef CONFIG_SENSORS_DS18B20_POLL
static int ds18b20_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR unsigned long *period_us);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Timeout definitions after measurement according to the data sheet */

static const uint32_t g_res_timeout[DS18B20_NRES] =
{
  93750,
  187500,
  375000,
  750000
};

static const struct sensor_ops_s g_ds18b20_ops =
{
#ifdef CONFIG_SENSORS_DS18B20_POLL
  .set_interval = ds18b20_set_interval,
#endif
  .activate     = ds18b20_active,
  .fetch        = ds18b20_fetch,
  .control      = ds18b20_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_rawdata
 *
 * Description:
 *   Helper for extract temperature data from scratchpad
 *
 * Return:
 *   Raw temperature data
 *
 ****************************************************************************/

static inline int16_t ds18b20_rawtemp(FAR const uint8_t *spad)
{
  return (spad[DS18B20_SPAD_LSB_OFFSET] |
          (spad[DS18B20_SPAD_MSB_OFFSET] << 8));
}

/****************************************************************************
 * Name: ds18b20_tempdata
 *
 * Description:
 *   Helper for converting temperature data from raw sensor data
 *
 * Return:
 *   Temperature data
 *
 ****************************************************************************/

static inline float ds18b20_temp(FAR const uint8_t *spad)
{
  int8_t predec = DS18B20_TEMP_PREDEC(spad[DS18B20_SPAD_LSB_OFFSET],
                                      spad[DS18B20_SPAD_MSB_OFFSET]);
  uint8_t dec   = DS18B20_TEMP_DEC(spad[DS18B20_SPAD_LSB_OFFSET],
                                   spad[DS18B20_SPAD_RES_OFFSET]);

  return predec + dec / 16.0;
}

/****************************************************************************
 * Name: ds18b20_alarm_cb
 *
 * Description:
 *   Call back function for searching a single alarm device.
 *
 * Input Parameters:
 *   romcode - Unique romcode of a device with alarm flag set
 *   arg     - Pointer to struct onewire_alarm_s
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20_POLL
static void ds18b20_alarm_cb(int family, uint64_t romcode, FAR void *arg)
{
  FAR struct ds18b20_cb_alarm_s *priv = (FAR struct ds18b20_cb_alarm_s *)arg;

  if (romcode == priv->romcode)
    {
      priv->isalarm = true;
    }
}
#endif

/****************************************************************************
 * Name: ds18b20_isalarm
 *
 * Description:
 *   Check if a 1wire device has set the alarm flag in an atomic operation.
 *
 * Input Parameters:
 *   master  - Pointer to the allocated 1-wire interface
 *   config  - Described the 1WIRE configuration
 *
 * Return Value:
 *   0   - no alarm flag is set
 *   1   - alarm flag is set
 *   < 0 - in case of error
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20_POLL
static int ds18b20_isalarm(FAR struct onewire_master_s *master,
                           FAR const struct onewire_config_s *config)
{
  int ret;
  struct ds18b20_cb_alarm_s alarm;
  alarm.romcode = config->romcode;
  alarm.isalarm = false;

  ret = onewire_search(master, DS18B20_DEVICE_FAMILY, true, ds18b20_alarm_cb,
                       &alarm);
  if (ret > 0)
    {
      /* Check if our device is in the alarm list */

      if (alarm.isalarm == true)
        {
          ret = 1;
        }
      else
        {
          /* Device found, but not our */

          ret = 0;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: ds18b20_measure
 *
 * Description: Perform a measurement
 *
 * Parameter:
 *   dev - Internal private lower half driver instance
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_measure(FAR struct ds18b20_dev_s *dev)
{
  int ret;
  uint8_t buf = DS18B20_CMD_CONVERTT;

  ret = onewire_write(dev->master, &dev->config, &buf, 1);
  if (ret < 0)
    {
      snerr("ERROR: Unable to perform a temperature measurement\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ds18b20_write_spad
 *
 * Description: Write scratchpad data to chip
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   th   - High alarm temperature
 *   lh   - Low alarm temperature
 *   res  - Temperature resolution
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_write_spad(FAR struct ds18b20_dev_s *dev, int8_t th,
                              int8_t tl, uint8_t res)
{
  int ret;
  uint8_t wbuf[DS18B20_WSPAD_SIZE + 1];

  wbuf[DS18B20_WSPAD_CMD_OFFSET]     = ONEWIRE_CMD_WRITE_SCRATCHPAD;
  wbuf[DS18B20_WSPAD_TH_OFFSET + 1]  = th;
  wbuf[DS18B20_WSPAD_TL_OFFSET + 1]  = tl;
  wbuf[DS18B20_WSPAD_RES_OFFSET + 1] = res;

  ret = onewire_write(dev->master, &dev->config, wbuf, sizeof(wbuf));
  if (ret < 0)
    {
      snerr("ERROR: Unable to write scratchpad\n");
    }

  return ret;
}

/****************************************************************************
 * Name: ds18b20_read_spad
 *
 * Description: Read scratchpad data from chip
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   spad - Pointer to store the scratchpad data
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_read_spad(FAR struct ds18b20_dev_s *dev,
                             FAR uint8_t *spad)
{
  int ret;
  uint8_t crc;
  uint8_t wbuf = ONEWIRE_CMD_READ_SCRATCHPAD;
  FAR struct ds18b20_config_s *reg = &dev->reg;

  ret = onewire_writeread(dev->master, &dev->config, &wbuf, 1, spad,
                          DS18B20_SPAD_SIZE);
  if (ret < 0)
    {
      snerr("ERROR: Unable to read scratchpad\n");
      return ret;
    }

  crc = onewire_crc8(spad, DS18B20_SPAD_SIZE - 1);
  if (crc != spad[DS18B20_SPAD_CRC_OFFSET - 1])
    {
      snerr("ERROR: crc mismatch in scratchpad\n");
      return -EIO;
    }

  /* Update configurable register state each time reading the scratchpad */

  reg->res = spad[DS18B20_SPAD_RES_OFFSET];

  if (reg->res < DS18B20_RES_CONV(DS18B20_RESMIN) ||
      reg->res > DS18B20_RES_CONV(reg->res))
    {
      /* Usually this should not happen, but be sure that we did not run into
       * a buffer overflow later.
       */

      swarn("WARNING: Sensor responded unknown resolution: %d\n", reg->res);
      reg->res = DS18B20_RES_CONV(DS18B20_RESMAX);
    }

  reg->alarm.thigh = spad[DS18B20_SPAD_TH_OFFSET];
  reg->alarm.tlow  = spad[DS18B20_SPAD_TL_OFFSET];

  return OK;
}

/****************************************************************************
 * Name: ds18b20_set_res
 *
 * Description: Set resolution for temperature measurement
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   res  - Resolution, can be 9 to 12 bit
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_set_res(FAR struct ds18b20_dev_s *dev, uint8_t res)
{
  int ret;
  uint8_t spad[DS18B20_SPAD_SIZE];

  /* Read current scratchpad first */

  ret = ds18b20_read_spad(dev, spad);
  if (ret < 0)
    {
      return ret;
    }

  ret = ds18b20_write_spad(dev, dev->reg.alarm.thigh, dev->reg.alarm.tlow,
                           res);
  if (ret < 0)
    {
      return ret;
    }

  /* Read current scratchpad again and verify that res is set */

  ret = ds18b20_read_spad(dev, spad);
  if (ret < 0)
    {
      return ret;
    }

  if (DS18B20_RES_VAL(spad[DS18B20_SPAD_RES_OFFSET]) != DS18B20_RES_VAL(res))
    {
      snerr("ERROR: Expected resolution not matched, received RES: %d\n",
            spad[DS18B20_SPAD_RES_OFFSET]);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: ds18b20_set_alarm
 *
 * Description: Set temperature alarm
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   alarm - Pointer to store the alarm temperature
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_set_alarm(FAR struct ds18b20_dev_s *dev,
                             FAR const struct ds18b20_alarm_s *alarm)
{
  int ret;
  uint8_t spad[DS18B20_SPAD_SIZE];

  /* Read current scratchpad first, to get current resolution */

  ret = ds18b20_read_spad(dev, spad);
  if (ret < 0)
    {
      return ret;
    }

  ret = ds18b20_write_spad(dev, alarm->thigh, alarm->tlow, dev->reg.res);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_SENSORS_DS18B20_POLL
    dev->reg.alarm.wakeup = alarm->wakeup;
#endif

  /* Read current scratchpad again and verify that alarm is set */

  ret = ds18b20_read_spad(dev, spad);
  if (ret < 0)
    {
      return ret;
    }

  if (spad[DS18B20_SPAD_TH_OFFSET] != alarm->thigh ||
      spad[DS18B20_SPAD_TL_OFFSET] != alarm->tlow)
    {
      snerr("ERROR: Expected alarm trigger does not match, " \
            "received TH: %d, TL: %d\n",
            spad[DS18B20_SPAD_TH_OFFSET],
            spad[DS18B20_SPAD_TL_OFFSET]);
      return -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: ds18b20_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in nsec
 ****************************************************************************/

static unsigned long ds18b20_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: ds18b20_notify
 *
 * Description:
 *   Notify upper about data has been changed.
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   data - Read sensor data
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20_POLL
static void ds18b20_notify(FAR struct ds18b20_dev_s *dev,
                           FAR struct ds18b20_sensor_data_s *data)
{
  FAR struct ds18b20_sensor_s *sensor = &dev->sensor;
  struct sensor_temp temp;

  temp.temperature = ds18b20_temp(data->spad);
  temp.timestamp   = data->timestamp;
  sensor->lower.push_event(sensor->lower.priv, &temp,
                           sizeof(struct sensor_temp));
}
#endif

/****************************************************************************
 * Name: ds18b20_measure_read
 *
 * Description: Perform a measurement and read last measured temperature
 *
 * Parameter:
 *   dev  - Internal private lower half driver instance
 *   data - Pointer to store sensor data
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_measure_read(FAR struct ds18b20_dev_s *dev,
                                FAR struct ds18b20_sensor_data_s *data)
{
  int ret;

  ret = ds18b20_measure(dev);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(g_res_timeout[DS18B20_RES_VAL(dev->reg.res)]);

  ret = ds18b20_read_spad(dev, data->spad);
  if (ret < 0)
    {
      return ret;
    }

  data->timestamp = ds18b20_curtime();
  return OK;
}

/****************************************************************************
 * Name: ds18b20_fetch
 *
 * Description: Performs a measuremnt cylce and data read with data
 *              conversion.
 *
 * Parameter:
 *   lower  - Pointer to lower half sensor driver instance
 *   buffer - Pointer to the buffer for reading data
 *   buflen - Size of the buffer
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR char *buffer, size_t buflen)
{
  int ret;
  struct ds18b20_sensor_data_s data;
  FAR struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)lower;

  /* Check if the user is reading the right size */

  if (buflen != sizeof(struct sensor_temp))
    {
      snerr("ERROR: You need to read %d bytes from this sensor!\n",
            sizeof(struct sensor_temp));
      return -EINVAL;
    }

  ret = ds18b20_measure_read(priv, &data);
  if (!ret)
    {
      FAR struct sensor_temp *temp =
        (FAR struct sensor_temp *)buffer;
      temp->temperature = ds18b20_temp(data.spad);
      temp->timestamp   = data.timestamp;
    }
  else
    {
      return ret;
    }

  return buflen;
}

/****************************************************************************
 * Name: ds18b20_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_control(FAR struct sensor_lowerhalf_s *lower,
                          int cmd, unsigned long arg)
{
  int ret;
  struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)lower;

  switch (cmd)
    {
      /* Read rom code */

      case SNIOC_READROMCODE:
        {
          FAR uint64_t *ptr = (FAR uint64_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->config.romcode;
          sninfo("read romcode: %16llx\n", *ptr);
          ret = OK;
        }
        break;

      /* Set new temperature alarm */

      case SNIOC_SETALARM:
        {
          FAR struct ds18b20_alarm_s *ptr =
            (FAR struct ds18b20_alarm_s *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          sninfo("set alarm: [TH/TL] = %d/%d °C\n", ptr->thigh, ptr->tlow);
          ret = ds18b20_set_alarm(priv, ptr);
        }
        break;

      /* Set sensor resolution */

      case SNIOC_SETRESOLUTION:
        {
          uint8_t res = arg - 9;
          sninfo("set resolution: %ld\n", arg);
          if (res >= DS18B20_RESMIN && res <= DS18B20_RESMAX)
            {
              ret = ds18b20_set_res(priv, DS18B20_RES_CONV(res));
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      default:
        snerr("ERROR: Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: ds18b20_active
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int ds18b20_active(FAR struct sensor_lowerhalf_s *lower,
                          bool enabled)
{
#ifdef CONFIG_SENSORS_DS18B20_POLL
  bool start_thread = false;
  struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)lower;

  if (enabled)
    {
      if (!priv->sensor.enabled)
        {
          start_thread = true;
        }
    }

  priv->sensor.enabled = enabled;

  if (start_thread == true)
    {
      /* Wake up the thread */

      nxsem_post(&priv->run);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: ds18b20_set_interval
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20_POLL
static int ds18b20_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned long *period_us)
{
  FAR struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)lower;
  priv->interval = *period_us;
  return OK;
}
#endif

/****************************************************************************
 * Name: ds18b20_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

#ifdef CONFIG_SENSORS_DS18B20_POLL
static int ds18b20_thread(int argc, char** argv)
{
  uint16_t orawdata;
  FAR struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)
        ((uintptr_t)strtoul(argv[1], NULL, 0));

  /* Set initial value to out of measurement range to ensure that the first
   * data read leads to an upper notification.
   */

  orawdata = 0xffff;

  while (true)
    {
      int ret;
      struct ds18b20_sensor_data_s data;
      FAR struct ds18b20_sensor_s *sensor = &priv->sensor;
      FAR struct ds18b20_alarm_s *alarm = &priv->reg.alarm;

      if (!sensor->enabled)
        {
          /* Waiting to be woken up */

          nxsem_wait(&priv->run);
        }

      if (alarm->wakeup == true)
        {
          /* Perform a temperature conversion to update the alarm flag */

          ret = ds18b20_measure(priv);
          if (!ret)
            {
              /* Wait until temperature conversion is done and the alarm flag
               * is up to date.
               */

              nxsig_usleep(g_res_timeout[DS18B20_RES_VAL(priv->reg.res)]);

              /* Check for existing temperature alarm */

              ret = ds18b20_isalarm(priv->master, &priv->config);
              if (ret == 1)
                {
                  ret = ds18b20_measure_read(priv, &data);
                  if (!ret && sensor->enabled == true)
                    {
                      /* Notify upper */

                      ds18b20_notify(priv, &data);
                    }
                }
            }
        }
      else
        {
          /* Default nofitication when temperature has been changed */

          ret = ds18b20_measure_read(priv, &data);
          if (!ret)
            {
              uint16_t rawtemp = ds18b20_rawtemp(data.spad);

              if (sensor->enabled == true && orawdata != rawtemp)
                {
                  ds18b20_notify(priv, &data);
                }

              /* Store the last sensor data for later comparison */

              orawdata = ds18b20_rawtemp(data.spad);
            }
        }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(priv->interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ds18b20_register
 *
 * Description:
 *   Register the DS18B20 character device.
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   onewire - An instance of the 1wire interface to communicate with DS18B20
 *             sensor.
 *   romcode - The ROM code of the DS18B20.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int ds18b20_register(int devno, FAR struct onewire_master_s *onewire,
                     uint64_t romcode)
{
  int ret;
  struct ds18b20_sensor_s *tmp;
#ifdef CONFIG_SENSORS_DS18B20_POLL
  FAR char *argv[2];
  char arg1[32];
#endif

  /* Sanity check */

  if (!onewire)
    {
      snerr("Invalid 1wire instance\n");
      return -EINVAL;
    }

  /* Initialize the DS18B20 device structure */

  FAR struct ds18b20_dev_s *priv = (FAR struct ds18b20_dev_s *)
    kmm_malloc(sizeof(struct ds18b20_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->master           = onewire;
  priv->config.romcode   = romcode;
  priv->reg.res          = DS18B20_RES_CONV(DS18B20_RESMAX);
  priv->reg.alarm.thigh  = DS18B20_TALARM_MAX;
  priv->reg.alarm.tlow   = DS18B20_TALARM_MIN;
#ifdef CONFIG_SENSORS_DS18B20_POLL
  priv->reg.alarm.wakeup = false;
  priv->interval         = CONFIG_SENSORS_DS18B20_POLL_INTERVAL;

  nxsem_init(&priv->run, 0, 0);
  nxsem_set_protocol(&priv->run, SEM_PRIO_NONE);
#endif

  /* Temperature register */

  tmp = &priv->sensor;
#ifdef CONFIG_SENSORS_DS18B20_POLL
  tmp->enabled = false;
#endif
  tmp->lower.ops = &g_ds18b20_ops;
  tmp->lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  tmp->lower.uncalibrated = false;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto sensor_err;
    }

#ifdef CONFIG_SENSORS_DS18B20_POLL

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("ds18b20_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_DS18B20_THREAD_STACKSIZE,
                       ds18b20_thread, argv);
  if (ret > 0)
#endif
    {
      return OK;
    }

sensor_err:
  sensor_unregister(&priv->sensor.lower, devno);
  kmm_free(priv);
  return ret;
}
#endif /* CONFIG_1WIRE && CONFIG_SENSORS_DS18B20 */
