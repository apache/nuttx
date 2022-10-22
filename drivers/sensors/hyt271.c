/****************************************************************************
 * drivers/sensors/hyt271.c
 * Character driver for HYT271 Digital Humidity and Temperature Module.
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

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hyt271.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_HYT271)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HYT271_TEMPDATA_MASK 0x3fff
#define HYT271_HUMIDATA_MASK 0x3fff

#define HYT271_TEMPDATA_SHIFT(x) ((x) >> 2)
#define HYT271_HUMIDATA_SHIFT(x) ((x) >> 16)

#define HYT271_TEMPRAWDATA(x) (HYT271_TEMPDATA_SHIFT(x) & HYT271_TEMPDATA_MASK)
#define HYT271_TEMPRAWEQUAL(x, y) \
  (HYT271_TEMPRAWDATA(x) == HYT271_TEMPRAWDATA(y))

#define HYT271_HUMIRAWDATA(x) (HYT271_HUMIDATA_SHIFT(x) & HYT271_HUMIDATA_MASK)
#define HYT271_HUMIRAWEQUAL(x, y) \
  (HYT271_HUMIRAWDATA(x) == HYT271_HUMIRAWDATA(y))

#define HYT271_TEMPDATA(x) (HYT271_TEMPRAWDATA(x) * 165.0 / 16383.0 - 40.0)
#define HYT271_HUMIDATA(x) (HYT271_HUMIRAWDATA(x) * 100.0 / 16383.0)

#define HYT271_SENSOR_HUMI    0
#define HYT271_SENSOR_TEMP    1
#define HYT271_SENSOR_MAX     2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hyt271_sensor_data_s
{
  uint32_t data;
  uint64_t timestamp;
};

struct hyt271_dev_s;
struct hyt271_sensor_s
{
  struct sensor_lowerhalf_s lower;       /* Common lower interface */
  unsigned int              buffer_size; /* Data size for reading */
#ifdef CONFIG_SENSORS_HYT271_POLL
  bool                      enabled;     /* Sensor activated */
#endif
  FAR struct hyt271_dev_s   *dev;        /* Driver instance */
};

struct hyt271_dev_s
{
  struct hyt271_sensor_s sensor[HYT271_SENSOR_MAX]; /* Sensor types */
  FAR struct i2c_master_s *i2c;                     /* I2C interface */
  FAR struct hyt271_bus_s *bus;                     /* Bus power interface */
  mutex_t                 lock_measure_cycle;       /* Locks measure cycle */
  uint32_t                freq;                     /* I2C Frequency */
#ifdef CONFIG_SENSORS_HYT271_POLL
  unsigned long           interval;                 /* Polling interval */
  sem_t                   run;                      /* Locks sensor thread */
  bool                    initial_read;             /* Already read */
#endif
  uint8_t                 addr;                     /* I2C address */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sensor functions */

static int hyt271_active(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enabled);

static int hyt271_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);

static int hyt271_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg);

#ifdef CONFIG_SENSORS_HYT271_POLL
static int hyt271_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_hyt271_ops =
{
#ifdef CONFIG_SENSORS_HYT271_POLL
  .set_interval = hyt271_set_interval,
#endif
  .activate = hyt271_active,
  .fetch = hyt271_fetch,
  .control = hyt271_control
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hyt271_humi_from_rawdata
 *
 * Description: Helper for converting raw data to temperature.
 *
 * Parameter:
 *   data - Pointer to internal datat structure for measured data
 *   temp - Pointer to sensor data structure for humidity
 ****************************************************************************/

static void hyt271_humi_from_rawdata(FAR struct hyt271_sensor_data_s *data,
                                     FAR struct sensor_humi *humi)
{
  humi->timestamp   = data->timestamp;
  humi->humidity    = HYT271_HUMIDATA(data->data);
}

/****************************************************************************
 * Name: hyt271_temp_from_rawdata
 *
 * Description: Helper for converting raw data to temperature.
 *
 * Parameter:
 *   data - Pointer to internal datat structure for measured data
 *   temp - Pointer to sensor data structure for temperature
 ****************************************************************************/

static void hyt271_temp_from_rawdata(FAR struct hyt271_sensor_data_s *data,
                                     FAR struct sensor_temp *temp)
{
  temp->timestamp   = data->timestamp;
  temp->temperature = HYT271_TEMPDATA(data->data);
}

/****************************************************************************
 * Name: hyt271_curtime
 *
 * Description: Helper to get current timestamp.
 *
 * Return:
 *   Timestamp in nsec
 ****************************************************************************/

static unsigned long hyt271_curtime(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: hyt271_init_rw_buffer
 *
 * Description: Helper for buffer initializing.
 *
 * Parameter:
 *   buffer - Pointer to the buffers memory region
 *   cmd    - The command to set in the buffer
 *   size   - The buffer size
 ****************************************************************************/

static void hyt271_init_rw_buffer(FAR uint8_t *buffer, uint8_t cmd,
                                  uint8_t size)
{
  int n;

  buffer[0] = cmd;

  for (n = 1; n < size; n++)
    {
      buffer[n] = 0x00;
    }
}

/****************************************************************************
 * Name: hyt271_df
 *
 * Description: Helper for reading data by a data fetch.
 *
 * Parameter:
 *   dev    - Pointer to private driver instance
 *   config - I2C configuration
 *   buffer - Pointer to the buffers memory region
 *   size   - The buffer size
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_df(FAR struct hyt271_dev_s *dev,
                     FAR struct i2c_config_s *config, FAR uint8_t *buffer,
                     uint8_t size)
{
  int ret;

  /* Read data from the device */

  ret = i2c_read(dev->i2c, config, buffer, size);
  if (ret < 0)
    {
      snerr ("i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: hyt271_mr
 *
 * Description: Helper for sending a measurement cycle request.
 *
 * Parameter:
 *   dev    - Pointer to private driver instance
 *   config - I2C configuration
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_mr(FAR struct hyt271_dev_s *dev,
                     FAR struct i2c_config_s *config)
{
  uint8_t buffer;
  int ret;

  /* We only must send the i2c address with write bit enabled here. This
   * isn't provided by the i2c api, so instead sending a null byte seems
   * to work fine.
   */

  buffer = 0x00;

  /* Send address only with write bit enabled */

  ret = i2c_write(dev->i2c, config, &buffer, 1);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
    }

  /* Wait until measure cycle is done. This takes between 60 - 100 ms. */

  nxsig_usleep(100000);
  return ret;
}

/****************************************************************************
 * Name: hyt271_cmd
 *
 * Description: Helper for sending a command.
 *
 * Parameter:
 *   dev    - Pointer to private driver instance
 *   config - I2C configuration
 *   buffer - Pointer to the buffers memory region
 *   size   - The buffer size
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_cmd(FAR struct hyt271_dev_s *dev,
                      FAR struct i2c_config_s *config, FAR uint8_t *buffer,
                      uint8_t size)
{
  int ret;

  /* Write command data to the device */

  ret = i2c_write(dev->i2c, config, buffer, size);
  if (ret < 0)
    {
      snerr ("i2c_write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: hyt271_cmd_response
 *
 * Description: Helper for sending a command fetching the response after a
 *              necessary timeout.
 * Parameter:
 *   dev    - Pointer to private driver instance
 *   config - I2C configuration
 *   buffer - Pointer to the buffers memory region
 *   wsize  - Bytes to write from the buffer
 *   rsize  - Bytes  to read into the buffer
 *   usec   - Timeout between write and read operation in usec
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_cmd_response(FAR struct hyt271_dev_s *dev,
                               FAR struct i2c_config_s *config,
                               FAR uint8_t *buffer, uint8_t wsize,
                               uint8_t rsize, unsigned int usec)
{
  int ret;

  /* Send command */

  ret = hyt271_cmd(dev, config, buffer, wsize);
  if (ret < 0)
    {
      return ret;
    }

  /* Sleep for usec µs until response is ready */

  nxsig_usleep(usec);

  /* Read response and return */

  hyt271_init_rw_buffer(buffer, 0x00, rsize);
  ret = hyt271_df(dev, config, buffer, rsize);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if ack response is valid */

  if (buffer[0] != 0x81)
    {
      snerr ("wrong ack response: 0x%x\n", buffer[0]);
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: hyt271_change_addr
 *
 * Description: Change I2C address of the chip.
 *
 * Parameter:
 *   dev  - Pointer to private driver instance
 *   addr - The new I2C address
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_change_addr(FAR struct hyt271_dev_s *dev, uint8_t addr)
{
  int ret;
  irqstate_t irq_flags;
  struct i2c_config_s config;
  uint8_t buffer[3];

  if (dev->bus == NULL)
    {
      return -ENOSYS;
    }

  if (addr & 0x80)
    {
      return -EINVAL;
    }

  /* Reset and subsequent operations must be mutually exclusive.  Prevents
   * from being used by another open driver instance during this address
   * change operation.
   */

  ret = nxmutex_lock(&dev->lock_measure_cycle);
  if (ret < 0)
    {
      return ret;
    }

  /* Set up the I2C configuration */

  config.frequency = dev->freq;
  config.address   = dev->addr;
  config.addrlen   = 7;

  /* Enter in command mode */

  hyt271_init_rw_buffer(buffer, HYT271_CMD_START_CM, 3);

  /* The following section must be done within a time-constrained window.
   * It should be enough to lock this cpu than in the case of SMP all the
   * others.
   */

  irq_flags = up_irq_save();

  /* Power on reset the bus.
   * Entering in command mode must be done within the next 10 ms after power
   * up.
   */

  ret = dev->bus->pwonreset(dev->bus);
  if (ret < 0)
    {
      ret = -EIO;
      goto err_unlock;
    }

  ret = hyt271_cmd_response(dev, &config, buffer, 3, 1, 100);

  /* The chip should be in command mode now */

  up_irq_restore(irq_flags);

  if (ret < 0)
    {
      goto err_unlock;
    }

  /* Fetch current I2C address */

  hyt271_init_rw_buffer(buffer, HYT271_CMD_EEPROM_READ, 3);
  ret = hyt271_cmd_response(dev, &config, buffer, 3, 3, 100);
  if (ret < 0)
    {
      goto err_unlock;
    }

  if ((buffer[2] & 0x7f) != dev->addr)
    {
      snerr("wrong current I2C address response: 0x%x\n", buffer[2] & 0x7f);
      ret = -EIO;
      goto err_unlock;
    }

  /* Write new I2C address */

  hyt271_init_rw_buffer(buffer, HYT271_CMD_EEPROM_WRITE, 3);
  buffer[2] = addr;
  ret = hyt271_cmd_response(dev, &config, buffer, 3, 3, 12000);
  if (ret < 0)
    {
      goto err_unlock;
    }

  /* Fetch new I2C address again for confirmation */

  hyt271_init_rw_buffer(buffer, HYT271_CMD_EEPROM_READ, 3);
  ret = hyt271_cmd_response(dev, &config, buffer, 3, 3, 100);
  if (ret < 0)
    {
      goto err_unlock;
    }

  if ((buffer[2] & 0x7f) != addr)
    {
      snerr("wrong changed I2C address response: 0x%x\n", buffer[2] & 0x7f);
      ret = -EIO;
      goto err_unlock;
    }

  /* Finally go back to normal mode */

  hyt271_init_rw_buffer(buffer, HYT271_CMD_START_NOM, 3);
  ret = hyt271_cmd(dev, &config, buffer, 3);
  if (ret < 0)
    {
      goto err_unlock;
    }

  /* Set new address */

  dev->addr = addr;

  nxmutex_unlock(&dev->lock_measure_cycle);
  return OK;

err_unlock:
  nxmutex_unlock(&dev->lock_measure_cycle);
  return ret;
}

/****************************************************************************
 * Name: hyt271_measure_read
 *
 * Description:
 *   Performs a measurement cycle and reads measured data.
 *
 * Parameter:
 *   dev  - Pointer to private driver instance
 *   data - Pointer to internal data structure for measured data

 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_measure_read(FAR struct hyt271_dev_s *dev,
                               FAR struct hyt271_sensor_data_s *data)
{
  int ret;
  struct i2c_config_s config;
  uint8_t buffer[4];

  /* Measure request and read operation must be mutually exclusive.
   * Prevents from being used by another open driver instance during this
   * read operation.
   */

  ret = nxmutex_lock(&dev->lock_measure_cycle);
  if (ret < 0)
    {
      return ret;
    }

  /* Set up the I2C configuration */

  config.frequency = dev->freq;
  config.address   = dev->addr;
  config.addrlen   = 7;

  /* Start a measurement request */

  ret = hyt271_mr(dev, &config);
  if (ret < 0)
    {
      snerr("ERROR: Error starting measure cycle\n");
      goto err_unlock;
    }

  /* Read humity and temperature from the sensor */

  ret = hyt271_df(dev, &config, buffer, 4);
  if (ret < 0)
    {
      snerr("ERROR: Error reading sensor data!\n");
      goto err_unlock;
    }

  sninfo("value: %02x %02x %02x %02x ret: %d\n",
         buffer[0], buffer[1], buffer[2], buffer[3], ret);

  data->data = buffer[0] << 24 | buffer[1] << 16 | \
               buffer[2] << 8 | buffer[3];
  data->timestamp = hyt271_curtime();

  nxmutex_unlock(&dev->lock_measure_cycle);

  return OK;

err_unlock:
  nxmutex_unlock(&dev->lock_measure_cycle);
  return ret;
}

/****************************************************************************
 * Name: hyt271_fetch
 *
 * Description: Performs a measurement cycle and data read with data
 *              conversion.
 *
 * Parameter:
 *   lower  - Pointer to lower half sensor driver instance.
 *   filep  - The pointer of file, represents each user using the sensor.
 *   buffer - Pointer to the buffer for reading data.
 *   buflen - Size of the buffer.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  int ret;
  struct hyt271_sensor_data_s data;
  FAR struct hyt271_sensor_s *priv = (FAR struct hyt271_sensor_s *)lower;

  /* Check if the user is reading the right size */

  if (buflen != priv->buffer_size)
    {
      snerr("ERROR: You need to read %d bytes from this sensor!\n",
          priv->buffer_size);
      return -EINVAL;
    }

  ret = hyt271_measure_read(priv->dev, &data);
  if (ret < 0)
    {
      return ret;
    }

  switch (lower->type)
    {
      case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        {
            struct sensor_temp temp;
            hyt271_temp_from_rawdata(&data, &temp);
            memcpy(buffer, &temp, sizeof(temp));
        }
        break;
      case SENSOR_TYPE_RELATIVE_HUMIDITY:
        {
            struct sensor_humi humi;
            hyt271_humi_from_rawdata(&data, &humi);
            memcpy(buffer, &humi, sizeof(humi));
        }
        break;
      default:
        return -EINVAL;
    }

  return buflen;
}

/****************************************************************************
 * Name: hyt271_control
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg)
{
  int ret;
  struct hyt271_sensor_s *priv = (FAR struct hyt271_sensor_s *)lower;

  switch (cmd)
    {
      /* Read i2c address */

      case SNIOC_READADDR:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          DEBUGASSERT(ptr != NULL);
          *ptr = priv->dev->addr;
          sninfo("readaddr: %02x\n", *ptr);
          ret = OK;
        }
        break;

      /* Change i2c address */

      case SNIOC_CHANGEADDR:
        {
          uint8_t addr = arg;
          ret = hyt271_change_addr(priv->dev, addr);
          sninfo("changeaddr: %02x\n", addr);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: hyt271_active
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

static int hyt271_active(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, bool enabled)
{
#ifdef CONFIG_SENSORS_HYT271_POLL
  bool start_thread = false;
  struct hyt271_sensor_s *priv = (FAR struct hyt271_sensor_s *)lower;

  if (enabled)
    {
      if (!priv->dev->sensor[HYT271_SENSOR_TEMP].enabled &&
          !priv->dev->sensor[HYT271_SENSOR_HUMI].enabled)
        {
          start_thread = true;
        }
    }

  priv->enabled = enabled;

  if (start_thread == true)
    {
      /* Wake up the thread */

      nxsem_post(&priv->dev->run);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: hyt271_set_interval
 *
 * Description: Interface function of struct sensor_ops_s.
 *
 * Return:
 *   OK - on success
 ****************************************************************************/

#ifdef CONFIG_SENSORS_HYT271_POLL
static int hyt271_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct hyt271_sensor_s *priv = (FAR struct hyt271_sensor_s *)lower;
  priv->dev->interval = *period_us;
  return OK;
}
#endif

/****************************************************************************
 * Name: hyt271_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 ****************************************************************************/

#ifdef CONFIG_SENSORS_HYT271_POLL
static int hyt271_thread(int argc, char** argv)
{
  FAR struct hyt271_dev_s *priv = (FAR struct hyt271_dev_s *)
    ((uintptr_t)strtoul(argv[1], NULL, 0));
  uint32_t orawdata = 0;

  while (true)
    {
      int ret;
      struct hyt271_sensor_data_s data;
      struct hyt271_sensor_s *hsensor = &priv->sensor[HYT271_SENSOR_HUMI];
      struct hyt271_sensor_s *tsensor = &priv->sensor[HYT271_SENSOR_TEMP];

      if (!hsensor->enabled && !tsensor->enabled)
        {
          /* Reset initial read identifier */

          priv->initial_read = false;

          /* Waiting to be woken up */

          ret = nxsem_wait(&priv->run);
          if (ret < 0)
            {
              continue;
            }
        }

      ret = hyt271_measure_read(priv, &data);
      if (!ret)
        {
          /* Notify upper */

          if (priv->initial_read == false || (hsensor->enabled == true &&
              !HYT271_HUMIRAWEQUAL(orawdata, data.data)))
            {
              struct sensor_humi humi;
              hyt271_humi_from_rawdata(&data, &humi);
              hsensor->lower.push_event(hsensor->lower.priv, &humi,
                                        sizeof(struct sensor_humi));
            }

          if (priv->initial_read == false || (tsensor->enabled == true &&
              !HYT271_TEMPRAWEQUAL(orawdata, data.data)))
            {
              struct sensor_temp temp;
              hyt271_temp_from_rawdata(&data, &temp);
              tsensor->lower.push_event(tsensor->lower.priv, &temp,
                                        sizeof(struct sensor_temp));
            }

          if (priv->initial_read == false)
            {
              priv->initial_read = true;
            }

          /* Store the last sensor data for later comparison */

          orawdata = data.data;
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
 * Name: hyt271_register
 *
 * Description:
 *   Register the HYT271 character device.
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   i2c     - An instance of the I2C interface to communicate with HYT271
 *             sensor.
 *
 *   addr    - The I2C address of the HYT271.
 *   bus     - Callback to board specific logic for i2c bus power.
 *             Will be used for changing i2c address of the sensor and can be
 *             set to NULL when not supported.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int hyt271_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr,
                    FAR struct hyt271_bus_s *bus)
{
  int ret;
  struct hyt271_sensor_s *tmp;
#ifdef CONFIG_SENSORS_HYT271_POLL
  FAR char *argv[2];
  char arg1[32];
#endif

  /* Sanity check */

  if (!i2c)
    {
      snerr("Invalid i2c instance\n");
      return -EINVAL;
    }

  /* Initialize the HYT271 device structure */

  FAR struct hyt271_dev_s *priv = (FAR struct hyt271_dev_s *)
    kmm_malloc(sizeof(struct hyt271_dev_s));

  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->freq = 100000;
  priv->addr = addr;
  priv->bus  = bus;
#ifdef CONFIG_SENSORS_HYT271_POLL
  priv->interval = CONFIG_SENSORS_HYT271_POLL_INTERVAL;
  priv->initial_read = false;
#endif

  nxmutex_init(&priv->lock_measure_cycle);
#ifdef CONFIG_SENSORS_HYT271_POLL
  nxsem_init(&priv->run, 0, 0);
#endif

  /* Humidity register */

  tmp = &priv->sensor[HYT271_SENSOR_HUMI];
  tmp->dev = priv;
#ifdef CONFIG_SENSORS_HYT271_POLL
  tmp->enabled = false;
#endif
  tmp->buffer_size = sizeof(struct sensor_humi);
  tmp->lower.ops = &g_hyt271_ops;
  tmp->lower.type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  tmp->lower.uncalibrated = false;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto humi_err;
    }

  /* Temperature register */

  tmp = &priv->sensor[HYT271_SENSOR_TEMP];
  tmp->dev = priv;
#ifdef CONFIG_SENSORS_HYT271_POLL
  tmp->enabled = false;
#endif
  tmp->buffer_size = sizeof(struct sensor_temp);
  tmp->lower.ops = &g_hyt271_ops;
  tmp->lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  tmp->lower.uncalibrated = false;
  tmp->lower.nbuffer = 1;
  ret = sensor_register(&tmp->lower, devno);
  if (ret < 0)
    {
      goto temp_err;
    }

#ifdef CONFIG_SENSORS_HYT271_POLL
  /* Create thread for sensor */

  snprintf(arg1, 16, "0x%" PRIxPTR, (uintptr_t)priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("hyt271_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_HYT271_THREAD_STACKSIZE,
                       hyt271_thread, argv);
  if (ret > 0)
#endif
    {
      return OK;
    }

humi_err:
  sensor_unregister(&priv->sensor[HYT271_SENSOR_HUMI].lower, devno);
temp_err:
  sensor_unregister(&priv->sensor[HYT271_SENSOR_TEMP].lower, devno);

  nxmutex_destroy(&priv->lock_measure_cycle);
  kmm_free(priv);
  return ret;
}
#endif /* CONFIG_I2C && CONFIG_SENSORS_HYT271 */
