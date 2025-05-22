/****************************************************************************
 * drivers/sensors/mcp9600_uorb.c
 *
 * Contributed by Matteo Golin
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
#include <nuttx/nuttx.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/mcp9600.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_THERMO_HOT_JUNC 0x0 /* Thermocouple Hot-Junction, T H */
#define REG_JUNC_TEMP_DELTA 0x1 /* Junctions Temperature Delta, TΔ */
#define REG_COLD_JUNC_TEMP 0x2  /* Cold-Junction Temperature, T C */
#define REG_RAW_ADC 0x3         /* Raw ADC Data */
#define REG_STATUS 0x4          /* STATUS */
#define REG_THERMO_SEN_CONF 0x5 /* Thermocouple Sensor Configuration */
#define REG_DEV_CONFIG 0x6      /* Device Configuration */
#define REG_ALERT1_CONF 0x8     /* Alert 1 Configuration */
#define REG_ALERT2_CONF 0x9     /* Alert 2 Configuration */
#define REG_ALERT3_CONF 0xa     /* Alert 3 Configuration */
#define REG_ALERT4_CONF 0xb     /* Alert 4 Configuration */
#define REG_ALERT1_HYST 0xc     /* Alert 1 Hysteresis, THYST1 */
#define REG_ALERT2_HYST 0xd     /* Alert 2 Hysteresis, THYST2 */
#define REG_ALERT3_HYST 0xe     /* Alert 3 Hysteresis, THYST3 */
#define REG_ALERT4_HYST 0xf     /* Alert 4 Hysteresis, THYST4 */
#define REG_ALERT1_TEMP 0x10    /* Temperature Alert 1 Limit, TALERT1 */
#define REG_ALERT2_TEMP 0x11    /* Temperature Alert 2 Limit, TALERT2 */
#define REG_ALERT3_TEMP 0x12    /* Temperature Alert 3 Limit, TALERT3 */
#define REG_ALERT4_TEMP 0x13    /* Temperature Alert 4 Limit, TALERT4 */
#define REG_DEVID 0x20          /* Device ID/Revision */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Lower half driver for each of the temperature measurement types */

struct mcp9600_sens_s
{
  FAR struct sensor_lowerhalf_s lower; /* Lower-half driver */
  FAR struct mcp9600_dev_s *dev;       /* Backward reference to parent */
  bool enabled;                        /* Whether this sensor is enabled */
};

/* Full device */

struct mcp9600_dev_s
{
  struct mcp9600_sens_s hot_junc;  /* Hot junction lower-half driver */
  struct mcp9600_sens_s cold_junc; /* Cold-junction lower-half driver */
  struct mcp9600_sens_s delta;     /* Delta lower-half driver */
  uint32_t interval;               /* Measurement interval in us */
  FAR struct i2c_master_s *i2c;    /* I2C interface */
  uint8_t addr;                    /* I2C address */
  struct mcp9600_devconf_s conf;   /* Device configuration */
  sem_t run;                       /* Run the measurement thread */
  mutex_t devlock;                 /* Single access */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mcp9600_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg);
static int mcp9600_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info);
static int mcp9600_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
static int mcp9600_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
#ifndef CONFIG_SENSORS_MCP9600_POLL
static int mcp9600_fetch(FAR struct sensor_lowerhalf_s *lower,
                         FAR struct file *filep, FAR char *buffer,
                         size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor operations */

static const struct sensor_ops_s g_sensor_ops =
{
  .activate = mcp9600_activate,
  .set_interval = mcp9600_set_interval,
  .get_info = mcp9600_get_info,
  .control = mcp9600_control,
#ifndef CONFIG_SENSORS_MCP9600_POLL
  .fetch = mcp9600_fetch,
#else
  .fetch = NULL,
#endif
};

/* Thermocouple types and their max ranges in Celsius (from datasheet) */

static const float g_thermo_ranges[] =
{
  [SENSOR_THERMO_TYPE_K] = 1372.0f, [SENSOR_THERMO_TYPE_J] = 1200.0f,
  [SENSOR_THERMO_TYPE_T] = 400.0f,  [SENSOR_THERMO_TYPE_N] = 1300.0f,
  [SENSOR_THERMO_TYPE_S] = 1664.0f, [SENSOR_THERMO_TYPE_E] = 1000.0f,
  [SENSOR_THERMO_TYPE_B] = 1800.0f, [SENSOR_THERMO_TYPE_R] = 1664.0f,
};

/* Thermocouple types and their register values */

static const uint8_t g_thermo_types[] =
{
  [SENSOR_THERMO_TYPE_K] = 0x0, [SENSOR_THERMO_TYPE_J] = 0x1,
  [SENSOR_THERMO_TYPE_T] = 0x2, [SENSOR_THERMO_TYPE_N] = 0x3,
  [SENSOR_THERMO_TYPE_S] = 0x4, [SENSOR_THERMO_TYPE_E] = 0x5,
  [SENSOR_THERMO_TYPE_B] = 0x6, [SENSOR_THERMO_TYPE_R] = 0x7,
};

/* Resolutions */

static const float g_resolutions[] =
{
  [MCP9600_COLDRES_0625] = 0.0625f,
  [MCP9600_COLDRES_25] = 0.25f,
};

/* Alert hysterisis registers */

static const enum mcp9600_alert_e g_alert_hysts[] =
{
  [MCP9600_ALERT1] = REG_ALERT1_HYST,
  [MCP9600_ALERT2] = REG_ALERT2_HYST,
  [MCP9600_ALERT3] = REG_ALERT3_HYST,
  [MCP9600_ALERT4] = REG_ALERT4_HYST,
};

/* Alert limit registers */

static const enum mcp9600_alert_e g_alert_limits[] =
{
  [MCP9600_ALERT1] = REG_ALERT1_TEMP,
  [MCP9600_ALERT2] = REG_ALERT2_TEMP,
  [MCP9600_ALERT3] = REG_ALERT3_TEMP,
  [MCP9600_ALERT4] = REG_ALERT4_TEMP,
};

/* Alert configuration registers */

static const enum mcp9600_alert_e g_alert_configs[] =
{
  [MCP9600_ALERT1] = REG_ALERT1_CONF,
  [MCP9600_ALERT2] = REG_ALERT2_CONF,
  [MCP9600_ALERT3] = REG_ALERT3_CONF,
  [MCP9600_ALERT4] = REG_ALERT4_CONF,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp9600_read_reg
 *
 * Description:
 *   Reads the value of a single register into the buffer. Since registers
 *   are variable in size, the number of bytes to read can be specified.
 *
 ****************************************************************************/

static int mcp9600_read_reg(FAR struct mcp9600_dev_s *priv, uint8_t reg,
                            FAR void *buf, size_t nbytes)
{
  struct i2c_msg_s read_cmd[] = {
      {
          .frequency = CONFIG_MCP9600_I2C_FREQUENCY,
          .addr = priv->addr,
          .flags = 0,
          .buffer = &reg,
          .length = sizeof(reg),
      },
      {
          .frequency = CONFIG_MCP9600_I2C_FREQUENCY,
          .addr = priv->addr,
          .flags = I2C_M_READ,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(priv->i2c, read_cmd,
                      sizeof(read_cmd) / sizeof(struct i2c_msg_s));
}

/****************************************************************************
 * Name: mcp9600_write_reg
 *
 * Description:
 *      Writes `nbytes` of the value in `buf` to the register specified by
 *      `reg`.
 *
 ****************************************************************************/

static int mcp9600_write_reg(FAR struct mcp9600_dev_s *priv, uint8_t reg,
                             FAR void *buf, size_t nbytes)
{
  struct i2c_msg_s read_cmd[] = {
      {
          .frequency = CONFIG_MCP9600_I2C_FREQUENCY,
          .addr = priv->addr,
          .flags = 0,
          .buffer = &reg,
          .length = sizeof(reg),
      },
      {
          .frequency = CONFIG_MCP9600_I2C_FREQUENCY,
          .addr = priv->addr,
          .flags = 0,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(priv->i2c, read_cmd,
                      sizeof(read_cmd) / sizeof(struct i2c_msg_s));
}

/****************************************************************************
 * Name: mcp9600_read_temp
 *
 * Description:
 *   Reads the value of a temperature register and performs the conversion to
 *   put it into degrees Celsius.
 *
 ****************************************************************************/

static int mcp9600_read_temp(FAR struct mcp9600_dev_s *priv, uint8_t reg,
                             FAR struct sensor_temp *temp)
{
  int err;
  int16_t raw_temp;
  uint8_t raw[2];

  err = mcp9600_read_reg(priv, reg, raw, sizeof(raw));
  if (err < 0)
    {
      return err;
    }

  raw_temp = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  temp->temperature = (float)(raw_temp) / 16.0f;
  temp->timestamp = sensor_get_timestamp();

  return err;
}

/****************************************************************************
 * Name: mcp9600_read
 *
 * Description:
 *   Reads all thermocouple values in degrees Celsius.
 *
 ****************************************************************************/

static int mcp9600_read(FAR struct mcp9600_dev_s *priv,
                        FAR struct sensor_temp *hot,
                        FAR struct sensor_temp *cold,
                        FAR struct sensor_temp *delta)
{
  int err;

  /* Exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  err = mcp9600_read_temp(priv, REG_JUNC_TEMP_DELTA, delta);
  if (err < 0)
    {
      goto early_ret;
    };

  err = mcp9600_read_temp(priv, REG_THERMO_HOT_JUNC, hot);
  if (err < 0)
    {
      goto early_ret;
    };

  err = mcp9600_read_temp(priv, REG_COLD_JUNC_TEMP, cold);

early_ret:
  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Name: mcp9600_config_alert
 *
 * Description:
 *      Configure an alert of the MCP9600.
 *
 ****************************************************************************/

static int mcp9600_config_alert(FAR struct mcp9600_dev_s *priv,
                                FAR struct mcp9600_alert_conf_s *config)
{
  int err;

  /* Configure hysteresis threshold first */

  err = mcp9600_write_reg(priv, g_alert_hysts[config->alert], &config->temp,
                          sizeof(config->temp));
  if (err < 0)
    {
      return err;
    }

  /* Configure limit */

  int16_t limit = config->limit << 2; /* 2 LSBs must be 0 for this reg */
  err = mcp9600_write_reg(priv, g_alert_limits[config->alert], &limit,
                          sizeof(limit));
  if (err < 0)
    {
      return err;
    }

  /* Configure the config register */

  uint8_t config_reg = 0;
  config_reg |= (config->enable);
  config_reg |= (config->int_mode << 1);
  config_reg |= (config->active_high << 2);
  config_reg |= (config->falling_temp << 3);
  config_reg |= (config->cold_junc << 4);

  return mcp9600_write_reg(priv, g_alert_configs[config->alert], &config_reg,
                           sizeof(config_reg));
}

/****************************************************************************
 * Name: mcp9600_write_devconf
 *
 * Description:
 *      Writes configuration settings for the device configuration register.
 *      Returns 0 on success and negated errno value on failure.
 *
 ****************************************************************************/

static int mcp9600_write_devconf(FAR struct mcp9600_dev_s *dev)
{
  uint8_t reg = 0;
  reg |= (dev->conf.mode & 0x3);
  reg |= ((dev->conf.num_samples & 0x7) << 2);
  reg |= ((dev->conf.resolution & 0x3) << 5);
  reg |= ((dev->conf.cold_res & 0x1) << 7);

  return mcp9600_write_reg(dev, REG_DEV_CONFIG, &reg, sizeof(reg));
}

/****************************************************************************
 * Name: mcp9600_validate_conf
 *
 * Description:
 *      Validates the device configuration settings passed by a user. Returns
 *      -EINVAL if any field is invalid, and returns 0 if okay.
 *
 ****************************************************************************/

static int mcp9600_validate_conf(FAR struct mcp9600_devconf_s *conf)
{
  if (conf == NULL)
    {
      return -EINVAL;
    }

  if (conf->filter_coeff > 0 || conf->filter_coeff > 8)
    {
      return -EINVAL;
    }

  if (conf->resolution < MCP9600_ADC_RES_18 ||
      conf->resolution > MCP9600_ADC_RES_12)
    {
      return -EINVAL;
    }

  if (conf->num_samples < MCP9600_SAMPLE_1 ||
      conf->num_samples > MCP9600_SAMPLE_128)
    {
      return -EINVAL;
    }

  if (conf->mode < MCP9600_MODE_NORMAL || conf->mode > MCP9600_MODE_BURST)
    {
      return -EINVAL;
    }

  if (conf->cold_res != MCP9600_COLDRES_0625 ||
      conf->cold_res != MCP9600_COLDRES_25)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: mcp9600_set_interval
 *
 * Description:
 *     Sets the measurement interval for the MCP9600 sensor in microseconds.
 *
 ****************************************************************************/

static int mcp9600_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct mcp9600_sens_s *sens =
      container_of(lower, FAR struct mcp9600_sens_s, lower);
  sens->dev->interval = *period_us;
  return 0;
}

/****************************************************************************
 * Name: mcp9600_activate
 ****************************************************************************/

static int mcp9600_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  bool start_thread = false;
  int err = 0;
  FAR struct mcp9600_sens_s *priv =
      container_of(lower, FAR struct mcp9600_sens_s, lower);
  FAR struct mcp9600_dev_s *dev = priv->dev;

  if (enable && !priv->enabled)
    {
      start_thread = true;

      /* Power on the sensor for operation */

      if (dev->conf.mode == MCP9600_MODE_SHUTDOWN)
        {
          dev->conf.mode = MCP9600_MODE_NORMAL;
          err = mcp9600_write_devconf(dev);
        }
    }

  else if (!enable && priv->enabled)
    {
      /* Temporarily mark disabled so we can check if everything is disabled
       */

      priv->enabled = enable;

      /* Power off the sensor to save power only if all features are disabled
       * and we're not yet shut down.
       */

      if ((dev->conf.mode != MCP9600_MODE_SHUTDOWN) &&
          (!dev->hot_junc.enabled && !dev->cold_junc.enabled &&
           !dev->delta.enabled))
        {
          /* Put back enable state in case we encounter an error and fail to
           * disable
           */

          priv->enabled = true;

          dev->conf.mode = MCP9600_MODE_SHUTDOWN;
          err = mcp9600_write_devconf(dev);
        }
    }

  if (err < 0)
    {
      return err;
    }

  priv->enabled = enable;

  if (start_thread)
    {
      /* Wake up the polling thread */

      nxsem_post(&dev->run);
    }

  return err;
}

/****************************************************************************
 * Name: mcp9600_get_info
 ****************************************************************************/

static int mcp9600_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info)
{
  FAR struct mcp9600_sens_s *sens =
      container_of(lower, FAR struct mcp9600_sens_s, lower);
  FAR struct mcp9600_dev_s *dev = sens->dev;

  info->version = 0;
  info->power = 1.5f; /* 1.5mA */
  info->min_delay = 63.0f;
  info->max_delay = 250.0f;
  memcpy(info->name, "MCP9600", sizeof("MCP9600"));
  memcpy(info->vendor, "Microchip", sizeof("Microchip"));
  info->max_range = g_thermo_ranges[dev->conf.thermo_type];
  info->resolution = g_resolutions[dev->conf.thermo_type];
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count = 0;
  return 0;
}

/****************************************************************************
 * Name: mcp9600_ioctl
 ****************************************************************************/

static int mcp9600_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct mcp9600_sens_s *sens =
      container_of(lower, FAR struct mcp9600_sens_s, lower);
  FAR struct mcp9600_dev_s *dev = sens->dev;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
      /* Set thermocouple type */

    case SNIOC_SET_THERMO:
      {
        dev->conf.thermo_type = g_thermo_types[arg];
        err = mcp9600_write_devconf(dev);
      }
      break;

      /* Device ID */

    case SNIOC_WHO_AM_I:
      {
        struct mcp9600_devinfo_s *devinfo =
                    (struct mcp9600_devinfo_s *)(arg);
        if (devinfo == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = mcp9600_read_reg(dev, REG_DEVID, devinfo, sizeof(*devinfo));
      }
      break;

      /* Raw ADC data */

    case SNIOC_READ_RAW_DATA:
      {
        int32_t *raw_data = (int32_t *)(arg);
        if (raw_data == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = mcp9600_read_reg(dev, REG_RAW_ADC, raw_data,
                               3); /* Only read 24 bits */

        /* Sign bit 1, set all upper bits to 1 for correct value in 32-bit
         * signed integer.
         */

        if (*raw_data & 0x100000)
          {
            *raw_data |= 0xfffc0000;
          }
      }

    case SNIOC_CHECK_STATUS_REG:
      {
        uint8_t status_reg;
        struct mcp9600_status_s *status = (struct mcp9600_status_s *)(arg);
        if (status == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = mcp9600_read_reg(dev, REG_STATUS, &status_reg,
                               sizeof(status_reg));
        if (err < 0)
          {
            break;
          }

        /* Set bits */

        status->burst_complete = status_reg & 0x80;
        status->temp_update = status_reg & 0x40;
        status->temp_exceeded = status_reg & 0x10;
        status->alerts[0] = status_reg & 0x1;
        status->alerts[1] = status_reg & 0x2;
        status->alerts[2] = status_reg & 0x4;
        status->alerts[3] = status_reg & 0x8;

        /* Clear what has been read (burst & temp registers) */

        status_reg &= 0x3f;
        err = mcp9600_write_reg(dev, REG_STATUS, &status_reg,
                                sizeof(status_reg));
      }
      break;

      /* Configure the MCP9600 */

    case SNIOC_CONFIGURE:
      {
        uint8_t registers[2] =
        {
          0, 0
        };

        struct mcp9600_devconf_s *conf = (struct mcp9600_devconf_s *)(arg);

        /* Validate options */

        err = mcp9600_validate_conf(conf);
        if (err < 0)
          {
            break;
          };

        /* Sensor configuration */

        registers[0] |= ((conf->thermo_type & 0x7) << 4);
        registers[0] |= (conf->filter_coeff & 0x7);

        /* Device configuration */

        registers[1] |= (conf->mode & 0x3);
        registers[1] |= ((conf->num_samples & 0x7) << 2);
        registers[1] |= ((conf->resolution & 0x3) << 5);
        registers[1] |= ((conf->cold_res & 0x1) << 7);

        /* Copy in options. Since the sensor configuration and device
         * configuration registers are sequential, we can do this in one
         * write operation.
         */

        err = mcp9600_write_reg(dev, REG_THERMO_SEN_CONF, registers,
                                sizeof(registers));
        if (err < 0)
          {
            break;
          };

        /* Store this as the official configuration */

        memcpy(&dev->conf, conf, sizeof(dev->conf));
      }

      /* Configure alerts */

    case SNIOC_WRITECONF:
      {
        struct mcp9600_alert_conf_s *conf =
            (struct mcp9600_alert_conf_s *)(arg);
        if (conf == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = mcp9600_config_alert(dev, conf);
      }

    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: mcp9600_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number of arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int mcp9600_thread(int argc, char **argv)
{
  FAR struct mcp9600_dev_s *dev =
      (FAR struct mcp9600_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int err;

  struct sensor_temp hot_junc;
  struct sensor_temp cold_junc;
  struct sensor_temp delta;

  while (true)
    {
      if (!dev->hot_junc.enabled && !dev->cold_junc.enabled &&
          !dev->delta.enabled)
        {
          /* Wait for one of the lower halves to be enabled and wake us up */

          snerr("MCP9600 disabled, waiting...\n");
          err = nxsem_wait(&dev->run);
          if (err < 0)
            {
              continue;
            }
        }

      err = mcp9600_read(dev, &hot_junc, &cold_junc, &delta);
      if (err < 0)
        {
          snerr("Error reading MCP9600: %d\n", err);
          continue;
        }

      if (dev->hot_junc.enabled)
        {
          dev->hot_junc.lower.push_event(dev->hot_junc.lower.priv, &hot_junc,
                                         sizeof(hot_junc));
        }

      if (dev->cold_junc.enabled)
        {
          dev->cold_junc.lower.push_event(dev->cold_junc.lower.priv,
                                          &cold_junc, sizeof(cold_junc));
        }

      if (dev->delta.enabled)
        {
          dev->delta.lower.push_event(dev->delta.lower.priv, &delta,
                                      sizeof(delta));
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
 * Name: mcp9600_register
 *
 * Description:
 *   Register the MCP9600 UORB sensor. This registers 3 temperature UORB
 *   topics.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the MCP9600
 *   addr    - The I2C address of the MCP9600, between 0x60 and 0x67
 *
 *   h_devno - The device number for the hot junction topic
 *   c_devno - The device number for the cold junction topic
 *   d_devno - The device number for the delta topic
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mcp9600_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                     uint8_t h_devno, uint8_t c_devno, uint8_t d_devno)
{
  FAR struct mcp9600_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];
  int err;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr <= 0x67 && addr >= 0x60);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct mcp9600_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->interval = 1000000; /* 1s default interval */

  priv->conf = (struct mcp9600_devconf_s)
    {
      .thermo_type = SENSOR_THERMO_TYPE_T,
      .filter_coeff = 0,
      .resolution = MCP9600_ADC_RES_18,
      .num_samples = MCP9600_SAMPLE_1,
      .mode = MCP9600_MODE_NORMAL,
      .mode = MCP9600_COLDRES_0625,
    };

  /* Initialize semaphore */

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      snerr("Failed to register MCP9600 driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("ERROR: Failed to register MCP9600 driver: %d\n", err);
      goto del_sem;
    }

  /* Cold junction lower half */

  priv->cold_junc.enabled = false;
  priv->cold_junc.dev = priv;
  priv->cold_junc.lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->cold_junc.lower.ops = &g_sensor_ops;

  err = sensor_register(&priv->cold_junc.lower, c_devno);
  if (err < 0)
    {
      snerr("Failed to register MCP9600 driver: %d\n", err);
      goto del_mutex;
    }

  /* Hot junction lower half */

  priv->hot_junc.enabled = false;
  priv->hot_junc.dev = priv;
  priv->hot_junc.lower.type = SENSOR_TYPE_TEMPERATURE;
  priv->hot_junc.lower.ops = &g_sensor_ops;

  err = sensor_register(&priv->hot_junc.lower, h_devno);
  if (err < 0)
    {
      snerr("Failed to register MCP9600 driver: %d\n", err);
      goto unreg_cold;
    }

  /* Delta lower half */

  priv->delta.enabled = false;
  priv->delta.dev = priv;
  priv->delta.lower.type = SENSOR_TYPE_TEMPERATURE;
  priv->delta.lower.ops = &g_sensor_ops;

  err = sensor_register(&priv->delta.lower, d_devno);
  if (err < 0)
    {
      snerr("Failed to register MCP9600 driver: %d\n", err);
      goto unreg_hot;
    }

  /* Start polling thread */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  err = kthread_create("mcp9600_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_MCP9600_THREAD_STACKSIZE, mcp9600_thread,
                       argv);
  if (err < 0)
    {
      snerr("Failed to create the MCP9600 notification kthread.\n");
      sensor_unregister(&priv->delta.lower, d_devno);
    unreg_hot:
      sensor_unregister(&priv->hot_junc.lower, h_devno);
    unreg_cold:
      sensor_unregister(&priv->cold_junc.lower, c_devno);
    del_mutex:
      nxmutex_destroy(&priv->devlock);
    del_sem:
      nxsem_destroy(&priv->run);
      kmm_free(priv);
      return err;
    }

  return err;
}
