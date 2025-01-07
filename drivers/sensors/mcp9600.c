/****************************************************************************
 * drivers/sensors/mcp9600.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/mcp9600.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C frequency to use during transfers */

#ifdef CONFIG_MCP9600_I2C_FREQUENCY
#define CONFIG_MCP9600_I2C_FREQUENCY 100000
#endif

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

struct mcp9600_dev_s
{
  FAR struct i2c_master_s *i2c;  /* I2C interface */
  uint8_t addr;                  /* I2C address */
  struct mcp9600_devconf_s conf; /* Device configuration */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references. */
  bool unlinked; /* True, driver has been unlinked. */
#endif
  mutex_t devlock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mcp9600_open(FAR struct file *filep);
static int mcp9600_close(FAR struct file *filep);
static ssize_t mcp9600_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static ssize_t mcp9600_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int mcp9600_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int mcp9600_unlink(FAR struct inode *inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mcp9600fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .open = mcp9600_open,
    .close = mcp9600_close,
#else
    .open = NULL,
    .close = NULL,
#endif
    .read = mcp9600_read,
    .write = mcp9600_write,
    .seek = NULL,
    .ioctl = mcp9600_ioctl,
    .mmap = NULL,
    .truncate = NULL,
    .poll = NULL,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .unlink = mcp9600_unlink,
#endif
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
                             FAR int16_t *temp)
{
  int err;
  uint8_t raw[2];

  err = mcp9600_read_reg(priv, reg, raw, sizeof(raw));
  if (err < 0)
    {
      return err;
    }

  /* Positive temperature */

  *temp = (raw[0] * 16 + raw[1] / 16);

  /* Negative temperature */

  if (raw[0] & 0x80)
    {
      *temp -= 4096;
    }

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

  if (conf->thermo_type < MCP9600_THERMO_TYPE_K ||
      conf->thermo_type > MCP9600_THERMO_TYPE_R)
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
 * Name: mcp9600_open
 *
 * Description:
 *   This function is called whenever the MCP9600 device is opened.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int mcp9600_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mcp9600_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: mcp9600_close
 *
 * Description:
 *   This routine is called when the MCP9600 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int mcp9600_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mcp9600_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif

/****************************************************************************
 * Name: mcp9600_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int mcp9600_unlink(FAR struct inode *inode)
{
  FAR struct mcp9600_dev_s *priv;
  int err;

  DEBUGASSERT(inode->i_private != NULL);
  priv = inode->i_private;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Are there open references to the driver data structure? */

  if (priv->crefs <= 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  /* No. Just mark the driver as unlinked and free the resources when
   * the last client closes their reference to the driver.
   */

  priv->unlinked = true;
  nxmutex_unlock(&priv->devlock);
  return OK;
}
#endif

/****************************************************************************
 * Name: mcp9600_read
 *
 * Description:
 *     Character driver interface to sensor for debugging.
 *
 ****************************************************************************/

static ssize_t mcp9600_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mcp9600_dev_s *priv = inode->i_private;
  int err;
  int16_t hot_junc_temp;

  /* If file position is non-zero, then we're at the end of file. */

  if (filep->f_pos > 0)
    {
      return 0;
    }

  /* Get exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      goto finish_unlock;
    }
#endif

  err = mcp9600_read_temp(priv, REG_THERMO_HOT_JUNC, &hot_junc_temp);

  if (err < 0)
    {
      goto finish_unlock;
    }

  err = snprintf(buffer, buflen, "%d C\n", hot_junc_temp);

  if (err > buflen)
    {
      err = buflen;
    }

  filep->f_pos += err;

finish_unlock:
  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Name: mcp9600_write
 *
 * Description:
 *     Not implemented.
 ****************************************************************************/

static ssize_t mcp9600_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: mcp9600_ioctl
 ****************************************************************************/

static int mcp9600_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct mcp9600_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {
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

        err = mcp9600_read_reg(priv, REG_DEVID, devinfo, sizeof(*devinfo));
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

        err = mcp9600_read_reg(priv, REG_RAW_ADC, raw_data,
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

        err = mcp9600_read_reg(priv, REG_STATUS, &status_reg,
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
        err = mcp9600_write_reg(priv, REG_STATUS, &status_reg,
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

        err = mcp9600_write_reg(priv, REG_THERMO_SEN_CONF, registers,
                                sizeof(registers));
        if (err < 0)
          {
            break;
          };

        /* Store this as the official configuration */

        memcpy(&priv->conf, conf, sizeof(priv->conf));
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

        err = mcp9600_config_alert(priv, conf);
      }

      /* Read temperature data */

    case SNIOC_READTEMP:
      {
        struct mcp9600_temp_s *temps = (struct mcp9600_temp_s *)(arg);
        if (temps == NULL)
          {
            err = -EINVAL;
            break;
          }

        err =
            mcp9600_read_temp(priv, REG_JUNC_TEMP_DELTA, &temps->temp_delta);
        if (err < 0)
          {
            break;
          };

        err = mcp9600_read_temp(priv, REG_THERMO_HOT_JUNC, &temps->hot_junc);
        if (err < 0)
          {
            break;
          };

        err = mcp9600_read_temp(priv, REG_COLD_JUNC_TEMP, &temps->cold_junc);
      }

      /* Shutdown the device (argument unused) */

    case SNIOC_SHUTDOWN:
      {
        uint8_t reg = 0;
        priv->conf.mode = MCP9600_MODE_SHUTDOWN;

        reg |= (priv->conf.mode & 0x3);
        reg |= ((priv->conf.num_samples & 0x7) << 2);
        reg |= ((priv->conf.resolution & 0x3) << 5);
        reg |= ((priv->conf.cold_res & 0x1) << 7);

        err = mcp9600_write_reg(priv, REG_DEV_CONFIG, &reg, sizeof(reg));
      }

      /* Start the device again */

    case SNIOC_START:
      {
        uint8_t reg = 0;
        priv->conf.mode = MCP9600_MODE_NORMAL;

        reg |= (priv->conf.mode & 0x3);
        reg |= ((priv->conf.num_samples & 0x7) << 2);
        reg |= ((priv->conf.resolution & 0x3) << 5);
        reg |= ((priv->conf.cold_res & 0x1) << 7);

        err = mcp9600_write_reg(priv, REG_DEV_CONFIG, &reg, sizeof(reg));
      }

    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcp9600_register
 *
 * Description:
 *   Register the MCP9600 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the MCP9600
 *   addr    - The I2C address of the MCP9600, between 0x60 and 0x67
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int mcp9600_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                     uint8_t addr)
{
  FAR struct mcp9600_dev_s *priv;
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

  priv->conf = (struct mcp9600_devconf_s)
    {
      .thermo_type = MCP9600_THERMO_TYPE_T,
      .filter_coeff = 0,
      .resolution = MCP9600_ADC_RES_18,
      .num_samples = MCP9600_SAMPLE_1,
      .mode = MCP9600_MODE_NORMAL,
      .mode = MCP9600_COLDRES_0625,
    };

  priv->unlinked = false;
  priv->crefs = 0;

  /* Initialize mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("ERROR: Failed to register MCP9600 driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Register the character driver */

  err = register_driver(devpath, &g_mcp9600fops, 0666, priv);
  if (err < 0)
    {
      snerr("ERROR: Failed to register MCP9600 driver: %d\n", err);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return err;
}
