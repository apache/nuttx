/****************************************************************************
 * drivers/sensors/ct7117.c
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

#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ct7117.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CT7117_DEVID              0x59        /* Device id. */

/* Temperature resolution. */

#define CT7117_RESOLUTION         0.0078125f  /* degC. */

/* default ODR */

#define CT7117_DEFAULT_ODR        4000000     /* default 4s. */

/* Register address. */

#define CT7117_REG_DATA           0x00        /* Temperature data. */
#define CT7117_REG_CONF           0x01        /* Config information. */
#define CT7117_REG_WIA            0x07        /* Manufacture id. */

/* Conversion time bit. */

#define CT7117_CONVERSION_0p25HZ  0x00        /* Conversion Rate Selection 0.25Hz. */
#define CT7117_CONVERSION_1HZ     0x01        /* Conversion Rate Selection 1.0Hz. */
#define CT7117_CONVERSION_4HZ     0x02        /* Conversion Rate Selection 4.0Hz. */
#define CT7117_CONVERSION_8HZ     0x03        /* Conversion Rate Selection 8.0Hz. */

/* Mode Control bit. */

#define CT7117_SHOTDOWN_MODE      0x01        /* Control device in shotdown mode. */
#define CT7117_NORMAL_MODE        0x00        /* Control device in normal mode. */

#define CT7117_MAKE_S16(u8h, u8l) \
        (int16_t)(((uint16_t)(u8h) << 8) | (uint16_t)(u8l))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Device Register Bit. */

union ct7117_control_u
{
  uint16_t ctrl_w;
  struct
  {
    uint16_t os         : 1;     /* One shot action enable bit. */
    uint16_t cr         : 2;     /* Conversion rate selection bits. */
    uint16_t pec        : 1;     /* Packet error checking bit. */
    uint16_t to         : 1;     /* Time-out enable bit. */
    uint16_t re         : 2;     /* Resolution selection bits. */
    uint16_t em         : 1;     /* Extended mode bit. */
    uint16_t sd         : 1;     /* Shutdown bit. */
    uint16_t altm       : 1;     /* Over temperature alert mode selection bit. */
    uint16_t res1       : 1;     /* Reserved. */
    uint16_t fq         : 2;     /* Fault queue bits. */
    uint16_t res2       : 2;     /* Reserved. */
    uint16_t ots        : 1;     /* Over temperature ALERT status bit. */
  } ctrl_b;
};

typedef union ct7117_control_u ct7117_control_t;

/* Structure for ct7117 odr. */

struct ct7117_odr_s
{
  uint8_t regval;              /* the data of register. */
  unsigned long odr;           /* the unit is us. */
};

/* Structure for ct7117 device. */

struct ct7117_dev_s
{
  struct sensor_lowerhalf_s lower;          /* Lower half driver. */
  FAR const struct ct7117_config_s *config; /* The board config function. */
  bool activated;                           /* Sensor working state. */
  unsigned long interval;                   /* Sensor acquisition interval. */
  struct work_s work;                       /* Work queue for reading data. */
  int devno;                                /* The device number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions. */

static int ct7117_i2c_write(FAR struct ct7117_dev_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval);
static int ct7117_i2c_read(FAR struct ct7117_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval,
                           uint32_t cnt);
static int ct7117_i2c_readcfg(FAR struct ct7117_dev_s *priv,
                              FAR ct7117_control_t *ctrl);
static int ct7117_i2c_writecfg(FAR struct ct7117_dev_s *priv,
                               FAR ct7117_control_t *ctrl);

/* Sensor handle functions. */

static int ct7117_enable(FAR struct ct7117_dev_s *priv, bool enable);
static int ct7117_checkid(FAR struct ct7117_dev_s *priv);
static int ct7117_setmode(FAR struct ct7117_dev_s *priv, uint8_t mode);
static int ct7117_readtemp(FAR struct ct7117_dev_s *priv,
                           FAR struct sensor_temp *data);
static int ct7117_findodr(FAR unsigned long *expect_period_us);

/* Sensor ops functions. */

static int ct7117_activate(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           bool enable);
static int ct7117_set_interval(FAR struct file *filep,
                               FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned long *interval_us);
static int ct7117_selftest(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg);

/* Sensor poll functions. */

static void ct7117_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ct7117_odr_s g_ct7117_odr[] =
{
  {CT7117_CONVERSION_8HZ, 125000},          /* Sampling interval is 1.25s. */
  {CT7117_CONVERSION_4HZ, 250000},          /* Sampling interval is 2.5s. */
  {CT7117_CONVERSION_1HZ, 1000000},         /* Sampling interval is 1s. */
  {CT7117_CONVERSION_0p25HZ, 4000000},      /* Sampling interval is 4s. */
};

static const struct sensor_ops_s g_ct7117_ops =
{
  .activate = ct7117_activate,              /* Enable/disable snesor. */
  .set_interval = ct7117_set_interval,      /* Set output data period. */
  .selftest = ct7117_selftest               /* Sensor selftest function. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: ct7117_i2c_read
 *
 * Description:
 *   Read register data
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -Register value
 *   cnt      -Data number
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_i2c_read(FAR struct ct7117_dev_s *priv,
                           uint8_t regaddr, FAR uint8_t *regval,
                           uint32_t cnt)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->config->freq;
  msg[0].addr      = priv->config->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->config->freq;
  msg[1].addr      = priv->config->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = cnt;

  ret = I2C_TRANSFER(priv->config->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_i2c_write
 *
 * Description:
 *   Write ct7117 register
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -To be write value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_i2c_write(FAR struct ct7117_dev_s *priv,
                            uint8_t regaddr, FAR uint8_t *regval)
{
  struct i2c_msg_s msg;
  uint8_t w_buf[3];
  int ret;

  w_buf[0] = regaddr;
  w_buf[1] = regval[0];
  w_buf[2] = regval[1];

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = w_buf;
  msg.length    = 3;

  ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_i2c_readcfg
 *
 * Description:
 *   Read config regsiter
 *
 * Input Parameters
 *   priv     -Device struct
 *   ctrl     -ctrl value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_i2c_readcfg(FAR struct ct7117_dev_s *priv,
                              FAR ct7117_control_t *ctrl)
{
  int ret;
  uint8_t r_buf[2];

  ret = ct7117_i2c_read(priv, CT7117_REG_CONF, r_buf, 2);
  if (ret < 0)
    {
      snerr("Fail to read config register: %d\n", ret);
    }
  else
    {
      ctrl->ctrl_w = (uint16_t)((r_buf[0] << 8) | (r_buf[1]));
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_i2c_writecfg
 *
 * Description:
 *   Write ct7117 config register
 *
 * Input Parameters
 *   priv     -Device struct
 *   ctrl     -ctrl value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_i2c_writecfg(FAR struct ct7117_dev_s *priv,
                               FAR ct7117_control_t *ctrl)
{
  int ret;
  uint8_t w_buf[2];

  w_buf[0] = (uint8_t)((ctrl->ctrl_w) >> 8);
  w_buf[1] = (uint8_t)(ctrl->ctrl_w);

  ret = ct7117_i2c_write(priv, CT7117_REG_CONF, w_buf);
  if (ret < 0)
    {
      snerr("Fail to write config register: %d\n", ret);
    }

  return ret;
}

/* Sensor handle functions */

/****************************************************************************
 * Name: ct7117_setmode
 *
 * Description:
 *   Set work mode for CT7117
 *
 * Input Parameters
 *   priv     -Device struct
 *   mode     -Work mode
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_setmode(FAR struct ct7117_dev_s *priv,
                          uint8_t mode)
{
  ct7117_control_t ctrl;
  int ret;

  ret = ct7117_i2c_readcfg(priv, &ctrl);
  if (ret < 0)
    {
      snerr("Failed to read config register: %d\n", ret);
      return ret;
    }

  if (mode == CT7117_NORMAL_MODE)
    {
      ctrl.ctrl_b.sd = CT7117_NORMAL_MODE;
    }
  else
    {
      ctrl.ctrl_b.sd = CT7117_SHOTDOWN_MODE;
    }

  ret = ct7117_i2c_writecfg(priv, &ctrl);
  if (ret < 0)
    {
      snerr("Failed to set work mode: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_readtemp
 *
 * Description:
 *   Read temperature data
 *
 * Input Parameters
 *   priv     -Device struct
 *   data     -temperature data buffer
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_readtemp(FAR struct ct7117_dev_s *priv,
                           FAR struct sensor_temp *data)
{
  int ret;
  uint8_t buffer[2];

  ret = ct7117_i2c_read(priv, CT7117_REG_DATA, buffer, 2);
  if (ret < 0)
    {
      snerr("Failed to read temperature: %d\n", ret);
      return ret;
    }

  data->temperature = CT7117_RESOLUTION *
                      CT7117_MAKE_S16(buffer[0], buffer[1]);

  return ret;
}

/****************************************************************************
 * Name: ct7117_checkid
 *
 * Description:
 *   Read and verify the CT7117 chip ID
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_checkid(FAR struct ct7117_dev_s *priv)
{
  int ret;
  uint8_t devid;

  ret = ct7117_i2c_read(priv, CT7117_REG_WIA, &devid, 1);
  if (ret < 0 || devid != CT7117_DEVID)
    {
      snerr("Wrong Device ID! %02x\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_enable
 *
 * Description:
 *   CT7117 control on/off
 *
 * Input Parameters
 *   priv     -Device struct
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_enable(FAR struct ct7117_dev_s *priv, bool enable)
{
  int ret;

  if (enable)
    {
      /* Set normal mode. */

      ret = ct7117_setmode(priv, CT7117_NORMAL_MODE);
      if (ret < 0)
        {
          snerr("Failed to set normal mode: %d\n", ret);
          return ret;
        }

      work_queue(LPWORK, &priv->work,
                 ct7117_worker, priv,
                 priv->interval / USEC_PER_TICK);

      syslog(LOG_WARNING, "---CT7117 device %d power state: start---\n",
             priv->devno);
    }
  else
    {
      work_cancel(LPWORK, &priv->work);

      /* Set shotdown mode. */

      ret = ct7117_setmode(priv, CT7117_SHOTDOWN_MODE);
      if (ret < 0)
        {
          snerr("Failed to set shotdown mode: %d\n", ret);
          return ret;
        }

      syslog(LOG_WARNING, "---CT7117 device %d power state: end---\n",
             priv->devno);
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_findodr
 *
 * Description:
 *   Find the period that matches best.
 *
 * Input Parameters
 *   expect_period_us  -the time(expext) between report data, in us.
 *
 * Returned Value
 *   index of odr param.
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_findodr(FAR unsigned long *expect_period_us)
{
  int i;
  int len = sizeof(g_ct7117_odr) / sizeof(struct ct7117_odr_s);

  for (i = 0; i < len; i++)
    {
      if (*expect_period_us <= g_ct7117_odr[i].odr)
        {
          return i;
        }
    }

  return len - 1;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: ct7117_set_interval
 *
 * Description:
 *   Set ODR
 *
 * Input Parameters
 *   filep       - The pointer of file, represents each user using the sensor.
 *   lower       - The instance of lower half sensor driver.
 *   interval_us - Sample interval.
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_set_interval(FAR struct file *filep,
                               FAR struct sensor_lowerhalf_s *lower,
                               FAR unsigned long *interval_us)
{
  FAR struct ct7117_dev_s *priv = (FAR struct ct7117_dev_s *)lower;
  ct7117_control_t ctrl;
  int ret = 0;
  int idx;

  DEBUGASSERT(priv != NULL);

  /* Find the period that matches best. */

  idx = ct7117_findodr(interval_us);
  if (priv->interval != g_ct7117_odr[idx].odr)
    {
      ret = ct7117_i2c_readcfg(priv, &ctrl);
      if (ret < 0)
        {
          snerr("Failed to read config register: %d\n", ret);
          return ret;
        }

      ctrl.ctrl_b.cr = g_ct7117_odr[idx].regval;
      ret = ct7117_i2c_writecfg(priv, &ctrl);
      if (ret < 0)
        {
          snerr("Failed to set interval: %d\n", ret);
        }
      else
        {
          priv->interval = g_ct7117_odr[idx].odr;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ct7117_activate
 *
 * Description:
 *   Enable or disable sensor device
 *
 * Input Parameters
 *   filep  - The pointer of file, represents each user using the sensor.
 *   lower  - The instance of lower half sensor driver.
 *   enable - True(enable) and false(disable).
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ct7117_activate(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           bool enable)
{
  FAR struct ct7117_dev_s *priv = (FAR struct ct7117_dev_s *)lower;
  int ret;

  if (lower->type != SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (priv->activated != enable)
    {
      ret = ct7117_enable(priv, enable);
      if (ret < 0)
        {
          snerr("Failed to enable temperature sensor: %d\n", ret);
          return ret;
        }

      priv->activated = enable;
    }

  return OK;
}

/****************************************************************************
 * Name: ct7117_selftest
 *
 * Description:
 *   Mainly used in the self-test link, including device ID inspection
 *   and device functional inspection.
 *
 * Input Parameters:
 *   filep - The pointer of file, represents each user using the sensor.
 *   lower - The instance of lower half sensor driver.
 *   arg   - The parameters associated with cmd.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *   -ENOTTY    - The cmd don't support.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

static int ct7117_selftest(FAR struct file *filep,
                           FAR struct sensor_lowerhalf_s *lower,
                           unsigned long arg)
{
  FAR struct ct7117_dev_s *priv = (FAR struct ct7117_dev_s *)lower;
  int ret;

  DEBUGASSERT(lower != NULL);

  /* Process ioctl commands. */

  switch (arg)
    {
      case SNIOC_SIMPLE_CHECK:       /* Simple communication check. */
        {
          ret = ct7117_checkid(priv);
          if (ret < 0)
            {
              snerr("Failed to get DeviceID: %d\n", ret);
            }
        }
        break;

      default:                       /* Other cmd tag. */
        {
          ret = -ENOTTY;
          snerr("The cmd don't support: %d\n", ret);
        }
        break;
    }

    return ret;
}

/****************************************************************************
 * Name: ct7117_worker
 *
 * Description:
 *   Reading data according to the sampling time
 *
 * Input Parameters
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static void ct7117_worker(FAR void *arg)
{
  struct sensor_temp tmp;
  FAR struct ct7117_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  /* Get the timestamp. */

  tmp.timestamp = sensor_get_timestamp();

  /* Set work queue. */

  work_queue(LPWORK, &priv->work, ct7117_worker,
             priv, priv->interval / USEC_PER_TICK);

  if (ct7117_readtemp(priv, &tmp) >= 0)
    {
      priv->lower.push_event(priv->lower.priv, &tmp,
                             sizeof(struct sensor_temp));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ct7117_register
 *
 * Description:
 *   Register the CT7117 character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/tempN
 *   config  - configuration for the temperature driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ct7117_register(int devno, FAR const struct ct7117_config_s *config)
{
  FAR struct ct7117_dev_s *priv;
  int ret;

  /* Sanity check. */

  DEBUGASSERT(config != NULL);

  /* Initialize the CT7117 device structure. */

  priv = kmm_zalloc(sizeof(struct ct7117_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->lower.nbuffer = CONFIG_SENSORS_CT7117_BUFFER_NUMBER;
  priv->lower.ops = &g_ct7117_ops;
  priv->interval = CT7117_DEFAULT_ODR;
  priv->devno = devno;

  /* Check Device ID. */

  ret = ct7117_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  /* Enter shotdown mode. */

  ret = ct7117_enable(priv, false);
  if (ret < 0)
    {
      snerr("Failed to enter shotdown mode : %d\n", ret);
      goto err;
    }

  /* Register the sensor driver. */

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("Failed to register driver:%d\n", ret);
      goto err;
    }

  return ret;

err:
  kmm_free(priv);
  return ret;
}
