/****************************************************************************
 * drivers/sensors/ak09919c.c
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

#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ak09919c.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AK09919C_DEVID                  0x0E48       /* Device ID. */

/* Mag resolution. */

#define AK09919C_RESOLUTION             0.15f        /* uT/LSB. */

/* Register address. */

#define AK09919C_REG_WIA1               0x00         /* Company ID. */
#define AK09919C_REG_WIA2               0x01         /* Device ID. */
#define AK09919C_REG_ST1                0x10         /* Status 1 register. */
#define AK09919C_REG_HXH                0x11         /* X-axis data high byte. */
#define AK09919C_REG_HXL                0x12         /* X-axis data low byte. */
#define AK09919C_REG_HYH                0x13         /* Y-axis data high byte. */
#define AK09919C_REG_HYL                0x14         /* Y-axis data low byte. */
#define AK09919C_REG_HZH                0x15         /* Z-axis data high byte. */
#define AK09919C_REG_HZL                0x16         /* Z-axis data low byte. */
#define AK09919C_REG_TMPS               0x17         /* Dummy. */
#define AK09919C_REG_ST2                0x18         /* Status 2 register. */
#define AK09919C_REG_CNTL1              0x30         /* Control settings 1 register. */
#define AK09919C_REG_CNTL2              0x31         /* Control settings 2 register. */
#define AK09919C_REG_CNTL3              0x32         /* Control settings 3 register. */

/* Noise Suppression Filter. */

#define AK09919C_NSF_NONE               0x00         /* No noise suppression filter. */
#define AK09919C_NSF_LOW                0x01         /* Low noise suppression filter. */
#define AK09919C_NSF_MIDDLE             0x02         /* Middle noise suppression filter. */
#define AK09919C_NSF_HIGH               0x03         /* High noise suppression filter. */

/* Power mode. */

#define AK09919C_POWER_DOWN_MODE        0x00         /* Powerdown mode(default). */
#define AK09919C_SINGLESHOT_MODE        0x01         /* Single-shot mode. */
#define AK09919C_CONTINUOUS_MODE1       0x02         /* Continuous conversion mode 1. */
#define AK09919C_CONTINUOUS_MODE2       0x04         /* Continuous conversion mode 2. */
#define AK09919C_CONTINUOUS_MODE3       0x06         /* Continuous conversion mode 3. */
#define AK09919C_CONTINUOUS_MODE4       0x08         /* Continuous conversion mode 4. */
#define AK09919C_CONTINUOUS_MODE5       0x0E         /* Continuous conversion mode 5. */
#define AK09919C_SELF_TEST_MODE         0x10         /* Self test mode. */

/* Reset control bit. */

#define AK09919C_SOFT_RESET             0x01

/* Noise suppresion level control bit. */

#define AK09919C_NOISE_SUPPRESION       0x60

#define SET_BITSLICE(s, v, offset, mask)   \
do                                         \
  {                                        \
    s &= mask;                             \
    s |= (v << offset) & mask;             \
  }                                        \
while(0)                                   \

#define MAKE_S16(u8h, u8l)                 \
    (int16_t)(((uint16_t)(u8h) << 8) | (uint16_t)(u8l))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Structure for ak09919C device. */

struct ak09919c_dev_s
{
  struct sensor_lowerhalf_s lower;          /* The struct of lower half driver. */
  FAR struct ak09919c_config_s *config;     /* The board config function. */
  uint64_t timestamp;                       /* Units is microseconds. */
  bool activated;                           /* Sensor working state. */
  unsigned int interval;                    /* Sensor acquisition interval. */
  uint8_t workmode;                         /* Sensor work mode. */
  struct work_s work;                       /* Work queue for reading data. */
};

/* Structure for ak09919C odr. */

struct ak09919c_odr_s
{
  uint8_t regval;                           /* the data of register */
  unsigned int odr;                         /* the unit is us */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C functions. */

static int ak09919c_i2c_readreg(FAR struct ak09919c_dev_s *priv,
                                uint8_t regaddr, FAR uint8_t *regval);
static int ak09919c_i2c_writereg(FAR struct ak09919c_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval);
static int ak09919c_i2c_read(FAR struct ak09919c_dev_s *priv,
                             uint8_t regaddr, FAR uint8_t *buffer,
                             uint32_t cnt);

/* Sensor handle functions. */

static int ak09919c_enable(FAR struct ak09919c_dev_s *priv, bool enable);
static int ak09919c_init(FAR struct ak09919c_dev_s *priv);
static int ak09919c_checkid(FAR struct ak09919c_dev_s *priv);
static int ak09919c_setmode(FAR struct ak09919c_dev_s *priv, uint8_t mode);
static int ak09919c_setnoisefilter(FAR struct ak09919c_dev_s *priv,
                                   uint32_t nsf);
static int ak09919c_readmag(FAR struct ak09919c_dev_s *priv,
                            FAR struct sensor_event_mag *data);
static int ak09919c_softreset(FAR struct ak09919c_dev_s *priv);
static int ak09919c_findodr(FAR unsigned int *expect_period_us);

/* Sensor ops functions. */

static int ak09919c_activate(FAR struct sensor_lowerhalf_s *lower,
                             bool enable);
static int ak09919c_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned int * interval_us);

/* Sensor poll functions. */

static void ak09919c_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ak09919c_odr_s g_ak09919c_odr[] =
{
  {AK09919C_CONTINUOUS_MODE4, 10000},       /* Sampling interval is 10ms. */
  {AK09919C_CONTINUOUS_MODE3, 20000},       /* Sampling interval is 20ms. */
  {AK09919C_CONTINUOUS_MODE2, 50000},       /* Sampling interval is 50ms. */
  {AK09919C_CONTINUOUS_MODE1, 100000},      /* Sampling interval is 100ms. */
  {AK09919C_CONTINUOUS_MODE5, 200000},      /* Sampling interval is 200ms. */
};

static const struct sensor_ops_s g_ak09919c_ops =
{
  .activate = ak09919c_activate,            /* Enable/disable snesor. */
  .set_interval = ak09919c_set_interval,    /* Set output data period. */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* I2C functions */

/****************************************************************************
 * Name: ak09919c_i2c_read
 *
 * Description:
 *   Read mag data
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

static int ak09919c_i2c_read(FAR struct ak09919c_dev_s *priv,
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
 * Name: ak09919c_i2c_readreg
 *
 * Description:
 *   Read 8-bit ak09919c register
 *
 * Input Parameters
 *   priv     -Device struct
 *   regaddr  -Register address
 *   regval   -Register value
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ak09919c_i2c_readreg(FAR struct ak09919c_dev_s *priv,
                                uint8_t regaddr, FAR uint8_t *regval)
{
  return ak09919c_i2c_read(priv, regaddr, regval, 1);
}

/****************************************************************************
 * Name: ak09919c_i2c_writereg
 *
 * Description:
 *   Write 8-bit ak09919c register
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

static int ak09919c_i2c_writereg(FAR struct ak09919c_dev_s *priv,
                                 uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg;
  int ret;
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg.frequency = priv->config->freq;
  msg.addr      = priv->config->addr;
  msg.flags     = 0;
  msg.buffer    = txbuffer;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->config->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/* Sensor handle functions */

/****************************************************************************
 * Name: ak09919c_setmode
 *
 * Description:
 *   Set work mode for AK09919C
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

static int ak09919c_setmode(FAR struct ak09919c_dev_s *priv,
                            uint8_t mode)
{
  int ret;

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL2, mode);
  if (ret < 0)
    {
      snerr("Failed to set work mode: %d\n", ret);
    }
  else
    {
      up_udelay(1000);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_setnoisefilter
 *
 * Description:
 *   Set AK09919c noise suppression filter
 *
 * Input Parameters
 *   priv     -Device struct
 *   nsf     -noise suppression filter
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ak09919c_setnoisefilter(FAR struct ak09919c_dev_s *priv,
                                   uint32_t nsf)
{
  int ret;
  uint8_t ctrl1;

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_CNTL1, &ctrl1);
  if (ret < 0)
    {
      snerr("Failed to set noise filter: %d\n", ret);
    }

  SET_BITSLICE(ctrl1, nsf, 5, AK09919C_NOISE_SUPPRESION);

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL1, ctrl1);
  if (ret < 0)
    {
      snerr("Failed to set noise filter: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_readmag
 *
 * Description:
 *   Read mag data
 *
 * Input Parameters
 *   priv     -Device struct
 *   data     -mag data buffer
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ak09919c_readmag(FAR struct ak09919c_dev_s *priv,
                            FAR struct sensor_event_mag *data)
{
  int ret;
  uint8_t buffer[8];
  uint8_t state;

  /* AK09919C_REG_ST1 must be read separately before reading data. */

  ret = ak09919c_i2c_readreg(priv, AK09919C_REG_ST1, &state);
  if (ret < 0)
    {
      snerr("Failed to read drdy: %d\n", ret);
      return ret;
    }

  /* New data can be unlocked only when AK09919C_REG_ST2(8th byte) is read. */

  ret = ak09919c_i2c_read(priv, AK09919C_REG_HXH,
                          buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("Failed to read mag: %d\n", ret);
      return ret;
    }

  data->x = AK09919C_RESOLUTION * MAKE_S16(buffer[0], buffer[1]);
  data->y = AK09919C_RESOLUTION * MAKE_S16(buffer[2], buffer[3]);
  data->z = AK09919C_RESOLUTION * MAKE_S16(buffer[4], buffer[5]);
  data->timestamp = priv->timestamp;

  return ret;
}

/****************************************************************************
 * Name: ak09919c_checkid
 *
 * Description:
 *   Read and verify the AK09919c chip ID
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

static int ak09919c_checkid(FAR struct ak09919c_dev_s *priv)
{
  int ret;
  uint16_t devid;

  ret = ak09919c_i2c_read(priv, AK09919C_REG_WIA1, (uint8_t *)&devid, 2);
  if (ret < 0 || devid != AK09919C_DEVID)
    {
      snerr("Wrong Device ID! %02x\n", devid);
      ret = -ENODEV;
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_softreset
 *
 * Description:
 *   Reset AK09919c device
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

static int ak09919c_softreset(FAR struct ak09919c_dev_s *priv)
{
  int ret;

  ret = ak09919c_i2c_writereg(priv, AK09919C_REG_CNTL3,
                              AK09919C_SOFT_RESET);
  if (ret < 0)
    {
      snerr("Failed to reset AK09919C: %d\n", ret);
    }
  else
    {
      up_udelay(1000);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_enable
 *
 * Description:
 *   AK09919c control on/off
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

static int ak09919c_enable(FAR struct ak09919c_dev_s *priv, bool enable)
{
  int ret;

  if (enable)
    {
      /* Reset soft device */

      ret = ak09919c_softreset(priv);
      if (ret < 0)
        {
          snerr("Failed reset device: %d\n", ret);
          return ret;
        }

      /* Set noise filter */

      ret = ak09919c_setnoisefilter(priv, AK09919C_NSF_LOW);
      if (ret < 0)
        {
          snerr("Failed set noise filter: %d\n", ret);
          return ret;
        }

      /* Set work mode */

      ret = ak09919c_setmode(priv, priv->workmode);
      if (ret < 0)
        {
          snerr("Failed set work mode: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* Reset soft device */

      priv->workmode = AK09919C_POWER_DOWN_MODE;
      ret = ak09919c_softreset(priv);
      if (ret < 0)
        {
          snerr("Failed reset device: %d\n", ret);
          return ret;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_findodr
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

static int ak09919c_findodr(FAR unsigned int *expect_period_us)
{
  int i;
  int len = sizeof(g_ak09919c_odr) / sizeof(struct ak09919c_odr_s);

  for (i = 0; i < len; i++)
    {
      if (*expect_period_us <= g_ak09919c_odr[i].odr)
        {
          return i;
        }
    }

  return len - 1;
}

/****************************************************************************
 * Name: ak09919c_init
 *
 * Description:
 *   AK09919c init
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

static int ak09919c_init(FAR struct ak09919c_dev_s *priv)
{
  int ret;

  /* Reset soft device */

  ret = ak09919c_softreset(priv);
  if (ret < 0)
    {
      snerr("Failed reset device: %d\n", ret);
      return ret;
    }

  /* Set noise suppression level */

  ret = ak09919c_setnoisefilter(priv, AK09919C_NSF_LOW);
  if (ret < 0)
    {
      snerr("Failed set Noise suppression filter: %d\n", ret);
      return ret;
    }

  /* Set power down mode */

  ret = ak09919c_setmode(priv, priv->workmode);
  if (ret < 0)
    {
      snerr("Failed set work mode: %d\n", ret);
      return ret;
    }

  return ret;
}

/* Sensor ops functions */

/****************************************************************************
 * Name: ak09919c_set_interval
 *
 * Description:
 *   Set ODR
 *
 * Input Parameters
 *   lower       -The instance of lower half sensor driver
 *   interval_us -Sample interval
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ak09919c_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR unsigned int * interval_us)
{
  FAR struct ak09919c_dev_s *priv = (FAR struct ak09919c_dev_s *)lower;
  int ret;
  int idx;

  DEBUGASSERT(priv != NULL);

  /* Find the period that matches best. */

  idx = ak09919c_findodr(interval_us);
  priv->interval = g_ak09919c_odr[idx].odr;
  priv->workmode = g_ak09919c_odr[idx].regval;

  ret = ak09919c_setmode(priv, priv->workmode);
  if (ret < 0)
    {
      snerr("Failed set work mode: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ak09919c_activate
 *
 * Description:
 *   Enable or disable sensor device
 *
 * Input Parameters
 *   lower     -The instance of lower half sensor driver
 *   enable    -true(enable) and false(disable)
 *
 * Returned Value
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure;
 *
 * Assumptions/Limitations:
 *   none.
 *
 ****************************************************************************/

static int ak09919c_activate(FAR struct sensor_lowerhalf_s *lower,
                             bool enable)
{
  FAR struct ak09919c_dev_s *priv = (FAR struct ak09919c_dev_s *)lower;
  int ret;

  if (lower->type != SENSOR_TYPE_MAGNETIC_FIELD)
    {
      snerr("Failed to match sensor type.\n");
      return -EINVAL;
    }

  if (priv->activated != enable)
    {
      ret = ak09919c_enable(priv, enable);
      if (ret < 0)
        {
          snerr("Failed to enable mag sensor: %d\n", ret);
          return ret;
        }

      priv->activated = enable;

      if (enable)
        {
          work_queue(HPWORK, &priv->work,
                     ak09919c_worker, priv,
                     priv->interval / USEC_PER_TICK);
        }
      else
        {
          work_cancel(HPWORK, &priv->work);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ak09919c_worker
 *
 * Description:
 *   Polling the DRDY flag according to 1/5 of the sampling time,
 *   and read the data if DRDY is set
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

static void ak09919c_worker(FAR void *arg)
{
  struct sensor_event_mag tmp;
  FAR struct ak09919c_dev_s *priv = arg;

  DEBUGASSERT(priv != NULL);

  priv->timestamp = sensor_get_timestamp();
  work_queue(HPWORK, &priv->work, ak09919c_worker,
             priv, priv->interval / USEC_PER_TICK);

  if (ak09919c_readmag(priv, &tmp) >= 0)
    {
      priv->lower.push_event(priv->lower.priv, &tmp,
                             sizeof(struct sensor_event_mag));
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ak09919c_register
 *
 * Description:
 *   Register the AK09919C character device as 'devpath'.
 *
 * Input Parameters:
 *   devno   - The device number, used to build the device path
 *             as /dev/sensor/gyro_uncalN
 *   config  - configuration for the L3GD20 driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ak09919c_register(int devno, FAR struct ak09919c_config_s *config)
{
  FAR struct ak09919c_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(config != NULL);

  /* Initialize the AK09919C device structure */

  priv = kmm_zalloc(sizeof(struct ak09919c_dev_s));
  if (priv == NULL)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->config = config;
  priv->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  priv->lower.buffer_number = CONFIG_SENSORS_AK09919C_BUFFER_NUMBER;
  priv->lower.ops = &g_ak09919c_ops;
  priv->lower.uncalibrated = true;

  /* Check Device ID */

  ret = ak09919c_checkid(priv);
  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      goto err;
    }

  /* Init the device */

  ret = ak09919c_init(priv);
  if (ret < 0)
    {
      snerr("Failed to initialize physical device ak0991c:%d\n", ret);
      goto err;
    }

  /* Register the sensor driver */

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
