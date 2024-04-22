/****************************************************************************
 * drivers/sensors/bmm150_uorb.c
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

#include <debug.h>
#include <stdio.h>
#include <sys/param.h>

#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#include <nuttx/kmalloc.h>

#include <nuttx/sensors/bmm150.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMM150_I2C_FREQ          400000

#define BMM150_CHIPID_VAL        0x32

#define BMM150_CHIPID            0x40
#define BMM150_DATA_X_LSB        0x42
#define BMM150_DATA_X_MSB        0x43
#define BMM150_DATA_Y_LSB        0x44
#define BMM150_DATA_Y_MSB        0x45
#define BMM150_DATA_Z_LSB        0x46
#define BMM150_DATA_Z_MSB        0x47
#define BMM150_RHALL_LSB         0x48
#define BMM150_RHALL_MSB         0x49
#define BMM150_IRQSTAT           0x4a
#define BMM150_POWER             0x4b
#define BMM150_MODE              0x4c
#define BMM150_INTEN             0x4d
#define BMM150_INTCFG            0x4e
#define BMM150_LOW_THR           0x4f
#define BMM150_HIGH_THR          0x50
#define BMM150_REPXY             0x51
#define BMM150_REPZ              0x52

/* TRIM registers - NVM */

#define BMM150_TRIM_X1           0x5d /* 0 */
#define BMM150_TRIM_Y1           0x5e /* 1 */
#define BMM150_TRIM_Z4_LSB       0x62 /* 5 */
#define BMM150_TRIM_Z4_MSB       0x63 /* 6 */
#define BMM150_TRIM_X2           0x64 /* 7 */
#define BMM150_TRIM_Y2           0x65 /* 8 */
#define BMM150_TRIM_Z2_LSB       0x68 /* 11 */
#define BMM150_TRIM_Z2_MSB       0x69 /* 12 */
#define BMM150_TRIM_Z1_LSB       0x6a /* 13 */
#define BMM150_TRIM_Z1_MSB       0x6b /* 14 */
#define BMM150_TRIM_XYZ1_LSB     0x6c /* 15 */
#define BMM150_TRIM_XYZ1_MSB     0x6d /* 16 */
#define BMM150_TRIM_Z3_LSB       0x6e /* 17 */
#define BMM150_TRIM_Z3_MSB       0x6f /* 18 */
#define BMM150_TRIM_XY2          0x70 /* 19 */
#define BMM150_TRIM_XY1          0x71 /* 20 */

#define POWER_CONTROL            (1 << 0)

#define MODE_SELFTEST            0x01
#define MODE_OPMODE_NORMAL       (0 << 1)
#define MODE_OPMODE_FORCED       (1 << 1)
#define MODE_OPMODE_SLEEP        (2 << 1)
#define MODE_OPMODE_MASK         (3 << 1)

#define MODE_DATARATE_10HZ       (0 << 3)
#define MODE_DATARATE_2HZ        (1 << 3)
#define MODE_DATARATE_6HZ        (2 << 3)
#define MODE_DATARATE_8HZ        (3 << 3)
#define MODE_DATARATE_15HZ       (4 << 3)
#define MODE_DATARATE_20HZ       (5 << 3)
#define MODE_DATARATE_25HZ       (6 << 3)
#define MODE_DATARATE_30HZ       (7 << 3)

#define MODE_ADV_SELFTEST0       (1 << 6)
#define MODE_ADV_SELFTEST1       (1 << 7)

#define INTCFG_EN_X              (1 << 3)
#define INTCFG_EN_Y              (1 << 4)
#define INTCFG_EN_Z              (1 << 5)

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmm150_trim_s
{
  int8_t   x1;
  int8_t   y1;
  int8_t   x2;
  int8_t   y2;
  uint16_t z1;
  int16_t  z2;
  int16_t  z3;
  int16_t  z4;
  int8_t   xy1;
  int8_t   xy2;
  uint16_t xyz1;
};

/* Used by the driver to manage the device */

struct bmm150_sensor_dev_s
{
  struct sensor_lowerhalf_s       lower;
  struct bmm150_trim_s            trim;
  bool                            enabled;
  struct bmm150_config_s          config;
  uint32_t                        freq;
  mutex_t                         lock;
#ifdef CONFIG_SENSORS_BMM150_POLL
  unsigned long                   interval;
  uint64_t                        last_update;
  sem_t                           run;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bmm150_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
#ifndef CONFIG_SENSORS_BMM150_POLL
static int bmm150_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);
#endif
static int bmm150_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);

/* Helpers */

static float bmm150_getx(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data);
static float bmm150_gety(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data);
static float bmm150_getz(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_bmm150_sensor_ops =
{
  NULL,                 /* open */
  NULL,                 /* close */
  bmm150_activate,
  bmm150_set_interval,
  NULL,                 /* batch */
#ifdef CONFIG_SENSORS_BMM150_POLL
  NULL,                 /* fetch */
#else
  bmm150_fetch,
#endif
  NULL,                 /* selftest */
  NULL,                 /* set_calibvalue */
  NULL,                 /* calibrate */
  NULL,                 /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_getreg8
 *
 * Description:
 *   Read from an 8-bit BMM150 register
 *
 ****************************************************************************/

static uint8_t bmm150_getreg8(FAR struct bmm150_sensor_dev_s *priv,
                              uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return regval;
}

/****************************************************************************
 * Name: bmm150_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

void bmm150_getregs(FAR struct bmm150_sensor_dev_s *priv, uint8_t regaddr,
                    FAR uint8_t *regval, int len)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->config.addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->config.addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->config.i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: bmm150_putreg8
 *
 * Description:
 *   Write to an 8-bit BMM150 register
 *
 ****************************************************************************/

static void bmm150_putreg8(FAR struct bmm150_sensor_dev_s *priv,
                           uint8_t regaddr, uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->config.addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->config.i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: bmm150_initialize
 *
 * Description:
 *   Intialzie BMM150 driver
 *
 ****************************************************************************/

static int bmm150_initialzie(FAR struct bmm150_sensor_dev_s *dev)
{
  uint8_t  data[21];
  uint16_t u16;
  int16_t  i16;
  uint8_t  id  = 0;
  int      ret = OK;

  /* Set Power Control */

  bmm150_putreg8(dev, BMM150_POWER, POWER_CONTROL);

  /* Start-Up Time */

  up_mdelay(3);

  /* Read Chip ID */

  id = bmm150_getreg8(dev, BMM150_CHIPID);
  if (id != BMM150_CHIPID_VAL)
    {
      /* Chip ID is not Correct */

      snerr("Wrong chip ID! %02x\n", id);
      ret = -ENODEV;
      goto errout;
    }

  /* Get TRIM registers */

  bmm150_getregs(dev, BMM150_TRIM_X1, (FAR uint8_t *)data, 20);

  dev->trim.x1 = (int8_t)data[0];
  dev->trim.y1 = (int8_t)data[1];
  dev->trim.x2 = (int8_t)data[7];
  dev->trim.y2 = (int8_t)data[8];
  u16 = ((uint16_t)(data[14] << 8) + data[11]);
  dev->trim.z1 = u16;
  i16 = ((int16_t)(data[12] << 8) + data[11]);
  dev->trim.z2 = i16;
  i16 = ((int16_t)(data[18] << 8) + data[17]);
  dev->trim.z3 = i16;
  i16 = ((int16_t)(data[6] << 8) + data[7]);
  dev->trim.z4 = i16;
  dev->trim.xy1 = (int8_t)data[20];
  dev->trim.xy2 = (int8_t)data[19];
  u16 = ((uint16_t)(data[16] << 8) + data[15]);
  dev->trim.xyz1 = u16;

errout:

  /* Clear Power Control */

  bmm150_putreg8(dev, BMM150_POWER, 0);

  return ret;
}

/****************************************************************************
 * Name: bmm150_activate
 ****************************************************************************/

static int bmm150_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct bmm150_sensor_dev_s *dev          = NULL;
#ifdef CONFIG_SENSORS_BMM150_POLL
  bool                            start_thread = false;
#endif
  uint8_t                         val          = 0;

  dev = (FAR struct bmm150_sensor_dev_s *)lower;

  nxmutex_lock(&dev->lock);

  if (enable)
    {
#ifdef CONFIG_SENSORS_BMM150_POLL
      if (!dev->enabled)
        {
          start_thread      = true;
          dev->last_update = sensor_get_timestamp();
        }
#endif

      /* Set Power Control */

      bmm150_putreg8(dev, BMM150_POWER, POWER_CONTROL);

      /* Start-Up Time */

      up_mdelay(3);

      /* Set Normal Mode */

      val = bmm150_getreg8(dev, BMM150_MODE);
      val &= ~MODE_OPMODE_MASK;
      val |= MODE_OPMODE_NORMAL;
      bmm150_putreg8(dev, BMM150_MODE, val);

      /* Enable XYZ - active low */

      val = 0;
      bmm150_putreg8(dev, BMM150_INTCFG, val);
    }
  else
    {
      /* Set Sleep Mode */

      val = bmm150_getreg8(dev, BMM150_MODE);
      val |= MODE_OPMODE_SLEEP;
      bmm150_putreg8(dev, BMM150_MODE, val);
    }

  dev->enabled = enable;

#ifdef CONFIG_SENSORS_BMM150_POLL
  if (start_thread)
    {
      /* Wake up the thread */

      nxsem_post(&dev->run);
    }
#endif

  nxmutex_unlock(&dev->lock);

  return OK;
}

/****************************************************************************
 * Name: bmm150_getx
 ****************************************************************************/

static float bmm150_getx(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data)
{
  float tmp = 0.0f;
  float x0 = 0.0f;
  float x1 = 0.0f;
  float x2 = 0.0f;
  float x3 = 0.0f;
  float x4 = 0.0f;

  /* TODO: check overflow */

  x0 = dev->trim.xyz1 * 16384.0f / data[3];
  tmp = x0 - 16384.0f;
  x1 = dev->trim.xy2 * (tmp * tmp / 268435456.0f);
  x2 = x1 + tmp * dev->trim.xy1 / 16384.0f;
  x3 = dev->trim.x2 + 160.0f;
  x4 = data[0] * (x2 + 256.0f) * x3;
  tmp = ((x4 / 8192.0f) + dev->trim.x1 * 8.0f) / 16.0f;

  return tmp;
}

/****************************************************************************
 * Name: bmm150_gety
 ****************************************************************************/

static float bmm150_gety(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data)
{
  float tmp = 0.0f;
  float y0 = 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;
  float y3 = 0.0f;
  float y4 = 0.0f;

  /* TODO: check overflow */

  y0 = dev->trim.xyz1 * 16384.0f / data[3];
  tmp = y0 - 16384.0f;
  y1 = dev->trim.xy2 * (tmp * tmp / 268435456.0f);
  y2 = y1 + tmp * dev->trim.xy1 / 16384.0f;
  y3 = dev->trim.y2 + 160.0f;
  y4 = data[1] * (y2 + 256.0f) * y3;
  tmp = ((y4 / 8192.0f) + dev->trim.y1 * 8.0f) / 16.0f;

  return tmp;
}

/****************************************************************************
 * Name: bmm150_getz
 ****************************************************************************/

static float bmm150_getz(FAR struct bmm150_sensor_dev_s *dev,
                         FAR int16_t *data)
{
  float tmp = 0.0f;
  float z0 = 0.0f;
  float z1 = 0.0f;
  float z2 = 0.0f;
  float z3 = 0.0f;
  float z4 = 0.0f;
  float z5 = 0.0f;

  /* TODO: check overflow */

  z0 = data[2] - dev->trim.z4;
  z1 = data[3] - dev->trim.xyz1;
  z2 = dev->trim.z3 * z1;
  z3 = dev->trim.z1 * data[3] / 32768.0f;
  z4 = dev->trim.z2 + z3;
  z5 = z0 * 121072.0f - z2;
  tmp = (z5 / (z4 * 4.0f)) / 16.0f;

  return tmp;
}

#ifndef CONFIG_SENSORS_BMM150_POLL
/****************************************************************************
 * Name: bmm150_fetch
 ****************************************************************************/

static int bmm150_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  FAR struct bmm150_sensor_dev_s *dev =
    (FAR struct bmm150_sensor_dev_s *)lower;
  struct sensor_mag           mag_data;
  int16_t                     data[4];
  uint64_t                    now  = sensor_get_timestamp();
  int                         ret  = 0;

  nxmutex_lock(&dev->lock);

  if (!dev->enabled)
    {
      ret = -EACCES;
      goto errout;
    }

  if (buflen != sizeof(mag_data))
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get data */

  do
    {
      bmm150_getregs(dev, BMM150_DATA_X_LSB, (FAR uint8_t *)data, 8);
    }
  while (!(data[3] & 1));

  /* Correct data */

  data[0] >>= 3;
  data[1] >>= 3;
  data[2] >>= 1;
  data[3] >>= 2;

  /* Get compensated data */

  mag_data.timestamp = now;
  mag_data.x = bmm150_getx(dev, data);
  mag_data.y = bmm150_gety(dev, data);
  mag_data.z = bmm150_getz(dev, data);

  memcpy(buffer, &mag_data, sizeof(mag_data));
  ret = sizeof(mag_data);

errout:
  nxmutex_unlock(&dev->lock);
  return ret;
}
#endif

/****************************************************************************
 * Name: bmm150_set_interval
 ****************************************************************************/

static int bmm150_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR unsigned long *interval)
{
#ifdef CONFIG_SENSORS_BMM150_POLL
  FAR struct bmm150_sensor_dev_s *dev =
    (FAR struct bmm150_sensor_dev_s *)lower;

  dev->interval = *interval;
#endif

  return OK;
}

#ifdef CONFIG_SENSORS_BMM150_POLL
/****************************************************************************
 * Name: bmm150_thread
 *
 * Description: Thread for performing interval measurement cycle and data
 *              read.
 *
 * Parameter:
 *   argc - Number opf arguments
 *   argv - Pointer to argument list
 *
 ****************************************************************************/

static int bmm150_thread(int argc, FAR char **argv)
{
  FAR struct bmm150_sensor_dev_s *dev =
    (FAR struct bmm150_sensor_dev_s *)((uintptr_t)strtoul(argv[1], NULL,
                                                        16));
  struct sensor_mag           mag_data;
  int16_t                     data[4];
  int                         ret     = 0;

  while (true)
    {
      if (!dev->enabled)
        {
          /* Waiting to be woken up */

          ret = nxsem_wait(&dev->run);
          if (ret < 0)
            {
              continue;
            }
        }

      if (dev->enabled)
        {
          /* Get data */

          bmm150_getregs(dev, BMM150_DATA_X_LSB, (FAR uint8_t *)data, 8);

          /* Correct data */

          data[0] >>= 3;
          data[1] >>= 3;
          data[2] >>= 1;
          data[3] >>= 2;

          /* Get compensated data */

          mag_data.timestamp = sensor_get_timestamp();
          mag_data.x = bmm150_getx(dev, data);
          mag_data.y = bmm150_gety(dev, data);
          mag_data.z = bmm150_getz(dev, data);
          dev->lower.push_event(dev->lower.priv,
                                &mag_data, sizeof(mag_data));
        }

      /* Sleeping thread before fetching the next sensor data */

      nxsig_usleep(dev->interval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmm150_register_uorb
 *
 * Description:
 *   Register the BMM150 uorb device as 'devpath'
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   config  - device configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmm150_register_uorb(int devno, FAR struct bmm150_config_s *config)
{
  FAR struct bmm150_sensor_dev_s *dev = NULL;
#ifdef CONFIG_SENSORS_BMM150_POLL
  FAR char                       *argv[2];
  char                            arg1[32];
#endif
  int                             ret = OK;

  /* Without config info, we can't do anything. */

  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the device structure. */

  dev = kmm_malloc(sizeof(struct bmm150_sensor_dev_s));
  if (dev == NULL)
    {
      snerr("ERROR: Failed to allocate bmm150 device instance\n");
      return -ENOMEM;
    }

  memset(dev, 0, sizeof(struct bmm150_sensor_dev_s));
  nxmutex_init(&dev->lock);
#ifdef CONFIG_SENSORS_BMM150_POLL
  nxsem_init(&dev->run, 0, 0);
#endif

  /* Configure dev */

  dev->config.i2c  = config->i2c;
  dev->config.addr = config->addr;
  dev->freq        = BMM150_I2C_FREQ;

  /*  Register sensor */

  dev->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  dev->lower.ops  = &g_bmm150_sensor_ops;
#ifdef CONFIG_SENSORS_BMM150_POLL
  dev->enabled    = false;
  dev->interval   = CONFIG_SENSORS_BMM150_POLL_INTERVAL;
#endif

  /* Check Device ID */

  ret = bmm150_initialzie(dev);
  if (ret < 0)
    {
      snerr("Failed to initialize driver: %d\n", ret);
      return ret;
    }

  /* Regsiter driver */

  ret = sensor_register(&dev->lower, devno);
  if (ret < 0)
    {
      goto mag_err;
    }

#ifdef CONFIG_SENSORS_BMM150_POLL
  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", dev);
  argv[0] = arg1;
  argv[1] = NULL;

  ret = kthread_create("bmm150_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_BMM150_THREAD_STACKSIZE,
                       bmm150_thread,
                       argv);
  if (ret < 0)
    {
      goto thr_err;
    }
#endif

  return ret;

#ifdef CONFIG_SENSORS_BMM150_POLL
  thr_err:
#endif
  sensor_unregister(&dev->lower, devno);
mag_err:
  kmm_free(dev);
  return ret;
}
