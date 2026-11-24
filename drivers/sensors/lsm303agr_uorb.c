/****************************************************************************
 * drivers/sensors/lsm303agr_uorb.c
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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <string.h>
#include <nuttx/debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/sensors/lsm303agr.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM303AGR_I2C_FREQUENCY CONFIG_SENSORS_LSM303AGR_UORB_I2C_FREQUENCY

#define LSM303AGR_ACCEL_ADDR    0x19
#define LSM303AGR_MAG_ADDR      0x1e

/* Accelerometer registers */

#define REG_OUT_TEMP_L_A        0x0c
#define REG_WHO_AM_I_A          0x0f
#define REG_TEMP_CFG_REG_A      0x1f
#define REG_CTRL_REG1_A         0x20
#define REG_CTRL_REG4_A         0x23
#define REG_OUT_X_L_A           0x28

/* Magnetometer registers */

#define REG_WHO_AM_I_M          0x4f
#define REG_CFG_REG_A_M         0x60
#define REG_CFG_REG_C_M         0x62
#define REG_OUTX_L_REG_M        0x68

#define WHO_AM_I_A_VAL          0x33
#define WHO_AM_I_M_VAL          0x40

/* The accelerometer increments the register address in a multi-byte
 * transfer only when bit 7 of the address is set.
 */

#define REG_AUTOINC             0x80

/* Register bits */

#define CTRL_REG1_A_XYZEN       0x07         /* X, Y and Z axes enabled */
#define CTRL_REG1_A_ODR_SHIFT   4
#define CTRL_REG4_A_HR          (1 << 3)     /* High-resolution mode */
#define CTRL_REG4_A_FS_SHIFT    4
#define CTRL_REG4_A_BDU         (1 << 7)     /* Block data update */
#define TEMP_CFG_REG_A_TEMP_EN  (3 << 6)     /* Temperature sensor on */

#define CFG_REG_A_M_MD_CONT     0x00         /* Continuous mode */
#define CFG_REG_A_M_MD_IDLE     0x03         /* Idle mode */
#define CFG_REG_A_M_ODR_SHIFT   2
#define CFG_REG_A_M_COMP_TEMP   (1 << 7)     /* Temperature compensation */
#define CFG_REG_C_M_BDU         (1 << 4)     /* Block data update */

#define STANDARD_GRAVITY        9.80665f     /* m/s^2 */

/* Magnetometer sensitivity (1.5 milligauss/LSB = 0.15 uT/LSB) */

#define MAG_SCALE               sensor_data_ftof(0.15f)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lsm303agr_odr_s
{
  uint8_t  code;      /* ODR field of CTRL_REG1_A or CFG_REG_A_M */
  uint32_t interval;  /* Sample interval in microseconds */
};

struct lsm303agr_fs_s
{
  uint8_t       code;  /* CTRL_REG4_A FS field */
  uint8_t       range; /* Full scale in g */
  sensor_data_t scale; /* m/s^2 per LSB in high-resolution mode */
};

struct lsm303agr_dev_s;

struct lsm303agr_sens_s
{
  struct sensor_lowerhalf_s lower;  /* uORB lower half */
  FAR struct lsm303agr_dev_s *dev;  /* Owning device */

  /* Data rates of this sensor */

  FAR const struct lsm303agr_odr_s *odrs;
  uint8_t nodr;                     /* Number of data rates */
  uint8_t odr;                      /* Index into odrs */
  bool enabled;                     /* Sampling is on */
  uint64_t next;                    /* Time of the next sample */
};

struct lsm303agr_dev_s
{
  struct lsm303agr_sens_s accel;    /* Accelerometer */
  struct lsm303agr_sens_s mag;      /* Magnetometer */
  FAR struct i2c_master_s *i2c;     /* I2C interface */
  mutex_t lock;                     /* Exclusive access to the device */
  sem_t run;                        /* Wakes up the polling thread */
  uint8_t fs;                       /* Index into g_fs */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm303agr_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int lsm303agr_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us);
static int lsm303agr_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info);
static int lsm303agr_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Accelerometer data rates of the normal and high-resolution modes */

static const struct lsm303agr_odr_s g_accel_odr[] =
{
  { 1, 1000000 },     /* 1 Hz */
  { 2, 100000 },      /* 10 Hz */
  { 3, 40000 },       /* 25 Hz */
  { 4, 20000 },       /* 50 Hz */
  { 5, 10000 },       /* 100 Hz */
  { 6, 5000 },        /* 200 Hz */
  { 7, 2500 },        /* 400 Hz */
  { 9, 744 },         /* 1344 Hz */
};

/* Magnetometer data rates */

static const struct lsm303agr_odr_s g_mag_odr[] =
{
  { 0, 100000 },      /* 10 Hz */
  { 1, 50000 },       /* 20 Hz */
  { 2, 20000 },       /* 50 Hz */
  { 3, 10000 },       /* 100 Hz */
};

/* Full scales with the 12-bit high-resolution sensitivity (0.98, 1.95,
 * 3.9 and 11.72 mg/LSB)
 */

static const struct lsm303agr_fs_s g_fs[] =
{
  { 0, 2,  sensor_data_ftof(0.00098f * STANDARD_GRAVITY) },
  { 1, 4,  sensor_data_ftof(0.00195f * STANDARD_GRAVITY) },
  { 2, 8,  sensor_data_ftof(0.0039f * STANDARD_GRAVITY) },
  { 3, 16, sensor_data_ftof(0.01172f * STANDARD_GRAVITY) },
};

static const struct sensor_ops_s g_lsm303agr_ops =
{
  .activate     = lsm303agr_activate,
  .set_interval = lsm303agr_set_interval,
  .get_info     = lsm303agr_get_info,
  .control      = lsm303agr_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303agr_read
 *
 * Description:
 *   Read `len` bytes starting at register `reg` of the device at `addr`.
 *
 ****************************************************************************/

static int lsm303agr_read(FAR struct i2c_master_s *i2c, uint8_t addr,
                          uint8_t reg, FAR void *buf, size_t len)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = LSM303AGR_I2C_FREQUENCY;
  msg[0].addr      = addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = LSM303AGR_I2C_FREQUENCY;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  return I2C_TRANSFER(i2c, msg, 2);
}

/****************************************************************************
 * Name: lsm303agr_write
 *
 * Description:
 *   Write one register of the device at `addr`.
 *
 ****************************************************************************/

static int lsm303agr_write(FAR struct lsm303agr_dev_s *dev, uint8_t addr,
                           uint8_t reg, uint8_t val)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];

  buf[0] = reg;
  buf[1] = val;

  msg.frequency = LSM303AGR_I2C_FREQUENCY;
  msg.addr      = addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = sizeof(buf);

  return I2C_TRANSFER(dev->i2c, &msg, 1);
}

/****************************************************************************
 * Name: lsm303agr_accel_config
 *
 * Description:
 *   Program the data rate and full scale, or power the accelerometer down.
 *   Called with the device lock held.
 *
 ****************************************************************************/

static int lsm303agr_accel_config(FAR struct lsm303agr_dev_s *dev,
                                  bool enable)
{
  int ret;

  ret = lsm303agr_write(dev, LSM303AGR_ACCEL_ADDR, REG_CTRL_REG4_A,
                        CTRL_REG4_A_BDU | CTRL_REG4_A_HR |
                        (g_fs[dev->fs].code << CTRL_REG4_A_FS_SHIFT));
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm303agr_write(dev, LSM303AGR_ACCEL_ADDR, REG_TEMP_CFG_REG_A,
                        enable ? TEMP_CFG_REG_A_TEMP_EN : 0);
  if (ret < 0)
    {
      return ret;
    }

  return lsm303agr_write(dev, LSM303AGR_ACCEL_ADDR, REG_CTRL_REG1_A,
                         enable ? (g_accel_odr[dev->accel.odr].code <<
                                   CTRL_REG1_A_ODR_SHIFT) |
                                  CTRL_REG1_A_XYZEN : 0);
}

/****************************************************************************
 * Name: lsm303agr_mag_config
 *
 * Description:
 *   Program the data rate and start continuous measurement, or put the
 *   magnetometer in idle mode.  Called with the device lock held.
 *
 ****************************************************************************/

static int lsm303agr_mag_config(FAR struct lsm303agr_dev_s *dev,
                                bool enable)
{
  int ret;

  ret = lsm303agr_write(dev, LSM303AGR_MAG_ADDR, REG_CFG_REG_C_M,
                        CFG_REG_C_M_BDU);
  if (ret < 0)
    {
      return ret;
    }

  return lsm303agr_write(dev, LSM303AGR_MAG_ADDR, REG_CFG_REG_A_M,
                         CFG_REG_A_M_COMP_TEMP |
                         (g_mag_odr[dev->mag.odr].code <<
                          CFG_REG_A_M_ODR_SHIFT) |
                         (enable ? CFG_REG_A_M_MD_CONT :
                                   CFG_REG_A_M_MD_IDLE));
}

/****************************************************************************
 * Name: lsm303agr_config
 ****************************************************************************/

static int lsm303agr_config(FAR struct lsm303agr_sens_s *sens, bool enable)
{
  if (sens == &sens->dev->accel)
    {
      return lsm303agr_accel_config(sens->dev, enable);
    }

  return lsm303agr_mag_config(sens->dev, enable);
}

/****************************************************************************
 * Name: lsm303agr_push_accel
 *
 * Description:
 *   Read one accelerometer and temperature sample and push it to the upper
 *   half.  Called with the device lock held.
 *
 ****************************************************************************/

static int lsm303agr_push_accel(FAR struct lsm303agr_dev_s *dev)
{
  struct sensor_accel data;
  int16_t raw[3];
  int16_t temp;
  sensor_data_t scale;
  int ret;

  ret = lsm303agr_read(dev->i2c, LSM303AGR_ACCEL_ADDR,
                       REG_OUT_X_L_A | REG_AUTOINC, raw, sizeof(raw));
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm303agr_read(dev->i2c, LSM303AGR_ACCEL_ADDR,
                       REG_OUT_TEMP_L_A | REG_AUTOINC, &temp, sizeof(temp));
  if (ret < 0)
    {
      return ret;
    }

  /* The 12-bit samples are left-justified in little-endian 16-bit words */

  scale = g_fs[dev->fs].scale;

  data.timestamp   = sensor_get_timestamp();
  data.x           = sensor_data_muli(scale, (raw[0] >> 4));
  data.y           = sensor_data_muli(scale, (raw[1] >> 4));
  data.z           = sensor_data_muli(scale, (raw[2] >> 4));
  data.temperature = sensor_data_add(sensor_data_divi(sensor_data_itof(temp),
                                                      256),
                                     sensor_data_itof(25));
  data.status      = 0;

  return dev->accel.lower.push_event(dev->accel.lower.priv, &data,
                                     sizeof(data));
}

/****************************************************************************
 * Name: lsm303agr_push_mag
 *
 * Description:
 *   Read one magnetometer sample and push it to the upper half.  Called
 *   with the device lock held.
 *
 ****************************************************************************/

static int lsm303agr_push_mag(FAR struct lsm303agr_dev_s *dev)
{
  struct sensor_mag data;
  int16_t raw[3];
  int ret;

  ret = lsm303agr_read(dev->i2c, LSM303AGR_MAG_ADDR, REG_OUTX_L_REG_M,
                       raw, sizeof(raw));
  if (ret < 0)
    {
      return ret;
    }

  memset(&data, 0, sizeof(data));
  data.timestamp = sensor_get_timestamp();
  data.x         = sensor_data_muli(MAG_SCALE, raw[0]);
  data.y         = sensor_data_muli(MAG_SCALE, raw[1]);
  data.z         = sensor_data_muli(MAG_SCALE, raw[2]);

  return dev->mag.lower.push_event(dev->mag.lower.priv, &data,
                                   sizeof(data));
}

/****************************************************************************
 * Name: lsm303agr_push_data
 *
 * Description:
 *   Push one sample of `sens` if it is still enabled.
 *
 ****************************************************************************/

static int lsm303agr_push_data(FAR struct lsm303agr_sens_s *sens)
{
  FAR struct lsm303agr_dev_s *dev = sens->dev;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!sens->enabled)
    {
      ret = -EAGAIN;
    }
  else if (sens == &dev->accel)
    {
      ret = lsm303agr_push_accel(dev);
    }
  else
    {
      ret = lsm303agr_push_mag(dev);
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: lsm303agr_activate
 ****************************************************************************/

static int lsm303agr_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct lsm303agr_sens_s *sens =
    container_of(lower, struct lsm303agr_sens_s, lower);
  FAR struct lsm303agr_dev_s *dev = sens->dev;
  bool wake;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm303agr_config(sens, enable);
  if (ret >= 0)
    {
      wake = enable && !sens->enabled;
      sens->enabled = enable;
      if (wake)
        {
          /* The first sample is ready after one data rate interval */

          sens->next = sensor_get_timestamp() +
                       sens->odrs[sens->odr].interval;
          nxsem_post(&dev->run);
        }
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: lsm303agr_set_interval
 *
 * Description:
 *   Select the slowest data rate that is at least as fast as requested.
 *
 ****************************************************************************/

static int lsm303agr_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us)
{
  FAR struct lsm303agr_sens_s *sens =
    container_of(lower, struct lsm303agr_sens_s, lower);
  uint8_t odr;
  int ret;

  for (odr = 0; odr < sens->nodr - 1; odr++)
    {
      if (sens->odrs[odr].interval <= *period_us)
        {
          break;
        }
    }

  ret = nxmutex_lock(&sens->dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  sens->odr = odr;
  if (sens->enabled)
    {
      ret = lsm303agr_config(sens, true);
    }

  nxmutex_unlock(&sens->dev->lock);

  *period_us = sens->odrs[odr].interval;
  return ret;
}

/****************************************************************************
 * Name: lsm303agr_get_info
 ****************************************************************************/

static int lsm303agr_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info)
{
  FAR struct lsm303agr_sens_s *sens =
    container_of(lower, struct lsm303agr_sens_s, lower);
  FAR struct lsm303agr_dev_s *dev = sens->dev;

  memset(info, 0, sizeof(*info));
  strlcpy(info->name, "LSM303AGR", sizeof(info->name));
  strlcpy(info->vendor, "STMicro", sizeof(info->vendor));

  info->min_delay = sens->odrs[sens->nodr - 1].interval;
  info->max_delay = sens->odrs[0].interval;

  if (sens == &dev->accel)
    {
      info->max_range  = sensor_data_muli(sensor_data_ftof(STANDARD_GRAVITY),
                                          g_fs[dev->fs].range);
      info->resolution = g_fs[dev->fs].scale;
    }
  else
    {
      info->max_range  = sensor_data_muli(MAG_SCALE, INT16_MAX);
      info->resolution = MAG_SCALE;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_control
 ****************************************************************************/

static int lsm303agr_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct lsm303agr_sens_s *sens =
    container_of(lower, struct lsm303agr_sens_s, lower);
  FAR struct lsm303agr_dev_s *dev = sens->dev;
  bool accel = sens == &dev->accel;
  uint8_t fs;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case SNIOC_WHO_AM_I:
        if (arg == 0)
          {
            ret = -EINVAL;
            break;
          }

        ret = lsm303agr_read(dev->i2c,
                             accel ? LSM303AGR_ACCEL_ADDR :
                                     LSM303AGR_MAG_ADDR,
                             accel ? REG_WHO_AM_I_A : REG_WHO_AM_I_M,
                             (FAR uint8_t *)arg, 1);
        break;

      /* Accelerometer full scale in g: 2, 4, 8 or 16 */

      case SNIOC_SETFULLSCALE:
        if (!accel)
          {
            ret = -ENOTTY;
            break;
          }

        for (fs = 0; fs < nitems(g_fs); fs++)
          {
            if (g_fs[fs].range == arg)
              {
                break;
              }
          }

        if (fs == nitems(g_fs))
          {
            ret = -EINVAL;
            break;
          }

        dev->fs = fs;
        if (sens->enabled)
          {
            ret = lsm303agr_accel_config(dev, true);
          }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: lsm303agr_poll
 *
 * Description:
 *   Push a sample of `sens` when it is due.  Return the time in
 *   microseconds until its next sample, or UINT32_MAX if it is disabled.
 *
 ****************************************************************************/

static uint32_t lsm303agr_poll(FAR struct lsm303agr_sens_s *sens,
                               uint64_t now)
{
  if (!sens->enabled)
    {
      return UINT32_MAX;
    }

  if (now >= sens->next)
    {
      lsm303agr_push_data(sens);
      sens->next = now + sens->odrs[sens->odr].interval;
    }

  return sens->next - now;
}

/****************************************************************************
 * Name: lsm303agr_thread
 *
 * Description:
 *   Poll the enabled sensors at their configured data rates.
 *
 ****************************************************************************/

static int lsm303agr_thread(int argc, FAR char *argv[])
{
  FAR struct lsm303agr_dev_s *dev =
    (FAR struct lsm303agr_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  uint64_t now;
  uint32_t wait;

  while (true)
    {
      if (!dev->accel.enabled && !dev->mag.enabled)
        {
          nxsem_wait_uninterruptible(&dev->run);
          continue;
        }

      now  = sensor_get_timestamp();
      wait = MIN(lsm303agr_poll(&dev->accel, now),
                 lsm303agr_poll(&dev->mag, now));
      nxsched_usleep(wait);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_init_sens
 ****************************************************************************/

static void lsm303agr_init_sens(FAR struct lsm303agr_dev_s *dev,
                                FAR struct lsm303agr_sens_s *sens, int type,
                                FAR const struct lsm303agr_odr_s *odrs,
                                uint8_t nodr)
{
  sens->dev           = dev;
  sens->odrs          = odrs;
  sens->nodr          = nodr;
  sens->lower.ops     = &g_lsm303agr_ops;
  sens->lower.type    = type;
  sens->lower.nbuffer = CONFIG_SENSORS_LSM303AGR_UORB_BUFSIZE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303agr_register_uorb
 *
 * Description:
 *   Register the LSM303AGR as the uORB topics sensor_accel<devno> and
 *   sensor_mag<devno>.  Both sensors are polled.
 *
 * Input Parameters:
 *   i2c   - The I2C bus the LSM303AGR is on.
 *   devno - The device number of both topics.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm303agr_register_uorb(FAR struct i2c_master_s *i2c, int devno)
{
  FAR struct lsm303agr_dev_s *dev;
  FAR char *argv[2];
  char arg[32];
  uint8_t id = 0;
  int ret;

  DEBUGASSERT(i2c != NULL);

  ret = lsm303agr_read(i2c, LSM303AGR_ACCEL_ADDR, REG_WHO_AM_I_A, &id, 1);
  if (ret < 0 || id != WHO_AM_I_A_VAL)
    {
      snerr("ERROR: No LSM303AGR accelerometer: %d %02x\n", ret, id);
      return ret < 0 ? ret : -ENODEV;
    }

  ret = lsm303agr_read(i2c, LSM303AGR_MAG_ADDR, REG_WHO_AM_I_M, &id, 1);
  if (ret < 0 || id != WHO_AM_I_M_VAL)
    {
      snerr("ERROR: No LSM303AGR magnetometer: %d %02x\n", ret, id);
      return ret < 0 ? ret : -ENODEV;
    }

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->i2c = i2c;
  dev->fs  = 0;   /* +/- 2 g */

  lsm303agr_init_sens(dev, &dev->accel, SENSOR_TYPE_ACCELEROMETER,
                      g_accel_odr, nitems(g_accel_odr));
  dev->accel.odr = 1;   /* 10 Hz */

  lsm303agr_init_sens(dev, &dev->mag, SENSOR_TYPE_MAGNETIC_FIELD,
                      g_mag_odr, nitems(g_mag_odr));
  dev->mag.odr = 0;     /* 10 Hz */

  nxmutex_init(&dev->lock);
  nxsem_init(&dev->run, 0, 0);

  ret = sensor_register(&dev->accel.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register the accelerometer: %d\n", ret);
      goto err_free;
    }

  ret = sensor_register(&dev->mag.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register the magnetometer: %d\n", ret);
      goto err_accel;
    }

  snprintf(arg, sizeof(arg), "%p", dev);
  argv[0] = arg;
  argv[1] = NULL;

  ret = kthread_create("lsm303agr", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_LSM303AGR_UORB_THREAD_STACKSIZE,
                       lsm303agr_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to create the polling thread: %d\n", ret);
      goto err_mag;
    }

  return OK;

err_mag:
  sensor_unregister(&dev->mag.lower, devno);
err_accel:
  sensor_unregister(&dev->accel.lower, devno);
err_free:
  nxsem_destroy(&dev->run);
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return ret;
}
