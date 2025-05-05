/****************************************************************************
 * drivers/sensors/hts221_uorb.c
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
#include <stdio.h>
#include <string.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/hts221.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register map */

#define HTS221_WHO_AM_I             0x0f
#define HTS221_AV_CONF              0x10
#define HTS221_CTRL_REG1            0x20
#define HTS221_CTRL_REG2            0x21
#define HTS221_CTRL_REG3            0x22
#define HTS221_STATUS_REG           0x27
#define HTS221_HUM_OUT_L            0x28
#define HTS221_HUM_OUT_H            0x29
#define HTS221_TEMP_OUT_L           0x2a
#define HTS221_TEMP_OUT_H           0x2b

/* Calibration registers */

#define HTS221_CALIB_H0_RH_X2       0x30
#define HTS221_CALIB_H1_RH_X2       0x31
#define HTS221_CALIB_T0_DEGC_X8     0x32
#define HTS221_CALIB_T1_DEGC_X8     0x33
#define HTS221_CALIB_T1_T0_MSB      0x35
#define HTS221_CALIB_H0T0_OUT_L     0x36
#define HTS221_CALIB_H0T0_OUT_H     0x37
#define HTS221_CALIB_H1T0_OUT_L     0x3a
#define HTS221_CALIB_H1T0_OUT_H     0x3b
#define HTS221_CALIB_T0_OUT_L       0x3c
#define HTS221_CALIB_T0_OUT_H       0x3d
#define HTS221_CALIB_T1_OUT_L       0x3e
#define HTS221_CALIB_T1_OUT_H       0x3f

/* HTS221_AV_CONF: AVGH in bits 0-2, AVGT in bits 3-5 */

#define HTS221_AV_CONF_AVGH_SHIFT   0
#define HTS221_AV_CONF_AVGT_SHIFT   3
#define HTS221_AV_CONF_MASK         0x3f
#define HTS221_AVGT_DEFAULT         3        /* 16 samples */
#define HTS221_AVGH_DEFAULT         3        /* 32 samples */
#define HTS221_AVG_CODES            8

/* HTS221_CTRL_REG1 */

#define HTS221_CTRL_REG1_PD         (1 << 7)
#define HTS221_CTRL_REG1_BDU        (1 << 2)
#define HTS221_CTRL_REG1_ODR_MASK   0x03
#define HTS221_ODR_ONESHOT          0
#define HTS221_ODR_1HZ              1
#define HTS221_ODR_7HZ              2
#define HTS221_ODR_12_5HZ           3

/* HTS221_CTRL_REG2 */

#define HTS221_CTRL_REG2_BOOT       (1 << 7)
#define HTS221_CTRL_REG2_ONE_SHOT   (1 << 0)

/* HTS221_CTRL_REG3 */

#define HTS221_CTRL_REG3_DRDY_L_H   (1 << 7)
#define HTS221_CTRL_REG3_PP_OD      (1 << 6)
#define HTS221_CTRL_REG3_DRDY_EN    (1 << 2)

/* HTS221_STATUS_REG */

#define HTS221_STATUS_REG_H_DA      (1 << 1)
#define HTS221_STATUS_REG_T_DA      (1 << 0)
#define HTS221_STATUS_REG_DA        (HTS221_STATUS_REG_H_DA | \
                                     HTS221_STATUS_REG_T_DA)

#define HTS221_DEVID                0xbc

#define HTS221_BOOT_RETRIES         5
#define HTS221_ONE_SHOT_RETRIES     10

/* Output data rate periods in us */

#define HTS221_PERIOD_12_5HZ        80000
#define HTS221_PERIOD_7HZ           142857
#define HTS221_PERIOD_1HZ           1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Each measurement channel needs its own lower-half. */

struct hts221_sensor_s
{
  FAR struct sensor_lowerhalf_s lower;   /* Lower-half driver */
  FAR struct hts221_dev_s      *dev;     /* Parent device */
  bool                          enabled; /* Channel activated */
};

struct hts221_dev_s
{
  struct hts221_sensor_s   temp;     /* Temperature lower-half */
  struct hts221_sensor_s   hum;      /* Humidity lower-half */
  FAR struct i2c_master_s *i2c;      /* I2C interface */
  FAR hts221_config_t     *config;   /* Board callbacks, may be NULL */
  uint8_t                  addr;     /* I2C address */
  uint8_t                  avgt;     /* Temperature averaging code */
  uint8_t                  avgh;     /* Humidity averaging code */
  uint8_t                  odr;      /* Output data rate code */
  uint32_t                 interval; /* Requested interval in us */
  struct work_s            work;     /* Measurement work */
  mutex_t                  lock;     /* Serialize device access */
  bool                     running;  /* Sampling in progress */
  struct
  {
    int16_t t0_out;
    int16_t t1_out;
    int16_t h0_t0_out;
    int16_t h1_t0_out;
    unsigned int t0_x8:10;
    unsigned int t1_x8:10;
    uint8_t h0_x2;
    uint8_t h1_x2;
  } calib;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int hts221_read_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval);
static int hts221_write_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                            uint8_t regval);
static int hts221_modify_reg(FAR struct hts221_dev_s *priv,
                             uint8_t regaddr, uint8_t mask, uint8_t value);
static int hts221_load_calibration(FAR struct hts221_dev_s *priv);
static int hts221_boot(FAR struct hts221_dev_s *priv);
static bool hts221_use_irq(FAR struct hts221_dev_s *priv);
static int hts221_apply_settings(FAR struct hts221_dev_s *priv);
static int hts221_board_power(FAR struct hts221_dev_s *priv, bool on);
static int hts221_power(FAR struct hts221_dev_s *priv, bool on);
static int hts221_wait_data(FAR struct hts221_dev_s *priv, int retries);
static int hts221_read_raw(FAR struct hts221_dev_s *priv,
                           FAR int16_t *t_out, FAR int16_t *h_out);
static sensor_data_t hts221_interpolate(int y0, int y1, int x0, int x1,
                                        int x, int div);
static void hts221_push(FAR struct hts221_dev_s *priv, int16_t t_out,
                        int16_t h_out, bool temp_en, bool hum_en);
static void hts221_worker(FAR void *arg);
static int hts221_int_handler(int irq, FAR void *context, FAR void *arg);
static int hts221_start(FAR struct hts221_dev_s *priv);
static void hts221_stop(FAR struct hts221_dev_s *priv);
static int hts221_set_oversampling(FAR struct hts221_sensor_s *sensor,
                                   unsigned long samples);

/* Sensor lower-half operations */

static int hts221_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int hts221_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
static int hts221_selftest(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, unsigned long arg);
static int hts221_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info);
static int hts221_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_hts221_ops =
{
  .activate     = hts221_activate,
  .set_interval = hts221_set_interval,
  .selftest     = hts221_selftest,
  .get_info     = hts221_get_info,
  .control      = hts221_control,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hts221_read_reg
 ****************************************************************************/

static int hts221_read_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                           FAR uint8_t *regval)
{
  struct i2c_config_s config;

  config.frequency = CONFIG_HTS221_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  return i2c_writeread(priv->i2c, &config, &regaddr, 1, regval, 1);
}

/****************************************************************************
 * Name: hts221_write_reg
 ****************************************************************************/

static int hts221_write_reg(FAR struct hts221_dev_s *priv, uint8_t regaddr,
                            uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buf[2];

  config.frequency = CONFIG_HTS221_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  buf[0] = regaddr;
  buf[1] = regval;

  return i2c_write(priv->i2c, &config, buf, 2);
}

/****************************************************************************
 * Name: hts221_modify_reg
 ****************************************************************************/

static int hts221_modify_reg(FAR struct hts221_dev_s *priv,
                             uint8_t regaddr, uint8_t mask, uint8_t value)
{
  uint8_t regval;
  int ret;

  ret = hts221_read_reg(priv, regaddr, &regval);
  if (ret < 0)
    {
      return ret;
    }

  regval &= ~mask;
  regval |= value & mask;

  return hts221_write_reg(priv, regaddr, regval);
}

/****************************************************************************
 * Name: hts221_load_calibration
 *
 * Description:
 *   Read the factory calibration coefficients used to convert raw readings
 *   into physical units.
 *
 ****************************************************************************/

static int hts221_load_calibration(FAR struct hts221_dev_s *priv)
{
  uint8_t t0_degc_x8   = 0;
  uint8_t t1_degc_x8   = 0;
  uint8_t t1_t0_msb    = 0;
  uint8_t t0_out_lsb   = 0;
  uint8_t t0_out_msb   = 0;
  uint8_t t1_out_lsb   = 0;
  uint8_t t1_out_msb   = 0;
  uint8_t h0_rh_x2     = 0;
  uint8_t h1_rh_x2     = 0;
  uint8_t h0t0_out_lsb = 0;
  uint8_t h0t0_out_msb = 0;
  uint8_t h1t0_out_lsb = 0;
  uint8_t h1t0_out_msb = 0;
  int ret;

  ret = hts221_read_reg(priv, HTS221_CALIB_T0_DEGC_X8, &t0_degc_x8);
  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_DEGC_X8, &t1_degc_x8);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_T0_MSB, &t1_t0_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T0_OUT_L, &t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T0_OUT_H, &t0_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_OUT_L, &t1_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_T1_OUT_H, &t1_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0_RH_X2, &h0_rh_x2);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1_RH_X2, &h1_rh_x2);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0T0_OUT_L, &h0t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H0T0_OUT_H, &h0t0_out_msb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1T0_OUT_L, &h1t0_out_lsb);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_CALIB_H1T0_OUT_H, &h1t0_out_msb);
    }

  if (ret < 0)
    {
      snerr("ERROR: Failed to read HTS221 calibration data: %d\n", ret);
      return ret;
    }

  priv->calib.t0_x8     = t0_degc_x8 | ((t1_t0_msb & 0x3) << 8);
  priv->calib.t1_x8     = t1_degc_x8 | ((t1_t0_msb & (0x3 << 2)) << (8 - 2));
  priv->calib.t0_out    = t0_out_lsb | (t0_out_msb << 8);
  priv->calib.t1_out    = t1_out_lsb | (t1_out_msb << 8);
  priv->calib.h0_x2     = h0_rh_x2;
  priv->calib.h1_x2     = h1_rh_x2;
  priv->calib.h0_t0_out = h0t0_out_lsb | (h0t0_out_msb << 8);
  priv->calib.h1_t0_out = h1t0_out_lsb | (h1t0_out_msb << 8);

  /* As calibration coefficients are unique to each sensor device,
   * they are a good candidate to be added to the entropy pool.
   */

  up_rngaddentropy(RND_SRC_HW, (FAR uint32_t *)&priv->calib,
                   sizeof(priv->calib) / sizeof(uint32_t));

  return OK;
}

/****************************************************************************
 * Name: hts221_boot
 *
 * Description:
 *   Reboot the sensor to reload the trimming parameters.  Resets the
 *   control registers, so it is done before they are programmed.
 *
 ****************************************************************************/

static int hts221_boot(FAR struct hts221_dev_s *priv)
{
  uint8_t regval;
  int retries = HTS221_BOOT_RETRIES;
  int ret;

  ret = hts221_write_reg(priv, HTS221_CTRL_REG2, HTS221_CTRL_REG2_BOOT);
  if (ret < 0)
    {
      return ret;
    }

  do
    {
      nxsig_usleep(10000);
      ret = hts221_read_reg(priv, HTS221_CTRL_REG2, &regval);
      if (ret < 0)
        {
          return ret;
        }
    }
  while ((regval & HTS221_CTRL_REG2_BOOT) && --retries);

  if (regval & HTS221_CTRL_REG2_BOOT)
    {
      return -ETIMEDOUT;
    }

  /* After boot bit is cleared, wait additional 5 msec as recommended in
   * HTS221 application note.
   */

  up_mdelay(5);
  return OK;
}

/****************************************************************************
 * Name: hts221_use_irq
 *
 * Description:
 *   True when the data-ready interrupt drives the sampling: the board
 *   provides the interrupt and the requested rate maps to a continuous
 *   output data rate.
 *
 ****************************************************************************/

static bool hts221_use_irq(FAR struct hts221_dev_s *priv)
{
  return priv->config != NULL && priv->config->irq_attach != NULL &&
         priv->odr != HTS221_ODR_ONESHOT;
}

/****************************************************************************
 * Name: hts221_apply_settings
 *
 * Description:
 *   Program averaging, data-ready pin and output data rate.  The PD bit is
 *   left untouched.
 *
 ****************************************************************************/

static int hts221_apply_settings(FAR struct hts221_dev_s *priv)
{
  uint8_t regval;
  int ret;

  regval = (priv->avgh << HTS221_AV_CONF_AVGH_SHIFT) |
           (priv->avgt << HTS221_AV_CONF_AVGT_SHIFT);
  ret = hts221_modify_reg(priv, HTS221_AV_CONF, HTS221_AV_CONF_MASK,
                          regval);
  if (ret < 0)
    {
      return ret;
    }

  regval = 0;
  if (hts221_use_irq(priv))
    {
      regval |= HTS221_CTRL_REG3_DRDY_EN;
      regval |= priv->config->drdy_active_low ?
                HTS221_CTRL_REG3_DRDY_L_H : 0;
      regval |= priv->config->drdy_open_drain ?
                HTS221_CTRL_REG3_PP_OD : 0;
    }

  ret = hts221_write_reg(priv, HTS221_CTRL_REG3, regval);
  if (ret < 0)
    {
      return ret;
    }

  return hts221_modify_reg(priv, HTS221_CTRL_REG1,
                           HTS221_CTRL_REG1_ODR_MASK | HTS221_CTRL_REG1_BDU,
                           priv->odr | HTS221_CTRL_REG1_BDU);
}

/****************************************************************************
 * Name: hts221_board_power
 *
 * Description:
 *   Switch the sensor supply through the board callback, if any.
 *
 ****************************************************************************/

static int hts221_board_power(FAR struct hts221_dev_s *priv, bool on)
{
  if (priv->config != NULL && priv->config->set_power != NULL)
    {
      return priv->config->set_power(priv->config, on);
    }

  return OK;
}

/****************************************************************************
 * Name: hts221_power
 *
 * Description:
 *   Set or clear the PD bit.
 *
 ****************************************************************************/

static int hts221_power(FAR struct hts221_dev_s *priv, bool on)
{
  return hts221_modify_reg(priv, HTS221_CTRL_REG1, HTS221_CTRL_REG1_PD,
                           on ? HTS221_CTRL_REG1_PD : 0);
}

/****************************************************************************
 * Name: hts221_wait_data
 *
 * Description:
 *   Wait until both temperature and humidity samples are available.
 *
 ****************************************************************************/

static int hts221_wait_data(FAR struct hts221_dev_s *priv, int retries)
{
  uint8_t regval;
  int ret;

  for (; ; )
    {
      ret = hts221_read_reg(priv, HTS221_STATUS_REG, &regval);
      if (ret < 0)
        {
          return ret;
        }

      if ((regval & HTS221_STATUS_REG_DA) == HTS221_STATUS_REG_DA)
        {
          return OK;
        }

      if (--retries <= 0)
        {
          return -EAGAIN;
        }

      nxsig_usleep(5000);
    }
}

/****************************************************************************
 * Name: hts221_read_raw
 *
 * Description:
 *   Read the raw temperature and humidity output registers.
 *
 ****************************************************************************/

static int hts221_read_raw(FAR struct hts221_dev_s *priv,
                           FAR int16_t *t_out, FAR int16_t *h_out)
{
  uint8_t h_l;
  uint8_t h_h;
  uint8_t t_l;
  uint8_t t_h;
  int ret;

  ret = hts221_read_reg(priv, HTS221_HUM_OUT_L, &h_l);
  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_HUM_OUT_H, &h_h);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_TEMP_OUT_L, &t_l);
    }

  if (ret >= 0)
    {
      ret = hts221_read_reg(priv, HTS221_TEMP_OUT_H, &t_h);
    }

  if (ret < 0)
    {
      return ret;
    }

  *t_out = t_h << 8 | t_l;
  *h_out = h_h << 8 | h_l;

  add_sensor_randomness(*h_out ^ *t_out);
  return OK;
}

/****************************************************************************
 * Name: hts221_interpolate
 *
 * Description:
 *   Linear interpolation between the two factory calibration points
 *   (x0, y0) and (x1, y1) at x, with the result divided by div.
 *
 ****************************************************************************/

static sensor_data_t hts221_interpolate(int y0, int y1, int x0, int x1,
                                        int x, int div)
{
  sensor_data_t slope;
  int span  = x1 - x0;
  int delta = x - x0;
  int rise  = y1 - y0;

  /* Keep the operands within the b16 range */

  while (span > 16383 || span < -16383)
    {
      span  /= 2;
      delta /= 2;
    }

  slope = sensor_data_div(sensor_data_itof(rise), sensor_data_itof(span));

  return sensor_data_divi(sensor_data_add(sensor_data_itof(y0),
                                          sensor_data_muli(slope, delta)),
                          div);
}

/****************************************************************************
 * Name: hts221_push
 *
 * Description:
 *   Convert a raw sample with the factory calibration and push it to the
 *   enabled topics.  Called without the lock held, since the upper half
 *   takes its own lock in push_event.
 *
 ****************************************************************************/

static void hts221_push(FAR struct hts221_dev_s *priv, int16_t t_out,
                        int16_t h_out, bool temp_en, bool hum_en)
{
  struct sensor_temp temp;
  struct sensor_humi humi;

  temp.timestamp   = sensor_get_timestamp();
  temp.temperature = hts221_interpolate(priv->calib.t0_x8,
                                        priv->calib.t1_x8,
                                        priv->calib.t0_out,
                                        priv->calib.t1_out, t_out, 8);
  humi.timestamp   = temp.timestamp;
  humi.humidity    = hts221_interpolate(priv->calib.h0_x2,
                                        priv->calib.h1_x2,
                                        priv->calib.h0_t0_out,
                                        priv->calib.h1_t0_out, h_out, 2);

  if (temp_en)
    {
      priv->temp.lower.push_event(priv->temp.lower.priv, &temp,
                                  sizeof(temp));
    }

  if (hum_en)
    {
      priv->hum.lower.push_event(priv->hum.lower.priv, &humi,
                                 sizeof(humi));
    }
}

/****************************************************************************
 * Name: hts221_worker
 *
 * Description:
 *   In one-shot mode trigger a conversion, wait for it and re-queue every
 *   interval.  With the data-ready interrupt the handler queues the worker
 *   and the sample is ready.
 *
 ****************************************************************************/

static void hts221_worker(FAR void *arg)
{
  FAR struct hts221_dev_s *priv = (FAR struct hts221_dev_s *)arg;
  bool temp_en;
  bool hum_en;
  int16_t t_out = 0;
  int16_t h_out = 0;
  int ret;

  nxmutex_lock(&priv->lock);

  if (!priv->running)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  if (!hts221_use_irq(priv))
    {
      /* Reading the output registers clears stale data-ready flags left by
       * a boot or a previous run, so that the wait below sees only the
       * conversion triggered here.
       */

      ret = hts221_read_raw(priv, &t_out, &h_out);
      if (ret >= 0)
        {
          ret = hts221_write_reg(priv, HTS221_CTRL_REG2,
                                 HTS221_CTRL_REG2_ONE_SHOT);
        }

      if (ret >= 0)
        {
          ret = hts221_wait_data(priv, HTS221_ONE_SHOT_RETRIES);
        }

      work_queue(LPWORK, &priv->work, hts221_worker, priv,
                 USEC2TICK(priv->interval));
    }
  else
    {
      ret = hts221_wait_data(priv, 1);
    }

  if (ret == OK)
    {
      ret = hts221_read_raw(priv, &t_out, &h_out);
    }

  temp_en = priv->temp.enabled;
  hum_en  = priv->hum.enabled;

  nxmutex_unlock(&priv->lock);

  if (ret == OK)
    {
      hts221_push(priv, t_out, h_out, temp_en, hum_en);
    }
  else if (ret != -EAGAIN)
    {
      snerr("ERROR: HTS221 measurement failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: hts221_int_handler
 ****************************************************************************/

static int hts221_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct hts221_dev_s *priv = (FAR struct hts221_dev_s *)arg;

  DEBUGASSERT(priv != NULL);

  if (priv->config->irq_clear != NULL)
    {
      priv->config->irq_clear(priv->config);
    }

  return work_queue(LPWORK, &priv->work, hts221_worker, priv, 0);
}

/****************************************************************************
 * Name: hts221_start
 *
 * Description:
 *   Power on, program the settings and start sampling.  Called with the
 *   lock held.
 *
 ****************************************************************************/

static int hts221_start(FAR struct hts221_dev_s *priv)
{
  int ret;

  ret = hts221_board_power(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = hts221_apply_settings(priv);
  if (ret >= 0)
    {
      ret = hts221_power(priv, true);
    }

  if (ret < 0)
    {
      hts221_board_power(priv, false);
      return ret;
    }

  priv->running = true;

  if (hts221_use_irq(priv))
    {
      if (priv->config->irq_clear != NULL)
        {
          priv->config->irq_clear(priv->config);
        }

      if (priv->config->irq_enable != NULL)
        {
          priv->config->irq_enable(priv->config, true);
        }
    }

  /* Take the first sample now; it also drains a sample that may already
   * be waiting so that the DRDY line goes low.
   */

  return work_queue(LPWORK, &priv->work, hts221_worker, priv, 0);
}

/****************************************************************************
 * Name: hts221_stop
 *
 * Description:
 *   Stop sampling and power off.  Called with the lock held.
 *
 ****************************************************************************/

static void hts221_stop(FAR struct hts221_dev_s *priv)
{
  priv->running = false;
  work_cancel(LPWORK, &priv->work);

  if (priv->config != NULL && priv->config->irq_enable != NULL)
    {
      priv->config->irq_enable(priv->config, false);
    }

  hts221_write_reg(priv, HTS221_CTRL_REG2, 0);
  hts221_power(priv, false);
  hts221_board_power(priv, false);
}

/****************************************************************************
 * Name: hts221_set_oversampling
 *
 * Description:
 *   Set the number of internal samples averaged for one output: 2 to 256
 *   for temperature, 4 to 512 for humidity, powers of two.  Called with
 *   the lock held.
 *
 ****************************************************************************/

static int hts221_set_oversampling(FAR struct hts221_sensor_s *sensor,
                                   unsigned long samples)
{
  FAR struct hts221_dev_s *priv = sensor->dev;
  unsigned long base;
  uint8_t code;

  base = sensor->lower.type == SENSOR_TYPE_AMBIENT_TEMPERATURE ? 2 : 4;

  for (code = 0; code < HTS221_AVG_CODES; code++)
    {
      if (samples == (base << code))
        {
          break;
        }
    }

  if (code == HTS221_AVG_CODES)
    {
      return -EINVAL;
    }

  if (sensor->lower.type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      priv->avgt = code;
    }
  else
    {
      priv->avgh = code;
    }

  return hts221_modify_reg(priv, HTS221_AV_CONF, HTS221_AV_CONF_MASK,
                           (priv->avgh << HTS221_AV_CONF_AVGH_SHIFT) |
                           (priv->avgt << HTS221_AV_CONF_AVGT_SHIFT));
}

/****************************************************************************
 * Name: hts221_activate
 ****************************************************************************/

static int hts221_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct hts221_sensor_s *sensor =
      container_of(lower, FAR struct hts221_sensor_s, lower);
  FAR struct hts221_dev_s *priv = sensor->dev;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  sensor->enabled = enable;

  if (enable && !priv->running)
    {
      ret = hts221_start(priv);
    }
  else if (!enable && priv->running &&
           !priv->temp.enabled && !priv->hum.enabled)
    {
      hts221_stop(priv);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: hts221_set_interval
 *
 * Description:
 *   Without the data-ready interrupt the part is polled in one-shot mode
 *   every interval.  With it the interval selects the continuous output
 *   data rate: 12.5 Hz, 7 Hz or 1 Hz.
 *
 ****************************************************************************/

static int hts221_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct hts221_sensor_s *sensor =
      container_of(lower, FAR struct hts221_sensor_s, lower);
  FAR struct hts221_dev_s *priv = sensor->dev;
  uint8_t odr = HTS221_ODR_ONESHOT;
  int ret = OK;

  if (priv->config != NULL && priv->config->irq_attach != NULL)
    {
      if (*period_us <= HTS221_PERIOD_12_5HZ)
        {
          odr = HTS221_ODR_12_5HZ;
          *period_us = HTS221_PERIOD_12_5HZ;
        }
      else if (*period_us <= HTS221_PERIOD_7HZ)
        {
          odr = HTS221_ODR_7HZ;
          *period_us = HTS221_PERIOD_7HZ;
        }
      else
        {
          odr = HTS221_ODR_1HZ;
        }
    }

  nxmutex_lock(&priv->lock);

  priv->interval = *period_us;

  if (odr != priv->odr)
    {
      priv->odr = odr;
      if (priv->running)
        {
          hts221_stop(priv);
          ret = hts221_start(priv);
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: hts221_selftest
 *
 * Description:
 *   Check that the part answers with its device ID.
 *
 ****************************************************************************/

static int hts221_selftest(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, unsigned long arg)
{
  FAR struct hts221_sensor_s *sensor =
      container_of(lower, FAR struct hts221_sensor_s, lower);
  FAR struct hts221_dev_s *priv = sensor->dev;
  uint8_t id = 0;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = hts221_read_reg(priv, HTS221_WHO_AM_I, &id);
  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      return ret;
    }

  return id == HTS221_DEVID ? OK : -ENODEV;
}

/****************************************************************************
 * Name: hts221_get_info
 ****************************************************************************/

static int hts221_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info)
{
  memset(info, 0, sizeof(*info));

  info->version   = 0;
  info->power     = sensor_data_ftof(0.002f);     /* 2 uA at 1 Hz */
  info->min_delay = HTS221_PERIOD_12_5HZ;
  info->max_delay = HTS221_PERIOD_1HZ;
  strlcpy(info->name, "HTS221", sizeof(info->name));
  strlcpy(info->vendor, "STMicroelectronics", sizeof(info->vendor));

  if (lower->type == SENSOR_TYPE_AMBIENT_TEMPERATURE)
    {
      info->max_range  = sensor_data_itof(120);
      info->resolution = sensor_data_ftof(0.016f); /* 64 LSB / degC */
    }
  else
    {
      info->max_range  = sensor_data_itof(100);
      info->resolution = sensor_data_ftof(0.004f); /* 256 LSB / %rH */
    }

  return OK;
}

/****************************************************************************
 * Name: hts221_control
 *
 * Description:
 *   SNIOC_OVERSAMPLING - number of averaged samples for this topic
 *
 ****************************************************************************/

static int hts221_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct hts221_sensor_s *sensor =
      container_of(lower, FAR struct hts221_sensor_s, lower);
  FAR struct hts221_dev_s *priv = sensor->dev;
  int ret;

  nxmutex_lock(&priv->lock);

  switch (cmd)
    {
      case SNIOC_OVERSAMPLING:
        ret = hts221_set_oversampling(sensor, arg);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hts221_register_uorb
 *
 * Description:
 *   Register the HTS221 humidity and temperature sensor as UORB devices.
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path.
 *   i2c    - The I2C bus driver instance.
 *   addr   - The I2C address of the HTS221.
 *   config - Board callbacks for the data-ready interrupt and power, or
 *            NULL.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hts221_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr, FAR hts221_config_t *config)
{
  FAR struct hts221_dev_s *priv;
  uint8_t id;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(struct hts221_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate HTS221 instance\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->config   = config;
  priv->avgt     = HTS221_AVGT_DEFAULT;
  priv->avgh     = HTS221_AVGH_DEFAULT;
  priv->odr      = HTS221_ODR_ONESHOT;
  priv->interval = HTS221_PERIOD_1HZ;
  nxmutex_init(&priv->lock);

  ret = hts221_board_power(priv, true);
  if (ret < 0)
    {
      snerr("ERROR: Failed to power on HTS221: %d\n", ret);
      goto errout;
    }

  ret = hts221_read_reg(priv, HTS221_WHO_AM_I, &id);
  if (ret < 0 || id != HTS221_DEVID)
    {
      snerr("ERROR: HTS221 not found: ret=%d id=0x%02x\n", ret, id);
      ret = ret < 0 ? ret : -ENODEV;
      goto errout_power;
    }

  ret = hts221_boot(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to boot HTS221: %d\n", ret);
      goto errout_power;
    }

  ret = hts221_load_calibration(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to calibrate HTS221: %d\n", ret);
      goto errout_power;
    }

  hts221_board_power(priv, false);

  if (config != NULL && config->irq_attach != NULL)
    {
      if (config->irq_clear != NULL)
        {
          config->irq_clear(config);
        }

      ret = config->irq_attach(config, hts221_int_handler, priv);
      if (ret < 0)
        {
          snerr("ERROR: Failed to attach HTS221 interrupt: %d\n", ret);
          goto errout;
        }

      if (config->irq_enable != NULL)
        {
          config->irq_enable(config, false);
        }
    }

  /* Register the temperature lower-half */

  priv->temp.dev           = priv;
  priv->temp.lower.ops     = &g_hts221_ops;
  priv->temp.lower.type    = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  priv->temp.lower.nbuffer = 1;

  ret = sensor_register(&priv->temp.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register HTS221 temperature: %d\n", ret);
      goto errout;
    }

  /* Register the humidity lower-half */

  priv->hum.dev           = priv;
  priv->hum.lower.ops     = &g_hts221_ops;
  priv->hum.lower.type    = SENSOR_TYPE_RELATIVE_HUMIDITY;
  priv->hum.lower.nbuffer = 1;

  ret = sensor_register(&priv->hum.lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register HTS221 humidity: %d\n", ret);
      sensor_unregister(&priv->temp.lower, devno);
      goto errout;
    }

  sninfo("HTS221 registered\n");
  return OK;

errout_power:
  hts221_board_power(priv, false);
errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
