/****************************************************************************
 * drivers/sensors/bmp280.c
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

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmp280.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BMP280)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMP280_ADDR         0x76
#define BMP280_FREQ         CONFIG_BMP280_I2C_FREQUENCY
#define DEVID               0x58

#define BMP280_DIG_T1_LSB   0x88
#define BMP280_DIG_T1_MSB   0x89
#define BMP280_DIG_T2_LSB   0x8a
#define BMP280_DIG_T2_MSB   0x8b
#define BMP280_DIG_T3_LSB   0x8c
#define BMP280_DIG_T3_MSB   0x8d
#define BMP280_DIG_P1_LSB   0x8e
#define BMP280_DIG_P1_MSB   0x8f
#define BMP280_DIG_P2_LSB   0x90
#define BMP280_DIG_P2_MSB   0x91
#define BMP280_DIG_P3_LSB   0x92
#define BMP280_DIG_P3_MSB   0x93
#define BMP280_DIG_P4_LSB   0x94
#define BMP280_DIG_P4_MSB   0x95
#define BMP280_DIG_P5_LSB   0x96
#define BMP280_DIG_P5_MSB   0x97
#define BMP280_DIG_P6_LSB   0x98
#define BMP280_DIG_P6_MSB   0x99
#define BMP280_DIG_P7_LSB   0x9a
#define BMP280_DIG_P7_MSB   0x9b
#define BMP280_DIG_P8_LSB   0x9c
#define BMP280_DIG_P8_MSB   0x9d
#define BMP280_DIG_P9_LSB   0x9e
#define BMP280_DIG_P9_MSB   0x9f

#define BMP280_DEVID        0xd0
#define BMP280_SOFT_RESET   0xe0
#define BMP280_STAT         0xf3
#define BMP280_CTRL_MEAS    0xf4
#define BMP280_CONFIG       0xf5
#define BMP280_PRESS_MSB    0xf7
#define BMP280_PRESS_LSB    0xf8
#define BMP280_PRESS_XLSB   0xf9
#define BMP280_TEMP_MSB     0xfa
#define BMP280_TEMP_LSB     0xfb
#define BMP280_TEMP_XLSB    0xfc

/* Power modes */

#define BMP280_SLEEP_MODE   (0x00)
#define BMP280_FORCED_MODE  (0x01)
#define BMP280_NORMAL_MODE  (0x03)

/* Oversampling for temperature. */

#define BMP280_OST_SKIPPED (0x00 << 5)
#define BMP280_OST_X1      (0x01 << 5)
#define BMP280_OST_X2      (0x02 << 5)
#define BMP280_OST_X4      (0x03 << 5)
#define BMP280_OST_X8      (0x04 << 5)
#define BMP280_OST_X16     (0x05 << 5)

/* Oversampling for pressure. */

#define BMP280_OSP_SKIPPED (0x00 << 2)
#define BMP280_OSP_X1      (0x01 << 2)
#define BMP280_OSP_X2      (0x02 << 2)
#define BMP280_OSP_X4      (0x03 << 2)
#define BMP280_OSP_X8      (0x04 << 2)
#define BMP280_OSP_X16     (0x05 << 2)

/* Predefined oversampling combinations. */

#define BMP280_OS_ULTRA_HIGH_RES  (BMP280_OSP_X16 | BMP280_OST_X2)
#define BMP280_OS_STANDARD_RES    (BMP280_OSP_X4  | BMP280_OST_X1)
#define BMP280_OS_ULTRA_LOW_POWER (BMP280_OSP_X1  | BMP280_OST_X1)

/* Data combined from bytes to int */

#define COMBINE(d) (((int)(d)[0] << 12) | ((int)(d)[1] << 4) | ((int)(d)[2] >> 4))

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

struct bmp280_dev_s
{
  FAR struct sensor_lowerhalf_s sensor_lower;
  FAR struct i2c_master_s *i2c; /* I2C interface */
  uint8_t addr;                 /* BMP280 I2C address */
  int freq;                     /* BMP280 Frequency <= 3.4MHz */
  bool activated;

  struct bmp280_calib_s
  {
    uint16_t t1;
    int16_t  t2;
    int16_t  t3;
    uint16_t p1;
    int16_t  p2;
    int16_t  p3;
    int16_t  p4;
    int16_t  p5;
    int16_t  p6;
    int16_t  p7;
    int16_t  p8;
    int16_t  p9;
  } calib;

  int32_t  tempfine;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv,
                              uint8_t regaddr);
static int bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);

/* Sensor methods */

static int bmp280_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us);
static int bmp280_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);
static int bmp280_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_sensor_ops =
{
  .activate      = bmp280_activate,
  .fetch         = bmp280_fetch,
  .set_interval  = bmp280_set_interval,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_getreg8
 *
 * Description:
 *   Read from an 8-bit BMP280 register
 *
 ****************************************************************************/

static uint8_t bmp280_getreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = &regval;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: bmp280_getregs
 *
 * Description:
 *   Read two 8-bit from a BMP280 register
 *
 ****************************************************************************/

static int bmp280_getregs(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                          uint8_t *rxbuffer, uint8_t length)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = rxbuffer;
  msg[1].length    = length;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280_putreg8
 *
 * Description:
 *   Write to an 8-bit BMP280 register
 *
 ****************************************************************************/

static int bmp280_putreg8(FAR struct bmp280_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = txbuffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bmp280_checkid
 *
 * Description:
 *   Read and verify the BMP280 chip ID
 *
 ****************************************************************************/

static int bmp280_checkid(FAR struct bmp280_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = bmp280_getreg8(priv, BMP280_DEVID);
  up_mdelay(1);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280_set_standby
 *
 * Description:
 *   set standby duration
 *
 ****************************************************************************/

static int bmp280_set_standby(FAR struct bmp280_dev_s *priv, uint8_t value)
{
  uint8_t v_data_u8;
  uint8_t v_sb_u8;

  /* Set the standby duration value */

  v_data_u8 = bmp280_getreg8(priv, BMP280_CONFIG);
  v_data_u8 = (v_data_u8 & ~(0x07 << 5)) | (value << 5);
  bmp280_putreg8(priv, BMP280_CONFIG, v_data_u8);

  /* Check the standby duration value */

  v_data_u8 = bmp280_getreg8(priv, BMP280_CONFIG);
  v_sb_u8 = (v_data_u8 >> 5) & 0x07;

  if (v_sb_u8 != value)
    {
      snerr("Failed to set value for standby time.");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: bmp280_initialize
 *
 * Description:
 *   Initialize BMP280 device
 *
 ****************************************************************************/

static int bmp280_initialize(FAR struct bmp280_dev_s *priv)
{
  uint8_t buf[24];
  int ret;

  /* Get calibration data. */

  ret = bmp280_getregs(priv, BMP280_DIG_T1_LSB, buf, 24);
  if (ret < 0)
    {
      return ret;
    }

  priv->calib.t1 = (uint16_t)buf[1]  << 8 | buf[0];
  priv->calib.t2 = (int16_t) buf[3]  << 8 | buf[2];
  priv->calib.t3 = (int16_t) buf[5]  << 8 | buf[4];

  priv->calib.p1 = (uint16_t)buf[7]  << 8 | buf[6];
  priv->calib.p2 = (int16_t) buf[9]  << 8 | buf[8];
  priv->calib.p3 = (int16_t) buf[11] << 8 | buf[10];
  priv->calib.p4 = (int16_t) buf[13] << 8 | buf[12];
  priv->calib.p5 = (int16_t) buf[15] << 8 | buf[14];
  priv->calib.p6 = (int16_t) buf[17] << 8 | buf[16];
  priv->calib.p7 = (int16_t) buf[19] << 8 | buf[18];
  priv->calib.p8 = (int16_t) buf[21] << 8 | buf[20];
  priv->calib.p9 = (int16_t) buf[23] << 8 | buf[22];

  sninfo("T1 = %u\n", priv->calib.t1);
  sninfo("T2 = %d\n", priv->calib.t2);
  sninfo("T3 = %d\n", priv->calib.t3);

  sninfo("P1 = %u\n", priv->calib.p1);
  sninfo("P2 = %d\n", priv->calib.p2);
  sninfo("P3 = %d\n", priv->calib.p3);
  sninfo("P4 = %d\n", priv->calib.p4);
  sninfo("P5 = %d\n", priv->calib.p5);
  sninfo("P6 = %d\n", priv->calib.p6);
  sninfo("P7 = %d\n", priv->calib.p7);
  sninfo("P8 = %d\n", priv->calib.p8);
  sninfo("P9 = %d\n", priv->calib.p9);

  /* Set power mode to sleep */

  bmp280_putreg8(priv, BMP280_CTRL_MEAS, BMP280_SLEEP_MODE);

  /* Set stand-by time to 0.5 ms, no IIR filter */

  ret = bmp280_set_standby(priv, BMP280_STANDBY_05_MS);
  if (ret != OK)
    {
      snerr("Failed to set value for standby time.\n");
      return -1;
    }

  return ret;
}

/****************************************************************************
 * Name: bmp280_compensate
 *
 * Description:
 *   calculate compensate tempreture
 *
 * Input Parameters:
 *   temp - uncompensate value of tempreture.
 *
 * Returned Value:
 *   calculate result of compensate tempreture.
 *
 ****************************************************************************/

static int32_t bmp280_compensate_temp(FAR struct bmp280_dev_s *priv,
                                   int32_t temp)
{
  struct bmp280_calib_s *c = &priv->calib;
  int32_t var1;
  int32_t var2;

  var1 = ((((temp >> 3) - ((int32_t)c->t1 << 1))) * ((int32_t)c->t2)) >> 11;
  var2 = (((((temp >> 4) - ((int32_t)c->t1)) *
            ((temp >> 4) - ((int32_t)c->t1))) >> 12) *
          ((int32_t)c->t3)) >> 14;

  priv->tempfine = var1 + var2;

  return (priv->tempfine * 5 + 128) >> 8;
}

/****************************************************************************
 * Name: bmp280_compensate_press
 *
 * Description:
 *   calculate compensate pressure
 *
 * Input Parameters:
 *   press - uncompensate value of pressure.
 *
 * Returned Value:
 *   calculate result of compensate pressure.
 *
 ****************************************************************************/

static uint32_t bmp280_compensate_press(FAR struct bmp280_dev_s *priv,
                                        uint32_t press)
{
  struct bmp280_calib_s *c = &priv->calib;
  int32_t var1;
  int32_t var2;
  uint32_t p;

  var1 = (priv->tempfine >> 1) - 64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)c->p6);
  var2 = var2 + ((var1 * ((int32_t)c->p5)) << 1);
  var2 = (var2 >> 2) + (((int32_t)c->p4) << 16);
  var1 = (((c->p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
          ((((int32_t)c->p2) * var1) >> 1)) >> 18;
  var1 = (((32768 + var1) * ((int32_t)c->p1)) >> 15);

  /* avoid exception caused by division by zero */

  if (var1 == 0)
    {
      return 0;
    }

  p = (((uint32_t)((0x100000) - press) - (var2 >> 12))) * 3125;

  if (p < 0x80000000)
    {
      p = (p << 1) / ((uint32_t)var1);
    }
  else
    {
      p = (p / (uint32_t)var1) * 2;
    }

  var1 = ((int32_t)c->p9 * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = ((int32_t)(p >> 2) * c->p8) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + c->p7) >> 4));

  return p;
}

/****************************************************************************
 * Name: bmp280_set_interval
 ****************************************************************************/

static int bmp280_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR unsigned long *period_us)
{
  FAR struct bmp280_dev_s *priv = container_of(lower,
                                               FAR struct bmp280_dev_s,
                                               sensor_lower);
  int ret = 0;

  uint8_t regval;

  switch (*period_us)
    {
      case 500:
        regval = BMP280_STANDBY_05_MS;
        break;
      case 62500:
        regval = BMP280_STANDBY_63_MS;
        break;
      case 125000:
        regval = BMP280_STANDBY_125_MS;
        break;
      case 250000:
        regval = BMP280_STANDBY_250_MS;
        break;
      case 500000:
        regval = BMP280_STANDBY_500_MS;
        break;
      case 1000000:
        regval = BMP280_STANDBY_1000_MS;
        break;
      case 2000000:
        regval = BMP280_STANDBY_2000_MS;
        break;
      case 4000000:
        regval = BMP280_STANDBY_4000_MS;
        break;
      default:
        ret = -EINVAL;
        break;
    }

  if (ret == 0)
    {
      ret = bmp280_set_standby(priv, regval);
    }

  return ret;
}

/****************************************************************************
 * Name: bmp280_activate
 ****************************************************************************/

static int bmp280_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable)
{
  FAR struct bmp280_dev_s *priv = container_of(lower,
                                               FAR struct bmp280_dev_s,
                                               sensor_lower);
  int ret;

  if (enable)
    {
      /* Set power mode to normal and standard sampling resolution. */

      ret = bmp280_putreg8(priv, BMP280_CTRL_MEAS, BMP280_NORMAL_MODE |
                                 BMP280_OS_STANDARD_RES);
    }
  else
    {
      /* Set to sleep mode */

      ret = bmp280_putreg8(priv, BMP280_CTRL_MEAS, BMP280_SLEEP_MODE);
    }

  if (ret >= 0)
    {
      priv->activated = enable;
    }

  return ret;
}

/****************************************************************************
 * Name: bmp280_fetch
 ****************************************************************************/

static int bmp280_fetch(FAR struct sensor_lowerhalf_s *lower,
                        FAR struct file *filep,
                        FAR char *buffer, size_t buflen)
{
  FAR struct bmp280_dev_s *priv = container_of(lower,
                                               FAR struct bmp280_dev_s,
                                               sensor_lower);

  uint8_t buf[6];
  uint32_t press;
  int32_t temp;
  int ret;
  struct timespec ts;
  struct sensor_baro baro_data;

  if (buflen != sizeof(baro_data))
    {
      return -EINVAL;
    }

  if (!priv->activated)
    {
      /* Sensor is asleep, go to force mode to read once */

      ret = bmp280_putreg8(priv, BMP280_CTRL_MEAS, BMP280_FORCED_MODE |
                                 BMP280_OS_ULTRA_LOW_POWER);

      if (ret < 0)
        {
          return ret;
        }

      /* Wait time according to ultra low power mode set during sleep */

      up_mdelay(6);
    }

  /* Read pressure & data */

  ret = bmp280_getregs(priv, BMP280_PRESS_MSB, buf, 6);

  if (ret < 0)
    {
      return ret;
    }

  press = (uint32_t)COMBINE(buf);
  temp = COMBINE(&buf[3]);

  sninfo("press = %"PRIu32", temp = %"PRIi32"\n", press, temp);

  temp = bmp280_compensate_temp(priv, temp);
  press = bmp280_compensate_press(priv, press);

  clock_systime_timespec(&ts);

  baro_data.timestamp = 1000000ull * ts.tv_sec + ts.tv_nsec / 1000;
  baro_data.pressure = press / 100.0f;
  baro_data.temperature = temp / 100.0f;

  memcpy(buffer, &baro_data, sizeof(baro_data));

  return buflen;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_register
 *
 * Description:
 *   Register the BMP280 character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_register(int devno, FAR struct i2c_master_s *i2c)
{
  FAR struct bmp280_dev_s *priv;
  int ret;

  /* Initialize the BMP280 device structure */

  priv = (FAR struct bmp280_dev_s *)kmm_zalloc(sizeof(struct bmp280_dev_s));
  if (!priv)
    {
      snerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = BMP280_ADDR;
  priv->freq = BMP280_FREQ;

  priv->sensor_lower.ops = &g_sensor_ops;
  priv->sensor_lower.type = SENSOR_TYPE_BAROMETER;

  /* Check Device ID */

  ret = bmp280_checkid(priv);

  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  ret = bmp280_initialize(priv);

  if (ret < 0)
    {
      snerr("Failed to initialize physical device bmp280:%d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = sensor_register(&priv->sensor_lower, devno);

  if (ret < 0)
    {
      snerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  sninfo("BMP280 driver loaded successfully!\n");
  return ret;
}

#endif
