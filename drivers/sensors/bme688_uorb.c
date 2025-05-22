/****************************************************************************
 * drivers/sensors/bme688_uorb.c
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

#include <stdio.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bme688.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BME688)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BME688_ADDR                0x76 /* I2C Slave Address */
#define BME688_FREQ                CONFIG_BME688_I2C_FREQUENCY
#define BME688_DEVID               0x61

/* Sub-sensor definitions */

#ifndef CONFIG_BME688_DISABLE_TEMP_MEAS
#  define BME688_TEMP_IDX          0
#else
#  define BME688_TEMP_IDX          (-1)
#endif

#define BME688_TEMP_IDX_OFF        BME688_TEMP_IDX

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
#  define BME688_PRESS_IDX         (BME688_TEMP_IDX_OFF + 1)
#  define BME688_PRESS_IDX_OFF     BME688_PRESS_IDX
#else
#  define BME688_PRESS_IDX         (-1)
#  define BME688_PRESS_IDX_OFF     BME688_TEMP_IDX_OFF
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
#  define BME688_HUM_IDX           (BME688_PRESS_IDX_OFF + 1)
#  define BME688_HUM_IDX_OFF       BME688_HUM_IDX
#else
#  define BME688_HUM_IDX           (-1)
#  define BME688_HUM_IDX_OFF       BME688_PRESS_IDX_OFF
#endif

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
#  define BME688_GAS_IDX           (BME688_HUM_IDX_OFF + 1)
#  define BME688_GAS_IDX_OFF       BME688_GAS_IDX
#else
#  define BME688_GAS_IDX           (-1)
#  define BME688_GAS_IDX_OFF       BME688_HUM_IDX_OFF
#endif

#define BME688_SENSORS_COUNT       (BME688_GAS_IDX_OFF + 1)

/* Register addresses */

#define BME688_STATUS_REG_ADDR     0x73
#define BME688_RESET_REG_ADDR      0xe0
#define BME688_ID_REG_ADDR         0xd0
#define BME688_CTRL_HUM_ADDR       0x72
#define BME688_CTRL_MEAS_ADDR      0x74
#define BME688_CONFIG_REG_ADDR     0x75
#define BME688_CTRL_GAS0           0x70
#define BME688_CTRL_GAS1           0x71
#define BME688_PRESS_MSB           0x1f
#define BME688_PRESS_LSB           0x20
#define BME688_PRESS_XLSB          0x21
#define BME688_TEMP_MSB            0x22
#define BME688_TEMP_LSB            0x23
#define BME688_TEMP_XLSB           0x24
#define BME688_HUM_MSB             0x25
#define BME688_HUM_LSB             0x26
#define BME688_GAS_R_MSB           0x2a
#define BME688_GAS_R_LSB           0x2b
#define BME688_IDAC_HEAT_ADDR      0x50
#define BME688_RES_HEAT_ADDR       0x5a
#define BME688_GAS_WAIT_ADDR       0x64
#define BME688_MEAS_STAT0          0x1d

/* nbconv boundaries */

#define BME688_NBCONV_MIN          0
#define BME688_NBCONV_MAX          9

/* Power modes */

#define BME688_SLEEP_MODE          0x00
#define BME688_FORCED_MODE         0x01
#define BME688_PARALLEL_MODE       0x02

/* Soft reset, same effect as a power-on reset */

#define BME688_SOFT_RESET          0xb6

/* Start addresses for coefficient arrays */

#define BME688_COEFF_ADDR1         0x89
#define BME688_COEFF_ADDR2         0xe1

#define BME688_COEFF_SIZE          41
#define BME688_COEFF_ADDR1_LEN     25
#define BME688_COEFF_ADDR2_LEN     16

/* Start address for measurements and status regs */

#define BME688_DATA_ADDR           0x1d
#define BME688_DATA_LEN            17

/* Gas coefficients */

#define BME688_RES_HEAT_RANGE_ADDR 0x02
#define BME688_RES_HEAT_VAL_ADDR   0x00
#define BME688_RANGE_SW_ERR_ADDR   0x04

/* Array index for mapping calibration data */

#define BME688_T2_LSB_REG           1
#define BME688_T2_MSB_REG           2
#define BME688_T3_REG               3
#define BME688_P1_LSB_REG           5
#define BME688_P1_MSB_REG           6
#define BME688_P2_LSB_REG           7
#define BME688_P2_MSB_REG           8
#define BME688_P3_REG               9
#define BME688_P4_LSB_REG           11
#define BME688_P4_MSB_REG           12
#define BME688_P5_LSB_REG           13
#define BME688_P5_MSB_REG           14
#define BME688_P7_REG               15
#define BME688_P6_REG               16
#define BME688_P8_LSB_REG           19
#define BME688_P8_MSB_REG           20
#define BME688_P9_LSB_REG           21
#define BME688_P9_MSB_REG           22
#define BME688_P10_REG              23
#define BME688_H2_MSB_REG           25
#define BME688_H2_LSB_REG           26
#define BME688_H1_LSB_REG           26
#define BME688_H1_MSB_REG           27
#define BME688_H3_REG               28
#define BME688_H4_REG               29
#define BME688_H5_REG               30
#define BME688_H6_REG               31
#define BME688_H7_REG               32
#define BME688_T1_LSB_REG           33
#define BME688_T1_MSB_REG           34
#define BME688_GH2_LSB_REG          35
#define BME688_GH2_MSB_REG          36
#define BME688_GH1_REG              37
#define BME688_GH3_REG              38

/* Masks for register values */

#define BME688_GAS_MEAS_MSK         0x30
#define BME688_NBCONV_MSK           0x0f
#define BME688_FILTER_MSK           0x1c
#define BME688_OST_MSK              0xe0
#define BME688_OSP_MSK              0x1c
#define BME688_OSH_MSK              0x07
#define BME688_HCTRL_MSK            0x08
#define BME688_RUN_GAS_MSK          0x10
#define BME688_MODE_MSK             0x03
#define BME688_RHRANGE_MSK          0x30
#define BME688_RSERROR_MSK          0xf0
#define BME688_NEW_DATA_MSK         0x80
#define BME688_GAS_INDEX_MSK        0x0f
#define BME688_GAS_RANGE_MSK        0x0f
#define BME688_GASM_VALID_MSK       0x20
#define BME688_HEAT_STAB_MSK        0x10
#define BME688_MEM_PAGE_MSK         0x10
#define BME688_BIT_H1_DATA_MSK      0x0f

/* Bounds for tpg */

#define MIN_HOT_PLATE_TEMP          200   /* Celsius */
#define MAX_HOT_PLATE_TEMP          400   /* Celsius */

#define BME688_MAX_OVERFLOW_VAL     0x40000000ull

#define CHECK_OS_BOUNDS(type) \
  ((type) >= BME688_OS_SKIPPED && (type) <= BME688_OS_16X)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bme688_data_s
{
  uint64_t timestamp;   /* Units is microseconds */
  float temperature;    /* Temperature in degrees Celsius */
  float pressure;       /* Pressure in millibar or hPa */
  float humidity;       /* Relative humidity in rH */
  float gas_resistance; /* Gas resistance in Ohm */
};

struct bme688_calib_s
{
  /* Temperature coefficients */

  uint16_t t1;
  int16_t t2;
  int8_t t3;

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  /* Pressure coefficients */

  uint16_t p1;
  int16_t p2;
  int8_t p3;
  int16_t p4;
  int16_t p5;
  int8_t p6;
  int8_t p7;
  int16_t p8;
  int16_t p9;
  uint8_t p10;
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS

  /* Humidity coefficients */

  uint16_t h1;
  uint16_t h2;
  int8_t h3;
  int8_t h4;
  int8_t h5;
  uint8_t h6;
  int8_t h7;
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  /* Gas heater coefficients */

  int8_t gh1;
  int16_t gh2;
  int8_t gh3;

  uint8_t res_heat_range; /* Heater resistance range */
  int8_t res_heat_val;    /* Heater resistance value */
  int8_t range_sw_err;    /* Error switching range */

#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

  int32_t t_fine;
};

struct bme688_sensor_s
{
  /* Lowerhalfs for every sub-sensor */

  struct sensor_lowerhalf_s lower[BME688_SENSORS_COUNT];
  struct bme688_calib_s calib;   /* Calibration data */
  struct bme688_config_s config; /* Configuration data */
  bool calibrated;               /* Is the device set up? */
};

struct bme688_dev_s
{
  struct bme688_sensor_s dev;   /* Sensor private data */
  FAR struct i2c_master_s *i2c; /* I2C interface */
  mutex_t dev_lock;             /* Manages exclusive access to the device */
  sem_t run;                    /* Locks sensor thread */
  bool enabled;                 /* Enable/Disable BME688 */
};

typedef int (*push_data_func)(FAR struct bme688_dev_s *priv,
                              FAR struct bme688_data_s *data);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint8_t bme688_getreg8(FAR struct bme688_dev_s *priv,
                              uint8_t regaddr);
static int bme688_putreg8(FAR struct bme688_dev_s *priv, uint8_t regaddr,
                          uint8_t regval);
static int bme688_getregs(FAR struct bme688_dev_s *priv, uint8_t regaddr,
                          uint8_t *rxbuffer, uint8_t length);

#ifndef CONFIG_BME688_DISABLE_TEMP_MEAS
static int bme688_push_temp_data(FAR struct bme688_dev_s *priv,
                                 FAR struct bme688_data_s *data);
#endif

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
static int bme688_push_press_data(FAR struct bme688_dev_s *priv,
                                  FAR struct bme688_data_s *data);
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
static int bme688_push_hum_data(FAR struct bme688_dev_s *priv,
                                FAR struct bme688_data_s *data);
#endif

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
static int bme688_push_gas_data(FAR struct bme688_dev_s *priv,
                                FAR struct bme688_data_s *data);
#endif

/* Sensor methods */

static int bme688_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           bool enable);
static int bme688_calibrate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            unsigned long arg);
static int bme688_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const push_data_func deliver_data[BME688_SENSORS_COUNT] =
{
#ifndef CONFIG_BME688_DISABLE_TEMP_MEAS
  bme688_push_temp_data,
#endif

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  bme688_push_press_data,
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  bme688_push_hum_data,
#endif

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  bme688_push_gas_data,
#endif
};

static const struct sensor_ops_s g_sensor_ops =
{
  NULL,             /* open */
  NULL,             /* close */
  bme688_activate,  /* activate */
  NULL,             /* set_interval */
  NULL,             /* batch */
  NULL,             /* fetch */
  NULL,             /* flush */
  NULL,             /* selftest */
  NULL,             /* set_calibvalue */
  bme688_calibrate, /* calibrate */
  NULL,             /* get_info */
  bme688_control    /* control */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bme688_getreg8
 *
 * Description:
 *   Read from an 8-bit BME688 register
 *
 ****************************************************************************/

static uint8_t bme688_getreg8(FAR struct bme688_dev_s *priv, uint8_t regaddr)
{
  struct i2c_msg_s msg[2];
  uint8_t regval = 0;
  int ret;

  msg[0].frequency = BME688_FREQ;
  msg[0].addr = BME688_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = BME688_FREQ;
  msg[1].addr = BME688_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = &regval;
  msg[1].length = 1;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

  return regval;
}

/****************************************************************************
 * Name: bme688_getregs
 *
 * Description:
 *   Read <length> bytes starting from a BME688 register addr
 *
 ****************************************************************************/

static int bme688_getregs(FAR struct bme688_dev_s *priv, uint8_t regaddr,
                          uint8_t *rxbuffer, uint8_t length)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = BME688_FREQ;
  msg[0].addr = BME688_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = &regaddr;
  msg[0].length = 1;

  msg[1].frequency = BME688_FREQ;
  msg[1].addr = BME688_ADDR;
  msg[1].flags = I2C_M_READ;
  msg[1].buffer = rxbuffer;
  msg[1].length = length;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Name: bme688_putreg8
 *
 * Description:
 *   Write to an 8-bit BME688 register
 *
 ****************************************************************************/

static int bme688_putreg8(FAR struct bme688_dev_s *priv, uint8_t regaddr,
                          uint8_t regval)
{
  struct i2c_msg_s msg[2];
  uint8_t txbuffer[2];
  int ret;

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  msg[0].frequency = BME688_FREQ;
  msg[0].addr = BME688_ADDR;
  msg[0].flags = 0;
  msg[0].buffer = txbuffer;
  msg[0].length = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 1);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: bme688_checkid
 *
 * Description:
 *   Read and verify the BME688 chip ID
 *
 ****************************************************************************/

static int bme688_checkid(FAR struct bme688_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = bme688_getreg8(priv, BME688_ID_REG_ADDR);
  up_mdelay(1);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint8_t)BME688_DEVID)
    {
      /* ID is not Correct */

      snerr("Wrong Device ID! %02x\n", devid);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: bme688_get_calib_data
 *
 * Description:
 *   Read sensor-specific parameters and store them for later
 * use in computing the compensated values.
 *
 ****************************************************************************/

static int bme688_get_calib_data(FAR struct bme688_dev_s *priv)
{
  uint8_t coeff[BME688_COEFF_SIZE];
  uint8_t temp_val;
  int ret;

  /* Get first part of the calibration data. */

  ret = bme688_getregs(priv, BME688_COEFF_ADDR1, coeff,
                       BME688_COEFF_ADDR1_LEN);
  if (ret < 0)
    {
      return ret;
    }

  /* Concatenate the second part of the data to coeff */

  ret = bme688_getregs(priv, BME688_COEFF_ADDR2,
                       &coeff[BME688_COEFF_ADDR1_LEN],
                       BME688_COEFF_ADDR2_LEN);
  if (ret < 0)
    {
      return ret;
    }

  /* Get data */

  priv->dev.calib.t1 = coeff[BME688_T1_MSB_REG] << 8
                      | coeff[BME688_T1_LSB_REG];
  priv->dev.calib.t2 = coeff[BME688_T2_MSB_REG] << 8
                      | coeff[BME688_T2_LSB_REG];
  priv->dev.calib.t3 = coeff[BME688_T3_REG];

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  priv->dev.calib.p1 = coeff[BME688_P1_MSB_REG] << 8
                      | coeff[BME688_P1_LSB_REG];
  priv->dev.calib.p2 = coeff[BME688_P2_MSB_REG] << 8
                      | coeff[BME688_P2_LSB_REG];
  priv->dev.calib.p3 = coeff[BME688_P3_REG];
  priv->dev.calib.p4 = coeff[BME688_P4_MSB_REG] << 8
                      | coeff[BME688_P4_LSB_REG];
  priv->dev.calib.p5 = coeff[BME688_P5_MSB_REG] << 8
                      | coeff[BME688_P5_LSB_REG];
  priv->dev.calib.p6 = coeff[BME688_P6_REG];
  priv->dev.calib.p7 = coeff[BME688_P7_REG];
  priv->dev.calib.p8 = coeff[BME688_P8_MSB_REG] << 8
                      | coeff[BME688_P8_LSB_REG];
  priv->dev.calib.p9 = coeff[BME688_P9_MSB_REG] << 8
                      | coeff[BME688_P9_LSB_REG];
  priv->dev.calib.p10 = coeff[BME688_P10_REG];
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  priv->dev.calib.h1 = (uint16_t)(((uint16_t)coeff[BME688_H1_MSB_REG] << 4)
                      | (coeff[BME688_H1_LSB_REG] & BME688_BIT_H1_DATA_MSK));
  priv->dev.calib.h2 = (uint16_t)(((uint16_t)coeff[BME688_H2_MSB_REG] << 4)
                      | ((coeff[BME688_H2_LSB_REG]) >> 4));
  priv->dev.calib.h3 = coeff[BME688_H3_REG];
  priv->dev.calib.h4 = coeff[BME688_H4_REG];
  priv->dev.calib.h5 = coeff[BME688_H5_REG];
  priv->dev.calib.h6 = coeff[BME688_H6_REG];
  priv->dev.calib.h7 = coeff[BME688_H7_REG];
#endif /* !CONFIG_BME688_DISABLE_HUM_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS

  /* Gas-related coefficients */

  priv->dev.calib.gh1 = coeff[BME688_GH1_REG];
  priv->dev.calib.gh2 = coeff[BME688_GH2_MSB_REG] << 8
                      | coeff[BME688_GH2_LSB_REG];
  priv->dev.calib.gh3 = coeff[BME688_GH3_REG];

  ret = bme688_getregs(priv, BME688_RES_HEAT_RANGE_ADDR, &temp_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  priv->dev.calib.res_heat_range = ((temp_val & BME688_RHRANGE_MSK)) / 16;

  ret = bme688_getregs(priv, BME688_RES_HEAT_VAL_ADDR, &temp_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  priv->dev.calib.res_heat_val = (int8_t)temp_val;

  ret = bme688_getregs(priv, BME688_RANGE_SW_ERR_ADDR, &temp_val, 1);
  if (ret < 0)
    {
      return ret;
    }

  priv->dev.calib.range_sw_err = ((int8_t)temp_val
                                 & (int8_t)BME688_RSERROR_MSK) / 16;

#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

  return ret;
}

/****************************************************************************
 * Name: bme688_set_mode
 *
 * Description:
 *   Set sensor mode and wait for it to change accordingly.
 *
 ****************************************************************************/

static int bme688_set_mode(FAR struct bme688_dev_s *priv, uint8_t mode)
{
  int ret;
  uint8_t power_mode;
  uint8_t regval;

  /* Get current sensor mode */

  ret = bme688_getregs(priv, BME688_CTRL_MEAS_ADDR, &regval, 1);

  if (ret < 0)
    {
      return ret;
    }

  power_mode = regval & BME688_MODE_MSK;

  if (power_mode != mode)
    {
      regval &= (uint8_t)(~BME688_MODE_MSK);
      regval |= (mode & BME688_MODE_MSK);

      ret = bme688_putreg8(priv, BME688_CTRL_MEAS_ADDR, regval);

      if (ret < 0)
        {
          return ret;
        }

      ret = bme688_getregs(priv, BME688_CTRL_MEAS_ADDR, &regval, 1);

      /* Check if the mode has changed and wait if it hasn't */

      while ((regval & BME688_MODE_MSK) != mode)
        {
          up_mdelay(100);
          ret = bme688_getregs(priv, BME688_CTRL_MEAS_ADDR, &regval, 1);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: bme688_set_oversamp
 *
 * Description:
 *   Set temperature, pressure and humidity oversampling.
 *
 ****************************************************************************/

static int bme688_set_oversamp(FAR struct bme688_dev_s *priv)
{
  struct bme688_config_s config = priv->dev.config;

  int ret;
  uint8_t regval;

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  /* Set humidity oversampling */

  regval = config.hum_os & BME688_OSH_MSK;
  ret = bme688_putreg8(priv, BME688_CTRL_HUM_ADDR, regval);
#endif

  /* Set temperature and pressure oversampling */

  regval = 0;
  ret = bme688_getregs(priv, BME688_CTRL_MEAS_ADDR, &regval, 1);

  if (ret < 0)
    {
      return ret;
    }

  regval &= BME688_MODE_MSK;
  regval |= ((config.temp_os << 5) & BME688_OST_MSK);

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  regval |= ((config.press_os << 2) & BME688_OSP_MSK);
#endif

  ret = bme688_putreg8(priv, BME688_CTRL_MEAS_ADDR, regval);

  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

#ifndef CONFIG_BME688_DISABLE_TEMP_MEAS
/****************************************************************************
 * Name: bme688_push_temp_data
 ****************************************************************************/

static int bme688_push_temp_data(FAR struct bme688_dev_s *priv,
                                 FAR struct bme688_data_s *data)
{
  struct sensor_temp temp_data;
  int ret;

  struct sensor_lowerhalf_s lower = priv->dev.lower[BME688_TEMP_IDX];

  temp_data.timestamp = data->timestamp;
  temp_data.temperature = data->temperature;

  ret = lower.push_event(lower.priv, &temp_data, sizeof(struct sensor_temp));

  if (ret < 0)
    {
      snerr("Pushing temperature data failed\n");
      return ret;
    }

  return OK;
}
#endif

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
/****************************************************************************
 * Name: bme688_push_press_data
 ****************************************************************************/

static int bme688_push_press_data(FAR struct bme688_dev_s *priv,
                                  FAR struct bme688_data_s *data)
{
  struct sensor_baro press_data;
  int ret;

  struct sensor_lowerhalf_s lower = priv->dev.lower[BME688_PRESS_IDX];

  press_data.timestamp = data->timestamp;
  press_data.temperature = data->temperature;
  press_data.pressure = data->pressure / 100.f;

  ret = lower.push_event(lower.priv, &press_data,
                         sizeof(struct sensor_baro));

  if (ret < 0)
    {
      snerr("Pushing baro data failed\n");
      return ret;
    }

  return OK;
}
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
/****************************************************************************
 * Name: bme688_push_hum_data
 ****************************************************************************/

static int bme688_push_hum_data(FAR struct bme688_dev_s *priv,
                                FAR struct bme688_data_s *data)
{
  struct sensor_humi hum_data;
  int ret;

  struct sensor_lowerhalf_s lower = priv->dev.lower[BME688_HUM_IDX];

  hum_data.timestamp = data->timestamp;
  hum_data.humidity = data->humidity;

  ret = lower.push_event(lower.priv, &hum_data, sizeof(struct sensor_humi));

  if (ret < 0)
    {
      snerr("Pushing humidity data failed\n");
      return ret;
    }

  return OK;
}
#endif /* !CONFIG_BME688_DISABLE_HUM_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
/****************************************************************************
 * Name: bme688_push_gas_data
 ****************************************************************************/

static int bme688_push_gas_data(FAR struct bme688_dev_s *priv,
                                FAR struct bme688_data_s *data)
{
  struct sensor_gas gas_data;
  int ret;

  struct sensor_lowerhalf_s lower = priv->dev.lower[BME688_GAS_IDX];

  gas_data.timestamp = data->timestamp;
  gas_data.gas_resistance = data->gas_resistance / 1000.f;

  ret = lower.push_event(lower.priv, &gas_data, sizeof(struct sensor_gas));

  if (ret < 0)
    {
      snerr("Pushing gas data failed\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: calc_heater_res
 *
 * Description:
 *   Compute the heater resistance using the target temperature.
 *
 ****************************************************************************/

static uint8_t calc_heater_res(FAR const struct bme688_dev_s *priv)
{
  uint8_t res_heat;
  int32_t var1;
  int32_t var2;
  int32_t var3;
  int32_t var4;
  int32_t var5;
  int32_t res_heat_x100;

  int16_t temp;
  int16_t amb_temp;

  struct bme688_sensor_s dev = priv->dev;

  temp = dev.config.target_temp;

  if (temp > 400)
    {
      temp = 400;
    }

  amb_temp = dev.config.amb_temp;

  var1 = (((int32_t)amb_temp * dev.calib.gh3) / 10) * 256;
  var2 = (dev.calib.gh1 + 784) * (((((dev.calib.gh2 + 154009)
        * temp * 5) / 100) + 3276880) / 10);
  var3 = var1 + (var2 / 2);
  var4 = (var3 / (dev.calib.res_heat_range + 4));
  var5 = (131 * dev.calib.res_heat_val) + 65536;
  res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
  res_heat = (uint8_t)((res_heat_x100 + 50) / 100);

  return res_heat;
}

/****************************************************************************
 * Name: calc_heater_dur
 *
 * Description:
 *   Compute the heater duration to be written to heat_dur register.
 *
 ****************************************************************************/

static uint8_t calc_heater_dur(FAR const struct bme688_dev_s *priv)
{
  uint16_t heat_dur = priv->dev.config.heater_duration;
  uint8_t gas_wait_val;
  uint8_t factor;

  /* Max value of duration is 4032 ms */

  if (heat_dur > 0xfc0)
    {
      heat_dur = 0xfc0;
    }

  /* Compute multiplication factor */

  factor = 0;

  while (heat_dur > 0x3f)
    {
      heat_dur = heat_dur / 4;
      factor++;
    }

  gas_wait_val = (factor << 6) | heat_dur;

  return gas_wait_val;
}

/****************************************************************************
 * Name: bme688_set_gas_config
 ****************************************************************************/

static int bme688_set_gas_config(FAR struct bme688_dev_s *priv)
{
  int ret;
  uint8_t heat_res;
  uint8_t heat_dur;
  uint8_t run_gas;
  uint8_t regval;
  uint8_t nb_conv = priv->dev.config.nb_conv;

  /* Set heater resistance */

  heat_res = calc_heater_res(priv);

  ret = bme688_putreg8(priv, (BME688_RES_HEAT_ADDR + nb_conv), heat_res);
  if (ret < 0)
    {
      return ret;
    }

  /* Set heater duration */

  heat_dur = calc_heater_dur(priv);

  ret = bme688_putreg8(priv, (BME688_GAS_WAIT_ADDR + nb_conv), heat_dur);
  if (ret < 0)
    {
      return ret;
    }

  /* Set nbconv and run_gas */

  run_gas = priv->dev.config.target_temp ? 1 : 0;
  regval = (run_gas << 5) | nb_conv;

  ret = bme688_putreg8(priv, BME688_CTRL_GAS1, regval);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}
#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

/****************************************************************************
 * Name: bme688_write_config
 *
 * Description:
 *   Write the configuration of the sensor into its registers
 * (oversampling, heater temp, heater duration, etc).
 *
 ****************************************************************************/

static int bme688_write_config(FAR struct bme688_dev_s *priv)
{
  int ret;

#ifdef CONFIG_BME688_ENABLE_IIR_FILTER
  uint8_t regval;
#endif /* CONFIG_BME688_ENABLE_IIR_FILTER */

  nxmutex_lock(&priv->dev_lock);

  /* Before anything is written, make sure it is in sleep mode */

  ret = bme688_set_mode(priv, BME688_SLEEP_MODE);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Set oversampling */

  ret = bme688_set_oversamp(priv);
  if (ret < 0)
    {
      goto err_out;
    }

#ifdef CONFIG_BME688_ENABLE_IIR_FILTER
  /* Set filter */

  regval = priv->dev.config.filter_coef << 2;
  ret = bme688_putreg8(priv, BME688_CONFIG_REG_ADDR, regval);
  if (ret < 0)
    {
      goto err_out;
    }
#endif /* CONFIG_ENABLE_IIR_FILTER */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  /* Set gas configs */

  ret = bme688_set_gas_config(priv);
  if (ret < 0)
    {
      goto err_out;
    }
#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

  nxmutex_unlock(&priv->dev_lock);
  return OK;

err_out:
  snerr("Failed to calibrate sensor.\n");
  nxmutex_unlock(&priv->dev_lock);
  return ret;
}

/****************************************************************************
 * Name: bme688_configure
 ****************************************************************************/

static int bme688_configure(FAR struct bme688_dev_s *priv,
                            FAR struct bme688_config_s *config)
{
  int ret;

  /* Sanity checks */

  if (!CHECK_OS_BOUNDS(config->temp_os))
    {
      return -EINVAL;
    }

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  if (!CHECK_OS_BOUNDS(config->press_os))
    {
      return -EINVAL;
    }
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  if (!CHECK_OS_BOUNDS(config->hum_os))
    {
      return -EINVAL;
    }
#endif

#ifdef CONFIG_BME688_ENABLE_IIR_FILTER
  if (config->filter_coef < BME688_FILTER_COEF0 ||
      config->filter_coef > BME688_FILTER_COEF127)
    {
      return -EINVAL;
    }
#endif

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  if (config->target_temp < MIN_HOT_PLATE_TEMP ||
      config->target_temp > MAX_HOT_PLATE_TEMP)
    {
      return -EINVAL;
    }

  if (config->nb_conv > 9)
    {
      return -EINVAL;
    }
#endif

  /* Update config in priv */

  memcpy(&priv->dev.config, config, sizeof(struct bme688_config_s));

  ret = bme688_write_config(priv);
  if (ret < 0)
    {
      snerr("Failed to calibrate sensor.\n");
      return ret;
    }

  priv->dev.calibrated = true;

  return ret;
}

/****************************************************************************
 * Name: bme688_comp_temp
 ****************************************************************************/

static float bme688_comp_temp(FAR struct bme688_dev_s *priv,
                              uint32_t adc_temp)
{
  float var1 = 0.0f;
  float var2 = 0.0f;
  float calc_temp = 0.0f;

  struct bme688_sensor_s dev = priv->dev;

  var1 = ((((float)adc_temp / 16384.0f) - ((float)dev.calib.t1 / 1024.0f))
       * ((float)dev.calib.t2));

  var2 = (((((float)adc_temp / 131072.0f) - ((float)dev.calib.t1 / 8192.0f))
        * (((float)adc_temp / 131072.0f) - ((float)dev.calib.t1 / 8192.0f)))
        * ((float)dev.calib.t3 * 16.0f));

  priv->dev.calib.t_fine = (var1 + var2);

  /* Compensated temperature data */

  calc_temp = (priv->dev.calib.t_fine) / 5120.0f;

  return calc_temp;
}

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
/****************************************************************************
 * Name: bme688_comp_press
 ****************************************************************************/

static float bme688_comp_press(FAR struct bme688_dev_s *priv,
                               uint32_t adc_press)
{
  float var1 = 0.0f;
  float var2 = 0.0f;
  float var3 = 0.0f;
  float calc_pres = 0.0f;

  struct bme688_sensor_s dev = priv->dev;

  var1 = (((float)dev.calib.t_fine / 2.0f) - 64000.0f);
  var2 = var1 * var1 * (((float)dev.calib.p6) / (131072.0f));
  var2 = var2 + (var1 * ((float)dev.calib.p5) * 2.0f);
  var2 = (var2 / 4.0f) + (((float)dev.calib.p4) * 65536.0f);
  var1 = (((((float)dev.calib.p3 * var1 * var1) / 16384.0f)
        + ((float)dev.calib.p2 * var1)) / 524288.0f);
  var1 = ((1.0f + (var1 / 32768.0f)) * ((float)dev.calib.p1));
  calc_pres = (1048576.0f - ((float)adc_press));

  /* Avoid exception caused by division by zero */

  if ((int)var1 != 0)
    {
      calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
      var1 = (((float)dev.calib.p9) * calc_pres * calc_pres) / 2147483648.0f;
      var2 = calc_pres * (((float)dev.calib.p8) / 32768.0f);
      var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f)
          * (calc_pres / 256.0f) * (dev.calib.p10 / 131072.0f));
      calc_pres = (calc_pres + (var1 + var2 + var3
          + ((float)dev.calib.p7 * 128.0f)) / 16.0f);
    }

  return calc_pres;
}
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
/****************************************************************************
 * Name: bme688_comp_hum
 ****************************************************************************/

static float bme688_comp_hum(FAR struct bme688_dev_s *priv,
                             uint16_t adc_hum)
{
  float calc_hum = 0.0f;
  float var1 = 0.0f;
  float var2 = 0.0f;
  float var3 = 0.0f;
  float var4 = 0.0f;
  float temp_comp;

  struct bme688_sensor_s dev = priv->dev;

  /* Compensated temperature data */

  temp_comp = ((dev.calib.t_fine) / 5120.0f);

  var1 = (float)((float)adc_hum) - (((float)dev.calib.h1 * 16.0f)
      + (((float)dev.calib.h3 / 2.0f) * temp_comp));

  var2 = var1 * ((float)(((float)dev.calib.h2 / 262144.0f)
      * (1.0f + (((float)dev.calib.h4 / 16384.0f) * temp_comp)
      + (((float)dev.calib.h5 / 1048576.0f) * temp_comp * temp_comp))));

  var3 = (float)dev.calib.h6 / 16384.0f;

  var4 = (float)dev.calib.h7 / 2097152.0f;

  calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

  if (calc_hum > 100.0f)
    {
      calc_hum = 100.0f;
    }
  else if (calc_hum < 0.0f)
    {
      calc_hum = 0.0f;
    }

  return calc_hum;
}
#endif /* !CONFIG_BME688_DISABLE_HUM_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
/****************************************************************************
 * Name: bme688_calc_gas_res
 ****************************************************************************/

static float bme688_calc_gas_res(FAR struct bme688_dev_s *priv,
                                 uint16_t adc_gas_res, uint8_t gas_range)
{
  uint32_t var1 = 262144 >> gas_range;
  int32_t var2 = (int32_t)adc_gas_res - 512;

  var2 *= 3;
  var2 = 4096 + var2;
  return 1000000.0f * (float)var1 / (float)var2;
}
#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

/****************************************************************************
 * Name: bme688_read_measurements
 *
 * Description:
 *   Reads the raw data from the sensor and computes the compensated
 *   values, storing them in the data struct.
 *
 ****************************************************************************/

static int bme688_read_measurements(FAR struct bme688_dev_s *priv,
                                    FAR struct bme688_data_s *data)
{
  uint8_t status;
  uint32_t adc_temp;

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  uint32_t adc_press;
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  uint16_t adc_hum;
#endif /* !CONFIG_BME688_DISABLE_HUM_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  uint16_t adc_gas_res;
  uint8_t gas_range;
  uint8_t gas_valid;
  uint8_t heat_stab;
#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

  int ret;

  uint8_t data_regs[BME688_DATA_LEN];

  ret = bme688_getregs(priv, BME688_DATA_ADDR, data_regs, BME688_DATA_LEN);
  if (ret < 0)
    {
      snerr("Failed to read data registers.\n");
      return ret;
    }

  status = data_regs[0] & BME688_NEW_DATA_MSK;

  /* No new data, return */

  if (!status)
    {
      sninfo("No new data\n");
      return -ENODATA;
    }

  adc_temp = (uint32_t)(((uint32_t)data_regs[5] << 12)
          | ((uint32_t)data_regs[6] << 4)
          | ((uint32_t)data_regs[7] >> 4));

  data->temperature = bme688_comp_temp(priv, adc_temp);

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  adc_press = (uint32_t)(((uint32_t)data_regs[2] << 12)
          | ((uint32_t)data_regs[3] << 4)
          | ((uint32_t)data_regs[4] >> 4));

  data->pressure = bme688_comp_press(priv, adc_press);
#endif /* !CONFIG_BME688_DISABLE_PRESS_MEAS */

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  adc_hum = (uint16_t)(((uint32_t)data_regs[8] << 8)
          | (uint32_t)data_regs[9]);

  data->humidity = bme688_comp_hum(priv, adc_hum);
#endif /* !CONFIG_BME688_DISABLE_HUM_MEAS */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  adc_gas_res = (uint16_t)((uint32_t)data_regs[15] << 2
               | (((uint32_t)data_regs[16]) >> 6));
  gas_range = data_regs[16] & BME688_GAS_RANGE_MSK;

  /* Is measured gas valid? */

  gas_valid = data_regs[16] & BME688_GASM_VALID_MSK;
  if (!gas_valid)
    {
      sninfo("Invalid gas measurement.\n");
      return -1;
    }

  heat_stab = data_regs[16] & BME688_HEAT_STAB_MSK;
  if (!heat_stab)
    {
      sninfo("The heater did not stabilize.\n");
      return -1;
    }

  priv->dev.config.amb_temp = data->temperature; /* Update ambient temp */

  data->gas_resistance = bme688_calc_gas_res(priv, adc_gas_res, gas_range);
#endif /* !CONFIG_BME688_DISABLE_GAS_MEAS */

  return OK;
}

/****************************************************************************
 * Name: bme688_get_tphg_dur
 *
 * Description:
 *   Compute the duration of a tphg cycle in us, taking into consideration
 *   the settings of the sensor.
 *
 ****************************************************************************/

static uint16_t bme688_get_tphg_dur(FAR struct bme688_dev_s *priv)
{
  uint32_t tph_dur; /* Calculate in us */
  uint32_t meas_cycles;
  uint16_t duration;
  uint8_t os_to_meas_cycles[6] =
  {
    0, 1, 2, 4, 8, 16
  };

  meas_cycles = os_to_meas_cycles[priv->dev.config.temp_os];

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  meas_cycles += os_to_meas_cycles[priv->dev.config.press_os];
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  meas_cycles += os_to_meas_cycles[priv->dev.config.hum_os];
#endif

  /* TPH measurement duration */

  tph_dur = meas_cycles * 1963;
  tph_dur += 477 * 4; /* TPH switching duration */

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  tph_dur += 477 * 5; /* Gas measurement duration */
#endif

  tph_dur += 500;
  tph_dur /= 1000; /* Convert to ms */

  tph_dur += 1; /* Wake up duration of 1ms */

  duration = (uint16_t)tph_dur;

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  /* The remaining time should be used for heating */

  duration += priv->dev.config.heater_duration;
#endif

  return duration;
}

/****************************************************************************
 * Name: bme688_activate
 ****************************************************************************/

static int bme688_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  int offset;
  FAR struct bme688_sensor_s *dev;
  FAR struct bme688_dev_s *priv;

  /* Get offset inside array of lowerhalfs */

  switch (lower->type)
    {
      case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        offset = BME688_TEMP_IDX;
        break;

      case SENSOR_TYPE_BAROMETER:
        offset = BME688_PRESS_IDX;
        break;

      case SENSOR_TYPE_RELATIVE_HUMIDITY:
        offset = BME688_HUM_IDX;
        break;

      case SENSOR_TYPE_GAS:
        offset = BME688_GAS_IDX;
        break;

      default:
        offset = 0;
        break;
    }

  dev = (FAR struct bme688_sensor_s *)
        ((uintptr_t)lower - offset * sizeof(*lower));

  priv = container_of(dev, FAR struct bme688_dev_s, dev);

  /* Wake the thread only once (the activate method will be called
   *  multiple times for the bme688 sub-sensors)
   */

  if (!priv->enabled && enable)
    {
      priv->enabled = enable;

      /* Wake up the polling thread */

      nxsem_post(&priv->run);

      return OK;
    }

  priv->enabled = enable;

  return OK;
}

/****************************************************************************
 * Name: bme688_calibrate
 ****************************************************************************/

static int bme688_calibrate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, unsigned long arg)
{
  int offset;
  FAR struct bme688_sensor_s *dev;
  FAR struct bme688_dev_s *priv;
  FAR struct bme688_config_s *calibval = (FAR struct bme688_config_s *)arg;

  /* Get offset inside array of lowerhalfs */

  switch (lower->type)
    {
      case SENSOR_TYPE_AMBIENT_TEMPERATURE:
        offset = BME688_TEMP_IDX;
        break;

      case SENSOR_TYPE_BAROMETER:
        offset = BME688_PRESS_IDX;
        break;

      case SENSOR_TYPE_RELATIVE_HUMIDITY:
        offset = BME688_HUM_IDX;
        break;

      case SENSOR_TYPE_GAS:
        offset = BME688_GAS_IDX;
        break;

      default:
        offset = 0;
        break;
    }

  dev = (FAR struct bme688_sensor_s *)
        ((uintptr_t)lower - offset * sizeof(*lower));

  priv = container_of(dev, FAR struct bme688_dev_s, dev);

  return bme688_configure(priv, calibval);
}

/****************************************************************************
 * Name: bme688_control
 ****************************************************************************/

static int bme688_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep,
                          int cmd, unsigned long arg)
{
  FAR struct bme688_sensor_s *dev = container_of(lower,
                                                 FAR struct bme688_sensor_s,
                                                 lower);
  FAR struct bme688_dev_s *priv = container_of(dev,
                                               FAR struct bme688_dev_s,
                                               dev);
  int ret;

  switch (cmd)
    {
      case SNIOC_RESET:
        {
          /* Perform Soft Reset */

          uint8_t regval = 0xb6;
          ret = bme688_putreg8(priv, BME688_RESET_REG_ADDR, regval);

          if (ret < 0)
            {
              return ret;
            }

          /* Wait for the device to reset */

          up_mdelay(100);
        }
        break;

      default:
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: bme688_thread
 ****************************************************************************/

static int bme688_thread(int argc, char **argv)
{
  FAR struct bme688_dev_s *priv =
    (FAR struct bme688_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  struct bme688_data_s data;
  int ret;

  while (true)
    {
      int sensor;

      if (!priv->enabled)
        {
          /* Wait for the sensor to be enabled */

          nxsem_wait(&priv->run);
        }

      /* No measurements are done unless the sensor is calibrated */

      if (!priv->dev.calibrated)
        {
          sninfo("The sensor is not calibrated!\n");
          goto thread_sleep;
        }

      /* Trigger a measurement */

      ret = bme688_set_mode(priv, BME688_FORCED_MODE);

      /* Wait for the TPHG cycle to complete */

      uint16_t cycle_duration = bme688_get_tphg_dur(priv);

      up_mdelay(cycle_duration);

      ret = bme688_read_measurements(priv, &data);
      if (ret < 0)
        {
          goto thread_sleep;
        }

      data.timestamp = sensor_get_timestamp();

      for (sensor = 0; sensor < BME688_SENSORS_COUNT; sensor++)
        {
          deliver_data[sensor](priv, &data);
        }

    thread_sleep:
      nxsig_usleep(CONFIG_SENSORS_BME688_POLL_INTERVAL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bme688_register
 *
 * Description:
 *   Register the BME688 character device
 *
 * Input Parameters:
 *   devno - The user-specified device number, starting from 0
 *   i2c   - An instance of the I2C interface to use to communicate with
 *           BME688
 *   config - optional sensor initial configuration
 *
 * Return value:
 *   Zero (OK) on success; a negated errno value on failure
 *
 ****************************************************************************/

int bme688_register(int devno, FAR struct i2c_master_s *i2c,
                    FAR struct bme688_config_s *config)
{
  FAR struct sensor_lowerhalf_s *lower;
  FAR struct bme688_dev_s *priv;
  FAR char *argv[2];
  char arg1[32];
  int ret = OK;
  int i;

  DEBUGASSERT(i2c != NULL);

  /* Initialize the BME688 device structure */

  priv = kmm_zalloc(sizeof(struct bme688_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance (err = %d)\n", ret);
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->enabled = false;
  nxmutex_init(&priv->dev_lock);
  nxsem_init(&priv->run, 0, 0);

  /* Check Device ID */

  ret = bme688_checkid(priv);
  if (ret < 0)
    {
      snerr("ERROR: Wrong device ID!\n");
      goto err_init;
    }

  /* Get Calibration Data */

  ret = bme688_get_calib_data(priv);
  if (ret < 0)
    {
      snerr("Failed to read calib data from bme688:%d\n", ret);
      goto err_init;
    }

  /* Sensor configuration */

  priv->dev.calibrated = false;

  if (config)
    {
      ret = bme688_configure(priv, config);
      if (ret < 0)
        {
          snerr("Failed to configure bme688:%d\n", ret);
          goto err_init;
        }
    }

#ifndef CONFIG_BME688_DISABLE_TEMP_MEAS
  /* Register the temperature driver */

  lower = &priv->dev.lower[BME688_TEMP_IDX];
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register temperature driver (err = %d)\n",
            ret);
      goto err_init;
    }
#endif

#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  /* Register the barometer driver */

  lower = &priv->dev.lower[BME688_PRESS_IDX];
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_BAROMETER;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register barometer driver (err = %d)\n",
            ret);
      goto err_init;
    }
#endif

#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  /* Register the humidity driver */

  lower = &priv->dev.lower[BME688_HUM_IDX];
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_RELATIVE_HUMIDITY;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register humidity driver (err = %d)\n",
            ret);
      goto err_init;
    }
#endif

#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  /* Register the gas driver */

  lower = &priv->dev.lower[BME688_GAS_IDX];
  lower->ops = &g_sensor_ops;
  lower->type = SENSOR_TYPE_GAS;

  ret = sensor_register(lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register gas driver (err = %d)\n",
            ret);
      goto err_init;
    }
#endif

  /* Create thread for polling sensor data */

  snprintf(arg1, 16, "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("bme688_thread", SCHED_PRIORITY_DEFAULT,
                       CONFIG_SENSORS_BME688_THREAD_STACKSIZE,
                       bme688_thread, argv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to create poll thread (err = %d)\n", ret);
      goto err_register;
    }

  sninfo("BME688 driver loaded successfully!\n");
  return OK;

err_register:
  for (i = 0; i < BME688_SENSORS_COUNT; i++)
    {
      sensor_unregister(&priv->dev.lower[i], devno);
    }

err_init:
  nxsem_destroy(&priv->run);
  nxmutex_destroy(&priv->dev_lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BME688 */
