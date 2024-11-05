/****************************************************************************
 * drivers/sensors/lsm9ds1_base.c
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

#include "lsm9ds1_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Accelerometer Operations */

static int lsm9ds1accelgyro_config(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accel_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accel_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1accelgyro_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                          uint32_t samplerate);
static int lsm9ds1accel_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                     uint32_t fullscale);

/* Gyroscope Operations */

static int lsm9ds1gyro_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1gyro_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1gyro_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t fullscale);

/* Magnetometer Operations */

static int lsm9ds1mag_config(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_start(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_stop(FAR struct lsm9ds1_dev_s *priv);
static int lsm9ds1mag_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                   uint32_t fullscale);
static int lsm9ds1mag_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t samplerate);

/****************************************************************************
 * Private Types
 ****************************************************************************/

const struct lsm9ds1_ops_s g_lsm9ds1accel_ops =
{
  lsm9ds1accelgyro_config,
  lsm9ds1accel_start,
  lsm9ds1accel_stop,
  lsm9ds1accelgyro_setsamplerate,
  lsm9ds1accel_setfullscale,
};

const struct lsm9ds1_ops_s g_lsm9ds1gyro_ops =
{
  lsm9ds1accelgyro_config,
  lsm9ds1gyro_start,
  lsm9ds1gyro_stop,
  lsm9ds1accelgyro_setsamplerate,
  lsm9ds1gyro_setfullscale,
};

const struct lsm9ds1_ops_s g_lsm9ds1mag_ops =
{
  lsm9ds1mag_config,
  lsm9ds1mag_start,
  lsm9ds1mag_stop,
  lsm9ds1mag_setsamplerate,
  lsm9ds1mag_setfullscale,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1accelgyro_config
 *
 * Description:
 *   Configure the accelerometer and gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1accelgyro_config(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t regval;
  int     ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm9ds1_readreg8(priv, LSM9DS1_WHO_AM_I, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  if (regval != LSM9DS1_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1accel_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/

static int lsm9ds1accel_start(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (priv->samplerate < lsm9ds1_midpoint(10, 50))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_10HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(50, 119))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_50HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(119, 238))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_119HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(238, 476))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_238HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(476, 952))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_476HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG6_XL_ODR_XL_952HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1accel_stop
 *
 * Description:
 *   Stop the accelerometer.
 *
 ****************************************************************************/

static int lsm9ds1accel_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_MASK,
                            LSM9DS1_CTRL_REG6_XL_ODR_XL_POWERDOWN);
}

/****************************************************************************
 * Name: lsm9ds1accelgyro_setsamplerate
 *
 * Description:
 *   Set the accelerometer or gyroscope's sample rate.
 *
 ****************************************************************************/

static int lsm9ds1accelgyro_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                          uint32_t samplerate)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  priv->samplerate = samplerate;
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1accel_setfullscale
 *
 * Description:
 *   Set the accelerometer's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1accel_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                     uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(2, 4))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_2G;
    }
  else if (fullscale < lsm9ds1_midpoint(4, 8))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_4G;
    }
  else if (fullscale < lsm9ds1_midpoint(8, 16))
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_8G;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG6_XL_FS_XL_16G;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG6_XL,
                            LSM9DS1_CTRL_REG6_XL_FS_XL_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1gyro_start
 *
 * Description:
 *   Start the gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1gyro_start(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (priv->samplerate < lsm9ds1_midpoint(14, 59))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_14p9HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(59, 119))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_59p5HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(119, 238))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_119HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(238, 476))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_238HZ;
    }
  else if (priv->samplerate < lsm9ds1_midpoint(476, 952))
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_476HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_G_ODR_G_952HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_ODR_G_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1gyro_stop
 *
 * Description:
 *   Stop the gyroscope.
 *
 ****************************************************************************/

static int lsm9ds1gyro_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_ODR_G_MASK,
                            LSM9DS1_CTRL_REG1_G_ODR_G_POWERDOWN);
}

/****************************************************************************
 * Name: lsm9ds1gyro_setfullscale
 *
 * Description:
 *   Set the gyroscope's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1gyro_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(245, 500))
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_245DPS;
    }
  else if (fullscale < lsm9ds1_midpoint(500, 2000))
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_500DPS;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_G_FS_G_2000DPS;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_G,
                            LSM9DS1_CTRL_REG1_G_FS_G_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1mag_config
 *
 * Description:
 *   Configure the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_config(FAR struct lsm9ds1_dev_s *priv)
{
  uint8_t regval;
  int     ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm9ds1_readreg8(priv, LSM9DS1_WHO_AM_I_M, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  if (regval != LSM9DS1_WHO_AM_I_M_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1mag_start
 *
 * Description:
 *   Start the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_start(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG3_M,
                            LSM9DS1_CTRL_REG3_M_MD_MASK,
                            LSM9DS1_CTRL_REG3_M_MD_CONT);
}

/****************************************************************************
 * Name: lsm9ds1mag_stop
 *
 * Description:
 *   Stop the magnetometer.
 *
 ****************************************************************************/

static int lsm9ds1mag_stop(FAR struct lsm9ds1_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG3_M,
                            LSM9DS1_CTRL_REG3_M_MD_MASK,
                            LSM9DS1_CTRL_REG3_M_MD_POWERDOWN2);
}

/****************************************************************************
 * Name: lsm9ds1mag_setfullscale
 *
 * Description:
 *   Set the magnetometer's full-scale range.
 *
 ****************************************************************************/

static int lsm9ds1mag_setfullscale(FAR struct lsm9ds1_dev_s *priv,
                                   uint32_t fullscale)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  if (fullscale < lsm9ds1_midpoint(4, 8))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_4GAUSS;
    }
  else if (fullscale < lsm9ds1_midpoint(8, 12))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_8GAUSS;
    }
  else if (fullscale < lsm9ds1_midpoint(12, 16))
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_12GAUSS;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG2_M_FS_16GAUSS;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG2_M,
                            LSM9DS1_CTRL_REG2_M_FS_MASK, setbits);
}

/****************************************************************************
 * Name: lsm9ds1mag_setsamplerate
 *
 * Description:
 *   Set the magnetometer's sample rate.
 *
 ****************************************************************************/

static int lsm9ds1mag_setsamplerate(FAR struct lsm9ds1_dev_s *priv,
                                    uint32_t samplerate)
{
  uint8_t setbits;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* The magnetometer can change its sample rate without exiting
   * power-down mode, so we don't need to save the value for later,
   * unlike the accelerometer and gyroscope.
   */

  if (samplerate < lsm9ds1_midpoint(0, 1))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_0p625HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(1, 2))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_1p25HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(2, 5))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_2p5HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(5, 10))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_5HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(10, 20))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_10HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(20, 40))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_20HZ;
    }
  else if (samplerate < lsm9ds1_midpoint(40, 80))
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_40HZ;
    }
  else
    {
      setbits = LSM9DS1_CTRL_REG1_M_DO_80HZ;
    }

  return lsm9ds1_modifyreg8(priv, LSM9DS1_CTRL_REG1_M,
                            LSM9DS1_CTRL_REG1_M_DO_MASK, setbits);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm9ds1_readreg8
 *
 * Description:
 *   Read from an 8-bit register.
 *
 ****************************************************************************/

int lsm9ds1_readreg8(FAR struct lsm9ds1_dev_s *priv,
                     uint8_t regaddr, FAR uint8_t *regval)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM9DS1_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval));
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, *regval);
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_readreg
 *
 * Description:
 *   Read bytes from registers
 *
 ****************************************************************************/

int lsm9ds1_readreg(FAR struct lsm9ds1_dev_s *priv,
                    uint8_t regaddr, FAR uint8_t *regval, uint8_t len)
{
  struct i2c_config_s config;
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM9DS1_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, sizeof(regaddr));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Restart and read 8 bits from the register */

  ret = i2c_read(priv->i2c, &config, regval, sizeof(*regval) * len);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_writereg8
 *
 * Description:
 *   Write to an 8-bit register.
 *
 ****************************************************************************/

int lsm9ds1_writereg8(FAR struct lsm9ds1_dev_s *priv,
                      uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t             buffer[2];
  int                 ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM9DS1_I2C_FREQUENCY;
  config.address   = priv->addr;
  config.addrlen   = 7;

  /* Write the register address followed by the data (no RESTART) */

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  sninfo("addr: %02x value: %02x\n", regaddr, regval);
  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_modifyreg8
 *
 * Description:
 *   Modify an 8-bit register.
 *
 ****************************************************************************/

int lsm9ds1_modifyreg8(FAR struct lsm9ds1_dev_s *priv,
                       uint8_t regaddr, uint8_t clearbits,
                       uint8_t setbits)
{
  uint8_t regval;
  int     ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  ret = lsm9ds1_readreg8(priv, regaddr, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_readreg8 failed: %d\n", ret);
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  ret = lsm9ds1_writereg8(priv, regaddr, regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds1_writereg8 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm9ds1_midpoint
 *
 * Description:
 *   Find the midpoint between two numbers.
 *
 ****************************************************************************/

uint32_t lsm9ds1_midpoint(uint32_t a, uint32_t b)
{
  return (uint32_t)(((uint64_t)a +
                     (uint64_t)b + (uint64_t)1) / (uint64_t)2);
}

