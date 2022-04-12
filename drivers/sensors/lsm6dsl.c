/****************************************************************************
 * drivers/sensors/lsm6dsl.c
 *
 *   Copyright (C) 2018 Inc. All rights reserved.
 *   Author: Ben vd Veen <disruptivesolutionsnl@gmail.com>
 *   Alias: DisruptiveNL
 *
 * Based on:
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <math.h>

#include <nuttx/kmalloc.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6dsl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSL)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LSM6DSL_I2C_FREQUENCY
#  define CONFIG_LSM6DSL_I2C_FREQUENCY 400000
#endif

/* Self test limits. */

#define LSM6DSL_MIN_ST_LIMIT_MG   50.0f      /* Accelerator min limit */
#define LSM6DSL_MAX_ST_LIMIT_MG   1700.0f    /* Accelerator max limit */
#define LSM6DSL_MIN_ST_LIMIT_MDPS 150000.0f  /* Gyroscope min limit */
#define LSM6DSL_MAX_ST_LIMIT_MDPS 700000.0f  /* Gyroscope max limit */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int lsm6dsl_readreg8(FAR struct lsm6dsl_dev_s *priv,
                            uint8_t regaddr, FAR uint8_t * regval);
static int lsm6dsl_writereg8(FAR struct lsm6dsl_dev_s *priv,
                             uint8_t regaddr, uint8_t regval);
static int lsm6dsl_modifyreg8(FAR struct lsm6dsl_dev_s *priv,
                              uint8_t regaddr,
                              uint8_t clearbits, uint8_t setbits);

/* Other Helpers */

static bool lsm6dsl_isbitset(int8_t b, int8_t n);

/* Accelerometer Operations */

static int lsm6dsl_sensor_config(FAR struct lsm6dsl_dev_s *priv);
static int lsm6dsl_sensor_start(FAR struct lsm6dsl_dev_s *priv);
static int lsm6dsl_sensor_stop(FAR struct lsm6dsl_dev_s *priv);
static int lsm6dsl_sensor_read(FAR struct lsm6dsl_dev_s *priv,
                               FAR struct lsm6dsl_sensor_data_s *sdata);
static int lsm6dsl_selftest(FAR struct lsm6dsl_dev_s *priv, uint32_t mode);

/* Character Driver Methods */

static ssize_t lsm6dsl_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lsm6dsl_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen);
static int lsm6dsl_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/* Common Register Function */

static int lsm6dsl_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c,
                            uint8_t addr,
                            FAR const struct lsm6dsl_ops_s *ops,
                            uint8_t datareg,
                            struct lsm6dsl_sensor_data_s sensor_data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static double g_accelerofactor = 0;
static double g_gyrofactor = 0;

static const struct file_operations g_fops =
{
  NULL,            /* open */
  NULL,            /* close */
  lsm6dsl_read,    /* read */
  lsm6dsl_write,   /* write */
  NULL,            /* seek */
  lsm6dsl_ioctl,   /* ioctl */
  NULL             /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* unlink */
#endif
};

static const struct lsm6dsl_ops_s g_lsm6dsl_sensor_ops =
{
  lsm6dsl_sensor_config,
  lsm6dsl_sensor_start,
  lsm6dsl_sensor_stop,
  lsm6dsl_sensor_read,
  lsm6dsl_selftest
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsl_resetsensor
 *
 * Description:
 *   Reset sensor values
 *
 ****************************************************************************/

static void lsm6dsl_resetsensor(FAR struct lsm6dsl_dev_s *priv)
{
  priv->sensor_data.x_data      = 0;
  priv->sensor_data.y_data      = 0;
  priv->sensor_data.z_data      = 0;
  priv->sensor_data.temperature = 0;
  priv->sensor_data.g_x_data    = 0;
  priv->sensor_data.g_y_data    = 0;
  priv->sensor_data.g_z_data    = 0;
  priv->sensor_data.timestamp   = 0;
}

/****************************************************************************
 * Name: lsm6dsl_readreg8
 *
 * Description:
 *   Read from an 8-bit register.
 *
 ****************************************************************************/

static int lsm6dsl_readreg8(FAR struct lsm6dsl_dev_s *priv,
                            uint8_t regaddr, FAR uint8_t * regval)
{
  struct i2c_config_s config;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM6DSL_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

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
 * Name: lsm6dsl_writereg8
 *
 * Description:
 *   Write to an 8-bit register.
 *
 ****************************************************************************/

static int lsm6dsl_writereg8(FAR struct lsm6dsl_dev_s *priv,
                             uint8_t regaddr, uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Set up a 2-byte message to send */

  buffer[0] = regaddr;
  buffer[1] = regval;

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM6DSL_I2C_FREQUENCY;
  config.address = priv->addr;
  config.addrlen = 7;

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
 * Name: lsm6dsl_modifyreg8
 *
 * Description:
 *   Modify an 8-bit register.
 *
 ****************************************************************************/

static int lsm6dsl_modifyreg8(FAR struct lsm6dsl_dev_s *priv,
                              uint8_t regaddr,
                              uint8_t clearbits, uint8_t setbits)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  ret = lsm6dsl_readreg8(priv, regaddr, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm6dsl_readreg8 failed: %d\n", ret);
      return ret;
    }

  regval &= ~clearbits;
  regval |= setbits;

  ret = lsm6dsl_writereg8(priv, regaddr, regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm6dsl_writereg8 failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_sensor_config
 *
 * Description:
 *   Configure the accelerometer and gyroscope.
 *
 ****************************************************************************/

static int lsm6dsl_sensor_config(FAR struct lsm6dsl_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm6dsl_readreg8(priv, LSM6DSL_WHO_AM_I, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm6dsl_readreg8 failed: %d\n", ret);
      return ret;
    }

  if (regval != LSM6DSL_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_isbitset
 *
 * Description:
 *   Check if bit is set from mask, not bit number.
 *
 ****************************************************************************/

static bool lsm6dsl_isbitset(int8_t b, int8_t m)
{
  if ((b & m) != 0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: lsm6dsl_sensor_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/

static int lsm6dsl_sensor_start(FAR struct lsm6dsl_dev_s *priv)
{
  /* Enable the accelerometer */

  /* Reset values */

  lsm6dsl_resetsensor(priv);

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  sninfo("Starting....");

  /* Accelerometer config registers:
   * Turn on the accelerometer: 833Hz, +- 16g
   */

  lsm6dsl_writereg8(priv, LSM6DSL_CTRL1_XL, 0x74);
  g_accelerofactor = 0.488;

  /* Gyro config registers Turn on the gyro: FS=2000dps, ODR=833Hz Not using
   * modifyreg with empty value!!!! Then read value first!!!
   */

  lsm6dsl_writereg8(priv, LSM6DSL_CTRL2_G, 0x7c);
  g_gyrofactor = 70;

  lsm6dsl_writereg8(priv, LSM6DSL_CTRL6_C, 0x00);

  /* Timestamp registers */

  lsm6dsl_writereg8(priv, LSM6DSL_CTRL10_C, 0x20);

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_sensor_stop
 *
 * Description:
 *   Stop the accelerometer.
 *
 ****************************************************************************/

static int lsm6dsl_sensor_stop(FAR struct lsm6dsl_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Stop accelerometer */

  lsm6dsl_modifyreg8(priv,
                     LSM6DSL_CTRL1_XL_ODR_XL_SHIFT,
                     LSM6DSL_CTRL1_XL_ODR_XL_MASK,
                     LSM6DSL_CTRL1_XL_ODR_XL_POWER_DOWN);

  /* Stop gyro */

  lsm6dsl_modifyreg8(priv,
                     LSM6DSL_CTRL2_G_ODR_G_SHIFT,
                     LSM6DSL_CTRL2_G_ODR_G_MASK,
                     LSM6DSL_CTRL2_G_ODR_G_POWER_DOWN);

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_selftest
 *
 * Description:
 *   Selftesting the sensor.
 *   Mode 0 = selftest accelerometer and mode 1 = selftest gyro
 *
 ****************************************************************************/

static int lsm6dsl_selftest(FAR struct lsm6dsl_dev_s *priv, uint32_t mode)
{
  int samples = 5;
  int i;
  int i2;
  int i3;

  uint8_t value = 0;
  int8_t lox    = 0;
  int8_t loxst  = 0;
  int8_t hix    = 0;
  int8_t hixst  = 0;
  int8_t loy    = 0;
  int8_t loyst  = 0;
  int8_t hiy    = 0;
  int8_t hiyst  = 0;
  int8_t loz    = 0;
  int8_t lozst  = 0;
  int8_t hiz    = 0;
  int8_t hizst  = 0;

  int16_t OUTX_NOST[samples];
  int16_t OUTY_NOST[samples];
  int16_t OUTZ_NOST[samples];
  int16_t OUTX_ST[samples];
  int16_t OUTY_ST[samples];
  int16_t OUTZ_ST[samples];

  int16_t avr_x   = 0;
  int16_t avr_y   = 0;
  int16_t avr_z   = 0;
  int16_t avr_xst = 0;
  int16_t avr_yst = 0;
  int16_t avr_zst = 0;
  int16_t test_x  = 0;
  int16_t test_y  = 0;
  int16_t test_z  = 0;

  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  int16_t raw_xst = 0;
  int16_t raw_yst = 0;
  int16_t raw_zst = 0;

  float st_limit_min = 0.0;
  float st_limit_max = 0.0;

  /* mode = 0 then add hex 0x06 to OUT registers */

  int8_t registershift;

  /* Keep the device still during the self-test procedure. Setting registers
   * Power up, wait for 100ms for stable output.
   */

  if (mode == 0)
    {
      registershift = 0x06;

      /* Accelero ; power down gyro CTRL2_G 4g factor: 1000 for mg -> g value
       * is in mg/LSB FS=4g,52Hz 4000mg=65535
       */

      lsm6dsl_writereg8(priv, LSM6DSL_CTRL1_XL, 0x38);
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL2_G, 0x00);
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL3_C, 0x44);
      g_accelerofactor = (0.122 / 1000);
      st_limit_min = LSM6DSL_MIN_ST_LIMIT_MG;
      st_limit_max = LSM6DSL_MAX_ST_LIMIT_MG;
    }
  else
    {
      registershift = 0x00;

      /* Gyro; power down accelero CTRL1_XL FS=2000dps,208Hz 2000dps=65535 */

      lsm6dsl_writereg8(priv, LSM6DSL_CTRL1_XL, 0x00);
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL2_G, 0x5c);
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL3_C, 0x44);
      g_gyrofactor = (70 / 1000); /* 2000dps */
      st_limit_min = LSM6DSL_MIN_ST_LIMIT_MDPS;
      st_limit_max = LSM6DSL_MAX_ST_LIMIT_MDPS;
    }

  lsm6dsl_writereg8(priv, LSM6DSL_CTRL4_C, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL5_C, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL6_C, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL7_G, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL8_XL, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL9_XL, 0x00);
  lsm6dsl_writereg8(priv, LSM6DSL_CTRL10_C, 0x00);

  nxsig_usleep(100000);         /* 100ms */

  /* Read the output registers after checking XLDA bit 5 times */

  bool checkbit = false;

  /* Wait until first sample and data is available */

  while (checkbit)
    {
      lsm6dsl_readreg8(priv, LSM6DSL_STATUS_REG, &value);
      if (mode == 0)
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_XLDA);
        }
      else
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_GDA);
        }
    }

  nxsig_usleep(100000);    /* 100ms */

  /* Read OUT registers Gyro is starting at 22h and Accelero at 28h */

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTX_L_G + registershift,
                   (FAR uint8_t *)&lox);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTX_H_G + registershift,
                   (FAR uint8_t *)&hix);

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTY_L_G + registershift,
                   (FAR uint8_t *)&loy);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTY_H_G + registershift,
                   (FAR uint8_t *)&hiy);

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTZ_L_G + registershift,
                   (FAR uint8_t *)&loz);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTZ_H_G + registershift,
                   (FAR uint8_t *)&hiz);

  /* check XLDA 5 times */

  for (i = 0; i < samples; i++)
    {
      lsm6dsl_readreg8(priv, LSM6DSL_STATUS_REG, &value);
      if (mode == 0)
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_XLDA);
        }
      else
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_GDA);
        }

      /* Average the stored data on each axis
       * http://ozzmaker.com/accelerometer-to-g/
       */

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTX_L_G + registershift,
                       (FAR uint8_t *)&lox);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTX_H_G + registershift,
                       (FAR uint8_t *)&hix);
      raw_x = (int16_t) (((uint16_t) hix << 8U) | (uint16_t) lox);

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTY_L_G + registershift,
                       (FAR uint8_t *)&loy);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTY_H_G + registershift,
                       (FAR uint8_t *)&hiy);
      raw_y = (int16_t) (((uint16_t) hiy << 8U) | (uint16_t) loy);

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTZ_L_G + registershift,
                       (FAR uint8_t *)&loz);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTZ_H_G + registershift,
                       (FAR uint8_t *)&hiz);
      raw_z = (int16_t) (((uint16_t) hiz << 8U) | (uint16_t) loz);

      /* Selftest only uses raw values */

      OUTX_NOST[i] = raw_x;
      OUTY_NOST[i] = raw_y;
      OUTZ_NOST[i] = raw_z;
    }

  /* Enable Selftest */

  if (mode == 0)
    {
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL5_C, 0x01);
    }
  else
    {
      lsm6dsl_writereg8(priv, LSM6DSL_CTRL5_C, 0x04);
    }

  nxsig_usleep(100000);         /* 100ms */

  checkbit = false;
  while (checkbit)              /* wait until first sample and data is
                                 * available */
    {
      lsm6dsl_readreg8(priv, LSM6DSL_STATUS_REG, &value);
      if (mode == 0)
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_XLDA);
        }
      else
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_GDA);
        }
    }

  nxsig_usleep(100000);         /* 100ms */

  /* Now do all the ST values */

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTX_L_G + registershift,
                   (FAR uint8_t *)&loxst);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTX_H_G + registershift,
                   (FAR uint8_t *)&hixst);

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTY_L_G + registershift,
                   (FAR uint8_t *)&loyst);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTY_H_G + registershift,
                   (FAR uint8_t *)&hiyst);

  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTZ_L_G + registershift,
                   (FAR uint8_t *)&lozst);
  lsm6dsl_readreg8(priv,
                   LSM6DSL_OUTZ_H_G + registershift,
                   (FAR uint8_t *)&hizst);

  for (i2 = 0; i2 < samples; i2++)
    {
      lsm6dsl_readreg8(priv, LSM6DSL_STATUS_REG, &value);
      if (mode == 0)
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_XLDA);
        }
      else
        {
          checkbit = lsm6dsl_isbitset(value, LSM6DSL_STATUS_REG_GDA);
        }

      nxsig_usleep(100000);     /* 100ms */

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTX_L_G + registershift,
                       (FAR uint8_t *)&loxst);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTX_H_G + registershift,
                       (FAR uint8_t *)&hixst);
      raw_xst = (int16_t) (((uint16_t) hixst << 8U) | (uint16_t) loxst);

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTY_L_G + registershift,
                       (FAR uint8_t *)&loyst);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTY_H_G + registershift,
                       (FAR uint8_t *)&hiyst);
      raw_yst = (int16_t) (((uint16_t) hiyst << 8U) | (uint16_t) loyst);

      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTZ_L_G + registershift,
                       (FAR uint8_t *)&lozst);
      lsm6dsl_readreg8(priv,
                       LSM6DSL_OUTZ_H_G + registershift,
                       (FAR uint8_t *)&hizst);
      raw_zst = (int16_t) (((uint16_t) hizst << 8U) | (uint16_t) lozst);

      /* Selftest only uses raw values */

      OUTX_ST[i2] = raw_xst;
      OUTY_ST[i2] = raw_yst;
      OUTZ_ST[i2] = raw_zst;
    }

  /* Average stored data on each axis */

  for (i3 = 0; i3 < samples; i3++)
    {
      avr_x = avr_x + (int16_t) OUTX_NOST[i3];
      avr_y = avr_y + (int16_t) OUTY_NOST[i3];
      avr_z = avr_z + (int16_t) OUTZ_NOST[i3];

      avr_xst = avr_xst + (int16_t) OUTX_ST[i3];
      avr_yst = avr_yst + (int16_t) OUTY_ST[i3];
      avr_zst = avr_zst + (int16_t) OUTZ_ST[i3];
    }

  avr_x = (int16_t) avr_x / samples;
  avr_y = (int16_t) avr_y / samples;
  avr_z = (int16_t) avr_z / samples;

  avr_xst = (int16_t) avr_xst / samples;
  avr_yst = (int16_t) avr_yst / samples;
  avr_zst = (int16_t) avr_zst / samples;

  sninfo("avr_x: %d\n", avr_x);
  sninfo("avr_y: %d\n", avr_y);
  sninfo("avr_z: %d\n", avr_z);

  test_x = fabs(avr_xst - avr_xst);
  test_y = fabs(avr_yst - avr_yst);
  test_z = fabs(avr_zst - avr_zst);

  /* Validation Question is placed at ST FAE because the equation in the
   * datasheet is doubtful.
   */

  if (test_x >= st_limit_min && test_x <= st_limit_max)
    {
      sninfo("PASSED NOST AND ST FOR X!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR X!\n");
      sninfo("[test_x: %d min: %f - max: %f ]"
            , test_x
            , st_limit_min
            , st_limit_max);
      sninfo("\n");
    }

  if (test_y >= st_limit_min && test_y <= st_limit_max)
    {
      sninfo("PASSED NOST AND ST FOR Y!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR Y!\n");
      sninfo("[test_y: %d min: %f - max: %f ]"
            , test_y
            , st_limit_min
            , st_limit_max);
      sninfo("\n");
    }

  if (test_z >= st_limit_min && test_z <= st_limit_max)
    {
      sninfo("PASSED NOST AND ST FOR Z!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR Z!\n");
      sninfo("[test_z: %d min: %f - max: %f ]"
            , test_z
            , st_limit_min
            , st_limit_max);
      sninfo("\n");
    }

  nxsig_sleep(2);

  /* Disable test */

  switch (mode)
    {
    case 0:
      {
        sninfo("SELFTEST ACCELERO DISABLED\n");
        lsm6dsl_writereg8(priv, LSM6DSL_CTRL1_XL, 0x00);
        lsm6dsl_writereg8(priv, LSM6DSL_CTRL5_C, 0x00);
      }
      break;
    case 1:
      {
        sninfo("SELFTEST GYRO DISABLED\n");
        lsm6dsl_writereg8(priv, LSM6DSL_CTRL2_G, 0x00);
        lsm6dsl_writereg8(priv, LSM6DSL_CTRL5_C, 0x00);
      }
      break;

    default:
      break;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_sensor_read
 *
 * Description:
 *   Read the sensor.
 *   A sensor in a steady state on a horizontal surface will
 *   measure 0 g on both the X-axis and Y-axis, whereas the Z-axis will
 *   measure 1 g. (page 30 datasheet). The X- and Y-axis have an offset
 *   of 40 mg/LSB
 *
 ****************************************************************************/

static int lsm6dsl_sensor_read(FAR struct lsm6dsl_dev_s *priv,
                               FAR struct lsm6dsl_sensor_data_s *sdata)
{
  int16_t lox   = 0;
  int16_t loxg  = 0;
  int16_t hix   = 0;
  int16_t hixg  = 0;
  int16_t loy   = 0;
  int16_t loyg  = 0;
  int16_t hiy   = 0;
  int16_t hiyg  = 0;
  int16_t loz   = 0;
  int16_t lozg  = 0;
  int16_t hiz   = 0;
  int16_t hizg  = 0;

  int16_t templ = 0;
  int16_t temph = 0;

  uint8_t tstamp0 = 0;
  uint8_t tstamp1 = 0;
  uint8_t tstamp2 = 0;
  uint32_t ts     = 0;

  int16_t tempi = 0;
  int16_t temp_val = 0;

  int16_t x_valg = 0;
  int16_t y_valg = 0;
  int16_t z_valg = 0;

  int16_t xf_val = 0;
  int16_t yf_val = 0;
  int16_t zf_val = 0;

  /* Accelerometer */

  lsm6dsl_readreg8(priv, LSM6DSL_OUTX_L_XL, (FAR uint8_t *)&lox);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTX_H_XL, (FAR uint8_t *)&hix);

  lsm6dsl_readreg8(priv, LSM6DSL_OUTY_L_XL, (FAR uint8_t *)&loy);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTY_H_XL, (FAR uint8_t *)&hiy);

  lsm6dsl_readreg8(priv, LSM6DSL_OUTZ_L_XL, (FAR uint8_t *)&loz);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTZ_H_XL, (FAR uint8_t *)&hiz);

  /* Gyro */

  lsm6dsl_readreg8(priv, LSM6DSL_OUTX_L_G, (FAR uint8_t *)&loxg);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTX_H_G, (FAR uint8_t *)&hixg);

  lsm6dsl_readreg8(priv, LSM6DSL_OUTY_L_G, (FAR uint8_t *)&loyg);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTY_H_G, (FAR uint8_t *)&hiyg);

  lsm6dsl_readreg8(priv, LSM6DSL_OUTZ_L_G, (FAR uint8_t *)&lozg);
  lsm6dsl_readreg8(priv, LSM6DSL_OUTZ_H_G, (FAR uint8_t *)&hizg);

  /* Timestamp */

  lsm6dsl_readreg8(priv, LSM6DSL_TIMESTAMP0_REG, &tstamp0);
  lsm6dsl_readreg8(priv, LSM6DSL_TIMESTAMP1_REG, &tstamp1);
  lsm6dsl_readreg8(priv, LSM6DSL_TIMESTAMP2_REG, &tstamp2);

  ts = (tstamp2 << 16) | (tstamp1 << 8) | tstamp0;

  /* Temperature */

  lsm6dsl_readreg8(priv, LSM6DSL_OUT_TEMP_L, (FAR uint8_t *)&templ);
  lsm6dsl_readreg8(priv, LSM6DSL_OUT_TEMP_H, (FAR uint8_t *)&temph);

  xf_val = (int16_t) ((hix << 8) | lox);
  yf_val = (int16_t) ((hiy << 8) | loy);
  zf_val = (int16_t) ((hiz << 8) | loz);

  tempi = (int16_t) ((((int16_t) temph << 8) | (int16_t) templ));

  temp_val = (tempi / 256) + 25;

  sninfo("Data 16-bit XL_X--->: %d mg\n",
         (short)(xf_val * g_accelerofactor));
  sninfo("Data 16-bit XL_Y--->: %d mg\n",
         (short)(yf_val * g_accelerofactor));
  sninfo("Data 16-bit XL_Z--->: %d mg\n",
         (short)(zf_val * g_accelerofactor));
  sninfo("Data 16-bit TEMP--->: %d Celsius\n",
         temp_val);

  sdata->x_data = xf_val * g_accelerofactor;
  sdata->y_data = yf_val * g_accelerofactor;
  sdata->z_data = zf_val * g_accelerofactor;
  sdata->temperature = temp_val;
  sdata->timestamp = ts;

  x_valg = (int16_t) (((hixg) << 8) | loxg);
  y_valg = (int16_t) (((hiyg) << 8) | loyg);
  z_valg = (int16_t) (((hizg) << 8) | lozg);

  sninfo("Data 16-bit G_X--->: %d mdps\n", (short)(x_valg * g_gyrofactor));
  sninfo("Data 16-bit G_Y--->: %d mdps\n", (short)(y_valg * g_gyrofactor));
  sninfo("Data 16-bit G_Z--->: %d mdps\n", (short)(z_valg * g_gyrofactor));

  sdata->g_x_data = x_valg * g_gyrofactor;
  sdata->g_y_data = y_valg * g_gyrofactor;
  sdata->g_z_data = z_valg * g_gyrofactor;

  return OK;
}

/****************************************************************************
 * Name: lsm6dsl_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t lsm6dsl_read(FAR struct file *filep,
                            FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode;
  FAR struct lsm6dsl_dev_s *priv;
  int ret;
  size_t i;
  size_t j;
  size_t samplesize;
  size_t nsamples;
  uint16_t data;
  FAR int16_t *ptr;
  uint8_t regaddr;
  uint8_t lo;
  uint8_t hi;
  uint32_t merge = 0;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct lsm6dsl_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->datareg == LSM6DSL_OUTX_L_G_SHIFT ||
              priv->datareg == LSM6DSL_OUTX_L_XL_SHIFT);
  DEBUGASSERT(buffer != NULL);

  samplesize = 3 * sizeof(*ptr);
  nsamples = buflen / samplesize;
  ptr = (FAR int16_t *) buffer;

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      /* Reset the register address to the X low byte register */

      regaddr = priv->datareg;

      /* Read the X, Y and Z data */

      for (j = 0; j < 3; j++)
        {
          /* Read the low byte */

          ret = lsm6dsl_readreg8(priv, regaddr, &lo);
          if (ret < 0)
            {
              snerr("ERROR: lsm6dsl_readreg8 failed: %d\n", ret);
              return (ssize_t) ret;
            }

          regaddr++;

          /* Read the high byte */

          ret = lsm6dsl_readreg8(priv, regaddr, &hi);
          if (ret < 0)
            {
              snerr("ERROR: lsm6dsl_readreg8 failed: %d\n", ret);
              return (ssize_t) ret;
            }

          regaddr++;

          /* The data is 16 bits in two's complement representation */

          data = ((uint16_t) hi << 8) | (uint16_t) lo;

          /* Collect entropy */

          merge += data ^ (merge >> 16);

          /* The value is positive */

          if (data < 0x8000)
            {
              ptr[j] = (int16_t) data;
            }

          /* The value is negative, so find its absolute value by taking the
           * two's complement
           */

          else if (data > 0x8000)
            {
              data = ~data + 1;
              ptr[j] = -(int16_t) data;
            }

          /* The value is negative and can't be represented as a positive
           * int16_t value
           */

          else
            {
              ptr[j] = (int16_t) (-32768);
            }
        }
    }

  /* Feed sensor data to entropy pool */

  add_sensor_randomness(merge);

  return nsamples * samplesize;
}

/****************************************************************************
 * Name: lsm6dsl_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t lsm6dsl_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lsm6dsl_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int lsm6dsl_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct lsm6dsl_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct lsm6dsl_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Start converting. Arg: None. */

    case SNIOC_START:
      ret = priv->ops->start(priv);
      break;

      /* Stop converting. Arg: None. */

    case SNIOC_STOP:
      ret = priv->ops->stop(priv);
      break;

    case SNIOC_LSM6DSLSENSORREAD:
      ret = priv->ops->sensor_read(priv,
                                   (FAR struct lsm6dsl_sensor_data_s *) arg);
      break;

    case SNIOC_START_SELFTEST:
      ret = priv->ops->selftest(priv, (uint32_t) arg);
      break;

      /* Unrecognized commands */

    default:
      {
        snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
        ret = -ENOTTY;
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: lsm6dsl_register
 *
 * Description:
 *   Register the LSM6DSL accelerometer, gyroscope device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g.
 *             "/dev/lsm6dsl0", "/dev/gyro0" or "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM6DSL accelerometer, gyroscope or
 *             magnetometer.
 *   ops     - The device operations structure.
 *   datareg - The register address of the low byte of the X-coordinate data.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dsl_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c,
                            uint8_t addr,
                            FAR const struct lsm6dsl_ops_s *ops,
                            uint8_t datareg,
                            struct lsm6dsl_sensor_data_s sensor_data)
{
  FAR struct lsm6dsl_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(datareg == LSM6DSL_OUTX_L_XL_SHIFT ||
              datareg == LSM6DSL_OUTX_L_G_SHIFT);

  /* Initialize the device's structure */

  priv = (FAR struct lsm6dsl_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c         = i2c;
  priv->addr        = addr;
  priv->ops         = ops;
  priv->datareg     = datareg;
  priv->sensor_data = sensor_data;

  /* Configure the device */

  ret = priv->ops->config(priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to configure device: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  /* Register the character driver */

  ret = register_driver(devpath, &g_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsl_sensor_register
 *
 * Description:
 *   Register the LSM6DSL accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register,
 *             e.g. "/dev/lsm6dsl0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM6DSL accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dsl_sensor_register(FAR const char *devpath,
                            FAR struct i2c_master_s *i2c, uint8_t addr)
{
  struct lsm6dsl_sensor_data_s sensor_data;

  DEBUGASSERT(addr == LSM6DSLACCEL_ADDR0 || addr == LSM6DSLACCEL_ADDR1);

  sninfo("Trying to register accel\n");

  return lsm6dsl_register(devpath, i2c, addr, &g_lsm6dsl_sensor_ops,
                          LSM6DSL_OUTX_L_XL_SHIFT, sensor_data);
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM6DSL */
