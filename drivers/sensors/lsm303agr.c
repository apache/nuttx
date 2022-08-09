/****************************************************************************
 * drivers/sensors/lsm303agr.c
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

#include <nuttx/kmalloc.h>
#include <nuttx/random.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm303agr.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM303AGR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LSM303AGR_I2C_FREQUENCY
#  define CONFIG_LSM303AGR_I2C_FREQUENCY 400000
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C Helpers */

static int lsm303agr_readreg8(FAR struct lsm303agr_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t * regval);
static int lsm303agr_writereg8(FAR struct lsm303agr_dev_s *priv,
                               uint8_t regaddr, uint8_t regval);

/* Other Helpers */

static int lsm303agr_find_minimum(int16_t a[], int n);
static int lsm303agr_find_maximum(int16_t a[], int n);
static bool lsm303agr_isbitset(int8_t b, int8_t n);

/* Accelerometer Operations */

static int lsm303agr_sensor_config(FAR struct lsm303agr_dev_s *priv);
static int lsm303agr_sensor_start(FAR struct lsm303agr_dev_s *priv);
static int lsm303agr_sensor_stop(FAR struct lsm303agr_dev_s *priv);
static int lsm303agr_sensor_read(FAR struct lsm303agr_dev_s *priv,
                                 FAR struct lsm303agr_sensor_data_s *sdata);
static int lsm303agr_selftest(FAR struct lsm303agr_dev_s *priv,
                              uint32_t mode);

/* Character Driver Methods */

static ssize_t lsm303agr_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t lsm303agr_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int lsm303agr_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/* Common Register Function */

static int lsm303agr_register(FAR const char *devpath,
                              FAR struct i2c_master_s *i2c,
                              uint8_t addr,
                              FAR const struct lsm303agr_ops_s *ops,
                              uint8_t datareg,
                              struct lsm303agr_sensor_data_s sensor_data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static double g_accelerofactor = 0;
static double g_magnetofactor = 0;

static const struct file_operations g_fops =
{
  NULL,               /* open */
  NULL,               /* close */
  lsm303agr_read,     /* read */
  lsm303agr_write,    /* write */
  NULL,               /* seek */
  lsm303agr_ioctl,    /* ioctl */
  NULL                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};

static const struct lsm303agr_ops_s g_lsm303agrsensor_ops =
{
  lsm303agr_sensor_config,
  lsm303agr_sensor_start,
  lsm303agr_sensor_stop,
  lsm303agr_sensor_read,
  lsm303agr_selftest
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303agr_resetsensor
 *
 * Description:
 *   Reset sensor values
 *
 ****************************************************************************/

static void lsm303agr_resetsensor(FAR struct lsm303agr_dev_s *priv)
{
  priv->sensor_data.x_data      = 0;
  priv->sensor_data.y_data      = 0;
  priv->sensor_data.z_data      = 0;
  priv->sensor_data.temperature = 0;
  priv->sensor_data.m_x_data    = 0;
  priv->sensor_data.m_y_data    = 0;
  priv->sensor_data.m_z_data    = 0;
  priv->sensor_data.timestamp   = 0;
}

/****************************************************************************
 * Name: lsm303agr_readreg8
 *
 * Description:
 *   Read from an 8-bit register.
 *
 ****************************************************************************/

static int lsm303agr_readreg8(FAR struct lsm303agr_dev_s *priv,
                              uint8_t regaddr, FAR uint8_t * regval)
{
  struct i2c_config_s config;
  int ret;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(regval != NULL);

  /* Set up the I2C configuration */

  config.frequency = CONFIG_LSM303AGR_I2C_FREQUENCY;
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
 * Name: lsm303agr_writereg8
 *
 * Description:
 *   Write to an 8-bit register.
 *
 ****************************************************************************/

static int lsm303agr_writereg8(FAR struct lsm303agr_dev_s *priv,
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

  config.frequency = CONFIG_LSM303AGR_I2C_FREQUENCY;
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
 * Name: lsm303agr_find_minimum
 *
 * Description:
 *   Find the minimum value in an array of numbers.
 *
 ****************************************************************************/

static int lsm303agr_find_minimum(int16_t a[], int n)
{
  int c;
  int min;
  int index;

  min = a[0];
  index = 0;

  for (c = 1; c < n; c++)
    {
      if (a[c] < min)
        {
          index = c;
          min = a[c];
        }
    }

  return index;
}

/****************************************************************************
 * Name: lsm303agr_find_maximum
 *
 * Description:
 *   Find the maximum value in an array of numbers.
 *
 ****************************************************************************/

static int lsm303agr_find_maximum(int16_t am[], int n)
{
  int c;
  int max = am[0];
  int index = 0;

  for (c = 1; c < n; c++)
    {
      if (am[c] > max)
        {
          index = c;
          max = am[c];
        }
    }

  return index;
}

/****************************************************************************
 * Name: lsm303agr_sensor_config
 *
 * Description:
 *   Configure the accelerometer and gyroscope.
 *
 ****************************************************************************/

static int lsm303agr_sensor_config(FAR struct lsm303agr_dev_s *priv)
{
  int ret;
  uint8_t regval;

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Get the device identification */

  ret = lsm303agr_readreg8(priv, LSM303AGR_WHO_AM_I, &regval);
  if (ret < 0)
    {
      snerr("ERROR: lsm303agr_readreg8 failed: %d\n", ret);
      return ret;
    }

  sninfo("WHO AM I check address regval: %02x reg: %d....\n", regval, ret);

  if (regval != LSM303AGR_WHO_AM_I_VALUE)
    {
      snerr("ERROR: Invalid device identification %02x\n", regval);
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_isbitset
 *
 * Description:
 *   Check if bit is set from mask, not bit number.
 *
 ****************************************************************************/

static bool lsm303agr_isbitset(int8_t b, int8_t m)
{
  if ((b & m) != 0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: lsm303agr_sensor_start
 *
 * Description:
 *   Start the accelerometer.
 *
 ****************************************************************************/

static int lsm303agr_sensor_start(FAR struct lsm303agr_dev_s *priv)
{
  /* readreg8 is not necessary to modify. Clearbits can be 0x00 or 0xff */

  /* Enable the accelerometer */

  /* Reset values */

  lsm303agr_resetsensor(priv);

  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  sninfo("Starting....");

  /* Accelerometer config registers:
   * Turn on the accelerometer: 833Hz, +- 16g
   */

  lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG1_A, 0x77);
  lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG4_A, 0xb0);
  g_accelerofactor = 11.72;

  /* Gyro config registers Turn on the gyro: FS=2000dps, ODR=833Hz Not using
   * modifyreg with empty value!!!! Then read value first!!!
   * lsm303agr_modifyreg8(priv, lsm303agr_CTRL2_G, value, 0x7c);
   */

  lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_A_M, 0x8c);
  g_magnetofactor = 1.5;

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_sensor_stop
 *
 * Description:
 *   Stop the sensor.
 *
 ****************************************************************************/

static int lsm303agr_sensor_stop(FAR struct lsm303agr_dev_s *priv)
{
  /* Sanity check */

  DEBUGASSERT(priv != NULL);

  /* Stop accelero: not implemented [datasheet] */

  /* Stop magneto: not implemented [datasheet] */

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_selftest
 *
 * Description:
 *   Selftesting the sensor.
 *   Mode 0 = selftest accelerometer and mode 1 = selftest magneto
 *
 ****************************************************************************/

static int lsm303agr_selftest(FAR struct lsm303agr_dev_s *priv,
                              uint32_t mode)
{
  int samples = 5;
  int i;
  int i2;
  int i3;

  uint8_t value;
  int8_t lox   = 0;
  int8_t loxst = 0;
  int8_t hix   = 0;
  int8_t hixst = 0;
  int8_t loy   = 0;
  int8_t loyst = 0;
  int8_t hiy   = 0;
  int8_t hiyst = 0;
  int8_t loz   = 0;
  int8_t lozst = 0;
  int8_t hiz   = 0;
  int8_t hizst = 0;

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

  int16_t min_x = 0;
  int16_t min_y = 0;
  int16_t min_z = 0;
  int16_t max_x = 0;
  int16_t max_y = 0;
  int16_t max_z = 0;

  int16_t min_xst = 0;
  int16_t min_yst = 0;
  int16_t min_zst = 0;
  int16_t max_xst = 0;
  int16_t max_yst = 0;
  int16_t max_zst = 0;

  int16_t raw_xst = 0;
  int16_t raw_yst = 0;
  int16_t raw_zst = 0;

  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  /* mode = 0 then add hex 0x06 to OUT registers */

  int8_t registershift;

  /* Keep the device still during the self-test procedure. Setting registers
   * Power up, wait for 100ms for stable output.
   */

  if (mode == 0)
    {
      registershift = 0x00;
      lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG2_A, 0x00);
      lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG3_A, 0x00);
      lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG4_A, 0x81);
      lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG1_A, 0x57);
      g_accelerofactor = 1;
    }
  else
    {
      registershift = 0x40;
      lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_A_M, 0x8c);
      lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_B_M, 0x02);
      lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_C_M, 0x10);
      g_magnetofactor = 1;
    }

  nxsig_usleep(100000);         /* 100ms */

  /* Read the output registers after checking XLDA bit 5 times */

  bool checkbit = false;

  /* Wait until first sample and data is available */

  while (checkbit)
    {
      if (mode == 0)
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_A, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_A_ZYXDA);
        }
      else
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_M, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_M_ZYXDA);
        }
    }

  nxsig_usleep(100000);         /* 100ms */

  /* Read OUT registers Gyro is starting at 22h and Accelero at 28h */

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_X_L_A + registershift,
                     (FAR uint8_t *)&lox);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_X_H_A + registershift,
                     (FAR uint8_t *)&hix);

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Y_L_A + registershift,
                     (FAR uint8_t *)&loy);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Y_H_A + registershift,
                     (FAR uint8_t *)&hiy);

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Z_L_A + registershift,
                     (FAR uint8_t *)&loz);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Z_H_A + registershift,
                     (FAR uint8_t *)&hiz);

  /* check XLDA 5 times */

  for (i = 0; i < samples; i++)
    {
      if (mode == 0)
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_A, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_A_ZYXDA);
        }
      else
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_M, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_M_ZYXDA);
        }

      /* Average the stored data on each axis *
       * http://ozzmaker.com/accelerometer-to-g/
       */

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_X_L_A + registershift,
                         (FAR uint8_t *)&lox);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_X_H_A + registershift,
                         (FAR uint8_t *)&hix);
      raw_x = (int16_t) (((uint16_t) hix << 8U) | (uint16_t) lox);

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Y_L_A + registershift,
                         (FAR uint8_t *)&loy);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Y_H_A + registershift,
                         (FAR uint8_t *)&hiy);
      raw_y = (int16_t) (((uint16_t) hiy << 8U) | (uint16_t) loy);

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Z_L_A + registershift,
                         (FAR uint8_t *)&loz);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Z_H_A + registershift,
                         (FAR uint8_t *)&hiz);
      raw_z = (int16_t) (((uint16_t) hiz << 8U) | (uint16_t) loz);

      /* selftest only uses raw values */

      OUTX_NOST[i] = raw_x;
      OUTY_NOST[i] = raw_y;
      OUTZ_NOST[i] = raw_z;
    }

  /* Enable Selftest */

  if (mode == 0)
    {
      lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG4_A, 0x85);
    }
  else
    {
      lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_C_M, 0x12);
    }

  nxsig_usleep(100000);         /* 100ms */

  checkbit = false;
  while (checkbit)
    {
      if (mode == 0)
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_A, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_A_ZYXDA);
        }
      else
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_M, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_M_ZYXDA);
        }
    }

  nxsig_usleep(100000);    /* 100ms */

  /* Now do all the ST values */

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_X_L_A + registershift,
                     (FAR uint8_t *)&loxst);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_X_H_A + registershift,
                     (FAR uint8_t *)&hixst);

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Y_L_A + registershift,
                     (FAR uint8_t *)&loyst);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Y_H_A + registershift,
                     (FAR uint8_t *)&hiyst);

  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Z_L_A + registershift,
                     (FAR uint8_t *)&lozst);
  lsm303agr_readreg8(priv,
                     LSM303AGR_OUT_Z_H_A + registershift,
                     (FAR uint8_t *)&hizst);

  for (i2 = 0; i2 < samples; i2++)
    {
      if (mode == 0)
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_A, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_A_ZYXDA);
        }
      else
        {
          lsm303agr_readreg8(priv, LSM303AGR_STATUS_REG_M, &value);
          checkbit = lsm303agr_isbitset(value, LSM303AGR_STATUS_REG_M_ZYXDA);
        }

      nxsig_usleep(100000);    /* 100ms */

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_X_L_A + registershift,
                         (FAR uint8_t *)&loxst);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_X_H_A + registershift,
                         (FAR uint8_t *)&hixst);
      raw_xst = (int16_t) (((uint16_t) hixst << 8U) | (uint16_t) loxst);

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Y_L_A + registershift,
                         (FAR uint8_t *)&loyst);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Y_H_A + registershift,
                         (FAR uint8_t *)&hiyst);
      raw_yst = (int16_t) (((uint16_t) hiyst << 8U) | (uint16_t) loyst);

      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Z_L_A + registershift,
                         (FAR uint8_t *)&lozst);
      lsm303agr_readreg8(priv,
                         LSM303AGR_OUT_Z_H_A + registershift,
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

  min_x = OUTX_NOST[lsm303agr_find_minimum(OUTX_NOST, samples)];
  min_y = OUTY_NOST[lsm303agr_find_minimum(OUTY_NOST, samples)];
  min_z = OUTZ_NOST[lsm303agr_find_minimum(OUTZ_NOST, samples)];

  max_x = OUTX_NOST[lsm303agr_find_maximum(OUTX_NOST, samples)];
  max_y = OUTY_NOST[lsm303agr_find_maximum(OUTY_NOST, samples)];
  max_z = OUTZ_NOST[lsm303agr_find_maximum(OUTZ_NOST, samples)];

  min_xst = OUTX_ST[lsm303agr_find_minimum(OUTX_ST, samples)];
  min_yst = OUTY_ST[lsm303agr_find_minimum(OUTY_ST, samples)];
  min_zst = OUTZ_ST[lsm303agr_find_minimum(OUTZ_ST, samples)];

  max_xst = OUTX_ST[lsm303agr_find_maximum(OUTX_ST, samples)];
  max_yst = OUTY_ST[lsm303agr_find_maximum(OUTY_ST, samples)];
  max_zst = OUTZ_ST[lsm303agr_find_maximum(OUTZ_ST, samples)];

  sninfo("stdev_x: -%d %d +%d\n", avr_x - min_x, avr_x, max_x - avr_x);
  sninfo("stdev_y: -%d %d +%d\n", avr_y - min_y, avr_y, max_y - avr_y);
  sninfo("stdev_z: -%d %d +%d\n", avr_z - min_z, avr_z, max_z - avr_z);

  sninfo("stdev_xst: -%d %d +%d\n", avr_xst - min_xst, avr_xst,
         max_xst - avr_xst);
  sninfo("stdev_yst: -%d %d +%d\n", avr_yst - min_yst, avr_yst,
         max_yst - avr_yst);
  sninfo("stdev_zst: -%d %d +%d\n", avr_zst - min_zst, avr_zst,
         max_zst - avr_zst);

  sninfo("avr_x: %d\n", avr_x);
  sninfo("avr_y: %d\n", avr_y);
  sninfo("avr_z: %d\n", avr_z);
  sninfo("min_x: %d\n", min_x);
  sninfo("max_x: %d\n", max_x);
  sninfo("min_xst: %d\n", min_xst);
  sninfo("max_xst: %d\n", max_xst);

  /* Validation */

  if ((avr_x >= min_x && avr_x <= max_x) &&
      (avr_xst >= min_xst && avr_xst <= max_xst))
    {
      sninfo("PASSED NOST AND ST FOR X!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR X!\n");
      sninfo("[ %d - %d ]", min_x, min_xst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", avr_x, avr_xst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", max_x, max_xst);
      sninfo("\n");
    }

  if ((avr_y >= min_y && avr_y <= max_y) &&
      (avr_yst >= min_yst && avr_yst <= max_yst))
    {
      sninfo("PASSED NOST AND ST FOR Y!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR Y!\n");
      sninfo("[ %d - %d ]", min_y, min_yst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", avr_y, avr_yst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", max_y, max_yst);
      sninfo("\n");
    }

  if ((avr_z >= min_z && avr_z <= max_z) &&
      (avr_zst >= min_zst && avr_zst <= max_zst))
    {
      sninfo("PASSED NOST AND ST FOR Z!\n");
    }
  else
    {
      sninfo("FAILED NOST AND ST FOR Z!\n");
      sninfo("[ %d - %d ]", min_z, min_zst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", avr_z, avr_zst);
      sninfo(" <=\n ");
      sninfo("[ %d - %d ]", max_z, max_zst);
      sninfo("\n");
    }

  nxsig_sleep(2);

  /* Disable test */

  switch (mode)
    {
    case 0:
      {
        sninfo("SELFTEST ACCELERO DISABLED\n");
        lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG1_A, 0x00);
        lsm303agr_writereg8(priv, LSM303AGR_CTRL_REG4_A, 0x01);
      }
      break;

    case 1:
      {
        sninfo("SELFTEST MAGNETO DISABLED\n");
        lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_C_M, 0x10);
        lsm303agr_writereg8(priv, LSM303AGR_CFG_REG_A_M, 0x83);
      }
      break;

    default:
      break;
    }

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_sensor_read
 *
 * Description:
 *   Read the sensor.
 *   A sensor in a steady state on a horizontal surface will
 *   measure 0 g on both the X-axis and Y-axis, whereas the Z-axis will
 *   measure 1 g. (page 30 datasheet). The X- and Y-axis have an offset
 *   of 40 mg/LSB
 *
 ****************************************************************************/

static int lsm303agr_sensor_read(FAR struct lsm303agr_dev_s *priv,
                                 FAR struct lsm303agr_sensor_data_s *sdata)
{
  int16_t loxg  = 0;
  int16_t hixg  = 0;
  int16_t loyg  = 0;
  int16_t hiyg  = 0;
  int16_t lozg  = 0;
  int16_t hizg  = 0;

  int16_t templ = 0;
  int16_t temph = 0;

  int16_t tempi = 0;
  int16_t temp_val = 0;

  int16_t x_valg = 0;
  int16_t y_valg = 0;
  int16_t z_valg = 0;

  /* Magneto */

  lsm303agr_readreg8(priv, LSM303AGR_OUTX_L_REG_M, (FAR uint8_t *)&loxg);
  lsm303agr_readreg8(priv, LSM303AGR_OUTX_H_REG_M, (FAR uint8_t *)&hixg);

  lsm303agr_readreg8(priv, LSM303AGR_OUTY_L_REG_M, (FAR uint8_t *)&loyg);
  lsm303agr_readreg8(priv, LSM303AGR_OUTY_H_REG_M, (FAR uint8_t *)&hiyg);

  lsm303agr_readreg8(priv, LSM303AGR_OUTZ_L_REG_M, (FAR uint8_t *)&lozg);
  lsm303agr_readreg8(priv, LSM303AGR_OUTZ_H_REG_M, (FAR uint8_t *)&hizg);

  /* Temperature */

  lsm303agr_readreg8(priv, LSM303AGR_OUT_TEMP_L_A, (FAR uint8_t *)&templ);
  lsm303agr_readreg8(priv, LSM303AGR_OUT_TEMP_H_A, (FAR uint8_t *)&temph);

  tempi = (int16_t) ((((int16_t) temph << 8) | (int16_t) templ));

  temp_val = (tempi / 256) + 25;        /* TSen 256 LSB/°C */

  sninfo("Data 16-bit TEMP--->: %d Celsius\n", temp_val);

  sdata->temperature = temp_val;

  x_valg = (int16_t) (((hixg) << 8) | loxg);
  y_valg = (int16_t) (((hiyg) << 8) | loyg);
  z_valg = (int16_t) (((hizg) << 8) | lozg);

  sninfo("Data 16-bit M_X--->: %d mguass\n",
         (short)(x_valg * g_magnetofactor));
  sninfo("Data 16-bit M_Y--->: %d mguass\n",
         (short)(y_valg * g_magnetofactor));
  sninfo("Data 16-bit M_Z--->: %d mguass\n",
         (short)(z_valg * g_magnetofactor));

  sdata->m_x_data = x_valg;
  sdata->m_y_data = y_valg;
  sdata->m_z_data = z_valg;

  return OK;
}

/****************************************************************************
 * Name: lsm303agr_read
 *
 * Description:
 *   The standard read method.
 *
 ****************************************************************************/

static ssize_t lsm303agr_read(FAR struct file *filep,
                              FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode;
  FAR struct lsm303agr_dev_s *priv;
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
  priv = (FAR struct lsm303agr_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(priv->datareg == LSM303AGR_OUTX_L_A_SHIFT ||
              priv->datareg == LSM303AGR_OUTX_L_M_SHIFT);
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

          ret = lsm303agr_readreg8(priv, regaddr, &lo);
          if (ret < 0)
            {
              snerr("ERROR: lsm303agr_readreg8 failed: %d\n", ret);
              return (ssize_t) ret;
            }

          regaddr++;

          /* Read the high byte */

          ret = lsm303agr_readreg8(priv, regaddr, &hi);
          if (ret < 0)
            {
              snerr("ERROR: lsm303agr_readreg8 failed: %d\n", ret);
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
           * two's complement.
           */

          else if (data > 0x8000)
            {
              data = ~data + 1;
              ptr[j] = -(int16_t) data;
            }

          /* The value is negative and can't be represented as a positive
           * int16_t value.
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
 * Name: lsm303agr_write
 *
 * Description:
 *   A dummy write method.
 *
 ****************************************************************************/

static ssize_t lsm303agr_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lsm303agr_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int lsm303agr_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct lsm303agr_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(filep != NULL);
  inode = filep->f_inode;

  DEBUGASSERT(inode != NULL);
  priv = (FAR struct lsm303agr_dev_s *)inode->i_private;

  DEBUGASSERT(priv != NULL);

  /* Handle ioctl commands */

  switch (cmd)
    {
      /* Start converting. Arg: None. */

      case SNIOC_START:
        {
          ret = priv->ops->start(priv);
          break;
        }

      /* Stop converting. Arg: None. */

      case SNIOC_STOP:
        {
          ret = priv->ops->stop(priv);
          break;
        }

      case SNIOC_LSM303AGRSENSORREAD:
        {
          FAR struct lsm303agr_sensor_data_s *d =
            (FAR struct lsm303agr_sensor_data_s *)arg;

          ret = priv->ops->sensor_read(priv, d);
          break;
        }

      case SNIOC_START_SELFTEST:
        {
          ret = priv->ops->selftest(priv, (uint32_t)arg);
          break;
        }

      /* Unrecognized commands */

      default:
        {
          snerr("ERROR: Unrecognized cmd: %d arg: %lu\n", cmd, arg);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: lsm303agr_register
 *
 * Description:
 *   Register the LSM303AGR accelerometer, gyroscope device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register, e.g.
 *             "/dev/lsm303agr0", "/dev/gyro0" or "/dev/mag0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM303AGR accelerometer, gyroscope or
 *             magnetometer.
 *   ops     - The device operations structure.
 *   datareg - The register address of the low byte of the X-coordinate data.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm303agr_register(FAR const char *devpath,
                              FAR struct i2c_master_s *i2c,
                              uint8_t addr,
                              FAR const struct lsm303agr_ops_s *ops,
                              uint8_t datareg,
                              struct lsm303agr_sensor_data_s sensor_data)
{
  FAR struct lsm303agr_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(datareg == LSM303AGR_OUTX_L_A_SHIFT ||
              datareg == LSM303AGR_OUTX_L_M_SHIFT);

  /* Initialize the device's structure */

  priv = (FAR struct lsm303agr_dev_s *)kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;
  priv->ops = ops;
  priv->datareg = datareg;
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
 * Name: lsm303agr_sensor_register
 *
 * Description:
 *   Register the LSM303AGR accelerometer character device as 'devpath'.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register,
 *             e.g. "/dev/lsm303agr0".
 *   i2c     - An I2C driver instance.
 *   addr    - The I2C address of the LSM303AGR accelerometer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm303agr_sensor_register(FAR const char *devpath,
                              FAR struct i2c_master_s *i2c, uint8_t addr)
{
  struct lsm303agr_sensor_data_s sensor_data;

  DEBUGASSERT(addr == LSM303AGRACCELERO_ADDR ||
              addr == LSM303AGRMAGNETO_ADDR);

  sninfo("Trying to register accel\n");

  return lsm303agr_register(devpath, i2c, addr, &g_lsm303agrsensor_ops,
                            LSM303AGR_OUTX_L_A_SHIFT, sensor_data);
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM303AGR */
