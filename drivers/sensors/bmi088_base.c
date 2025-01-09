/****************************************************************************
 * drivers/sensors/bmi088_base.c
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

#include "bmi088_base.h"

#if defined(CONFIG_SENSORS_BMI088)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi088_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI088_SPI
static void bmi088_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMI088 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, BMI088_SPI_MAXFREQUENCY);
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi088_get_acc_reg8
 *
 * Description:
 *   Read from an 8-bit BMI088 ACC register
 *
 ****************************************************************************/

uint8_t bmi088_get_acc_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
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
    }

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

/****************************************************************************
 * Name: bmi088_get_gyro_reg8
 *
 * Description:
 *   Read from an 8-bit BMI088 GYRO register
 *
 ****************************************************************************/

uint8_t bmi088_get_gyro_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
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
    }

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), true);

  /* Send register to read and get the next byte */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);
#endif

  return regval;
}

/****************************************************************************
 * Name: bmi088_put_acc_reg8
 *
 * Description:
 *   Write a value to an 8-bit BMI088 ACC register
 *
 ****************************************************************************/

void bmi088_put_acc_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                    uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

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

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi088_put_gyro_reg8
 *
 * Description:
 *   Write a value to an 8-bit BMI088 GYRO register
 *
 ****************************************************************************/

void bmi088_put_gyro_reg8(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                    uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;
  uint8_t txbuffer[2];

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

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), true);

  /* Send register address and set the value */

  SPI_SEND(priv->spi, regaddr);
  SPI_SEND(priv->spi, regval);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi088_get_acc_regs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

void bmi088_get_acc_regs(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                    uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, 1);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_ACC), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

/****************************************************************************
 * Name: bmi088_get_gyro_regs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

void bmi088_get_gyro_regs(FAR struct bmi088_dev_s *priv, uint8_t regaddr,
                    uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI088_I2C
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = priv->freq;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &regaddr;
  msg[0].length    = 1;

  msg[1].frequency = priv->freq;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = regval;
  msg[1].length    = len;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI088_SPI */
  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(priv->spi, true);
  bmi088_configspi(priv->spi);

  /* Select the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), true);

  /* Send register to read and get the next 2 bytes */

  SPI_SEND(priv->spi, regaddr | 0x80);
  SPI_RECVBLOCK(priv->spi, regval, len);

  /* Deselect the BMI088 */

  SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(BMI088_SPI_DEV_GYRO), false);

  /* Unlock bus */

  SPI_LOCK(priv->spi, false);

#endif
}

#endif /* CONFIG_SENSORS_BMI088 */
