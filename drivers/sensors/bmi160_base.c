/****************************************************************************
 * drivers/sensors/bmi160_base.c
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

#include "bmi160_base.h"

#if defined(CONFIG_SENSORS_BMI160)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_configspi
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SPI
static void bmi160_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMI160 */

  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, BMI160_SPI_MAXFREQUENCY);
}
#endif

/****************************************************************************
 * Name: bmi160_transferspi
 *
 * Description:
 *   SPI transfer data
 *
 * spi:     Handle to the initialized SPI device structure
 * write:   Flag indicating the transfer direction r/w
 * regaddr: Address of the BMI160 sensor register to read/write
 * data:    Pointer to the data buffer for send/receive
 * len:     Length of the data to be transferred in bytes
 *
 ****************************************************************************/

static void bmi160_transferspi(FAR struct spi_dev_s *spi, bool write,
                               uint8_t regaddr, FAR void *data, int len)
{
  uint8_t addr = 0;
  uint8_t tmp[128];

  if (len > 127)
    {
      snerr("SPI_TRANSFER failed: len %d is too large\n", len);
      len = 127;
    }

  /* If SPI bus is shared then lock and configure it */

  SPI_LOCK(spi, true);
  bmi160_configspi(spi);

  /* Select the BMI160 */

  SPI_SELECT(spi, SPIDEV_ACCELEROMETER(0), true);

  if (write)
    {
      tmp[0] = regaddr;
#ifdef CONFIG_SPI_EXCHANGE
      memcpy(tmp + 1, data, len);
      SPI_EXCHANGE(spi, tmp, 0, len + 1);
#else
      /* Send register address and set the value */

      SPI_SEND(spi, regaddr);
      SPI_SNDBLOCK(spi, data, len);
#endif
    }
  else
    {
      addr = regaddr | 0x80;

#ifdef CONFIG_SPI_EXCHANGE
      SPI_EXCHANGE(spi, &addr, tmp, len + 1);
      memcpy(data, tmp + 1, len);
#else
      /* Send register to read and get the next byte */

      SPI_SEND(spi, addr);
      SPI_RECVBLOCK(spi, data, len);
#endif
    }

  /* Deselect the BMI160 */

  SPI_SELECT(spi, SPIDEV_ACCELEROMETER(0), false);

  /* Unlock bus */

  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmi160_getreg8
 *
 * Description:
 *   Read from an 8-bit BMI160 register
 *
 ****************************************************************************/

uint8_t bmi160_getreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr)
{
  uint8_t regval = 0;

#ifdef CONFIG_SENSORS_BMI160_I2C
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

#else /* CONFIG_SENSORS_BMI160_SPI */

  bmi160_transferspi(priv->spi, false, regaddr, &regval, sizeof(uint8_t));

#endif

  return regval;
}

/****************************************************************************
 * Name: bmi160_putreg8
 *
 * Description:
 *   Write a value to an 8-bit BMI160 register
 *
 ****************************************************************************/

void bmi160_putreg8(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                    uint8_t regval)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
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

#else /* CONFIG_SENSORS_BMI160_SPI */

  bmi160_transferspi(priv->spi, true, regaddr, &regval, sizeof(uint8_t));

#endif
}

/****************************************************************************
 * Name: bmi160_getreg16
 *
 * Description:
 *   Read 16-bits of data from an BMI160 register
 *
 ****************************************************************************/

uint16_t bmi160_getreg16(FAR struct bmi160_dev_s *priv, uint8_t regaddr)
{
  uint16_t regval = 0;

#ifdef CONFIG_SENSORS_BMI160_I2C
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
  msg[1].buffer    = (FAR uint8_t *)&regval;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("I2C_TRANSFER failed: %d\n", ret);
    }

#else /* CONFIG_SENSORS_BMI160_SPI */

  bmi160_transferspi(priv->spi, false, regaddr, &regval,
                     sizeof(uint16_t));

#endif

  return regval;
}

/****************************************************************************
 * Name: bmi160_getregs
 *
 * Description:
 *   Read cnt bytes from specified dev_addr and reg_addr
 *
 ****************************************************************************/

void bmi160_getregs(FAR struct bmi160_dev_s *priv, uint8_t regaddr,
                    uint8_t *regval, int len)
{
#ifdef CONFIG_SENSORS_BMI160_I2C
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

#else /* CONFIG_SENSORS_BMI160_SPI */

  bmi160_transferspi(priv->spi, false, regaddr, regval, len);

#endif
}

/****************************************************************************
 * Name: bmi160_checkid
 *
 * Description:
 *   Read and verify the BMI160 chip ID
 *
 ****************************************************************************/

int bmi160_checkid(FAR struct bmi160_dev_s *priv)
{
  uint8_t devid = 0;

  /* Read device ID  */

  devid = bmi160_getreg8(priv, BMI160_CHIP_ID);
  sninfo("devid: %04x\n", devid);

  if (devid != (uint16_t) DEVID)
    {
      /* ID is not Correct */

      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_SENSORS_BMI160 */
