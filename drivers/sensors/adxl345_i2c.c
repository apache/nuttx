/****************************************************************************
 * drivers/sensors/adxl345_i2c.c
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/adxl345.h>

#include "adxl345.h"

#if defined(CONFIG_SENSORS_ADXL345)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adxl345_getreg8
 *
 * Description:
 *   Read from an 8-bit ADXL345 register
 *
 ****************************************************************************/

#ifdef CONFIG_ADXL345_I2C
uint8_t adxl345_getreg8(FAR struct adxl345_dev_s *priv, uint8_t regaddr)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - ADXL345_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - ADXL345_Read_Data - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t regval;
  int ret;

  /* Setup 8-bit ADXL345 address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit ADXL345 data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = &regval;                  /* Transfer to this address */
  msg[1].length    = 1;                        /* Receive one byte following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x->%02x\n", regaddr, regval);
#endif
  return regval;
}
#endif

/****************************************************************************
 * Name: adxl345_putreg8
 *
 * Description:
 *   Write a value to an 8-bit ADXL345 register
 *
 ****************************************************************************/

#ifdef CONFIG_ADXL345_I2C
void adxl345_putreg8(FAR struct adxl345_dev_s *priv,
                      uint8_t regaddr, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   * Start-I2C_Write_Address-ADXL345_Reg_Address-ADXL345_Write_Data-STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x<-%02x\n", regaddr, regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The ADXL345 register
   * address followed by one byte of data.
   */

  txbuffer[0] = regaddr;
  txbuffer[1] = regval;

  /* Setup 8-bit ADXL345 address write message */

  msg.frequency = priv->config->frequency;  /* I2C frequency */
  msg.addr      = priv->config->address;    /* 7-bit address */
  msg.flags     = 0;                        /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                 /* Transfer from this address */
  msg.length    = 2;                        /* Send two byte following the address
                                             * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }
}
#endif

/****************************************************************************
 * Name: adxl345_getreg16
 *
 * Description:
 *   Read 16-bits of data from an STMPE-11 register
 *
 ****************************************************************************/

#ifdef CONFIG_ADXL345_I2C
uint16_t adxl345_getreg16(FAR struct adxl345_dev_s *priv, uint8_t regaddr)
{
  /* 16-bit data read sequence:
   *
   *  Start - I2C_Write_Address - ADXL345_Reg_Address -
   *    Repeated_Start - I2C_Read_Address  - ADXL345_Read_Data_1 -
   *      ADXL345_Read_Data_2 - STOP
   */

  struct i2c_msg_s msg[2];
  uint8_t rxbuffer[2];
  int ret;

  /* Setup 8-bit ADXL345 address write message */

  msg[0].frequency = priv->config->frequency;  /* I2C frequency */
  msg[0].addr      = priv->config->address;    /* 7-bit address */
  msg[0].flags     = 0;                        /* Write transaction, beginning with START */
  msg[0].buffer    = &regaddr;                 /* Transfer from this address */
  msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

  /* Set up the 8-bit ADXL345 data read message */

  msg[1].frequency = priv->config->frequency;  /* I2C frequency */
  msg[1].addr      = priv->config->address;    /* 7-bit address */
  msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
  msg[1].buffer    = rxbuffer;                 /* Transfer to this address */
  msg[1].length    = 2;                        /* Receive two bytes following the address
                                                * (then STOP) */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
      return 0;
    }

#ifdef CONFIG_ADXL345_REGDEBUG
  _err("%02x->%02x%02x\n", regaddr, rxbuffer[0], rxbuffer[1]);
#endif
  return (uint16_t)rxbuffer[0] << 8 | (uint16_t)rxbuffer[1];
}
#endif

#endif /* CONFIG_SENSORS_ADXL345 */
