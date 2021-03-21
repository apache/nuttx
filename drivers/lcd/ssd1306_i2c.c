/****************************************************************************
 * drivers/lcd/ssd1306_i2c.c
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
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/ssd1306.h>

#include "ssd1306.h"

#if defined(CONFIG_LCD_SSD1306) && defined(CONFIG_LCD_SSD1306_I2C)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ssd1306_sendbyte
 *
 * Description:
 *   Write an 8-bit value into SSD1306
 *
 ****************************************************************************/

int ssd1306_sendbyte(FAR struct ssd1306_dev_s *priv, uint8_t regval)
{
  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - SSD1306_Reg_Address - SSD1306_Write_Data
   *  - STOP
   */

  struct i2c_msg_s msg;
  uint8_t txbuffer[2];
  int ret;

#ifdef CONFIG_LCD_SSD1306_REGDEBUG
  _err("-> 0x%02x\n", regval);
#endif

  /* Setup to the data to be transferred.  Two bytes:  The SSD1306 register
   * address followed by one byte of data.
   */

  txbuffer[0]   = 0x00;
  txbuffer[1]   = regval;

  /* Setup 8-bit SSD1306 address write message */

  msg.frequency = CONFIG_SSD1306_I2CFREQ;  /* I2C frequency */
  msg.addr      = priv->addr;              /* 7-bit address */
  msg.flags     = 0;                       /* Write transaction, beginning with START */
  msg.buffer    = txbuffer;                /* Transfer from this address */
  msg.length    = 2;                       /* Send two bytes following the address
                                            * then STOP */

  /* Perform the transfer */

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ssd1306_sendblk
 *
 * Description:
 *   Write an array of bytes to SSD1306
 *
 ****************************************************************************/

int ssd1306_sendblk(FAR struct ssd1306_dev_s *priv, uint8_t *data,
                    uint8_t len)
{
  struct i2c_msg_s msg[2];
  uint8_t transfer_mode;
  int ret;

  /* 8-bit data read sequence:
   *
   *  Start - I2C_Write_Address - Data transfer select - SSD1306_Write_Data
   *  - STOP
   */

  /* Send the SSD1306 register address (with no STOP) */

  transfer_mode    = 0x40;                    /* Select data transfer */

  msg[0].frequency = CONFIG_SSD1306_I2CFREQ;  /* I2C frequency */
  msg[0].addr      = priv->addr;              /* 7-bit address */
  msg[0].flags     = I2C_M_NOSTOP;            /* Write transaction, beginning with START */
  msg[0].buffer    = &transfer_mode;          /* Transfer mode send */
  msg[0].length    = 1;                       /* Send the one byte register address */

  /* Followed by the SSD1306 write data (with no RESTART, then STOP) */

  msg[1].frequency = CONFIG_SSD1306_I2CFREQ;  /* I2C frequency */
  msg[1].addr      = priv->addr;              /* 7-bit address */
  msg[1].flags     = I2C_M_NOSTART;           /* Write transaction with no RESTART */
  msg[1].buffer    = data;                    /* Transfer from this address */
  msg[1].length    = len;                     /* Send the data, then STOP */

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      lcderr("ERROR: I2C_TRANSFER failed: %d\n", ret);
    }

  return ret;
}

#endif /* CONFIG_LCD_SSD1306 &7 CONFIG_LCD_SSD1306_I2C */
