/****************************************************************************
 * include/nuttx/eeprom/i2c_xx24xx.h
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

#ifndef __INCLUDE_NUTTX_EEPROM_I2C_XX24XX_H
#define __INCLUDE_NUTTX_EEPROM_I2C_XX24XX_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DO NOT CHANGE ORDER, IT MATCHES CODE IN drivers/eeprom/i2c_xx24xx.c */

enum eeprom_24xx_e
{
  /* Microchip geometries */

  EEPROM_24XX00,
  EEPROM_24XX01,
  EEPROM_24XX02,
  EEPROM_24XX04,
  EEPROM_24XX08,
  EEPROM_24XX16,
  EEPROM_24XX32,
  EEPROM_24XX64,
  EEPROM_24XX128,
  EEPROM_24XX256,
  EEPROM_24XX512,
  EEPROM_24XX1025,
  EEPROM_24XX1026,
  EEPROM_24CM02,

  /* Atmel geometries - none... */

  /* STM geometries */

  EEPROM_M24C01,
  EEPROM_M24C02,
  EEPROM_M24M02,

  /* Aliases (devices similar to previously defined ones) */

  EEPROM_AT24C01   = EEPROM_24XX01,
  EEPROM_AT24C02   = EEPROM_24XX02,
  EEPROM_AT24C04   = EEPROM_24XX04,
  EEPROM_AT24C08   = EEPROM_24XX08,
  EEPROM_AT24C16   = EEPROM_24XX16,
  EEPROM_AT24C32   = EEPROM_24XX32,
  EEPROM_AT24C64   = EEPROM_24XX64,
  EEPROM_AT24C128  = EEPROM_24XX128,
  EEPROM_AT24C256  = EEPROM_24XX256,
  EEPROM_AT24C512  = EEPROM_24XX512,
  EEPROM_AT24C1024 = EEPROM_24XX1026,
  EEPROM_AT24CM02  = EEPROM_24CM02,

  EEPROM_M24C04    = EEPROM_24XX04,
  EEPROM_M24C08    = EEPROM_24XX08,
  EEPROM_M24C16    = EEPROM_24XX16,
  EEPROM_M24C32    = EEPROM_24XX32,
  EEPROM_M24C64    = EEPROM_24XX64,
  EEPROM_M24128    = EEPROM_24XX128,
  EEPROM_M24256    = EEPROM_24XX256,
  EEPROM_M24512    = EEPROM_24XX512,
  EEPROM_M24M01    = EEPROM_24XX1026,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ee24xx_initialize
 *
 * Description: Bind a EEPROM driver to an I2C bus. The user MUST provide
 * a description of the device geometry, since it is not possible to read
 * this information from the device (contrary to the SPI flash devices).
 *
 ****************************************************************************/

struct i2c_master_s;
int ee24xx_initialize(FAR struct i2c_master_s *bus, uint8_t devaddr,
                      FAR char *devname, int devtype, int readonly);

#endif /* __INCLUDE__NUTTX_EEPROM_I2C_XX24XX_H */
