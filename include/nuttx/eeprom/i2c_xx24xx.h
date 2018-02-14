 /****************************************************************************
 * include/nuttx/eeprom/i2c_xx24xx.h
 *
 *   Copyright (C) 2018 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __INCLUDE_NUTTX_EEPROM_I2C_XX24XX_H
#define __INCLUDE_NUTTX_EEPROM_I2C_XX24XX_H 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DO NOT CHANGE ORDER, IT MACHES CODE IN drivers/eeprom/i2c_xx24xx.c */

enum eeprom_24xx_e
{
  /* Microchip geometries */

  EEPROM_24xx00,
  EEPROM_24xx01,
  EEPROM_24xx02,
  EEPROM_24xx04,
  EEPROM_24xx08,
  EEPROM_24xx16,
  EEPROM_24xx32,
  EEPROM_24xx64,
  EEPROM_24xx128,
  EEPROM_24xx256,
  EEPROM_24xx512,
  EEPROM_24xx1025,
  EEPROM_24xx1026,

  /* Atmel geometries - none... */

  /* STM geometries */

  EEPROM_M24C01,
  EEPROM_M24C02,
  EEPROM_M24M02,

  /* Aliases (devices similar to previously defined ones) */

  EEPROM_AT24C01   = EEPROM_24xx01,
  EEPROM_AT24C02   = EEPROM_24xx02,
  EEPROM_AT24C04   = EEPROM_24xx04,
  EEPROM_AT24C08   = EEPROM_24xx08,
  EEPROM_AT24C16   = EEPROM_24xx16,
  EEPROM_AT24C32   = EEPROM_24xx32,
  EEPROM_AT24C64   = EEPROM_24xx64,
  EEPROM_AT24C128  = EEPROM_24xx128,
  EEPROM_AT24C256  = EEPROM_24xx256,
  EEPROM_AT24C512  = EEPROM_24xx512,
  EEPROM_AT24C1024 = EEPROM_24xx1026,

  EEPROM_M24C04    = EEPROM_24xx04,
  EEPROM_M24C08    = EEPROM_24xx08,
  EEPROM_M24C16    = EEPROM_24xx16,
  EEPROM_M24C32    = EEPROM_24xx32,
  EEPROM_M24C64    = EEPROM_24xx64,
  EEPROM_M24128    = EEPROM_24xx128,
  EEPROM_M24256    = EEPROM_24xx256,
  EEPROM_M24512    = EEPROM_24xx512,
  EEPROM_M24M01    = EEPROM_24xx1026,
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
