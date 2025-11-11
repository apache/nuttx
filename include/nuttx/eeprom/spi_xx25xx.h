/****************************************************************************
 * include/nuttx/eeprom/spi_xx25xx.h
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

#ifndef __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H
#define __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DO NOT CHANGE ORDER, IT MATCHES CODE IN drivers/eeprom/spieeprom.c and
 * drivers/mtd/at25ee.c
 */

enum eeprom_25xx_e
{
  /* Microchip geometries */

  EEPROM_25XX010,
  EEPROM_25XX020,
  EEPROM_25XX040,
  EEPROM_25XX080A, /* 16 bytes pages */
  EEPROM_25XX080B, /* 32 bytes pages */
  EEPROM_25XX160A, /* 16 bytes pages */
  EEPROM_25XX160B, /* 32 bytes pages */
  EEPROM_25XX320,
  EEPROM_25XX640,
  EEPROM_25XX128,
  EEPROM_25XX256,
  EEPROM_25XX512,  /* Additional erase commands */
  EEPROM_25XX1024, /* Additional erase commands */

  /* Atmel geometries */

  EEPROM_AT25010B,
  EEPROM_AT25020B,
  EEPROM_AT25040B,
  EEPROM_AT25512,
  EEPROM_AT25M01,

  /* STM geometries */

  EEPROM_M95M02,

  /* Aliases (devices similar to previously defined ones) */

  EEPROM_AT25080B  = EEPROM_25XX080B,
  EEPROM_AT25160B  = EEPROM_25XX160B,
  EEPROM_AT25320B  = EEPROM_25XX320,
  EEPROM_AT25640B  = EEPROM_25XX640,
  EEPROM_AT25128B  = EEPROM_25XX128,
  EEPROM_AT225256B = EEPROM_25XX256,
  EEPROM_AT25M02   = EEPROM_25XX1024,
  EEPROM_M95010    = EEPROM_25XX010,
  EEPROM_M95020    = EEPROM_25XX020,
  EEPROM_M95040    = EEPROM_25XX040,
  EEPROM_M95080    = EEPROM_25XX080B,
  EEPROM_M95160    = EEPROM_25XX160B,
  EEPROM_M95320    = EEPROM_25XX320,
  EEPROM_M95640    = EEPROM_25XX640,
  EEPROM_M95128    = EEPROM_25XX128,
  EEPROM_M95256    = EEPROM_25XX256,
  EEPROM_M95512    = EEPROM_AT25512,
  EEPROM_M95M01    = EEPROM_AT25M01,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_initialize
 *
 * Description:
 *   Bind an EEPROM driver to an SPI bus. The user MUST provide a description
 *   of the device geometry, since it is not possible to read this
 *   information from the device (contrary to the SPI flash devices).
 *
 * Parameters:
 *   dev       - Pointer to the SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devname   - Device name
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

struct spi_dev_s;
int ee25xx_initialize(FAR struct spi_dev_s *dev, uint16_t spi_devid,
                      FAR char *devname, int devtype, int readonly);

#endif /* __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H */
