/****************************************************************************
 * include/nuttx/eeprom/eeprom.h
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

/* This file includes common definitions to be used in all EEPROM drivers
 * (when applicable).
 */

#ifndef __INCLUDE_NUTTX_EEPROM_EEPROM_H
#define __INCLUDE_NUTTX_EEPROM_EEPROM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EEPROM IOCTL Commands ****************************************************/

#define EEPIOC_GEOMETRY     _EEPIOC(0x000)  /* Similar to BIOC_GEOMETRY:
                                             * Return the geometry of the
                                             * EEPROM device.
                                             * IN:  Pointer to writable
                                             *      instance of struct
                                             *      eeprom_geometry_s to be
                                             *      populated
                                             * OUT: Data return in user-
                                             *      provided buffer.        */

#define EEPIOC_SETSPEED     _EEPIOC(0x001)  /* Overwrite the SPI/I2C bus speed
                                             * IN:  Bus speed in Hz
                                             * OUT: None (ioctl return value
                                             *      provides success/failure
                                             *      indication).            */

#define EEPIOC_PAGEERASE    _EEPIOC(0x002)  /* Erase a single page
                                             * IN:  Index of the page to be
                                             *      erased
                                             * OUT: None (ioctl return value
                                             *      provides success/failure
                                             *      indication).            */

#define EEPIOC_SECTORERASE  _EEPIOC(0x003)  /* Erase a single sector, this
                                             * behaves like page erase on
                                             * devices without sectors
                                             * IN:  Index of the sector to be
                                             *      erased
                                             * OUT: None (ioctl return value
                                             *      provides success/failure
                                             *      indication).            */

#define EEPIOC_CHIPERASE    _EEPIOC(0x004)  /* Wipe the entire EEPROM
                                             * IN:  None
                                             * OUT: None (ioctl return value
                                             *      provides success/failure
                                             *      indication).            */

#define EEPIOC_BLOCKPROTECT _EEPIOC(0x005)  /* Set which memory blocks to
                                             * protect.
                                             * IN:  Which blocks as integer.
                                             * OUT: OK (0) on success.      */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct eeprom_geometry_s
{
  blkcnt_t  npages;   /* Number of pages on the device */
  blksize_t sectsize; /* Size of one sector in bytes */
  blksize_t pagesize; /* Size of one page in bytes */
};

#if defined(CONFIG_MTD_AT25EE) || defined(CONFIG_MTD_AT24XX)
struct eeprom_dev_s
{
  /* The following methods operate on the EEPROM */

  CODE ssize_t (*read)(FAR struct eeprom_dev_s *dev, off_t offset,
                       size_t nbytes, FAR uint8_t *buffer);
  CODE ssize_t (*write)(FAR struct eeprom_dev_s *dev, off_t offset,
                        size_t nbytes, FAR const uint8_t *buffer);

  CODE int (*ioctl)(FAR struct eeprom_dev_s *dev, int cmd,
                    unsigned long arg);
};
#endif

#ifdef CONFIG_I2C_EE_24XX
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
#endif

#ifdef CONFIG_SPI_EE_25XX
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
  EEPROM_BR25G256  = EEPROM_25XX256,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ee24xx_initialize
 *
 * Description:
 *   Bind a EEPROM driver to an I2C bus. The user MUST provide a description
 *   of the device geometry, since it is not possible to read this
 *   information from the device (contrary to the SPI flash devices).
 *
 * Parameters:
 *   bus      - Pointer to the I2C device instance
 *   devaddr  - I2C device address
 *   devaddr  - Device name
 *   devtype  - 24xx device type, the geometry is derived from it
 *   readonly - Flag indicating whether the device should be opened read-only
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_EE_24XX
struct i2c_master_s;
int ee24xx_initialize(FAR struct i2c_master_s *bus, uint8_t devaddr,
                      FAR char *devname, int devtype, int readonly);
#endif

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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_EE_25XX
struct spi_dev_s;
int ee25xx_initialize(FAR struct spi_dev_s *dev, uint16_t spi_devid,
                      FAR char *devname, enum eeprom_25xx_e devtype,
                      int readonly);

/****************************************************************************
 * Name: ee25xx_initialize_as_eeprom_dev
 *
 * Description:
 *   Create an initialized EEPROM device instance for an xx25xx SPI EEPROM.
 *   The device is not registered in the file system, but created as an
 *   instance that can be bound to other functions.
 *
 * Parameters:
 *   dev       - Pointer to the SPI device instance
 *   spi_devid - SPI device ID to manage CS lines in board
 *   devtype   - 25xx device type, the geometry is derived from it
 *   readonly  - Sets driver to be readonly
 *
 * Returned Values:
 *   Initialized device structure (success) or NULL (fail)
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_AT25EE
FAR struct eeprom_dev_s *ee25xx_initialize_as_eeprom_dev(
                           FAR struct spi_dev_s *dev, uint16_t spi_devid,
                           enum eeprom_25xx_e devtype, int readonly);

/****************************************************************************
 * Name: ee25xx_teardown
 *
 * Description:
 *   Teardown a previously created ee25xx device.
 *
 * Input Parameters:
 *   dev - Pointer to the driver instance.
 *
 ****************************************************************************/

void ee25xx_teardown(FAR struct eeprom_dev_s *dev);

#endif /* CONFIG_MTD_AT25EE */
#endif /* CONFIG_SPI_EE_25XX */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __INCLUDE_NUTTX_EEPROM_EEPROM_H */
