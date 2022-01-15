/****************************************************************************
 * include/nuttx/lcd/p14201.h
 * Application interface to the RiT P14201 OLED driver
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

#ifndef __INCLUDE_NUTTX_LCD_P14201_H
#define __INCLUDE_NUTTX_LCD_P14201_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* P14201 Configuration Settings:
 *
 * CONFIG_P14201_SPIMODE - Controls the SPI mode
 * CONFIG_P14201_FREQUENCY - Define to use a different bus frequency
 * CONFIG_P14201_NINTERFACES - Specifies the number of physical P14201
 *   devices that will be supported.
 * CONFIG_P14201_FRAMEBUFFER - If defined, accesses will be performed using
 *   an in-memory copy of the OLEDs GDDRAM.  This cost of this buffer is
 *   128 * 96 / 2 = 6Kb.  If this is defined, then the driver will be fully
 *   functional. If not, then it will have the following limitations:
 *
 *   - Reading graphics memory cannot be supported, and
 *   - All pixel writes must be aligned to byte boundaries.
 *
 *   The latter limitation effectively reduces the 128x96 display to 64x96.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_P14201 - Enable P14201 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be
 *   accepted.
 * CONFIG_LCD_MAXPOWER must be 1
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Some important "colors" */

#define RIT_Y4_BLACK 0x00
#define RIT_Y4_WHITE 0x0f

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name:  rit_initialize
 *
 * Description:
 *   Initialize the P14201 video hardware.  The initial state of the OLED is
 *   fully initialized, display memory cleared, and the OLED ready to use,
 *   but with the powersetting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_P14201_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the
 *   specified OLED.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s; /* see nuttx/lcd.h */
struct spi_dev_s; /* see nuttx/spi/spi.h */
FAR struct lcd_dev_s *rit_initialize(FAR struct spi_dev_s *spi,
                                     unsigned int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_P14201_H */
