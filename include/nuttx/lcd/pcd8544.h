/****************************************************************************
 * include/nuttx/lcd/pcd8544.h
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

#ifndef __INCLUDE_NUTTX_LCD_PCD8544_H
#define __INCLUDE_NUTTX_LCD_PCD8544_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PCD8544 Configuration Settings:
 *
 * CONFIG_PCD8544_SPIMODE - Controls the SPI mode
 * CONFIG_PCD8544_FREQUENCY - Define to use a different bus frequency
 * CONFIG_PCD8544_NINTERFACES - Specifies the number of physical
 *   PCD8544 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_PCD8544 - Enable PCD8544 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be
 *                        accepted.
 * CONFIG_LCD_MAXPOWER should be 2:  0=off, 1=dim, 2=normal
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Some important "colors" */

#define PCD8544_Y1_BLACK  0
#define PCD8544_Y1_WHITE  1

/* Only two power settings are supported: */

#define PCD8544_POWER_OFF 0
#define PCD8544_POWER_ON  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  pcd8544_initialize
 *
 * Description:
 *   Initialize the PCD8544 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 thropcd8544h
 *            CONFIG_PCD8544_NINTERFACES-1.
 *     This allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s; /* see nuttx/lcd.h */
struct spi_dev_s; /* see nuttx/spi/spi.h */
FAR struct lcd_dev_s *pcd8544_initialize(FAR struct spi_dev_s *spi,
                                         unsigned int devno);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_PCD8544_H */
