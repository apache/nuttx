/****************************************************************************
 * include/nuttx/lcd/apa102.h
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

#ifndef __INCLUDE_NUTTX_LCD_APA102_H
#define __INCLUDE_NUTTX_LCD_APA102_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/* Configuration
 * CONFIG_SPI - Enables support for SPI drivers
 * CONFIG_LCD_APA102 - Enables support for the APA102 driver
 */

#if defined(CONFIG_SPI) && defined(CONFIG_LCD_APA102)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some important "colors" */

#define APA102_BLACK         0
#define APA102_WHITE         1

/* Only two power settings are supported: */

#define APA102_POWER_OFF     0
#define APA102_POWER_ON      1

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: apa102_initialize
 *
 * Description:
 *   Initialize the APA102 device as a LCD interface.
 *
 * Input Parameters:
 *   spi   - An instance of the SPI interface to use to communicate
 *           with the APA102.
 *   devno - Device number to identify current display.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *apa102_initialize(FAR struct spi_dev_s *spi,
                                        unsigned int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_APA102 */
#endif /* __INCLUDE_NUTTX_LCD_APA102_H */
