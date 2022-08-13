/****************************************************************************
 * include/nuttx/leds/apa102.h
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

#ifndef __INCLUDE_NUTTX_LEDS_APA102_H
#define __INCLUDE_NUTTX_LEDS_APA102_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Configuration
 * CONFIG_SPI         - Enables support for SPI drivers
 * CONFIG_LEDS_APA102 - Enables support for the APA102 driver
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI definitions */

#define APA102_SPI_MAXFREQUENCY    (100000) /* Default 4MHz */

/* APA102 register addresses */

#define APA102_START_FRAME         0x00000000
#define APA102_HEADER_FRAME        0xe1
#define APA102_END_FRAME           0x00

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct apa102_ledstrip_s
{
  uint8_t bright;
  uint8_t blue;
  uint8_t green;
  uint8_t red;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: apa102_register
 *
 * Description:
 *   Register the APA102 device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/leddrv0".
 *   spi     - An instance of the SPI interface to use to communicate
 *             with the APA102.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int apa102_register(FAR const char *devpath, FAR struct spi_dev_s *spi);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LEDS_APA102_H */
