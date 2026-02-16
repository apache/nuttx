/****************************************************************************
 * include/nuttx/analog/ads7046.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7046_H
#define __INCLUDE_NUTTX_ANALOG_ADS7046_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <stdint.h>

#if defined(CONFIG_ADC_ADS7046)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/

/* Cmd: ANIOC_ADS7046_READ                    Arg: uint16_t *value
 * Cmd: ANIOC_ADS7046_READ_FASTUNSAFE         Arg: uint16_t *value
 * Cmd: ANIOC_ADS7046_OFFCAL                  Arg: N/A
 */

#define ANIOC_ADS7046_READ                    _ANIOC(AN_ADS7046_FIRST + 0)
#define ANIOC_ADS7046_READ_FASTUNSAFE         _ANIOC(AN_ADS7046_FIRST + 1)
#define ANIOC_ADS7046_OFFCAL                  _ANIOC(AN_ADS7046_FIRST + 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ads7046_register
 *
 * Description:
 *   Register the ADS7046 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath  - The full path to the driver to register. E.g., "/dev/adc0"
 *   spi      - An instance of the SPI interface to use.
 *   devno    - SPI device number.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ads7046_register(FAR const char *devpath, FAR struct spi_dev_s *spi,
                     unsigned int devno);

#endif /* CONFIG_ADC_ADS7046 */
#endif /* __INCLUDE_NUTTX_ANALOG_ADS7046_H */
