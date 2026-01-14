/****************************************************************************
 * boards/arm/nrf53/common/include/nrf53_bmi270.h
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

#ifndef __BOARDS_ARM_NRF53_COMMON_INCLUDE_NRF53_BMI270_H
#define __BOARDS_ARM_NRF53_COMMON_INCLUDE_NRF53_BMI270_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bmi270spi_initialize
 *       nrf53_bmi270i2c_initialize
 *
 * Description:
 *   Initialize and register the BMI270 as uorb sensor
 *
 * Input Parameters:
 *   devno - The user specifies device number, from 0.
 *   busno - SPI or I2C bus number
 *   addr  - (I2C only) The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI270_SPI
int nrf53_bmi270spi_initialize(int devno, int busno);
#else  /* CONFIG_SENSORS_BMI270_I2C */
int nrf53_bmi270i2c_initialize(int devno, int bus, uint8_t addr);
#endif

#endif /* __BOARDS_ARM_NRF53_COMMON_INCLUDE_NRF53_BMI270_H */
