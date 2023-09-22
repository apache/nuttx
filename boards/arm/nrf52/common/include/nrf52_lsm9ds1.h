/****************************************************************************
 * boards/arm/nrf52/common/include/nrf52_lsm9ds1.h
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

#ifndef __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_LSM9DS1_H
#define __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_LSM9DS1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_lsm9ds1_initialize
 *
 * Description:
 *   Initialize I2C-based LSM9DS1.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM9DS1
int nrf52_lsm9ds1_initialize(int bus);
#endif

#endif /* __BOARDS_ARM_NRF52_COMMON_INCLUDE_NRF52_LSM9DS1_H */
