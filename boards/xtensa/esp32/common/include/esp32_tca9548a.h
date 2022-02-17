/****************************************************************************
 * boards/xtensa/esp32/common/include/esp32_tca9548a.h
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

#ifndef __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_TCA9548A_H
#define __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_TCA9548A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: board_tca9548a_initialize
 *
 * Description:
 *   Initialize and register the TCA9548A Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_tca9548a_initialize(int devno, int busno);

/****************************************************************************
 * Name: esp32_i2cmux_getmaster
 *
 * Description:
 *   Returns an I2C Master for TCA9548A multiplexer channel.
 *
 *   NOTE: esp32_i2cmux_getmaster() is generic name, it can be used to return
 *   an I2C Master for others I2C Master, not only TCA9548A.
 *
 * Input Parameters:
 *   devno   - The device number, it is the TCA9548A I2C_Addr minus 0x70.
 *   channel - The TCA9548A's channel where the device will be added.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

struct i2c_master_s *esp32_i2cmux_getmaster(int devno, uint8_t channel);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_XTENSA_ESP32_COMMON_INCLUDE_ESP32_TCA9548A_H */
