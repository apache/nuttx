/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_bmp280.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMP280_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMP280_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
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
 * Name: board_bmp280_initialize
 *
 * Description:
 *   Initialize BMP280 i2c driver and register the BMP280 device.
 *
 ****************************************************************************/

#if defined(CONFIG_SENSORS_BMP280) || defined(CONFIG_SENSORS_BMP280_SCU)
int board_bmp280_initialize(int bus);
#endif

#ifdef CONFIG_SENSORS_BMP280_SCU
/****************************************************************************
 * Name: bmp280_init
 *
 * Description:
 *   Initialize BMP280 pressure device
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_init(struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: bmp280press_register
 *
 * Description:
 *   Register the BMP280 pressure sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/press0"
 *   minor   - The number of sequencer
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280press_register(const char *devpath, int minor,
                         struct i2c_master_s *i2c, int port);

/****************************************************************************
 * Name: bmp280temp_register
 *
 * Description:
 *   Register the BMP280 temperature sensor character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The base path to the driver to register. E.g., "/dev/temp"
 *   minor   - The number of sequencer
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BMP280
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280temp_register(const char *devpath, int minor,
                        struct i2c_master_s *i2c, int port);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMP280_H */
