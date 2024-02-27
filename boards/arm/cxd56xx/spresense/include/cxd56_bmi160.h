/****************************************************************************
 * boards/arm/cxd56xx/spresense/include/cxd56_bmi160.h
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

#ifndef __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMI160_H
#define __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMI160_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
 * Name: board_bmi160_initialize
 *
 * Description:
 *   Initialize BMI160 i2c driver and register the BMI160 device.
 *
 ****************************************************************************/

int board_bmi160_initialize(int bus);

#ifdef CONFIG_SENSORS_BMI160_SCU
/****************************************************************************
 * Name: bmi160_init
 *
 * Description:
 *   Initialize BMI160 accelerometer/gyro device
 *
 * Input Parameters:
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160_init(struct i2c_master_s *dev, int port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160_init(struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: bmi160gyro_register
 *
 * Description:
 *   Register the BMI160 gyro character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gyro"
 *   minor   - The number of sequencer
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160gyro_register(const char *devpath, int minor,
                        struct i2c_master_s *dev, int port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160gyro_register(const char *devpath, int minor,
                        struct spi_dev_s *dev);
#endif

/****************************************************************************
 * Name: bmi160accel_register
 *
 * Description:
 *   Register the BMI160 accelerometer character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel"
 *   minor   - The number of sequencer
 *   dev     - An instance of the SPI or I2C interface to use to communicate
 *             with BMI160
 *   port    - I2C port number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMI160_SCU_I2C
int bmi160accel_register(const char *devpath, int minor,
                         struct i2c_master_s *dev, int port);
#else /* CONFIG_SENSORS_BMI160_SCU_SPI */
int bmi160accel_register(const char *devpath, int minor,
                         struct spi_dev_s *dev);
#endif

#endif /* CONFIG_SENSORS_BMI160_SCU */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_CXD56XX_SPRESENSE_INCLUDE_CXD56_BMI160_H */
