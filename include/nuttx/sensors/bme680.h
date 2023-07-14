/****************************************************************************
 * include/nuttx/sensors/bme680.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_BME680_H
#define __INCLUDE_NUTTX_SENSORS_BME680_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_BME680)
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Oversampling settings */

#define BME680_OS_SKIPPED    (0x00)   /* Output set to 0x8000 */
#define BME680_OS_1X         (0x01)
#define BME680_OS_2X         (0x02)
#define BME680_OS_4X         (0x03)
#define BME680_OS_8X         (0x04)
#define BME680_OS_16X        (0x05)

/* IIR filter settings */

#define BME680_FILTER_COEF0     (0)
#define BME680_FILTER_COEF1     (1)
#define BME680_FILTER_COEF3     (2)
#define BME680_FILTER_COEF7     (3)
#define BME680_FILTER_COEF15    (4)
#define BME680_FILTER_COEF31    (5)
#define BME680_FILTER_COEF63    (6)
#define BME680_FILTER_COEF127   (7)

/* Prerequisites:
 *
 * CONFIG_BME680
 *   Enables support for the BME680 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct bme680_config_s
{
  /* Oversampling settings */

  uint8_t temp_os;

#ifndef CONFIG_BME680_DISABLE_PRESS_MEAS
  uint8_t press_os;
#endif

#ifndef CONFIG_BME680_DISABLE_HUM_MEAS
  uint8_t hum_os;
#endif

#ifdef CONFIG_BME680_ENABLE_IIR_FILTER
  /* Filter coefficient */

  uint8_t filter_coef;
#endif

#ifndef CONFIG_BME680_DISABLE_GAS_MEAS
  /* Gas settings */

  int16_t target_temp;      /* degrees Celsius */
  uint16_t heater_duration; /* ms */
  uint8_t nb_conv;
  int16_t amb_temp;         /* degrees Celsius */
#endif
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
 * Name: bme680_register
 *
 * Description:
 *   Register the BME680 character device
 *
 * Input Parameters:
 *   devno   - Instance number for driver
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             BME680
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bme680_register(int devno, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_BME680 */
#endif /* __INCLUDE_NUTTX_BME680_H */
