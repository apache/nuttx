/****************************************************************************
 * include/nuttx/sensors/hyt271.h
 * Character driver for HYT271 Digital Humidity and Temperature Module.
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

#ifndef __INCLUDE_NUTTX_SENSORS_HYT271_H
#define __INCLUDE_NUTTX_SENSORS_HYT271_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_HYT271)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HYT271 I2C Commands */

#define HYT271_CMD_EEPROM_READ     (0x1C)          /* EEPROM Read Command */
#define HYT271_CMD_EEPROM_WRITE    (0x5C)          /* EEPROM Write Command */
#define HYT271_CMD_START_CM        (0xA0)          /* Start command mode */
#define HYT271_CMD_START_NOM       (0x80)          /* End command mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct hyt271_bus_s
{
  /**************************************************************************
   * Name: pwonreset
   *
   * Description:
   *   Callback to board specific logic for power on reset the I2C bus.
   *   The callback must ensure that the repowered bus is stable before
   *   returning.
   *
   * Parameter:
   *   bus    - Instance to this
   *
   * Return:
   *   OK on success
   **************************************************************************/

  int (*pwonreset)(FAR struct hyt271_bus_s *bus);
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
 * Name: hyt271_register
 *
 * Description:
 *   Register the HYT271 character device.
 *
 * Input Parameters:
 *   devno   - The user specifies device number, from 0.
 *   i2c     - An instance of the I2C interface to communicate with HYT271
 *             sensor.
 *
 *   addr    - The I2C address of the HYT271.
 *   bus     - Callback to board specific logic for i2c bus power.
 *             Will be used for changing i2c address of the sensor and can be
 *             set to NULL when not supported.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 ****************************************************************************/

int hyt271_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr,
                    FAR struct hyt271_bus_s *bus);
#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_HYT271 */
#endif /* __INCLUDE_NUTTX_SENSORS_HYT271_H */
