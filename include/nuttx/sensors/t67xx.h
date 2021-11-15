/****************************************************************************
 * include/nuttx/sensors/t67xx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_T67XX_H
#define __INCLUDE_NUTTX_SENSORS_T67XX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_T67XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T67XX_I2C_ADDR 0x15

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

struct t67xx_value_s
{
  uint16_t gas_ppm;  /* CO2 level, parts per million */
  bool warming_up;   /* Warm-up mode, gas_ppm might be imprecise */
  bool calibrating;  /* Calibration in progress, gas_ppm might be imprecise */
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
 * Name: t67xx_register
 *
 * Description:
 *   Register the T67XX character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/co2_0"
 *   i2c - An instance of the I2C interface to use to communicate with T67XX
 *   addr - The I2C address of the T67XX. For T6713 this is initially 0x15
 *   but it can be changed by the SLAVE ADDRESS command.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int t67xx_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                   uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_T67XX */
#endif /* __INCLUDE_NUTTX_SENSORS_T67XX_H */
