/****************************************************************************
 * include/nuttx/sensors/pat9126ja.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_PAT9126JA_H
#define __INCLUDE_NUTTX_SENSORS_PAT9126JA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>

#ifdef CONFIG_SENSORS_PAT9126JA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct pat9126ja_config_s
{
  uint8_t addr;                           /* I2C address */
  int pin;                                /* Interrupt pin */
  int freq;                               /* I2C frequency */
  FAR struct i2c_master_s *i2c;           /* I2C interface */
  FAR struct ioexpander_dev_s *ioedev;    /* Ioexpander device */
};

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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pat9126ja_register
 *
 * Description:
 *   Register the PAT9126JA character device as 'devno'
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path
 *              as /dev/sensor/otsN
 *   config - The board config function for the device.
 *
 * Returned Value:
 *   Return 0 if the driver was success; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int pat9126ja_register(int devno,
                       FAR const struct pat9126ja_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_PAT9126JA */
#endif /* __INCLUDE_NUTTX_SENSORS_PAT9126JA_H */
