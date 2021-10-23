/****************************************************************************
 * include/nuttx/sensors/lps27hhw.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LPS27HHW_H
#define __INCLUDE_NUTTX_SENSORS_LPS27HHW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>

#ifdef CONFIG_SENSORS_LPS27HHW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lps27hhw_config_s
{
  uint8_t addr;                           /* I2C address */
  int freq;                               /* I2C frequency */
#ifdef CONFIG_LPS27HHW_MODE_INT
  int pin;                                /* Interrupt pin */
  FAR struct ioexpander_dev_s *ioedev;    /* Ioexpander device */
#endif
  FAR struct i2c_master_s *i2c;           /* I2C interface */
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
 * Name: lps27hhw_register
 *
 * Description:
 *   Register the LPS27HHW character device as 'devno'
 *
 * Input Parameters:
 *   devno  - The device number, used to build the device path
 *            as /dev/sensor/baroN
 *   config - The board config function for the device.
 *
 * Returned Value:
 *   Return 0 if the driver succeeded; A negated errno
 *   value is returned on any failure.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

int lps27hhw_register(int devno, FAR const struct lps27hhw_config_s *config);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_LPS27HHW */
#endif /* __INCLUDE_NUTTX_SENSORS_LPS27HHW_H */
