/****************************************************************************
 * include/nuttx/i2c/tca9548a.h
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

#ifndef __INCLUDE_NUTTX_I2C_TCA9548A_H
#define __INCLUDE_NUTTX_I2C_TCA9548A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* It is possible to config. TCA9548A to communicate from 0x70-0x77 */

#define TCA9548A_BASEADDR0    0x70
#define TCA9548A_BASEADDR1    0x71
#define TCA9548A_BASEADDR2    0x72
#define TCA9548A_BASEADDR3    0x73
#define TCA9548A_BASEADDR4    0x74
#define TCA9548A_BASEADDR5    0x75
#define TCA9548A_BASEADDR6    0x76
#define TCA9548A_BASEADDR7    0x77

/* The TCA9548A can multiplex up to 8 channels */

#define TCA9548A_SEL_CH0      0x0
#define TCA9548A_SEL_CH1      0x1
#define TCA9548A_SEL_CH2      0x2
#define TCA9548A_SEL_CH3      0x3
#define TCA9548A_SEL_CH4      0x4
#define TCA9548A_SEL_CH5      0x5
#define TCA9548A_SEL_CH6      0x6
#define TCA9548A_SEL_CH7      0x7

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct tca9548a_dev_s
{
  FAR struct i2c_master_s *i2c;      /* I2C interface */
  uint8_t addr;
  uint8_t state;                     /* Control register state */
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
 * Name: tca9548a_lower_half
 *
 * Description:
 *   Initialize the lower half of the TCA9548A by creating a i2c_master_s
 *   for the virtual i2c master and link it to the associated TCA9548A and
 *   its port.
 *
 * Input Parameters:
 *   dev     - Pointer to the associated TCA9548A
 *   channel - The channel number to be selected to communicate
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
  tca9548a_lower_half(FAR struct tca9548a_dev_s *dev, uint8_t channel);

/****************************************************************************
 * Name: tca9548a_initialize
 *
 * Description:
 *   Initialize the TCA9548A device.
 *
 * Input Parameters:
 *   i2c  - An instance of the I2C interface to use to communicate with
 *          TCA9548A
 *   addr - The I2C address of the TCA9548A.  The base I2C address of the
 *          TCA9548A is 0x70.
 *
 * Returned Value:
 *   Common i2c multiplexer device instance; NULL on failure.
 *
 ****************************************************************************/

FAR struct tca9548a_dev_s *tca9548a_initialize(FAR struct i2c_master_s *i2c,
                                               uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_I2C_TCA9548A_H */
