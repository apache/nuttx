/****************************************************************************
 * include/nuttx/i2c/i2c_bitbang_ioexpander.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_BITBANG_IOEXPANDER_H
#define __INCLUDE_NUTTX_I2C_I2C_BITBANG_IOEXPANDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/ioexpander/ioexpander.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bitbang_ioexpander_initialize
 *
 * Description:
 *   Initialize i2c bitbang ioexapnder lower half driver.
 *
 * Input Parameters:
 *  ioe     - An instance of the ioexpander device to use for bitbanging
 *  scl_pin - The pin number to use for SCL
 *  sda_pin - The pin number to use for SDA
 *  busnum  - The I2C bus number to register
 *
 *  Returned Value:
 *  On success, a pointer to the initialized I2C driver for the specified.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
i2c_bitbang_ioexpander_initialize(FAR struct ioexpander_dev_s *ioe,
                                  int scl_pin, int sda_pin, int busnum);

#endif /* __INCLUDE_NUTTX_I2C_I2C_BITBANG_IOEXPANDER_H */
