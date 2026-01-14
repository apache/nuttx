/****************************************************************************
 * boards/arm64/bcm2711/raspberrypi-4b/src/bcm2711_i2cdev.h
 *
 * Author: Matteo Golin <matteo.golin@gmail.com>
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

#ifndef __BOARDS_ARM64_BCM2711_RASPBERRYPI_4B_INCLUDE_BCM2711_I2CDEV_H
#define __BOARDS_ARM64_BCM2711_RASPBERRYPI_4B_INCLUDE_BCM2711_I2CDEV_H

/****************************************************************************
 * Name: board_i2cdev_initialize
 *
 * Description:
 *   Initialize and register the I2C character driver for the specified I2C
 *   port/bus number.
 *   NOTE: Driver will be registered at /dev/i2c[`port`]
 *
 * Input parameters:
 *     port - The I2C bus number/port number to register the driver for.
 *
 ****************************************************************************/

#if defined(CONFIG_BCM2711_I2C_DRIVER)
int bcm2711_i2cdev_initialize(int port);
#endif /* defined(BCM2711_I2C_DRIVER) */

#endif /* __BOARDS_ARM64_BCM2711_RASPBERRYPI_4B_INCLUDE_BCM2711_I2CDEV_H */
