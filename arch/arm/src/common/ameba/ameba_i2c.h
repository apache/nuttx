/****************************************************************************
 * arch/arm/src/common/ameba/ameba_i2c.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_I2C_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The Ameba I2C controllers exposed to NuttX as I2C master buses.  The SCL
 * and SDA pads are given with the same AMEBA_PA()/AMEBA_PB() PinName code
 * used by the GPIO driver (see ameba_gpio.h); any pad can be routed to an
 * I2C bus through the pin mux.
 */

#define AMEBA_I2C0            0
#define AMEBA_I2C1            1

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
 * Name: ameba_i2c_register
 *
 * Description:
 *   Configure one Ameba I2C controller as a master bus and register it with
 *   the NuttX I2C character driver at /dev/i2cN, where N is the bus number.
 *
 * Input Parameters:
 *   bus    - The controller index, AMEBA_I2C0 or AMEBA_I2C1.  Also used as
 *            the /dev/i2cN minor number.
 *   sclpin - The SCL pad, encoded with AMEBA_PA()/AMEBA_PB().
 *   sdapin - The SDA pad, encoded with AMEBA_PA()/AMEBA_PB().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ameba_i2c_register(int bus, uint8_t sclpin, uint8_t sdapin);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_I2C_H */
