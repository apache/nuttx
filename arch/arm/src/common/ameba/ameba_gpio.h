/****************************************************************************
 * arch/arm/src/common/ameba/ameba_gpio.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_GPIO_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/ioexpander/gpio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A pin is identified with the same encoding the Ameba SDK uses for its
 * "PinName": bits[7:5] select the GPIO port and bits[4:0] the pin within
 * the port.  Boards build their pin table from these helpers so the shared
 * driver never needs to know a board's wiring.
 */

#define AMEBA_PORT_A          0
#define AMEBA_PORT_B          1

#define AMEBA_PIN(port, num)  ((uint8_t)(((port) << 5) | ((num) & 0x1f)))

/* Convenience helpers: one per port exposed by the family.
 * RTL8721Dx / RTL8720F: ports A and B.
 * RTL8721F (Green2): ports A, B and C.
 */

#define AMEBA_PORT_C          2

#define AMEBA_PA(num)         AMEBA_PIN(AMEBA_PORT_A, (num))
#define AMEBA_PB(num)         AMEBA_PIN(AMEBA_PORT_B, (num))
#define AMEBA_PC(num)         AMEBA_PIN(AMEBA_PORT_C, (num))

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
 * Name: ameba_gpio_register
 *
 * Description:
 *   Configure a single Ameba GPIO pin and register it with the NuttX GPIO
 *   (ioexpander) upper half at /dev/gpioN, where N is the given minor.
 *
 * Input Parameters:
 *   minor   - The /dev/gpioN minor number.
 *   pin     - The pin, encoded with AMEBA_PIN() / AMEBA_PA() / AMEBA_PB().
 *   pintype - One of enum gpio_pintype_e (input, output or one of the
 *             interrupt pin types).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int ameba_gpio_register(int minor, uint8_t pin, enum gpio_pintype_e pintype);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_GPIO_H */
