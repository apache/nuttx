/****************************************************************************
 * arch/arm/src/am67/am67_gpio.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_GPIO_H
#define __ARCH_ARM_SRC_AM67_AM67_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PX4 board_config.h compatibility */

#define GPIO_OUTPUT           0
#define GPIO_INPUT            1

typedef uint32_t am67_gpio_t;

enum am67_gpio_id_e
{
  AM67_GPIO_ID_LED_GREEN = 0,
  AM67_GPIO_ID_LED_RED,
  AM67_GPIO_ID_SPI_CS1,
  AM67_GPIO_ID_SPI_CS3,
  AM67_GPIO_ID_IMU_EN,
  AM67_GPIO_ID_SPI_CS2,
  AM67_GPIO_ID_COUNT
};

#define AM67_GPIO_OLDI0_A0P  ((am67_gpio_t)AM67_GPIO_ID_LED_GREEN)
#define AM67_GPIO_OLDI0_A0N  ((am67_gpio_t)AM67_GPIO_ID_LED_RED)
#define AM67_GPIO_HAT_CS1    ((am67_gpio_t)AM67_GPIO_ID_SPI_CS1)
#define AM67_GPIO_HAT_CS3    ((am67_gpio_t)AM67_GPIO_ID_SPI_CS3)
#define AM67_GPIO_MCU0_PIN12 ((am67_gpio_t)AM67_GPIO_ID_IMU_EN)
#define AM67_GPIO_HAT_CS2    ((am67_gpio_t)AM67_GPIO_ID_SPI_CS2)

/* HAT header GPIO mapping — filled in during F4 PWM bring-up */

#define AM67_GPIO_HAT(n)     am67_gpio_hat(n)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

am67_gpio_t am67_gpio_hat(unsigned int hat_pin);
void am67_configgpio(am67_gpio_t gpio, int pintype);
void am67_gpiowrite(am67_gpio_t gpio, bool value);
bool am67_gpioread(am67_gpio_t gpio);
void am67_sensors_power_enable(bool enable);

#endif /* __ARCH_ARM_SRC_AM67_AM67_GPIO_H */
