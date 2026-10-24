/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd-xd-w515-eval.h
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

#ifndef __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_SRC_GD_XD_W515_EVAL_H
#define __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_SRC_GD_XD_W515_EVAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GD30BC2416 battery charger on I2C0.  The charge current is set by its
 * ISET resistor, R34 = 30 kOhm: I = 10 / R(kOhm) A = 333 mA.
 */

#define GD32_CHARGER_I2C_FREQUENCY  100000
#define GD32_CHARGER_ISET_CURRENT   333

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Bring up board features.
 *
 ****************************************************************************/

int gd32_bringup(void);

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Configure the chip select and control GPIOs of the SPI devices.
 *
 ****************************************************************************/

#ifdef CONFIG_GD32W51X_SPI
void gd32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_gpio_initialize
 *
 * Description:
 *   Register the board GPIOs as /dev/gpioN.
 *
 ****************************************************************************/

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
int gd32_gpio_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_SRC_GD_XD_W515_EVAL_H */
