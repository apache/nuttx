/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/verdin-mx8mp.h
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

#ifndef __BOARDS_ARM_MX8MP_MX8MP_VERDIN_SRC_VERDIN_MX8MP_H
#define __BOARDS_ARM_MX8MP_MX8MP_VERDIN_SRC_VERDIN_MX8MP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "mx8mp_gpio.h"
#include "mx8mp_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Board LEDs */

#define GPIO_LED_1         (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN0)
#define GPIO_LED_2         (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_LED_3         (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN5)
#define GPIO_LED_4         (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN6)

#define IOMUX_LED_1         IOMUXC_GPIO1_IO00_GPIO1_IO00, 0, GPIO_PAD_CTRL
#define IOMUX_LED_2         IOMUXC_GPIO1_IO01_GPIO1_IO01, 0, GPIO_PAD_CTRL
#define IOMUX_LED_3         IOMUXC_GPIO1_IO05_GPIO1_IO05, 0, GPIO_PAD_CTRL
#define IOMUX_LED_4         IOMUXC_GPIO1_IO06_GPIO1_IO06, 0, GPIO_PAD_CTRL

/* Board buttons */

#define BUTTON_1_GPIO       (GPIO_INTERRUPT | GPIO_INTBOTH_EDGES | GPIO_PORT1 | GPIO_PIN7)
#define BUTTON_1_IRQ        MX8MP_IRQ_SOFT_GPIO1_7
#define BUTTON_1_IOMUX      IOMUXC_GPIO1_IO07_GPIO1_IO07, 0, GPIO_PAD_CTRL

/* SPIs */

#define IOMUX_SPI1_CLK      IOMUXC_ECSPI1_SCLK_ECSPI1_SCLK, 0, SPI_PAD_CTRL
#define IOMUX_SPI1_MOSI     IOMUXC_ECSPI1_MISO_ECSPI1_MISO, 0, SPI_PAD_CTRL
#define IOMUX_SPI1_MISO     IOMUXC_ECSPI1_MOSI_ECSPI1_MOSI, 0, SPI_PAD_CTRL
#define IOMUX_SPI1_CS       IOMUXC_ECSPI1_SS0_ECSPI1_SS0,   0, SPI_PAD_CTRL

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int mx8mp_bringup(void);

/****************************************************************************
 * Name: mx8mp_i2cdev_initialize
 *
 * Description:
 *   Called to configure all i2c
 *
 ****************************************************************************/

int mx8mp_i2cdev_initialize(void);

/****************************************************************************
 * Name: mx8mp_spidev_initialize
 *
 * Description:
 *   Called to configure all spi
 *
 ****************************************************************************/

int mx8mp_spidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_MX8MP_MX8MP_VERDIN_SRC_VERDIN_MX8MP_H */
