/****************************************************************************
 * boards/arm/rzv2h/rzv2h-rdk/src/rzv2h-rdk.h
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

#ifndef __BOARDS_ARM_RZV2H_RZV2H_RDK_SRC_H
#define __BOARDS_ARM_RZV2H_RZV2H_RDK_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_DRIVER_PATH "/dev/userleds"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declarations */

struct spi_dev_s;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_bringup
 ****************************************************************************/

int rzv2h_bringup(void);

/****************************************************************************
 * Name: rzv2h_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int rzv2h_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   Initialize buttons
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
void board_button_initialize(void);
#endif

/****************************************************************************
 * Name: board_spi_initialize
 *
 * Description:
 *   Initialize SPI buses
 *
 ****************************************************************************/

#ifdef CONFIG_RZV2H_SPI_B
int board_spi_initialize(void);
#endif

/****************************************************************************
 * Name: board_sci_spi_initialize
 *
 * Description:
 *   Initialize SCI_B SPI buses
 *
 ****************************************************************************/

#ifdef CONFIG_RZV2H_SCI_SPI
int board_sci_spi_initialize(void);
#endif

/****************************************************************************
 * Name: rzv2h_appexamples
 *
 * Description:
 *   Run all enabled board example applications
 *
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_serial_setup
 *
 * Description:
 *   Configure serial pins for UART operation
 *
 ****************************************************************************/

void rzv2h_serial_setup(void);

#ifdef CONFIG_RZV2H_EXAMPLE_SUPPORT
int rzv2h_appexamples(void);
#endif

/****************************************************************************
 * Example application initialization functions
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RZV2H_RZV2H_RDK_SRC_H */
