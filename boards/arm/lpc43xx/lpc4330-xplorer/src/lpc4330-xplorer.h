/****************************************************************************
 * boards/arm/lpc43xx/lpc4330-xplorer/src/lpc4330-xplorer.h
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

#ifndef __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_LPC4330_XPLORER_H
#define __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_LPC4330_XPLORER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "lpc43_pinconfig.h"
#include "lpc43_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 *   LEDs GPIO                         PIN     SIGNAL NAME
 *  -------------------------------- ------- --------------
 *  gpio1[12] - LED D2                J10-20  LED1
 *  gpio1[11] - LED D3                J10-17  LED2
 *
 ****************************************************************************/

/* Definitions to configure LED pins as GPIOs:
 *
 * - Floating
 * - Normal drive
 * - No buffering, glitch filtering, slew=slow
 */

#define PINCONFIG_LED1 PINCONF_GPIO4p0
#define PINCONFIG_LED2 PINCONF_GPIO1p1
#define GPIO_LED1      (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN0)
#define GPIO_LED2      (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN8)

/****************************************************************************
 *  Buttons GPIO
 *  ----------------------------
 *  gpio2[7]  - User Button USR1
 ****************************************************************************/

#define LPC4330_XPLORER_BUT1 (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN7)

/* Button IRQ numbers */

#define LPC4330_XPLORER_BUT1_IRQ  LPC43_IRQ_P0p23

#define GPIO_SSP0_SCK  GPIO_SSP0_SCK_1
#define GPIO_SSP0_SSEL GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI GPIO_SSP0_MOSI_1

/* We need to redefine USB_PWRD as GPIO to get USB Host working
 * Also remember to add 2 resistors of 15K to D+ and D- pins.
 */

#ifdef CONFIG_USBHOST
#  ifdef GPIO_USB_PWRD
#    undef  GPIO_USB_PWRD
#    define GPIO_USB_PWRD  (GPIO_INPUT | GPIO_PORT1 | GPIO_PIN22)
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Lincoln 80 board.
 *
 ****************************************************************************/

void weak_function lpc43_sspdev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_LPC4330_XPLORER_H */
