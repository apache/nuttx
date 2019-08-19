/****************************************************************************
 * boards/arm/lpc43xx/lpc4330-xplorer/src/lpc4330-xplorer.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_XPLORER_H
#define __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_XPLORER_H

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
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
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
#endif /* __BOARDSS_ARM_LPC43XX_LPC4330_XPLORER_SRC_XPLORER_H */
