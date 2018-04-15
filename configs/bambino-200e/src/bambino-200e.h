/****************************************************************************
 * configs/bambino-200e/src/bambino-200e.h
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis acassis@gmail.com [nuttx] <nuttx@googlegroups.com>
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

#ifndef _CONFIGS_BAMBINO_200E_SRC_BAMBINO_H
#define _CONFIGS_BAMBINO_200E_SRC_BAMBINO_H

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

#define HAVE_MMCSD    1

/****************************************************************************
 *   LEDs GPIO                         PIN     SIGNAL NAME
 *  -------------------------------- ------- --------------
 *  gpio3[7] - LED1                  101     GPIO3[7]
 *  gpio5[5] - LED2                  91      GPIO5[5]
 *
 ****************************************************************************/

/* Definitions to configure LED pins as GPIOs:
 *
 * - Floating
 * - Normal drive
 * - No buffering, glitch filtering, slew=slow
 */

#define PINCONFIG_LED1 PINCONF_GPIO3p7
#define PINCONFIG_LED2 PINCONF_GPIO5p5
#define GPIO_LED1      (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | \
                        GPIO_PIN7)
#define GPIO_LED2      (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT5 | \
                        GPIO_PIN5)

/****************************************************************************
 *  Buttons GPIO
 *  ----------------------------
 *  gpio0[7]  - User Button USR1
 ****************************************************************************/

#define BAMBINO_BUT1 (GPIO_INTBOTH | GPIO_FLOAT | GPIO_PORT0 | GPIO_PIN7)

/* Button IRQ numbers */

#define BAMBINO_BUT1_IRQ  LPC43_IRQ_P0p23

#define GPIO_SSP0_SCK  GPIO_SSP0_SCK_1
#define GPIO_SSP0_SSEL GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI GPIO_SSP0_MOSI_1

/* Max31855 Chip Select pins */

#define PINCONFIG_MAX31855_CS1 PINCONF_GPIO0p4
#define PINCONFIG_MAX31855_CS2 PINCONF_GPIO1p8
#define GPIO_MAX31855_CS1  (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | \
                            GPIO_PORT0 | GPIO_PIN4)
#define GPIO_MAX31855_CS2  (GPIO_MODE_OUTPUT | GPIO_VALUE_ONE | \
                            GPIO_PORT1 | GPIO_PIN8)

/* We need to redefine USB_PWRD as GPIO to get USB Host working
 * Also remember to add 2 resistors of 15K to D+ and D- pins.
 */

#ifdef CONFIG_USBHOST
#  ifdef GPIO_USB_PWRD
#    undef  GPIO_USB_PWRD
#    define GPIO_USB_PWRD  (GPIO_INPUT | GPIO_PORT1 | GPIO_PIN22)
#  endif
#endif

/* MMC/SD support */

#ifdef CONFIG_LPC43_SDMMC

#  ifndef CONFIG_MMCSD
#    warning MMC/SD support requires CONFIG_MMCSD
#    undef HAVE_MMCSD
#  endif

#  ifndef CONFIG_MMCSD_SDIO
#    warning MMC/SD support requires CONFIG_MMCSD_SDIO
#    undef HAVE_MMCSD
#  endif

#  ifdef CONFIG_DISABLE_MOUNTPOINT
#    warning MMC/SD cannot be supported with CONFIG_DISABLE_MOUNTPOINT
#    undef HAVE_MMCSD
#  endif

#  ifdef CONFIG_NSH_MMCSDMINOR
#    define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define MMCSD_MINOR 0
#  endif

#else
#  undef HAVE_MMCSD
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
 *   Called to configure SPI chip select GPIO pins for the Bambino-200e
 *   board.
 *
 ****************************************************************************/

void weak_function lpc43_sspdev_initialize(void);

/************************************************************************************
 * Name: lpc43_max31855initialize
 *
 * Description:
 *   Initialize and register the MAX31855 Temperature Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *   devid   - Minor device number. E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int lpc43_max31855initialize(FAR const char *devpath, int bus, uint16_t devid);

/************************************************************************************
 * Name: lpc43xx_timerinitialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the timer hardware.
 *
 ************************************************************************************/

#ifdef CONFIG_TIMER
int lpc43_timerinitialize(void);
#else
#  define lpc43_timerinitialize() (0)
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_BAMBINO_200E_SRC_BAMBINO_H */
