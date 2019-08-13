/****************************************************************************
 * boards/arm/lpc17xx_40xx/u-blox-c027/src/u-blox-c027.h
 *
 *   Copyright (C) 2016 Vladimir Komendantskiy. All rights reserved.
 *   Author: Vladimir Komendantskiy <vladimir@moixaenergy.com>
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_U_BLOX_C027_SRC_U_BLOX_C027_H
#define __BOARDS_ARM_LPC17XX_40XX_U_BLOX_C027_SRC_U_BLOX_C027_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define C027_LED (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT3 | GPIO_PIN25)

#define C027_SD_CS (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN2)
#ifdef CONFIG_LPC17_40_GPIOIRQ
#  define C027_SD_CD (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#else
#  define C027_SD_CD (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN11)
#endif

#define GPSADDR            (66<<1) /* GPS I2C Address */
#define GPSBAUD            9600    /* Default GPS Baud Rate */
#define MDMBAUD            115200  /* Default Modem Baud Rate */

#define C027_MDMUSBDET     (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN7)
#define C027_MDMILVLOE     (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN8)
#define C027_MDMLVLOE      (GPIO_OUTPUT | GPIO_PORT0 | GPIO_PIN9)
#define C027_MDMLDOEN      (GPIO_OUTPUT | GPIO_PORT2 | GPIO_PIN5)
#define C027_MDMPWR        (GPIO_OUTPUT | GPIO_PORT2 | GPIO_PIN6)
#define C027_MDMGPIO1      (GPIO_INPUT  | GPIO_PORT2 | GPIO_PIN7)
#define C027_MDMRST        (GPIO_OUTPUT | GPIO_PORT2 | GPIO_PIN8)
#define C027_GPSEN         (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN29)
#define C027_GPSRST        (GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN18)

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: c027_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the C027 board.
 *
 ****************************************************************************/

void weak_function c027_sspdev_initialize(void);

/****************************************************************************
 * Name: lpc17_40_ubxmdm_init
 *
 * Description:
 *   Initialisation function for the u-blox modem.
 *
 ****************************************************************************/

#if defined(CONFIG_MODEM_U_BLOX)
void lpc17_40_ubxmdm_init(bool usb_used);
#endif /* CONFIG_MODEM_U_BLOX */

/****************************************************************************
 * Name: lpc17_40_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int lpc17_40_pwm_setup(void);
#endif

/****************************************************************************
 * Name: lpc17_40_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int lpc17_40_adc_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_U_BLOX_C027_SRC_U_BLOX_C027_H */
