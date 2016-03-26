/************************************************************************************
 * configs/stm32l476vg-disco/src/stm32l476vg-disco.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Authors: Frank Bennett
 *            Gregory Nutt <gnutt@nuttx.org>
 *            Sebastien Lorquet <sebastien@lorquet.fr>
 *            dev@ziggurat29.com
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
 ************************************************************************************/

#ifndef __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H
#define __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

/* LED.
 * LD4: the red LED on PB2
 * LD5: the green LED on PE8
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LED_RED \
  (GPIO_PORTB | GPIO_PIN2 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

#define GPIO_LED_GRN \
  (GPIO_PORTE | GPIO_PIN8 | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_PUSHPULL | \
   GPIO_PULLUP | GPIO_SPEED_50MHz)

/* Buttons
 *
 *  There is a 4 way d-pad 'joystick' with center button
 *  connected to PA0,1,5,2,3
 *                 C L D R U
 */

#define MIN_IRQBUTTON   BUTTON_CENTER
#define MAX_IRQBUTTON   BUTTON_UP
#define NUM_IRQBUTTONS  5

#define GPIO_BTN_CENTER \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)
#define GPIO_BTN_LEFT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN1)
#define GPIO_BTN_DOWN \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN5)
#define GPIO_BTN_RIGHT \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN2)
#define GPIO_BTN_UP \
  (GPIO_INPUT |GPIO_PULLDOWN |GPIO_EXTI | GPIO_PORTA | GPIO_PIN3)

/* SPI1 off */
/* XXX is this used on disco? */

#define GPIO_SPI1_MOSI_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI1_MISO_OFF (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN14)
#define GPIO_SPI1_SCK_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN13)
#define GPIO_SPI1_NSS_OFF  (GPIO_INPUT | GPIO_PULLDOWN | \
                            GPIO_PORTE | GPIO_PIN12)

/* Devices on the onboard I2C bus.
 *
 * Note that these are unshifted addresses.
 */

/* XXX IS this 'unshifted'? */

#define NUCLEO_I2C_OBDEV_CS43L22   0x94

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Global driver instances */

#ifdef CONFIG_STM32_SPI1
extern struct spi_dev_s *g_spi1;
#endif
#ifdef CONFIG_STM32_SPI2
extern struct spi_dev_s *g_spi2;
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

void stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

/************************************************************************************
 * Name: board_adc_initialize
 *
 * Description:
 *   Initialize and register the ADC driver(s)
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int board_adc_initialize(void);
#endif

#endif /* __CONFIGS_STM32L476VG_DISCO_SRC_STM32L476VG_DISCO_H */
