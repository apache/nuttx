/****************************************************************************
 * boards/arm/stm32/nucleo-f303ze/src/nucleo-f303ze.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F303ZE_SRC_NUCLEO_F303ZE_H
#define __BOARDS_ARM_STM32_NUCLEO_F303ZE_SRC_NUCLEO_F303ZE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/
/* LED
 *
 * The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED, LD2 a
 * Blue LED and LD3 a Red LED, that can be controlled by software. The following
 * definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN0)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN7)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN14)

#define GPIO_LED_GREEN GPIO_LD1
#define GPIO_LED_BLUE  GPIO_LD2
#define GPIO_LED_RED   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/
/* The Nucleo F303ZE supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F303ZET6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F303ZET6.
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* Oled configuration */

#define OLED_I2C_PORT   1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

#endif /* __BOARDS_ARM_STM32_NUCLEO_F303ZE_SRC_NUCLEO_F303ZE_H */
