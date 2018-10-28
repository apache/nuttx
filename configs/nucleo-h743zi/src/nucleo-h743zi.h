/************************************************************************************
 * configs/nucleo-h743zi/src/nucleo-h743zi.h
 *
#   Copyright (C) 2017 Gwenhael Goavec-Merou. All rights reserved.
#   Author: Gwenhael Goavec-Merou<gwenhael.goavec-merou@trabucayre.com>
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

#ifndef __CONFIGS_NUCLEO_H743ZI_SRC_NUCLEO_H743ZI_H
#define __CONFIGS_NUCLEO_H743ZI_SRC_NUCLEO_H743ZI_H

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

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.  A high value
 * will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt on this
 *       IO.
 *    2) The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* X-NUCLEO IKS01A2 */

#define GPIO_LPS22HB_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN10)
#define GPIO_LSM6DSL_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN4)
#define GPIO_LSM6DSL_INT2 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTB | GPIO_PIN5)

/* NRF24L01
 * CS  - PA4
 * CE  - PF12 (D8)
 * IRQ - PD15 (D9)
 */

#define GPIO_NRF24L01_CS   (GPIO_OUTPUT | GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN4)
#define GPIO_NRF24L01_CE   (GPIO_OUTPUT | GPIO_SPEED_50MHz| \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTF | GPIO_PIN12)
#define GPIO_NRF24L01_IRQ  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTD | GPIO_PIN15)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ************************************************************************************/

int stm32_bringup(void);

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-H743ZI board.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32H7_SPI
void stm32_spidev_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

// TODO: implement ADC or delete this

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif


/*****************************************************************************
 * Name: stm32_lsm6dsl_initialize
 *
 * Description:
 *   Initialize I2C-based LSM6DSL.
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM303AGR
int stm32_lsm6dsl_initialize(char *devpath);
#endif

/*****************************************************************************
 * Name: stm32_lsm303agr_initialize
 *
 * Description:
 *   Initialize I2C-based LSM303AGR.
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LSM6DSL
int stm32_lsm303agr_initialize(char *devpath);
#endif

/*****************************************************************************
 * Name: stm32_wlinitialize
 *
 * Description:
 *   Initialize NRF24L01 wireless interaface.
 ****************************************************************************/

#ifdef CONFIG_WL_NRF24L01
int stm32_wlinitialize(void);
#endif

#endif /* __CONFIGS_NUCLEO_H743ZI_SRC_NUCLEO_H743ZI_H */
