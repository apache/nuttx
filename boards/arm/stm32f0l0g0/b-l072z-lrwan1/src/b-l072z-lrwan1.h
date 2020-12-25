/****************************************************************************
 * boards/arm/stm32f0l0g0/b-l072z-lrwan1/src/b-l072z-lrwan1.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_B_L072Z_LRWAN1_SRC_B_L072Z_LRWAN1_H
#define __BOARDS_ARM_STM32F0L0G0_B_L072Z_LRWAN1_SRC_B_L072Z_LRWAN1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Nucleo L073RZ board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32L073RZT6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

#define GPIO_LED1      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH |\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTA | GPIO_PIN5)
#define GPIO_LED2      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB | GPIO_PIN5)
#define GPIO_LED3      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB | GPIO_PIN6)
#define GPIO_LED4      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB | GPIO_PIN7)

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

/* The Nucleo L073RZ supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32L073RZT6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32L073RZT6.
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* SX1276
 * RESET - PC0 (active low)
 * CS    - PA15
 * DIO0  - PB4
 */

#define GPIO_SX127X_RESET (GPIO_PORTC | GPIO_PIN0)
#define GPIO_SX127X_CS    (GPIO_OUTPUT | GPIO_SPEED_HIGH | \
                           GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN15)
#define GPIO_SX127X_DIO0  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI |  \
                           GPIO_PORTB | GPIO_PIN4)

/* CMWX1ZZABZ-091 module antenna switch
 * CRF1 - RX RFI HF - PA1
 * CRF2 - TX RFO HF - PC2
 * CRF3 - TX BOOST  - PC1
 */

#define GPIO_SX127X_CRF1  (GPIO_SPEED_HIGH | GPIO_PORTA | GPIO_PIN1)
#define GPIO_SX127X_CRF2  (GPIO_SPEED_HIGH | GPIO_PORTC | GPIO_PIN2)
#define GPIO_SX127X_CRF3  (GPIO_SPEED_HIGH | GPIO_PORTC | GPIO_PIN1)

/* Oled configuration */

#define OLED_I2C_PORT   1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the B-L072Z-LRWAN1
 *   board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_SPI
void stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_lpwaninitialize
 *
 * Description:
 *   Initialize SX127X LPWAN interaface.
 *
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int stm32_lpwaninitialize(void);
#endif

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

#endif /* __BOARDS_ARM_STM32F0L0G0_B_L072Z_LRWAN1_SRC_B_L072Z_LRWAN1_H */
