/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-l073rz/src/nucleo-l073rz.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_L073RZ_SRC_NUCLEO_L073RZ_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_L073RZ_SRC_NUCLEO_L073RZ_H

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

#define GPIO_LED1      (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_HIGH | \
                        GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)

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

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | \
                        GPIO_PIN13)

/* NRF24L01
 * CS  - PA8 (D7)
 * CE  - PA9 (D8)
 * IRQ - PC7 (D9)
 */

#define GPIO_NRF24L01_CS   (GPIO_OUTPUT | GPIO_SPEED_HIGH |           \
                            GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN8)
#define GPIO_NRF24L01_CE   (GPIO_OUTPUT | GPIO_SPEED_HIGH |             \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN9)
#define GPIO_NRF24L01_IRQ  (GPIO_INPUT | GPIO_FLOAT | GPIO_PORTC | GPIO_PIN7)

/* Dragino LORA shield (v1.4) - RF98 module (based on SX127X)
 * RESET - PC7  (D9)
 * CS    - PB6  (D10)
 * DIO0  - PA10 (D2)
 */

#define GPIO_SX127X_RESET (GPIO_PORTC | GPIO_PIN7)
#define GPIO_SX127X_CS    (GPIO_OUTPUT | GPIO_SPEED_HIGH |            \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN6)
#define GPIO_SX127X_DIO0  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI |  \
                           GPIO_PORTA | GPIO_PIN10)

/* MFRC522
 * CS    - PB4
 * RESET - PB11
 */

#define GPIO_MFRC522_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_HIGH|  \
                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN4)
#define GPIO_MFRC522_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_HIGH|  \
                            GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN11)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/*****************************************************************************
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

/*****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-H743ZI board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_SPI
void stm32_spidev_initialize(void);
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

/*****************************************************************************
 * Name: stm32_lpwaninitialize
 *
 * Description:
 *   Initialize SX127X LPWAN interaface.
 ****************************************************************************/

#ifdef CONFIG_LPWAN_SX127X
int stm32_lpwaninitialize(void);
#endif

/*****************************************************************************
 * Name: stm32_mfrc522initialize
 *
 * Description:
 *   Function used to initialize the MFRC522 RFID Transceiver
 *
 ****************************************************************************/

#ifdef CONFIG_CL_MFRC522
int stm32_mfrc522initialize(FAR const char *devpath);
#endif

#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_L073RZ_SRC_NUCLEO_L073RZ_H */
