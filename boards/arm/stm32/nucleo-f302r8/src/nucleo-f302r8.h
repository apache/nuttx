/****************************************************************************
 * boards/arm/stm32/nucleo-f302r8/src/nucleo-f302r8.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F302R8_SRC_NUCLEO_F302R8_H
#define __BOARDS_ARM_STM32_NUCLEO_F302R8_SRC_NUCLEO_F302R8_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The Nucleo F302R8 board has three LEDs.  Two of these are controlled by
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
 *           STM32F302R8T6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

#define GPIO_LED1      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN5)

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

/* The Nucleo F302R8 supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F302R8T6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F302R8T6.
 *
 * NOTE that EXTI interrupts are configured.
 */

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* PWM */

#define NUCLEOF302R8_PWMTIMER   1

#ifdef CONFIG_SENSORS_HALL3PHASE
/* GPIO pins used by the 3-phase Hall effect sensor */

#  define GPIO_HALL_PHA (GPIO_INPUT | GPIO_SPEED_2MHz | \
                         GPIO_PORTA | GPIO_PIN15)
#  define GPIO_HALL_PHB (GPIO_INPUT | GPIO_SPEED_2MHz | \
                         GPIO_PORTB | GPIO_PIN3)
#  define GPIO_HALL_PHC (GPIO_INPUT | GPIO_SPEED_2MHz | \
                         GPIO_PORTB | GPIO_PIN10)
#endif

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
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINITIALIZE=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *  Initialize FOC peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FOC
int stm32_foc_setup(void);
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

#endif /* __BOARDS_ARM_STM32_NUCLEO_F302R8_SRC_NUCLEO_F302R8_H */
