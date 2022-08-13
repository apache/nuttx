/****************************************************************************
 * boards/arm/stm32/stm32f334-disco/src/stm32f334-disco.h
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

#ifndef __BOARDS_ARM_STM32_STM32F334_DISCO_SRC_STM32F334_DISCO_H
#define __BOARDS_ARM_STM32_STM32F334_DISCO_SRC_STM32F334_DISCO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LED definitions **********************************************************/

#define GPIO_LED1      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN6)
#define GPIO_LED2      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#define GPIO_LED3      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
#define GPIO_LED4      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                        GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN7)

#define LED_DRIVER_PATH "/dev/userleds"

/* Button definitions *******************************************************/

#define MIN_IRQBUTTON  BUTTON_USER
#define MAX_IRQBUTTON  BUTTON_USER
#define NUM_IRQBUTTONS 1

#define GPIO_BTN_USER  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* PWM definitions **********************************************************/

/* The STM32F334-DISCO has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using variously unused pins on the
 * board for PWM output (see board.h for details of pins).
 */

#ifdef CONFIG_PWM
#  if defined(CONFIG_STM32_TIM2_PWM)
#    define STM32F334_DISCO_PWMTIMER 2
#  elif defined(CONFIG_STM32_TIM3_PWM)
#    define STM32F334_DISCO_PWMTIMER 3
#  elif defined(CONFIG_STM32_TIM4_PWM)
#    define STM32F334_DISCO_PWMTIMER 4
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI
void weak_function stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_timer_driver_setup
 *
 * Description:
 *   Configure the timer driver.
 *
 * Input Parameters:
 *   devpath - The full path to the timer device.
 *             This should be of the form /dev/timer0
 *   timer   - The timer's number.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_TIMER
int stm32_timer_driver_setup(const char *devpath, int timer);
#endif

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Configure DAC peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
int stm32_dac_setup(void);
#endif

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
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_comp_setup
 *
 * Description:
 *  Initialize COMP peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_COMP
int stm32_comp_setup(void);
#endif

/****************************************************************************
 * Name: stm32_opamp_setup
 *
 * Description:
 *  Initialize OPAMP peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_OPAMP
int stm32_opamp_setup(void);
#endif

/****************************************************************************
 * Name: stm32_powerled_setup
 *
 * Description:
 *  Initialize POWERLED peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_POWERLED
int stm32_powerled_setup(void);
#endif

/****************************************************************************
 * Name: stm32_smps_setup
 *
 * Description:
 *  Initialize SMPS peripheral for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_SMPS
int stm32_smps_setup(void);
#endif

#endif /* __BOARDS_ARM_STM32_STM32F334_DISCO_SRC_STM32F334_DISCO_H */
