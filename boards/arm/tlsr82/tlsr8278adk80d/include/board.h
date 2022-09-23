/****************************************************************************
 * boards/arm/tlsr82/tlsr8278adk80d/include/board.h
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

#ifndef __BOARDS_ARM_TLSR82_TLSR8278ADK80D_INCLUDE_BOARD_H
#define __BOARDS_ARM_TLSR82_TLSR8278ADK80D_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/ioexpander/gpio.h>

#include "tlsr82_adc.h"
#include "tlsr82_gpio.h"
#include "tlsr82_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Board GPIO PIN Configuration
 * BOARD_NGPIO      : the gpio numbers, max 8
 * BOARD_GPIOx_PIN  : define the gpio pin number, see tlsr82_gpio.h
 * BOARD_GPIOx_TYPE : define the pio type, see enum gpio_pintype_e in
 *                    nuttx/ioexpander/gpio.h
 */

#define BOARD_NGPIO                    4

#if BOARD_NGPIO > 0
#  define BOARD_GPIO0_PIN              GPIO_PIN_PD6
#  define BOARD_GPIO0_TYPE             GPIO_INPUT_PIN_PULLDOWN
#endif
#if BOARD_NGPIO > 1
#  define BOARD_GPIO1_PIN              GPIO_PIN_PD0
#  define BOARD_GPIO1_TYPE             GPIO_OUTPUT_PIN
#endif
#if BOARD_NGPIO > 2
#  define BOARD_GPIO2_PIN              GPIO_PIN_PD1
#  define BOARD_GPIO2_TYPE             GPIO_OUTPUT_PIN
#endif
#if BOARD_NGPIO > 3
#  define BOARD_GPIO3_PIN              GPIO_PIN_PB3
#  define BOARD_GPIO3_TYPE             GPIO_INTERRUPT_FALLING_PIN
#endif
#if BOARD_NGPIO > 4
#  define BOARD_GPIO4_PIN              GPIO_INVLD_CFG
#  define BOARD_GPIO4_TYPE             GPIO_NPINTYPES
#endif
#if BOARD_NGPIO > 5
#  define BOARD_GPIO5_PIN              GPIO_INVLD_CFG
#  define BOARD_GPIO5_TYPE             GPIO_NPINTYPES
#endif
#if BOARD_NGPIO > 6
#  define BOARD_GPIO6_PIN              GPIO_INVLD_CFG
#  define BOARD_GPIO6_TYPE             GPIO_NPINTYPES
#endif
#if BOARD_NGPIO > 7
#  define BOARD_GPIO7_PIN              GPIO_INVLD_CFG
#  define BOARD_GPIO7_TYPE             GPIO_NPINTYPES
#endif
#if BOARD_NGPIO > 8
#  error "BOARD_NGPIO max value is 8"
#endif

/* Board UART PIN Configuration
 * BOARD_UARTx_RX/TX_PIN : define the uart pin number, see tlsr82_gpio.h
 * BOARD_UARTx_RX/TX_MUX : define the gpio multiplex function, should be
 *                         UART function, see the tlsr82 datasheet
 */

#define BOARD_UART0_RX_PIN             GPIO_PIN_PB0
#define BOARD_UART0_RX_MUX             GPIO_AF_MUX1
#define BOARD_UART0_TX_PIN             GPIO_PIN_PB1
#define BOARD_UART0_TX_MUX             GPIO_AF_MUX1

/* Board ADC PIN Configuration
 * BOARD_ADCx_CHAN      : the ADC channel
 * BOARD_ADCx_CHAN_TYPE : the ADC type,
 * BOARD_ADCx_PIN       : the ADC input gpio pin, please see the datasheet to
 *                        configure this pin
 * Note: For ADC_CHAN_TYPE_VBAT and ADC_CHAN_TYPE_TEMP, do not need configure
 *       the external sample pin, so configure it to GPIO_INVLD_CFG
 *
 */

#ifdef CONFIG_TLSR82_ADC_CHAN0
#  define BOARD_ADC0_CHAN              ADC_CHAN_0
#  define BOARD_ADC0_CHAN_TYPE         ADC_CHAN_TYPE_BASE
#  define BOARD_ADC0_PIN               GPIO_PIN_PB2
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN1
#  define BOARD_ADC1_CHAN              ADC_CHAN_1
#  define BOARD_ADC1_CHAN_TYPE         ADC_CHAN_TYPE_BASE
#  define BOARD_ADC1_PIN               GPIO_PIN_PB3
#endif

#ifdef CONFIG_TLSR82_ADC_CHAN2
#  define BOARD_ADC2_CHAN              ADC_CHAN_2
#  define BOARD_ADC2_CHAN_TYPE         ADC_CHAN_TYPE_BASE
#  define BOARD_ADC2_PIN               GPIO_PIN_PB5
#endif

#ifdef CONFIG_TLSR82_ADC_VBAT
#  define BOARD_ADCVBAT_CHAN           ADC_CHAN_VBAT
#  define BOARD_ADCVBAT_CHAN_TYPE      ADC_CHAN_TYPE_VBAT
#  define BOARD_ADCVBAT_PIN            GPIO_INVLD_CFG
#endif

/* Board PWM PIN Configuration
 * BOARD_PWMx_PIN : define the gpio pin numbers and multiplex function,
 *                  please refs the tlsr82 datasheet to configure the
 *                  multiplex function.
 */

#ifdef CONFIG_TLSR82_PWM0
#  define BOARD_PWM0_PIN               (GPIO_PIN_PD5 | GPIO_AF_MUX0)
#endif

#ifdef CONFIG_TLSR82_PWM1
#  define BOARD_PWM1_PIN               (GPIO_PIN_PD3 | GPIO_AF_MUX0)
#endif

#ifdef CONFIG_TLSR82_PWM2
#  define BOARD_PWM2_PIN               (GPIO_PIN_PD4 | GPIO_AF_MUX2)
#endif

#ifdef CONFIG_TLSR82_PWM3
#  define BOARD_PWM3_PIN               (GPIO_PIN_PB4 | GPIO_AF_MUX1)
#endif

#ifdef CONFIG_TLSR82_PWM4
#  define BOARD_PWM4_PIN               (GPIO_INVLD_CFG)
#endif

#ifdef CONFIG_TLSR82_PWM5
#  define BOARD_PWM5_PIN               (GPIO_INVLD_CFG)
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_TLSR82_TLSR8278ADK80D_INCLUDE_BOARD_H */
