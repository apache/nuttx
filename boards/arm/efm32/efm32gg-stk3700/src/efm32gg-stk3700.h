/****************************************************************************
 * boards/arm/efm32/efm32gg-stk3700/src/efm32gg-stk3700.h
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

#ifndef __BOARDS_ARM_EFM32_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H
#define __BOARDS_ARM_EFM32_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART0
 *
 * The kit contains a board controller that is responsible for performing
 * various board level tasks, such as handling the debugger and the Advanced
 * Energy Monitor. An interface is provided between the EFM32 and the board
 * controller in the form of a UART connection. The connection is enabled by
 * setting the EFM_BC_EN (PF7) line high, and using the lines EFM_BC_TX
 * (PE0) and EFM_BC_RX (PE1) for communicating.
 */

#ifdef CONFIG_EFM32GG_STK3700_BCEN
#  define GPIO_BC_EN  (GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET|\
                       GPIO_PORTF|GPIO_PIN7)
#else
#  define GPIO_BC_EN  (GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_CLEAR|\
                       GPIO_PORTF|GPIO_PIN7)
#endif

/* LEDs
 *
 * The EFM32 Giant Gecko Start Kit has two yellow LEDs marked LED0 and LED1.
 * These LEDs are controlled by GPIO pins on the EFM32.  The LEDs are
 * connected to pins PE2 and PE3 in an active high configuration:
 *
 * ------------------------------------- --------------------
 * EFM32 PIN                             BOARD SIGNALS
 * ------------------------------------- --------------------
 * E2/BCK_VOUT/EBI_A09 #0/               MCU_PE2 UIF_LED0
 *   TIM3_CC2 #1/U1_TX #3/ACMP0_O #1
 * E3/BCK_STAT/EBI_A10 #0/U1_RX #3/      MCU_PE3 UIF_LED1
 *   ACMP1_O #1
 * ------------------------------------- --------------------
 *
 * All LEDs are grounded and so are illuminated by outputting a high
 * value to the LED.
 */

#define GPIO_LED0     (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                       GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LED1     (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                       GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN3)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_EFM32_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H */
