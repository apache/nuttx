/****************************************************************************
 * boards/arm/efm32/efm32-g8xx-stk/src/efm32-g8xx-stk.h
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

#ifndef __BOARDS_ARM_EFM32_EFM32_G8XX_STK_SRC_EFM32_G8XX_STK_H
#define __BOARDS_ARM_EFM32_EFM32_G8XX_STK_SRC_EFM32_G8XX_STK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART0
 *
 *   The control MCU acts as a board controller (BC). There is a UART
 *   connection between the EFM and the BC. The connection is made by
 *   setting the EFM_BC_EN (PD13) line high. The EFM can then use the BSP to
 *   send commands to the BC. When EFM_BC_EN is low, EFM_BC_TX and EFM_BC_RX
 *   can be used by other applications.
 */

#ifdef CONFIG_EFM32G8STK_BCEN
#  define GPIO_BC_EN  (GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_SET|\
                       GPIO_PORTD|GPIO_PIN13)
#else
#  define GPIO_BC_EN  (GPIO_OUTPUT_PUSHPULL|GPIO_OUTPUT_CLEAR|\
                       GPIO_PORTD|GPIO_PIN13)
#endif

/* LEDs
 *
 * The EFM32 Gecko Start Kit has four yellow LEDs.  These LEDs are connected
 * as follows:
 *
 *   ------------------------------------- --------------------
 *   EFM32 PIN                             BOARD SIGNALS
 *   ------------------------------------- --------------------
 *   C0/USART1_TX#0/PCNT0_S0IN#2/ACMP0_CH0  MCU_PC0  UIF_LED0
 *   C1/USART1_RX#0/PCNT0_S1IN#2/ACMP0_CH1  MCU_PC1  UIF_LED1
 *   C2/USART2_TX#0/ACMP0_CH2               MCU_PC2  UIF_LED2
 *   C3/USART2_RX#0/ACMP0_CH3               MCU_PC3  UIF_LED3
 *   ------------------------------------- --------------------
 *
 * All LEDs are grounded and so are illuminated by outputting a high
 * value to the LED.
 */

#define GPIO_LED0       (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)
#define GPIO_LED1       (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LED2       (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN2)
#define GPIO_LED3       (GPIO_OUTPUT_WIREDOR_PULLDOWN|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN3)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __BOARDS_ARM_EFM32_EFM32_G8XX_STK_SRC_EFM32_G8XX_STK_H */
