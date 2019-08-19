/****************************************************************************
 * boards/arm/efm32/efm32-g8xx-stk/src/efm32-g8xx-stk.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
