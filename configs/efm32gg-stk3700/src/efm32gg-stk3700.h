/****************************************************************************
 * configs/efm32gg-stk3700/include/efm32gg-stk3700.h
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

#ifndef __CONFIGS_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H
#define __CONFIGS_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H

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

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_led_initialize(void);
#endif

#endif /* __CONFIGS_EFM32GG_STK3700_SRC_EFM32GG_STK3700_H */
