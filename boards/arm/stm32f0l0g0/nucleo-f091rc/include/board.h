/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-f091rc/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Four different clock sources can be used to drive the system clock
 * (SYSCLK):
 *
 * - HSI high-speed internal oscillator clock
 *   Generated from an internal 8 MHz RC oscillator
 * - HSE high-speed external oscillator clock
 *   Normally driven by an external crystal (X3). However, this crystal is
 *   not fitted on the Nucleo-F091RC board.
 * - PLL clock
 * - MSI multispeed internal oscillator clock
 *   The MSI clock signal is generated from an internal RC oscillator. Seven
 *   frequency ranges are available: 65.536 kHz, 131.072 kHz, 262.144 kHz,
 *   524.288 kHz, 1.048 MHz, 2.097 MHz (default value) and 4.194 MHz.
 *
 * The devices have the following two secondary clock sources
 * - LSI low-speed internal RC clock
 *   Drives the watchdog and RTC.  Approximately 37KHz
 * - LSE low-speed external oscillator clock
 *   Driven by 32.768KHz crystal (X2) on the OSC32_IN and OSC32_OUT pins.
 */

#define STM32_BOARD_XTAL         8000000ul        /* X3 on board (not fitted)*/

#define STM32_HSI_FREQUENCY      8000000ul        /* Approximately 8MHz */
#define STM32_HSI14_FREQUENCY    14000000ul       /* HSI14 for ADC */
#define STM32_HSI48_FREQUENCY    48000000ul       /* HSI48 for USB, only some STM32F0xx */
#define STM32_HSE_FREQUENCY      STM32_BOARD_XTAL
#define STM32_LSI_FREQUENCY      40000            /* Approximately 40KHz */
#define STM32_LSE_FREQUENCY      32768            /* X2 on board */

/* PLL Configuration
 *
 *   - PLL source is HSI       -> 8MHz input (nominal)
 *   - PLL source predivider 2 -> 4MHz divided down PLL VCO clock output
 *   - PLL multipler is 12     -> 48MHz PLL VCO clock output (for USB)
 *
 * Resulting SYSCLK frequency is 8MHz x 12 / 2 = 48MHz
 *
 * USB:
 *   If the USB interface is used in the application, it requires a precise
 *   48MHz clock which can be generated from either the (1) the internal
 *   main PLL with the HSE clock source using an HSE crystal oscillator.  In
 *   this case,  the PLL VCO clock (defined by STM32_CFGR_PLLMUL) must be
 *   programmed to output a 96 MHz frequency. This is required to provide a
 *   48MHz clock to the (USBCLK = PLLVCO/2).  Or (2) by using the internal
 *   48MHz oscillator in automatic trimming mode. The synchronization for
 *   this oscillator can be taken from the USB data stream itself (SOF
 *   signalization) which allows crystal-less operation.
 * SYSCLK
 *   The system clock is derived from the PLL VCO divided by the output
 *   division factor.
 * Limitations:
 *   - 96 MHz as PLLVCO when the product is in range 1 (1.8V),
 *   - 48 MHz as PLLVCO when the product is in range 2 (1.5V),
 *   - 24 MHz when the product is in range 3 (1.2V).
 *   - Output division to avoid exceeding 32 MHz as SYSCLK.
 *   - The minimum input clock frequency for PLL is 2 MHz (when using HSE as
 *     PLL source).
 */

#define STM32_CFGR_PLLSRC        RCC_CFGR_PLLSRC_HSId2       /* Source is HSI/2 */
#define STM32_PLLSRC_FREQUENCY   (STM32_HSI_FREQUENCY/2)     /* 8MHz / 2 = 4MHz */
#ifdef CONFIG_STM32F0L0G0_USB
#  undef  STM32_CFGR2_PREDIV                                 /* Not used with source HSI/2 */
#  define STM32_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx12      /* PLLMUL = 12 */
#  define STM32_PLL_FREQUENCY    (12*STM32_PLLSRC_FREQUENCY) /* PLL VCO Frequency is 48MHz */
#else
#  undef  STM32_CFGR2_PREDIV                                 /* Not used with source HSI/2 */
#  define STM32_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx12      /* PLLMUL = 12 */
#  define STM32_PLL_FREQUENCY    (12*STM32_PLLSRC_FREQUENCY) /* PLL VCO Frequency is 48MHz */
#endif

/* Use the PLL and set the SYSCLK source to be the divided down PLL VCO
 * output frequency (STM32_PLL_FREQUENCY divided by the PLLDIV value).
 */

#define STM32_SYSCLK_SW          RCC_CFGR_SW_PLL     /* Use the PLL as the SYSCLK */
#define STM32_SYSCLK_SWS         RCC_CFGR_SWS_PLL
#ifdef CONFIG_STM32F0L0G0_USB
#  define STM32_SYSCLK_FREQUENCY STM32_PLL_FREQUENCY /* SYSCLK frequency is PLL VCO = 48MHz */
#else
#  define STM32_SYSCLK_FREQUENCY STM32_PLL_FREQUENCY /* SYSCLK frequency is PLL VCO = 48MHz */
#endif

#define STM32_RCC_CFGR_HPRE      RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY     STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (48MHz) */

#define STM32_RCC_CFGR_PPRE1     RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (48MHz) */

#define STM32_RCC_CFGR_PPRE2     RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY    STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN         (STM32_PCLK2_FREQUENCY)

/* APB1 timers 1-3, 6-7, and 14-17 will receive PCLK1 */

#define STM32_APB1_TIM1_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM2_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN    (STM32_PCLK1_FREQUENCY)

#define STM32_APB1_TIM6_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN    (STM32_PCLK1_FREQUENCY)

#define STM32_APB1_TIM14_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM15_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN   (STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* LEDs
 *
 * The Nucleo-64 board has one user controllable LED, User LD2.  This green
 * LED is a user LED connected to Arduino signal D13 corresponding to STM32
 * I/O PA5 (PB13 on other some other Nucleo-64 boards).
 *
 *   - When the I/O is HIGH value, the LED is on
 *   - When the I/O is LOW, the LED is off
 */

/* LED index values for use with board_userled() */

#define BOARD_LD2         0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LD2_BIT     (1 << BOARD_LD2)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD2
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PC13 (pin 2) of the STM32
 * microcontroller.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate Pin Functions **************************************************/

/* I2C
 *   PB8 - D15
 *   PB9 - D14
 */

#define GPIO_I2C1_SCL  (GPIO_I2C1_SCL_2|GPIO_SPEED_LOW)     /* PB8 */
#define GPIO_I2C1_SDA  (GPIO_I2C1_SDA_2|GPIO_SPEED_LOW)     /* PB9 */

/* SPI */

#define GPIO_SPI1_MISO (GPIO_SPI1_MISO_1|GPIO_SPEED_MEDIUM) /* D12 - PA6 */
#define GPIO_SPI1_MOSI (GPIO_SPI1_MOSI_1|GPIO_SPEED_MEDIUM) /* D11 - PA7 */
#define GPIO_SPI1_SCK  (GPIO_SPI1_SCK_1|GPIO_SPEED_MEDIUM)  /* D13 - PA5 */

/* USART 1 */

#define GPIO_USART1_TX (GPIO_USART1_TX_2|GPIO_SPEED_HIGH)
#define GPIO_USART1_RX (GPIO_USART1_RX_2|GPIO_SPEED_HIGH)

/* USART 2 */

#define GPIO_USART2_TX (GPIO_USART2_TX_3|GPIO_SPEED_HIGH)
#define GPIO_USART2_RX (GPIO_USART2_RX_3|GPIO_SPEED_HIGH)

#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_F091RC_INCLUDE_BOARD_H */
