/************************************************************************************
 * configs/stm32ldiscovery/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIG_STM32LDISCOVERY_INCLUDE_BOARD_H
#define __CONFIG_STM32LDISCOVERY_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Four different clock sources can be used to drive the system clock (SYSCLK):
 *
 * - HSI high-speed internal oscillator clock
 *   Generated from an internal 16 MHz RC oscillator
 * - HSE high-speed external oscillator clock
 *   Normally driven by an external crystal (X3).  However, this crystal is not fitted
 *   on the STM32L-Discovery board.
 * - PLL clock
 * - MSI multispeed internal oscillator clock
 *   The MSI clock signal is generated from an internal RC oscillator. Seven frequency
 *   ranges are available: 65.536 kHz, 131.072 kHz, 262.144 kHz, 524.288 kHz, 1.048 MHz,
 *   2.097 MHz (default value) and 4.194 MHz.
 *
 * The devices have the following two secondary clock sources
 * - LSI low-speed internal RC clock
 *   Drives the watchdog and RTC.  Approximately 37KHz
 * - LSE low-speed external oscillator clock
 *   Driven by 32.768KHz crystal (X2) on the OSC32_IN and OSC32_OUT pins.
 */

#define STM32_BOARD_XTAL         8000000ul        /* X3 on board (not fitted)*/

#define STM32_HSI_FREQUENCY      16000000ul       /* Approximately 16MHz */
#define STM32_HSE_FREQUENCY      STM32_BOARD_XTAL
#define STM32_MSI_FREQUENCY      2097000          /* Default is approximately 2.097Mhz */
#define STM32_LSI_FREQUENCY      37000            /* Approximately 37KHz */
#define STM32_LSE_FREQUENCY      32768            /* X2 on board */

/* PLL Configuration
 *
 *   - PLL source is HSI      -> 16MHz input (nominal)
 *   - PLL multipler is 6     -> 96MHz PLL VCO clock output (for USB)
 *   - PLL output divider 3   -> 32MHz divided down PLL VCO clock output
 *
 * Resulting SYSCLK frequency is 16MHz x 6 / 3 = 32MHz
 *
 * USB/SDIO:
 *   If the USB or SDIO interface is used in the application, the PLL VCO
 *   clock (defined by STM32_CFGR_PLLMUL) must be programmed to output a 96
 *   MHz frequency. This is required to provide a 48 MHz clock to the USB or
 *   SDIO (SDIOCLK or USBCLK = PLLVCO/2).
 * SYSCLK
 *   The system clock is derived from the PLL VCO divided by the output division factor.
 * Limitations:
 *   96 MHz as PLLVCO when the product is in range 1 (1.8V),
 *   48 MHz as PLLVCO when the product is in range 2 (1.5V),
 *   24 MHz when the product is in range 3 (1.2V).
 *   Output division to avoid exceeding 32 MHz as SYSCLK.
 *   The minimum input clock frequency for PLL is 2 MHz (when using HSE as PLL source).
 */

#define STM32_CFGR_PLLSRC        0                       /* Source is 16MHz HSI */
#ifdef CONFIG_STM32_USB
#  define STM32_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx6   /* PLLMUL = 6 */
#  define STM32_CFGR_PLLDIV      RCC_CFGR_PLLDIV_3       /* PLLDIV = 3 */
#  define STM32_PLL_FREQUENCY    (6*STM32_HSI_FREQUENCY) /* PLL VCO Frequency is 96MHz */
#else
#  define STM32_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx4   /* PLLMUL = 4 */
#  define STM32_CFGR_PLLDIV      RCC_CFGR_PLLDIV_2       /* PLLDIV = 2 */
#  define STM32_PLL_FREQUENCY    (4*STM32_HSI_FREQUENCY) /* PLL VCO Frequency is 64MHz */
#endif

/* Use the PLL and set the SYSCLK source to be the divided down PLL VCO output
 * frequency (STM32_PLL_FREQUENCY divided by the PLLDIV value).
 */

#define STM32_SYSCLK_SW          RCC_CFGR_SW_PLL         /* Use the PLL as the SYSCLK */
#define STM32_SYSCLK_SWS         RCC_CFGR_SWS_PLL
#ifdef CONFIG_STM32_USB
#  define STM32_SYSCLK_FREQUENCY (STM32_PLL_FREQUENCY/3) /* SYSCLK frequence is 96MHz/PLLDIV = 32MHz */
#else
#  define STM32_SYSCLK_FREQUENCY (STM32_PLL_FREQUENCY/2) /* SYSCLK frequence is 64MHz/PLLDIV = 32MHz */
#endif

/* AHB clock (HCLK) is SYSCLK (32MHz) */

#define STM32_RCC_CFGR_HPRE      RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY     STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK         STM32_HCLK_FREQUENCY    /* Same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (32MHz) */

#define STM32_RCC_CFGR_PPRE2     RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY    STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN         (STM32_PCLK2_FREQUENCY)

/* APB2 timers 9, 10, and 11 will receive PCLK2. */

#define STM32_APB2_TIM9_CLKIN    (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK (32MHz) */

#define STM32_RCC_CFGR_PPRE1     RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY)

/* APB1 timers 2-7 will receive PCLK1 */

#define STM32_APB1_TIM2_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN    (STM32_PCLK1_FREQUENCY)

/* LED definitions ******************************************************************/
/* The STM32L-Discovery board has four LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:   LD2 default status is red. LD2 turns to green to indicate that
 *            communications are in progress between the PC and the ST-LINK/V2.
 * LD2 PWR:   Red LED indicates that the board is powered.
 *
 * And two LEDs can be controlled by software:
 *
 * User LD3:  Green LED is a user LED connected to the I/O PB7 of the STM32L152
 *            MCU.
 * User LD4:  Blue LED is a user LED connected to the I/O PB6 of the STM32L152
 *            MCU.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */

#define BOARD_LED1               0 /* User LD3 */
#define BOARD_LED2               1 /* User LD4 */
#define BOARD_NLEDS              2

/* LED bits for use with stm32_setleds() */

#define BOARD_LED1_BIT           (1 << BOARD_LED1)
#define BOARD_LED2_BIT           (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 8 LEDs on board the
 * STM32L-Discovery.  The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                   LED1     LED2
 *   -------------------  -----------------------  -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF
 *   LED_INIRQ            In an interrupt              No change
 *   LED_SIGNAL           In a signal handler          No change
 *   LED_ASSERTION        An assertion failed          No change
 *   LED_PANIC            The system has crashed     OFF      Blinking
 *   LED_IDLE             STM32 is is sleep mode       Not used
 */

#define LED_STARTED              0
#define LED_HEAPALLOCATE         0
#define LED_IRQSENABLED          0
#define LED_STACKCREATED         1
#define LED_INIRQ                2
#define LED_SIGNAL               2
#define LED_ASSERTION            2
#define LED_PANIC                3

/* Button definitions ***************************************************************/
/* The STM32L-Discovery supports two buttons; only one button is controllable by
 * software:
 *
 *   B1 USER: user and wake-up button connected to the I/O PA0 of the STM32L152RBT6.
 *   B2 RESET: pushbutton connected to NRST is used to RESET the STM32L152RBT6.
 */

#define BUTTON_USER              0
#define NUM_BUTTONS              1

#define BUTTON_USER_BIT          (1 << BUTTON_USER)

/* Alternate Pin Functions **********************************************************/
/* The STM32L-Discovery has no on-board RS-232 driver.  Further, there are no USART
 * pins that do not conflict with the on board resources, in particular, the LCD:
 * Most USART pins are available if the LCD is enabled; USART2 may be used if either
 * the LCD or the on-board LEDs are disabled.
 *
 *   PA9   USART1_TX  LCD glass COM1  P2, pin 22
 *   PA10  USART1_RX  LCD glass COM2  P2, pin 21
 *   PB6   USART1_TX  LED Blue        P2, pin 8
 *   PB7   USART1_RX  LED Green       P2, pin 7
 *
 *   PA2   USART2_TX  LCD SEG1        P1, pin 17
 *   PA3   USART2_RX  LCD SEG2        P1, pin 18
 *
 *   PB10  USART3_TX LCD SEG6         P1, pin 22
 *   PB11  USART3_RX LCD SEG7         P1, pin 23
 *   PC10  USART3_TX LCD SEG22        P2, pin 15
 *   PC11  USART3_RX LCD SEG23        P2, pin 14
 */

#if !defined(CONFIG_STM32_LCD)
/* Select PA9 and PA10 if the LCD is not enabled */

#  define GPIO_USART1_RX         GPIO_USART1_RX_1 /* PA10 */
#  define GPIO_USART1_TX         GPIO_USART1_TX_1 /* PA9 */

/* This there are no other options for USART1 on this part */

#  define GPIO_USART2_RX         GPIO_USART2_RX_1 /* PA3 */
#  define GPIO_USART2_TX         GPIO_USART2_TX_1 /* PA2 */

/* Arbirtrarily select PB10 and PB11 */

#  define GPIO_USART3_RX         GPIO_USART3_RX_1 /* PB11 */
#  define GPIO_USART3_TX         GPIO_USART3_TX_1 /* PB10 */

#elif !defined(CONFIG_ARCH_LEDS)

/* Select PB6 and PB7 if the LEDs are not enabled */

#  define GPIO_USART1_RX         GPIO_USART1_RX_2 /* PB7 */
#  define GPIO_USART1_TX         GPIO_USART1_TX_2 /* PB6 */

#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

/************************************************************************************
 * Name:  stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void stm32_ledinit(void);
void stm32_setled(int led, bool ledon);
void stm32_setleds(uint8_t ledset);
#endif

/****************************************************************************
 * Name: stm32_slcd_initialize
 *
 * Description:
 *   Initialize the STM32L-Discovery LCD hardware and register the character
 *   driver as /dev/slcd.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_LCD
int stm32_slcd_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_STM32LDISCOVERY_INCLUDE_BOARD_H */
