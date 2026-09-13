/****************************************************************************
 * boards/arm/n32h7/n32h762iil7/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

 #ifndef __BOARDS_ARM_N32H7_N32H762IIL7_INCLUDE_BOARD_H
 #define __BOARDS_ARM_N32H7_N32H762IIL7_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

 #include <nuttx/config.h>

 #ifndef __ASSEMBLY__
 #  include <stdint.h>
 #endif

/* Do not include N32H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The N32H762IIL7 board provides the following clock sources:
 *
 *   X1:  25 MHz crystal oscillator
 *   X2:  32.768 KHz crystal oscillator
 *
 * So we have these clock source available within the N32H7
 *
 *   HSI: 64 MHz RC factory-trimmed
 *   MSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 25 MHz crystal oscillator
 *   LSE: 32.768 KHz crystal oscillator
 */

 #define N32_BOARD_XTAL        25000000ul

 #define N32_HSI_FREQUENCY     64000000ul
 #define N32_MSI_FREQUENCY     16000000ul
 #define N32_LSI_FREQUENCY     32000ul
 #define N32_HSE_FREQUENCY     N32_BOARD_XTAL
 #define N32_LSE_FREQUENCY     32768ul

/* Main PLL Configuration.
 *
 * PLL source is HSE = 25,000,000
 *
 * PLL_VCOx  = N32_HSE_FREQUENCY*(PLLF/16384)/(PLLR+1)
 * PLL_BWAJx = PLLF/32768 - 1
 * Subject to:
 *
 *         1 <=   PLLF  <= 67,108,863
 *         0 <=   PLLR  <= 63
 *   438 MHz <= PLL_VCO <= 800MHz
 *
 * Use SysClkMode0: M7 from PLL1A, AXI from PLL1A, M4 from PLL1A.
 *
 * Configure the M7 clock is 600MHz,
 * M4, AXI, AHB(1,2,5,6,9) clock is 300MHz, APB(1,2,5,6) clock is 150MHz.
 *
 * 0010_11 00_0100_1000_0000_0000_0000_0000
 * Target PLL_VCOx: 600 MHz     PLL1CTRL1.BWAJ[11:0] = 0x08F
 * PLLF[25:0]: 4718592          PLL1CTRL2            = 0x2C480000
 * PLLR[5:0]: 11
 * PLL_BWAJx: 143
 * Actual PLL_VCOx: 600000000 Hz
 *
 * *************!!!WARNING!!!***************
 * *                                       *
 * *!!!OVERCLOCK CAN DAMAGE YOU HARDWARE!!!*
 * *                                       *
 * *************!!!WARNING!!!***************
 *
 * 0010_11 00_0110_0000_0000_0000_0000_0000
 * Target PLL_VCOx: 800 MHz     PLL1CTRL1.BWAJ[11:0] = 0x0C7
 * PLLF[25:0]: 6291456          PLL1CTRL2            = 0x2C600000
 * PLLR[5:0]: 11
 * PLL_BWAJx: 191
 * Actual PLL_VCOx: 800000000 Hz
 */

#define TCM_SIZE_VALUE N32H7_TCM_CFG(896, 128, 0)
#define CLK_TCK 1000

#define N32_BOARD_USEHSE
/* #define N32_BOARD_USEHSI */

#ifdef N32_BOARD_USEHSI
#   define N32_BOARD_HSIDIV    RCC_SYSBUSDIV1_HSIDIV_DIV(1)
#   define PLL_PLLF            153600U
#   define PLL_PLLR            0U
#   define PLL_BWAJ            3U
#   define PLL_SRC    RCC_PLL1CTRL1_PLL1SRC_HSI
#elif defined(N32_BOARD_USEHSE)
#   define N32_BOARD_HSEDIV    RCC_SYSBUSDIV1_HSEDIV_DIV(1)
#   define PLL_PLLF            4718592U
#   define PLL_PLLR            11U
#   define PLL_BWAJ            143U
#   define PLL_SRC    RCC_PLL1CTRL1_PLL1SRC_HSE
#else
#   error "PLL configuration not supported"
#endif

#define N32_BOARD_SYSCLKMODE    0
/* #define N32_BOARD_SYSCLKMODE    1
 * #define N32_BOARD_SYSCLKMODE    2
 */

#define N32_PLL1_VCO_FREQUENCY  600000000ul
#define N32_PLL1A_FREQUENCY     N32_PLL1_VCO_FREQUENCY/1
#define N32_PLL1B_FREQUENCY     N32_PLL1_VCO_FREQUENCY/6
#if N32_BOARD_SYSCLKMODE != 0
#   define N32_PLL2_VCO_FREQUENCY  600000000ul
#   define N32_PLL2A_FREQUENCY     N32_PLL2_VCO_FREQUENCY/1
#endif

#if   N32_BOARD_SYSCLKMODE == 2
#   define N32_M7CPU_FREQUENCY     (N32_PLL2A_FREQUENCY/1)
#   define N32_M4CPU_FREQUENCY     (N32_PLL1A_FREQUENCY/2)
#   define N32_AXI_FREQUENCY       (N32_PLL1A_FREQUENCY/2)
#   define N32_AHB_FREQUENCY       (N32_PLL1A_FREQUENCY/2)
#   define N32_APB_FREQUENCY       (N32_AHB_FREQUENCY/2)
#elif N32_BOARD_SYSCLKMODE == 1
#   define N32_M7CPU_FREQUENCY     (N32_PLL2A_FREQUENCY/1)
#   define N32_M4CPU_FREQUENCY     (N32_PLL1A_FREQUENCY/2)
#   define N32_AXI_FREQUENCY       (N32_PLL2A_FREQUENCY/2)
#   define N32_AHB_FREQUENCY       (N32_PLL1A_FREQUENCY/2)
#   define N32_APB_FREQUENCY       (N32_AHB_FREQUENCY/2)
#else
#   define N32_M7CPU_FREQUENCY     (N32_PLL1A_FREQUENCY/1)
#   define N32_M4CPU_FREQUENCY     (N32_PLL1A_FREQUENCY/2)
#   define N32_AXI_FREQUENCY       (N32_PLL1A_FREQUENCY/2)
#   define N32_AHB_FREQUENCY       (N32_PLL1A_FREQUENCY/2)
#   define N32_APB_FREQUENCY       (N32_AHB_FREQUENCY/2)
#endif

/* LED definitions **********************************************************/

/* The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0 /* PA1 */
#define BOARD_LED2        1 /* PA2 */
#define BOARD_LED3        2 /* PA3 */
#define BOARD_NLEDS       3

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_BLUE    BOARD_LED2
#define BOARD_LED_RED     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

#ifdef CONFIG_ARCH_LEDS
#   define CONFIG_N32H7_GPIOA
#endif

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                        Red   Green Blue
 *   ----------------------  --------------------------  ------ ------ ---
 */

#define LED_STARTED        0 /* NuttX has been started   OFF    OFF   OFF  */
#define LED_HEAPALLOCATE   1 /* Heap has been allocated  OFF    OFF   ON   */
#define LED_IRQSENABLED    2 /* Interrupts enabled       OFF    ON    OFF  */
#define LED_STACKCREATED   3 /* Idle stack created       OFF    ON    ON   */
#define LED_INIRQ          4 /* In an interrupt          N/C    N/C   GLOW */
#define LED_SIGNAL         5 /* In a signal handler      N/C    GLOW  N/C  */
#define LED_ASSERTION      6 /* An assertion failed      GLOW   N/C   GLOW */
#define LED_PANIC          7 /* The system has crashed   Blink  OFF   N/C  */
#define LED_IDLE           8 /* MCU is is sleep mode     ON     OFF   OFF  */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The NUCLEO board supports four button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PC0.
 * A high value will be sensed when the button is depressed.
 */

#define BUTTON_WKUP        0
#define BUTTON_KEY1        1
#define BUTTON_KEY2        2
#define BUTTON_KEY3        3
#define NUM_BUTTONS        4
#define BUTTON_WKUP_BIT    (1 << BUTTON_WKUP)
#define BUTTON_KEY1_BIT    (1 << BUTTON_KEY1)
#define BUTTON_KEY2_BIT    (1 << BUTTON_KEY2)
#define BUTTON_KEY3_BIT    (1 << BUTTON_KEY3)

/* Alternate function pin selections ****************************************/

/* SDMMC GPIO pin definitions */
#define GPIO_SDMMC1_CMD    (GPIO_SDMMC1_CMD_0 | GPIO_DRIVE_12mA)  /* PD2 */
#define GPIO_SDMMC1_D0     (GPIO_SDMMC1_D0_1  | GPIO_DRIVE_12mA)  /* PC8 */
#define GPIO_SDMMC1_D1     (GPIO_SDMMC1_D1_0  | GPIO_DRIVE_12mA)  /* PC9 */
#define GPIO_SDMMC1_D2     (GPIO_SDMMC1_D2_0  | GPIO_DRIVE_12mA)  /* PC10 */
#define GPIO_SDMMC1_D3     (GPIO_SDMMC1_D3_0  | GPIO_DRIVE_12mA)  /* PC11 */
#define GPIO_SDMMC1_CK     (GPIO_SDMMC1_CK_0  | GPIO_DRIVE_12mA)  /* PC12 */

#define GPIO_SDIO_NCD      (GPIO_INPUT | GPIO_PULLUP | GPIO_SLEW_RATE_SLOW | GPIO_PORTC |GPIO_PIN13)  /* PC13 */

/* USBHS GPIO pin definitions */
#define GPIO_USBHS1_DM    (GPIO_USBHS1_DM_0)   /* PA11 */
#define GPIO_USBHS1_DP    (GPIO_USBHS1_DP_0)   /* PA12 */

#define GPIO_USBHS1_PWRON (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SLEW_RATE_SLOW | GPIO_PORTC |GPIO_PIN6) /* PC6 */

#define GPIO_USBHS2_DM    (GPIO_USBHS2_DM_0)   /* PB14 */
#define GPIO_USBHS2_DP    (GPIO_USBHS2_DP_0)   /* PB15 */

#define GPIO_USBHS2_PWRON (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SLEW_RATE_SLOW | GPIO_PORTD |GPIO_PIN12) /* PD12 */

/* USBHS configuration */
#define N32_USBHS_WRPCFG_PHYCLKSEL N32_USBHS_WRPCFG_PHYCLKSEL_25

/* ep0-8 x 2 for IN and OUT but driver internals use byte to map + one
 * bit for direction
 */
#define N32_USBHS_NENDPOINTS      (9)

/* There is 4Kb of FIFO memory in the USBHS core */
#define N32_USBHS_FIFO_SIZE       4096

/* USART1 (Serial Shield) */
#define GPIO_USART1_RX    (GPIO_USART1_RX_2) /* PA10 */
#define GPIO_USART1_TX    (GPIO_USART1_TX_2) /* PA9  */

/* USART3 (Serial Shield) */
#define GPIO_USART3_RX    (GPIO_USART3_RX_0) /* PB10 */
#define GPIO_USART3_TX    (GPIO_USART3_TX_0) /* PB11 */

/* DMA mappings */
#define DMAMAP_USART1_TX (DMAMAP_DMA1_USART1_TX)
#define DMAMAP_USART1_RX (DMAMAP_DMA1_USART1_RX)
#define DMAMAP_USART3_TX (DMAMAP_DMA1_USART3_TX)
#define DMAMAP_USART3_RX (DMAMAP_DMA1_USART3_RX)

/* ATIM1 (CH Output) */
#define PWM_ATIM1_NCHANNELS 4
#define GPIO_ATIM1_CH1OUT (GPIO_ATIM1_CH1OUT_1) /* PE9 */
#define GPIO_ATIM1_CH2OUT (GPIO_ATIM1_CH2OUT_1) /* PE11 */
#define GPIO_ATIM1_CH3OUT (GPIO_ATIM1_CH3OUT_1) /* PE13 */
#define GPIO_ATIM1_CH4OUT (GPIO_ATIM1_CH4OUT_1) /* PE14 */

/* GTIMB1 (CH Output) */
#define PWM_GTIMB1_NCHANNELS 4
#define GPIO_GTIMB1_CH1OUT (GPIO_GTIMB1_CH1POUT_0) /* PE5 */
#define GPIO_GTIMB1_CH2OUT (GPIO_GTIMB1_CH2OUT_0)  /* PE6 */
#define GPIO_GTIMB1_CH3OUT (GPIO_GTIMB1_CH3OUT_0)  /* PE0 */
#define GPIO_GTIMB1_CH4OUT (GPIO_GTIMB1_CH4OUT_0)  /* PE1 */

/* GTIMA2 (CH Input) */
#define GPIO_GTIMA2_CH1IN  (GPIO_GTIMA2_CH1IN_0) /* PA6 */
#define GPIO_GTIMA2_CH2IN  (GPIO_GTIMA2_CH1IN_0) /* PA6 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_N32H7_N32H762IIL7_INCLUDE_BOARD_H */
