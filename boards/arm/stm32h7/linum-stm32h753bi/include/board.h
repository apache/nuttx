/****************************************************************************
 * boards/arm/stm32h7/linum-stm32h753bi/include/board.h
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

#ifndef __BOARDS_ARM_STM32H7_LINUM_STM32H753BI_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_LINUM_STM32H753BI_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The LINUM-STM32H753BI board provides the following clock sources:
 *
 *   MCO: 8 MHz from MCO output of ST-LINK is used as input clock (default)
 *   X2:  32.768 KHz crystal for LSE
 *   X3:  HSE crystal oscillator (not provided)
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 25 MHz oscillator X2
 *   LSE: 32.768 kHz
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     25000000ul
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 25,000,000
 *
 * When STM32_HSE_FREQUENCY / PLLM <= 2MHz VCOL must be selected.
 * VCOH otherwise.
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 400 MHz
 */

#define STM32_BOARD_USEHSE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1 - 25 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (25 MHz / 5) * 192 = 960 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 800 MHz / 2   = 480 MHz
 *   PLL1Q = PLL1_VCO/4  = 800 MHz / 4   = 240 MHz
 *   PLL1R = PLL1_VCO/4  = 800 MHz / 4   = 240 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE| \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ| \
                                  RCC_PLLCFGR_DIVP1EN| \
                                  RCC_PLLCFGR_DIVQ1EN| \
                                  RCC_PLLCFGR_DIVR1EN)

#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(5)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(192)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(4)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 5) * 192)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)

/* PLL2 - 25 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL2_VCO = (25 MHz / 2) * 48 = 600 MHz
 *
 *   PLL2P = PLL2_VCO/8  = 600 MHz / 8   = 75 MHz
 *   PLL2Q = PLL2_VCO/40 = 600 MHz / 40  = 15 MHz
 *   PLL2R = PLL2_VCO/3  = 600 MHz / 3   = 200 MHz
 */
#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL2VCOSEL_WIDE| \
                              RCC_PLLCFGR_PLL2RGE_4_8_MHZ| \
                              RCC_PLLCFGR_DIVP2EN| \
                              RCC_PLLCFGR_DIVQ2EN| \
                              RCC_PLLCFGR_DIVR2EN)

#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(48)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(8)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(40)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(3)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 48)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 8)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 40)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 3)

/* PLL3 - 25 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL3_VCO = (25 MHz / 1) * 35 = 875 MHz
 *
 *   PLL3P = PLL3_VCO/2  = 875 MHz / 2   = 437.7 MHz
 *   PLL3Q = PLL3_VCO/1  = 875 MHz / 1  = 875 MHz
 *   PLL3R = PLL3_VCO/20  = 875 MHz / 20   = 43.75 MHz
 */
#define STM32_PLLCFG_PLL3CFG (RCC_PLLCFGR_PLL3VCOSEL_WIDE| \
                              RCC_PLLCFGR_PLL3RGE_8_16_MHZ| \
                              RCC_PLLCFGR_DIVP3EN| \
                              RCC_PLLCFGR_DIVQ3EN| \
                              RCC_PLLCFGR_DIVR3EN)

#define STM32_PLLCFG_PLL3M       RCC_PLLCKSELR_DIVM3(1)
#define STM32_PLLCFG_PLL3N       RCC_PLL3DIVR_N3(35)
#define STM32_PLLCFG_PLL3P       RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q       RCC_PLL3DIVR_Q3(1)
#define STM32_PLLCFG_PLL3R       RCC_PLL3DIVR_R3(20)

#define STM32_VCO3_FREQUENCY     ((STM32_HSE_FREQUENCY / 1) * 35)
#define STM32_PLL3P_FREQUENCY    (STM32_VCO3_FREQUENCY / 2)
#define STM32_PLL3Q_FREQUENCY    (STM32_VCO3_FREQUENCY / 1)
#define STM32_PLL3R_FREQUENCY    (STM32_VCO3_FREQUENCY / 20)

/* SYSCLK = PLL1P = 480 MHz
 * CPUCLK = SYSCLK / 1 = 480 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (240 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */

/* APB1 clock (PCLK1) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (120 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (120 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2      /* PCLK3 = HCLK / 2 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/2 (120 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2      /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source - HSI */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL1Q */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL1

/* SPI45 clock source - APB (PCLK2?) */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_PLL2

/* SPI6 clock source - APB (PCLK4) */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PCLK4

/* USB 1 and 2 clock source - HSI48 */

#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48

/* ADC 1 2 3 clock source - pll2_pclk */

#define STM32_RCC_D3CCIPR_ADCSRC     RCC_D3CCIPR_ADCSEL_PLL2

/* FDCAN 1 2 clock source, use STM32_HSE_FREQUENCY  */

#define STM32_RCC_D2CCIP1R_FDCANSEL  RCC_D2CCIP1R_FDCANSEL_HSE

/* SDMMC 1 2 clock source, use STM32_PLL1Q_FREQUENCY  */

#define STM32_RCC_D1CCIPR_SDMMCSEL  RCC_D1CCIPR_SDMMC_PLL1

/* FMC clock source, use STM32_PLL1Q_FREQUENCY  */
#define BOARD_FMC_CLK               RCC_D1CCIPR_FMCSEL_HCLK

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 4

/* LED definitions **********************************************************/

/* The board has 1 user LED RGB that could be used this diagnostic LED too.
 * LED RED    PG2
 * LED GREEN  PG3
 * LED BLUE   PB2
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_NLEDS       3

#define BOARD_LED_RED      BOARD_LED1
#define BOARD_LED_GREEN    BOARD_LED2
#define BOARD_LED_BLUE     BOARD_LED3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)

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

/* The LINUM-STM32H753BI board does have user buttons */

/* Alternate function pin selections ****************************************/

/* USART1 (Serial Console) */

#define GPIO_USART1_RX   (GPIO_USART1_RX_1|GPIO_SPEED_100MHz)  /* PB15 */
#define GPIO_USART1_TX   (GPIO_USART1_TX_1|GPIO_SPEED_100MHz)  /* PB14 */

/* UART4 */

#ifdef CONFIG_UART4_RS485
  /* Lets use for RS485 */

#  define GPIO_UART4_TX        (GPIO_UART4_TX_3|GPIO_SPEED_100MHz) /* PB9 */
#  define GPIO_UART4_RX        (GPIO_UART4_RX_3|GPIO_SPEED_100MHz) /* PB8 */

  /* RS485 DIR pin: PA15 */

#  define GPIO_UART4_RS485_DIR (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_100MHz |\
                                GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN15)

#endif

/* USART6 */

#ifdef CONFIG_USART6_RS485
  /* Lets use for RS485 */

#  define GPIO_USART6_TX        (GPIO_USART6_TX_1|GPIO_SPEED_100MHz) /* PC6 */
#  define GPIO_USART6_RX        (GPIO_USART6_RX_1|GPIO_SPEED_100MHz) /* PC7 */

  /* RS485 DIR pin: PG12 */

#  define GPIO_USART6_RS485_DIR (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_100MHz|\
                                 GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN12)

#endif

/* OTGFS */

#define GPIO_OTGFS_DM  (GPIO_OTGFS_DM_0|GPIO_SPEED_100MHz) /* PA11 */
#define GPIO_OTGFS_DP  (GPIO_OTGFS_DP_0|GPIO_SPEED_100MHz) /* PA12 */

/* SDMMC1 - Used SD Card memory */

/* Init 400 kHz, PLL1Q/(2*300) = 240 MHz / (2*300) = 400 Khz */

#define STM32_SDMMC_INIT_CLKDIV     (300 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* Just set these to 24 MHz for now,
 * PLL1Q/(2*5) = 240 MHz / (2*5) = 24 MHz
 */

#define STM32_SDMMC_MMCXFR_CLKDIV   (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

#define GPIO_SDMMC1_CK   (GPIO_SDMMC1_CK_0|GPIO_SPEED_100MHz)  /* PC12 */
#define GPIO_SDMMC1_CMD  (GPIO_SDMMC1_CMD_0|GPIO_SPEED_100MHz) /* PD2 */
#define GPIO_SDMMC1_D0   (GPIO_SDMMC1_D0_0|GPIO_SPEED_100MHz)  /* PC8 */
#define GPIO_SDMMC1_D1   (GPIO_SDMMC1_D1_0|GPIO_SPEED_100MHz)  /* PC9 */
#define GPIO_SDMMC1_D2   (GPIO_SDMMC1_D2_0|GPIO_SPEED_100MHz)  /* PC10 */
#define GPIO_SDMMC1_D3   (GPIO_SDMMC1_D3_0|GPIO_SPEED_100MHz)  /* PC11 */

/* I2C3 - Used by eeprom memory and touch screen */

#define GPIO_I2C3_SCL (GPIO_I2C3_SCL_2|GPIO_SPEED_100MHz) /* PH7 */
#define GPIO_I2C3_SDA (GPIO_I2C3_SDA_2|GPIO_SPEED_100MHz) /* PH8 */

/* I2C - There is a FT5336 TouchPanel on I2C3 using these pins: */

#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_100MHz))
#define GPIO_TP_INT  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTH|GPIO_PIN9) /* PH9 */

#define FT5X06_I2C_ADDRESS          (0x38)

/* PWM - Buzzer */
#define GPIO_TIM4_CH2OUT  (GPIO_TIM4_CH2OUT_1|GPIO_SPEED_100MHz)   /* PB7 */

/* FDCAN1 */

#define GPIO_CAN1_RX  (GPIO_CAN1_RX_4|GPIO_SPEED_50MHz) /* PH14 */
#define GPIO_CAN1_TX  (GPIO_CAN1_TX_4|GPIO_SPEED_50MHz) /* PH13 */
#define GPIO_CAN1_STD (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_100MHz |\
                       GPIO_OUTPUT_CLEAR | GPIO_PORTI | GPIO_PIN2) /* PI2 */

/* FDCAN2 */

#define GPIO_CAN2_RX  (GPIO_CAN2_RX_1|GPIO_SPEED_50MHz) /* PB12 */
#define GPIO_CAN2_TX  (GPIO_CAN2_TX_1|GPIO_SPEED_50MHz) /* PB13 */
#define GPIO_CAN2_STD (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_100MHz |\
                       GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN3) /* PE3 */

/* QSPI Mapping  */

#define GPIO_QSPI_CS  (GPIO_QUADSPI_BK1_NCS_2|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz) /* PG6 */
#define GPIO_QSPI_IO0 (GPIO_QUADSPI_BK1_IO0_1|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz) /* PF8 */
#define GPIO_QSPI_IO1 (GPIO_QUADSPI_BK1_IO1_1|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz) /* PF9 */
#define GPIO_QSPI_IO2 (GPIO_QUADSPI_BK1_IO2_2|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz) /* PF7 */
#define GPIO_QSPI_IO3 (GPIO_QUADSPI_BK1_IO3_3|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz) /* PF6 */
#define GPIO_QSPI_SCK (GPIO_QUADSPI_CLK_2|GPIO_FLOAT|GPIO_PUSHPULL|GPIO_SPEED_100MHz)     /* PF10 */

/* SPI4 */
#define GPIO_SPI4_SCK  (GPIO_SPI4_SCK_2)   /* PE2 */
#define GPIO_SPI4_MISO (GPIO_SPI4_MISO_2)  /* PE5 */
#define GPIO_SPI4_MOSI (GPIO_SPI4_MOSI_2)  /* PE6 */

/* MFRC522 */
#define GPIO_RFID_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                        GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)          /* PE4 */

/* Select PLL2R to source clock of QSPI */
#define BOARD_QSPI_CLK  RCC_D1CCIPR_QSPISEL_PLL2

/* Ethernet */

#define BOARD_CFGR_MC01_SOURCE   (RCC_CFGR_MCO1_HSE)
#define BOARD_CFGR_MC01_DIVIDER  (RCC_CFGR_MCO1PRE(0))
#define GPIO_MCO1                (GPIO_MCO1_0)                            /* PA8 */

#define GPIO_ETH_MDC          (GPIO_ETH_MDC_0|GPIO_SPEED_100MHz)          /* PC1 */
#define GPIO_ETH_MDIO         (GPIO_ETH_MDIO_0|GPIO_SPEED_100MHz)         /* PA2 */
#define GPIO_ETH_RMII_CRS_DV  (GPIO_ETH_RMII_CRS_DV_0|GPIO_SPEED_100MHz)  /* PA7 */
#define GPIO_ETH_RMII_REF_CLK (GPIO_ETH_RMII_REF_CLK_0|GPIO_SPEED_100MHz) /* PA1 */
#define GPIO_ETH_RMII_RXD0    (GPIO_ETH_RMII_RXD0_0|GPIO_SPEED_100MHz)    /* PC4 */
#define GPIO_ETH_RMII_RXD1    (GPIO_ETH_RMII_RXD1_0|GPIO_SPEED_100MHz)    /* PC5 */
#define GPIO_ETH_RMII_TXD0    (GPIO_ETH_RMII_TXD0_2|GPIO_SPEED_100MHz)    /* PG13 */
#define GPIO_ETH_RMII_TXD1    (GPIO_ETH_RMII_TXD1_3|GPIO_SPEED_100MHz)    /* PG14 */
#define GPIO_ETH_RMII_TX_EN   (GPIO_ETH_RMII_TX_EN_2|GPIO_SPEED_100MHz)   /* PG11 */

/* SDRAM FMC definitions ****************************************************/

/* The following settings correspond to IS42S16400J SDRAM
 * part-number and FMC_SDCLK frequency of 120 MHz
 */

/* Though IS42S16400J SDRAM itself provides 16-bit data bus,
 * linum board routes only DQ[15:0] bits.
 */

#if CONFIG_STM32H7_FMC
#  define FMC_SDCLK_FREQUENCY  (STM32_HCLK_FREQUENCY / 2)
#  if FMC_SDCLK_FREQUENCY > 120000000
#    error "FMC SDRAM settings need to be adjusted for a higher FMC_SDCLK frequency"
#  elif FMC_SDCLK_FREQUENCY < 120000000
#    warning "The current FMC SDRAM settings may not be optimal for a lower FMC_SDCLK frequency"
#  endif
#endif

#define BOARD_SDRAM1_SIZE               (8*1024*1024)

/* BOARD_FMC_SDCR1 - Initial value for SDRAM control registers for SDRAM
 *      bank 1. Note bank 2 isn't used!
 */

#define BOARD_FMC_SDCR1  (FMC_SDCR_COLBITS_8|   /* numcols = 8 bits */ \
                          FMC_SDCR_ROWBITS_12|  /* numrows = 12 bits */ \
                          FMC_SDCR_WIDTH_16|    /* width = 16 bits */ \
                          FMC_SDCR_SDCLK_2X|    /* sdclk = 2 hclk */ \
                          FMC_SDCR_BANKS_4|     /* 4 internal banks */ \
                          FMC_SDCR_BURST_READ|  /* enable burst read */ \
                          FMC_SDCR_RPIPE_0|     /* rpipe = 0 hclk */ \
                          FMC_SDCR_CASLAT_3)    /* cas latency = 3 cycles */

/* BOARD_FMC_SDTR1 - Initial value for SDRAM timing registers for SDRAM
 *      bank 1.
 *
 * FMC_SDTR_TMRD - Load mode register to active delay
 * FMC_SDTR_TXSR - Exit self-refresh delay
 * FMC_SDTR_TRAS - Self-refresh time
 * FMC_SDTR_TRCD - SDRAM common row cycle delay
 * FMC_SDTR_TWR  - Write recovery time
 * FMC_SDTR_TRP  - SDRAM common row percharge delay
 * FMC_SDTR_TRC  - Row to collumn delay
 */

#define BOARD_FMC_SDTR1  (FMC_SDTR_TMRD(2)| /* tMRD     = 2CLK */ \
                          FMC_SDTR_TXSR(9)| /* tXSR min = ns */ \
                          FMC_SDTR_TRCD(8)| /* tRCD min = ns */ \
                          FMC_SDTR_TRAS(6)| /* tRAS min = ns */ \
                          FMC_SDTR_TWR(4)|  /* tWR      = ns */ \
                          FMC_SDTR_TRP(2)|  /* tRP  min = ns */ \
                          FMC_SDTR_TRC(8))  /* tRC  min = ns */

#define BOARD_FMC_SDRAM_REFR_CYCLES  4096
#define BOARD_FMC_SDRAM_REFR_PERIOD  64
#define BOARD_FMC_SDRAM_AUTOREFRESH  8
#define BOARD_FMC_SDRAM_MODE         (FMC_SDCMR_MRD_BURST_LENGTH_1| \
                                      FMC_SDCMR_MRD_BURST_TYPE_SEQUENTIAL| \
                                      FMC_SDCMR_MRD_CAS_LATENCY_3| \
                                      FMC_SDCMR_MRD_WRITEBURST_MODE_SINGLE)

#define BOARD_FMC_GPIO_CONFIGS \
  (GPIO_FMC_A0_0|GPIO_SPEED_100MHz),     /* PF0 */ \
  (GPIO_FMC_A1_0|GPIO_SPEED_100MHz),     /* PF1 */ \
  (GPIO_FMC_A2_0|GPIO_SPEED_100MHz),     /* PF2 */ \
  (GPIO_FMC_A3_0|GPIO_SPEED_100MHz),     /* PF3 */ \
  (GPIO_FMC_A4_0|GPIO_SPEED_100MHz),     /* PF4 */ \
  (GPIO_FMC_A5_0|GPIO_SPEED_100MHz),     /* PF5 */ \
  (GPIO_FMC_A6_0|GPIO_SPEED_100MHz),     /* PF12 */ \
  (GPIO_FMC_A7_0|GPIO_SPEED_100MHz),     /* PF13 */ \
  (GPIO_FMC_A8_0|GPIO_SPEED_100MHz),     /* PF14 */ \
  (GPIO_FMC_A9_0|GPIO_SPEED_100MHz),     /* PF15 */ \
  (GPIO_FMC_A10_0|GPIO_SPEED_100MHz),    /* PG0 */ \
  (GPIO_FMC_A11_0|GPIO_SPEED_100MHz),    /* PG1 */ \
  (GPIO_FMC_D0_0|GPIO_SPEED_100MHz),     /* PD14 */ \
  (GPIO_FMC_D1_0|GPIO_SPEED_100MHz),     /* PD15 */ \
  (GPIO_FMC_D2_0|GPIO_SPEED_100MHz),     /* PD0 */ \
  (GPIO_FMC_D3_0|GPIO_SPEED_100MHz),     /* PD1 */ \
  (GPIO_FMC_D4_0|GPIO_SPEED_100MHz),     /* PE7 */ \
  (GPIO_FMC_D5_0|GPIO_SPEED_100MHz),     /* PE8 */ \
  (GPIO_FMC_D6_0|GPIO_SPEED_100MHz),     /* PE9 */ \
  (GPIO_FMC_D7_0|GPIO_SPEED_100MHz),     /* PE10 */ \
  (GPIO_FMC_D8_0|GPIO_SPEED_100MHz),     /* PE11 */ \
  (GPIO_FMC_D9_0|GPIO_SPEED_100MHz),     /* PE12 */ \
  (GPIO_FMC_D10_0|GPIO_SPEED_100MHz),    /* PE13 */ \
  (GPIO_FMC_D11_0|GPIO_SPEED_100MHz),    /* PE14 */ \
  (GPIO_FMC_D12_0|GPIO_SPEED_100MHz),    /* PE15 */ \
  (GPIO_FMC_D13_0|GPIO_SPEED_100MHz),    /* PD8 */ \
  (GPIO_FMC_D14_0|GPIO_SPEED_100MHz),    /* PD9 */ \
  (GPIO_FMC_D15_0|GPIO_SPEED_100MHz),    /* PD10 */ \
  (GPIO_FMC_NBL0_0|GPIO_SPEED_100MHz),   /* PE0 */ \
  (GPIO_FMC_NBL1_0|GPIO_SPEED_100MHz),   /* PE1 */ \
  (GPIO_FMC_BA0_0|GPIO_SPEED_100MHz),    /* PG4 */ \
  (GPIO_FMC_BA1_0|GPIO_SPEED_100MHz),    /* PG5 */ \
  (GPIO_FMC_SDNCAS_0|GPIO_SPEED_100MHz), /* PG15 */ \
  (GPIO_FMC_SDNRAS_0|GPIO_SPEED_100MHz), /* PF11 */ \
  (GPIO_FMC_SDNWE_2|GPIO_SPEED_100MHz),  /* PC0 */ \
  (GPIO_FMC_SDNE0_1|GPIO_SPEED_100MHz),  /* PC2 */ \
  (GPIO_FMC_SDCKE0_1|GPIO_SPEED_100MHz), /* PC3 */ \
  (GPIO_FMC_SDCLK_0|GPIO_SPEED_100MHz)   /* PG8 */

/* QEncoder - TIM5: CH1 and CH2 */

#define GPIO_TIM5_CH1IN  GPIO_TIM5_CH1IN_1 /* PA0 */
#define GPIO_TIM5_CH2IN  GPIO_TIM5_CH2IN_2 /* PH11 */

/* LTDC */

#define GPIO_LTDC_R0     (GPIO_LTDC_R0_3|GPIO_SPEED_100MHz) /* PI15 */
#define GPIO_LTDC_R1     (GPIO_LTDC_R1_3|GPIO_SPEED_100MHz) /* PJ0 */
#define GPIO_LTDC_R2     (GPIO_LTDC_R2_4|GPIO_SPEED_100MHz) /* PJ1 */
#define GPIO_LTDC_R3     (GPIO_LTDC_R3_2|GPIO_SPEED_100MHz) /* PJ2 */
#define GPIO_LTDC_R4     (GPIO_LTDC_R4_4|GPIO_SPEED_100MHz) /* PJ3 */
#define GPIO_LTDC_R5     (GPIO_LTDC_R5_5|GPIO_SPEED_100MHz) /* PJ4 */
#define GPIO_LTDC_R6     (GPIO_LTDC_R6_3|GPIO_SPEED_100MHz) /* PJ5 */
#define GPIO_LTDC_R7     (GPIO_LTDC_R7_3|GPIO_SPEED_100MHz) /* PJ6 */

#define GPIO_LTDC_G0     (GPIO_LTDC_G0_3|GPIO_SPEED_100MHz) /* PJ7 */
#define GPIO_LTDC_G1     (GPIO_LTDC_G1_3|GPIO_SPEED_100MHz) /* PJ8 */
#define GPIO_LTDC_G2     (GPIO_LTDC_G2_3|GPIO_SPEED_100MHz) /* PJ9 */
#define GPIO_LTDC_G3     (GPIO_LTDC_G3_4|GPIO_SPEED_100MHz) /* PJ10 */
#define GPIO_LTDC_G4     (GPIO_LTDC_G4_4|GPIO_SPEED_100MHz) /* PJ11 */
#define GPIO_LTDC_G5     (GPIO_LTDC_G5_3|GPIO_SPEED_100MHz) /* PK0 */
#define GPIO_LTDC_G6     (GPIO_LTDC_G6_3|GPIO_SPEED_100MHz) /* PK1 */
#define GPIO_LTDC_G7     (GPIO_LTDC_G7_4|GPIO_SPEED_100MHz) /* PK2 */

#define GPIO_LTDC_B0     (GPIO_LTDC_B0_3|GPIO_SPEED_100MHz) /* PJ12 */
#define GPIO_LTDC_B1     (GPIO_LTDC_B1_3|GPIO_SPEED_100MHz) /* PJ13 */
#define GPIO_LTDC_B2     (GPIO_LTDC_B2_4|GPIO_SPEED_100MHz) /* PJ14 */
#define GPIO_LTDC_B3     (GPIO_LTDC_B3_4|GPIO_SPEED_100MHz) /* PJ15 */
#define GPIO_LTDC_B4     (GPIO_LTDC_B4_4|GPIO_SPEED_100MHz) /* PK3 */
#define GPIO_LTDC_B5     (GPIO_LTDC_B5_3|GPIO_SPEED_100MHz) /* PK4 */
#define GPIO_LTDC_B6     (GPIO_LTDC_B6_3|GPIO_SPEED_100MHz) /* PK5 */
#define GPIO_LTDC_B7     (GPIO_LTDC_B7_3|GPIO_SPEED_100MHz) /* PK6 */

#define GPIO_LTDC_DE     (GPIO_LTDC_DE_3|GPIO_SPEED_100MHz)    /* PK7 */
#define GPIO_LTDC_CLK    (GPIO_LTDC_CLK_3|GPIO_SPEED_100MHz)   /* PI14 */
#define GPIO_LTDC_VSYNC  (GPIO_LTDC_VSYNC_3|GPIO_SPEED_100MHz) /* PI9 */
#define GPIO_LTDC_HSYNC  (GPIO_LTDC_HSYNC_2|GPIO_SPEED_100MHz) /* PI10 */

/* LCD definitions **********************************************************/

#define BOARD_LTDC_WIDTH                1024
#define BOARD_LTDC_HEIGHT               600

#define BOARD_LTDC_OUTPUT_BPP           16
#define BOARD_LTDC_HFP                  160
#define BOARD_LTDC_HBP                  160 /* AccumulatedHBP */
#define BOARD_LTDC_VFP                  1
#define BOARD_LTDC_VBP                  23  /* AccumulatedVBP */

#define BOARD_LTDC_HSYNC                1   /* HorizontalSync */
#define BOARD_LTDC_VSYNC                1   /* VerticalSync */

/* Pixel Clock Polarity */

#define BOARD_LTDC_GCR_PCPOL            0 /* !LTDC_GCR_PCPOL */

/* Data Enable Polarity */

#define BOARD_LTDC_GCR_DEPOL            0 /* !LTDC_GCR_DEPOL */

/* Vertical Sync Polarity */

#define BOARD_LTDC_GCR_VSPOL            0 /* !LTDC_GCR_VSPOL */

/* Horizontal Sync Polarity */

#define BOARD_LTDC_GCR_HSPOL            0 /* !LTDC_GCR_HSPOL */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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
#endif /* __BOARDS_ARM_STM32H7_LINUM_STM32H753BI_INCLUDE_BOARD_H */
