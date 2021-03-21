/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/include/board.h
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

#ifndef __BOARDS_ARM_KINETIS_FREEDOM_K28F_INCLUDE_BOARD_H
#define __BOARDS_ARM_KINETIS_FREEDOM_K28F_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
# include <stdbool.h>

# include <arch/chip/kinetis_mcg.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Freedom K28F uses a 12MHz external Oscillator.
 * The Kinetis MCU startup from an internal digitally-controlled oscillator
 * (DCO). NuttX will enable the main external oscillator (EXTAL0/XTAL0).
 * The external oscillator/resonator can range from 32.768 KHz up to 50 MHz.
 * The default external source for the MCG oscillator inputs is 12 MHz
 * oscillator
 *
 * X501 a High-frequency, low-power Xtal
 */

#define BOARD_EXTAL_LP       1
#define BOARD_EXTAL_FREQ     12000000       /* 12MHz Oscillator */
#define BOARD_XTAL32_FREQ    32768          /* 32KHz RTC Oscillator */

/* PLL Configuration.
 * Either the external clock or crystal frequency is used to select the
 * PRDIV value. Only reference clock frequencies are supported that will
 * produce a KINETIS_MCG_PLL_REF_MIN >= PLLIN <=KINETIS_MCG_PLL_REF_MAX
 * reference clock to the PLL.
 *
 * PLL Input frequency:   PLLIN  = REFCLK / PRDIV = 12 MHz  / 1  = 12 MHz
 * PLL Output frequency:  PLLOUT = PLLIN  * VDIV  = 12 MHz  * 24 = 288 MHz
 * MCG Frequency:         PLLOUT = 144 MHz = 288 MHz /
 *                                           KINETIS_MCG_PLL_INTERNAL_DIVBY
 * PRDIV register value is the divider minus KINETIS_MCG_C5_PRDIV_BASE.
 * VDIV  register value is offset by KINETIS_MCG_C6_VDIV_BASE.
 */

#define BOARD_PRDIV          1        /* PLL External Reference Divider */
#define BOARD_VDIV           24       /* PLL VCO Divider (frequency multiplier) */

/* Define additional MCG_C2 Setting */

#define BOARD_MCG_C2_FCFTRIM 0              /* Do not enable FCFTRIM */
#define BOARD_MCG_C2_LOCRE0  MCG_C2_LOCRE0  /* Enable reset on loss of clock */

#define BOARD_PLLIN_FREQ     (BOARD_EXTAL_FREQ / BOARD_PRDIV)
#define BOARD_PLLOUT_FREQ    (BOARD_PLLIN_FREQ * BOARD_VDIV)
#define BOARD_MCG_FREQ       (BOARD_PLLOUT_FREQ / KINETIS_MCG_PLL_INTERNAL_DIVBY)

/* SIM CLKDIV1 dividers */

#define BOARD_OUTDIV1        1              /* Core        = MCG,    144   MHz */
#define BOARD_OUTDIV2        2              /* Bus         = MCG / 2, 72   MHz */
#define BOARD_OUTDIV3        2              /* FlexBus     = MCG / 2, 72   MHz */
#define BOARD_OUTDIV4        6              /* Flash clock = MCG / 6, 24   MHz */

#define BOARD_CORECLK_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV1)
#define BOARD_BUS_FREQ       (BOARD_MCG_FREQ / BOARD_OUTDIV2)
#define BOARD_FLEXBUS_FREQ   (BOARD_MCG_FREQ / BOARD_OUTDIV3)
#define BOARD_FLASHCLK_FREQ  (BOARD_MCG_FREQ / BOARD_OUTDIV4)

/* Use BOARD_MCG_FREQ as the output SIM_SOPT2 MUX selected by
 * SIM_SOPT2[PLLFLLSEL]
 */

#define BOARD_SOPT2_PLLFLLSEL   SIM_SOPT2_PLLFLLSEL_MCGPLLCLK
#define BOARD_SOPT2_FREQ        BOARD_MCG_FREQ

/* N.B. The above BOARD_SOPT2_FREQ precludes use of USB with a 12 MHz Xtal
 * Divider output clock = Divider input clock * ((USBFRAC+1) / (USBDIV+1))
 *     SIM_CLKDIV2_FREQ = BOARD_SOPT2_FREQ * ((USBFRAC+1) / (USBDIV+1))
 *     SIM_CLKDIV2_FREQ = BOARD_SOPT2_FREQ / (USBDIV+1)* (USBFRAC+1)
 *                48MHz = 144MHz / (2 + 1) * (1 + 0)
 */

#if (BOARD_SOPT2_FREQ == 144000000L)
#  define BOARD_SIM_CLKDIV2_USBFRAC     1
#  define BOARD_SIM_CLKDIV2_USBDIV      3
#  define BOARD_SIM_CLKDIV2_FREQ        (BOARD_SOPT2_FREQ / \
                                         BOARD_SIM_CLKDIV2_USBDIV * \
                                         BOARD_SIM_CLKDIV2_USBFRAC)
#endif

/* Divider output
 *  clock = Divider input clock * ((PLLFLLFRAC+1)/(PLLFLLDIV+1))
 *  SIM_CLKDIV3_FREQ = BOARD_SOPT2_FREQ * ((PLLFLLFRAC+1) / (PLLFLLDIV+1))
 *  SIM_CLKDIV3_FREQ = BOARD_SOPT2_FREQ / (PLLFLLDIV+1) * (PLLFLLFRAC+1)
 *                72MHz = 144MHz / (1 + 1) * (1 + 0)
 */

#define BOARD_SIM_CLKDIV3_PLLFLLFRAC  1
#define BOARD_SIM_CLKDIV3_PLLFLLDIV   2
#define BOARD_SIM_CLKDIV3_FREQ        (BOARD_SOPT2_FREQ / \
                                       BOARD_SIM_CLKDIV3_PLLFLLDIV * \
                                       BOARD_SIM_CLKDIV3_PLLFLLFRAC)

#define BOARD_LPUART0_CLKSRC SIM_SOPT2_LPUARTSRC_MCGCLK
#define BOARD_LPUART0_FREQ   BOARD_SIM_CLKDIV3_FREQ

#define BOARD_TPM_CLKSRC     SIM_SOPT2_TPMSRC_MCGCLK
#define BOARD_TPM_FREQ       BOARD_SIM_CLKDIV3_FREQ

/* SDHC clocking ************************************************************/

/* SDCLK configurations corresponding to various modes of operation.
 *   Formula is:
 *
 *   SDCLK  frequency = (base clock) / (prescaler * divisor)
 *
 * The SDHC module is always configure configured so that the core clock is
 * the baseclock.
 * Possible values for prescaler and divisor are:
 *
 *   SDCLKFS: {2, 4, 8, 16, 32, 63, 128, 256}
 *   DVS:     {1..16}
 */

/* Identification mode:
 *  Optimal 400KHz, Actual 144MHz / (32 * 12) = 375 Khz
 */

#define BOARD_SDHC_IDMODE_PRESCALER    SDHC_SYSCTL_SDCLKFS_DIV32
#define BOARD_SDHC_IDMODE_DIVISOR      SDHC_SYSCTL_DVS_DIV(12)

/* MMC normal mode:
 * Optimal 20MHz, Actual 144MHz / (2 * 4) = 18 MHz
 */

#define BOARD_SDHC_MMCMODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_MMCMODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(4)

/* SD normal mode (1-bit):
 * Optimal 20MHz, Actual 144MHz / (2 * 4) = 18 MHz
 */

#define BOARD_SDHC_SD1MODE_PRESCALER   SDHC_SYSCTL_SDCLKFS_DIV2
#define BOARD_SDHC_SD1MODE_DIVISOR     SDHC_SYSCTL_DVS_DIV(4)

/* SD normal mode (4-bit):
 * Optimal 25MHz, Actual 144MHz / (2 * 3) = 24 MHz (with DMA)
 * SD normal mode (4-bit):
 * Optimal 25MHz, Actual 144MHz / (2 * 3) = 24 MHz (no DMA)
 */

#ifdef CONFIG_SDIO_DMA
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(3)
#else
#  define BOARD_SDHC_SD4MODE_PRESCALER SDHC_SYSCTL_SDCLKFS_DIV2
#  define BOARD_SDHC_SD4MODE_DIVISOR   SDHC_SYSCTL_DVS_DIV(3)
#endif

/* Use the output of SIM_SOPT2[PLLFLLSEL] as the USB clock source */

#define BOARD_USB_CLKSRC               SIM_SOPT2_USBSRC
#define BOARD_USB_FREQ                 BOARD_SIM_CLKDIV2_FREQ

/* Allow USBOTG-FS Controller to Read from FLASH */

#define BOARD_USB_FLASHACCESS

/* PWM Configuration */

/* FTM0 Channels */

#define GPIO_FTM0_CH0OUT PIN_FTM0_CH0_2  /* Pin 22: PTC1 */
#define GPIO_FTM0_CH1OUT PIN_FTM0_CH1_2  /* Pin 23: PTC2 */
#define GPIO_FTM0_CH2OUT PIN_FTM0_CH2_2  /* Pin  9: PTC3 */
#define GPIO_FTM0_CH3OUT PIN_FTM0_CH3    /* Pin 10: PTC4 */
#define GPIO_FTM0_CH4OUT PIN_FTM0_CH4    /* Pin  6: PTD4 */
#define GPIO_FTM0_CH5OUT PIN_FTM0_CH5_2  /* Pin 20: PTD5 */
#define GPIO_FTM0_CH6OUT PIN_FTM0_CH6_2  /* Pin 21: PTD6 */
#define GPIO_FTM0_CH7OUT PIN_FTM0_CH7_2  /* Pin  5: PTD7 */

/* FTM1 Channels */

#define GPIO_FTM1_CH0OUT PIN_FTM1_CH0_1  /* Pin  3: PTA12 */
#define GPIO_FTM1_CH1OUT PIN_FTM1_CH1_1  /* Pin  4: PTA13 */

/* FTM2 Channels */

#define GPIO_FTM2_CH0OUT PIN_FTM2_CH0    /* Pin 25: PTB18 */
#define GPIO_FTM2_CH1OUT PIN_FTM2_CH1    /* Pin 32: PTB19 */

/* LED definitions **********************************************************/

/* A single LED is available driven by PTC5.  The LED is grounded so bringing
 * PTC5 high will illuminate the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED                    0
#define BOARD_NLEDS                  1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_BIT                (1 << BOARD_LED)

/* When CONFIG_ARCH_LEDS is defined in the NuttX configuration, NuttX will
 * control the LED as defined below.
 * Thus if the LED is statically on, NuttX has successfully booted and is,
 * apparently, running normally.
 * If the LED is flashing at approximately 2Hz, then a fatal error has been
 * detected and the system has halted.
 */

#define LED_STARTED                  0 /* STATUS LED=OFF */
#define LED_HEAPALLOCATE             0 /* STATUS LED=OFF */
#define LED_IRQSENABLED              0 /* STATUS LED=OFF */
#define LED_STACKCREATED             1 /* STATUS LED=ON */
#define LED_INIRQ                    2 /* STATUS LED=no change */
#define LED_SIGNAL                   2 /* STATUS LED=no change */
#define LED_ASSERTION                3 /* STATUS LED=no change */
#define LED_PANIC                    3 /* STATUS LED=flashing */

/* Button definitions *******************************************************/

/* The freedom-k28f board has no standard GPIO contact buttons */

/* Alternative pin resolution ***********************************************/

/* The Freedom K28F has five LPUARTs with pin availability as follows:
 *
 *    ----- --------------- -------------------------------
 *    GPIO  LPUART FUNCTION BOARD CONFIGURATION
 *    ----- --------------- -------------------------------
 *    PTA1  LPUART0_RX      PTA1  GPIO0
 *    PTA15 LPUART0_RX      PTA15 FXIO0_D21
 *    PTB14 LPUART0_RX      PTB14
 *    PTB16 LPUART0_RX      PTB16 SDRAM_D17
 *    PTC25 LPUART0_RX      PTC25 LPUART0_RX_TGTMCU
 *    PTD6  LPUART0_RX      PTD6  Arduino_D17_ADC0_SE7b
 *    PTA2  LPUART0_TX      PTA2  INT
 *    PTA14 LPUART0_TX      PTA14 FXIO0_D20
 *    PTB15 LPUART0_TX            N/C
 *    PTB17 LPUART0_TX      PTB17 SDRAM_D16
 *    PTC24 LPUART0_TX      PTC24 LPUART0_TX_TGTMCU
 *    PTD7  LPUART0_TX      PTD7  SDRAM_CKE
 *    PTA3  LPUART0_RTS     PTA3
 *    PTA17 LPUART0_RTS     PTA17 FXIO0_D23
 *    PTB2  LPUART0_RTS     PTB2  Arduino_D19_ADC0_SE12/I2C0_SCL/SDRAM_WE
 *    PTB12 LPUART0_RTS     PTB12 Arduino_D5_FTM1_CH0/FTM0_CH4
 *    PTC27 LPUART0_RTS     PTC27 FXOS8700CQ_RESET
 *    PTD4  LPUART0_RTS     PTD4  SDRAM_A10
 *    PTA0  LPUART0_CTS     PTA0  K28F_SWD_CLK
 *    PTA16 LPUART0_CTS     PTA16 FXIO0_D22
 *    PTB3  LPUART0_CTS     PTB3  Arduino_D18_ADC0_SE13/I2C0_SDA/SDRAM_CS0
 *    PTB13 LPUART0_CTS     PTB13 Arduino_D6_FTM1_CH1/FTM0_CH5
 *    PTC26 LPUART0_CTS     PTC26 FXOS8700CQ_INT
 *    PTD5  LPUART0_CTS     PTD5  SDRAM_A9
 *    ----- --------------- -------------------------------
 *    PTD8  LPUART1_RX      PTD8  FXIO0_D24
 *    PTC3  LPUART1_RX      PTC3  CLKOUT
 *    PTE1  LPUART1_RX      PTE1  QSPIA0_SCLK
 *    PTC4  LPUART1_TX      PTC4  SDRAM_A19
 *    PTD9  LPUART1_TX      PTD9  FXIO0_D25
 *    PTE0  LPUART1_TX      PTE0  QSPIA0_DATA3
 *    PTD10 LPUART1_RTS     PTD10 FXIO0_D26
 *    PTC1  LPUART1_RTS     PTC1  SDRAM_A21
 *    PTE3  LPUART1_RTS     PTE3  QSPIA0_DATA2
 *    PTC2  LPUART1_CTS     PTC1  SDRAM_A21
 *    PTD11 LPUART1_CTS     PTD11 FXIO0_D27
 *    PTE2  LPUART1_CTS     PTE2  QSPIA0_DATA0
 *    ----- --------------- -------------------------------
 *    PTA25 LPUART2_RX      PTA25 SDHC0_D0/Arduino_D0_LPUART2_RX
 *    PTD2  LPUART2_RX      PTD2  SDRAM_A12
 *    PTE13 LPUART2_RX            N/C
 *    PTE17 LPUART2_RX            N/C
 *    PTA24 LPUART2_TX      PTA24 SDHC0_D1/Arduino_D1_LPUART2_TX
 *    PTD3  LPUART2_TX      PTD3  SDRAM_A11
 *    PTE12 LPUART2_TX      PTE12 I2S0_TX_BCLK
 *    PTE16 LPUART2_TX            N/C
 *    PTD0  LPUART2_RTS     PTD0  Button_LLWU_P12
 *    PTA27 LPUART2_RTS     PTA27 SDHC0_CMD
 *    PTE19 LPUART2_RTS           N/C
 *    PTA26 LPUART2_CTS     PTA26 SDHC0_DCLK
 *    PTD1  LPUART2_CTS     PTD1  Arduino_D16_ADC0_SE5b
 *    PTE18 LPUART2_CTS           N/C
 *    ----- --------------- -------------------------------
 *    PTA29 LPUART3_RX      PTA29 SDHC0_D2
 *    PTB10 LPUART3_RX      PTB10 SDRAM_D19
 *    PTC16 LPUART3_RX      PTC16 SDRAM_DQM2
 *    PTE5  LPUART3_RX      PTE5  QSPIA0_SS0/USB0_SOF_OUT
 *    PTA28 LPUART3_TX      PTA28 SDHC0_D3
 *    PTB11 LPUART3_TX      PTB11 SDRAM_D18
 *    PTC17 LPUART3_TX      PTC17 SDRAM_DQM3
 *    PTE4  LPUART3_TX      PTE4  QSPIA0_DATA1
 *    PTB8  LPUART3_RTS     PTB8  SDRAM_D21
 *    PTA31 LPUART3_RTS     PTA31
 *    PTC18 LPUART3_RTS     PTC18 Arduino_D7
 *    PTE7  LPUART3_RTS     PTE7  I2S0_RXD0/LEDRGB_GREEN
 *    PTA30 LPUART3_CTS     PTA30
 *    PTB9  LPUART3_CTS     PTB9  SDRAM_D20
 *    PTC19 LPUART3_CTS     PTC19 Arduino_D8
 *    PTE6  LPUART3_CTS     PTE6  I2S0_MCK/LEDRGB_RED
 *    ----- --------------- -------------------------------
 *    PTA21 LPUART4_RX      PTA21 TE/FXIO0_D9
 *    PTC14 LPUART4_RX      PTC14 SDRAM_D25
 *    PTE21 LPUART4_RX            N/C
 *    PTA20 LPUART4_TX      PTA20 RD/FXIO0_D8
 *    PTC15 LPUART4_TX      PTC15 SDRAM_D24
 *    PTE20 LPUART4_TX            N/C
 *    PTA23 LPUART4_RTS     PTA23 WR/FXIO0_D7
 *    PTC12 LPUART4_RTS     PTC12 SDRAM_D27
 *    PTE23 LPUART4_RTS           N/C
 *    PTA22 LPUART4_CTS     PTA22 CS/FXIO0_D6
 *    PTC13 LPUART4_CTS     PTC13 SDRAM_D26
 *    PTE22 LPUART4_CTS           N/C
 *    ----- --------------- -------------------------------
 *
 * Virtual serial port
 * -------------------
 *
 * A serial port connection is available between the OpenSDA v2.2 MCU and
 * pins PTC24 and PTC25 of the K28 MCU:
 *
 *    ----- --------------- -------------------------------
 *    GPIO  LPUART FUNCTION BOARD CONFIGURATION
 *    ----- --------------- -------------------------------
 *    PTC25 LPUART0_RX      PTC25 LPUART0_RX_TGTMCU
 *    PTC24 LPUART0_TX      PTC24 LPUART0_TX_TGTMCU
 *    ----- --------------- -------------------------------
 */

#define PIN_LPUART0_RX      PIN_LPUART0_RX_5  /* PTC25 */
#define PIN_LPUART0_TX      PIN_LPUART0_TX_5  /* PTC24 */

/*  Arduino RS-232 Shield
 *  ---------------------
 *
 *    ----- --------------- -------------------------------
 *    GPIO  LPUART FUNCTION BOARD CONFIGURATION
 *    ----- --------------- -------------------------------
 *    PTA25 LPUART2_RX      PTA25 SDHC0_D0/Arduino_D0_LPUART2_RX
 *    PTA24 LPUART2_TX      PTA24 SDHC0_D1/Arduino_D1_LPUART2_TX
 *    ----- --------------- -------------------------------
 *
 *  Note:  PTA24 and PTA25 are shared between Micro SD Card circuit and
 *  Arduino connectors. Remove R106 and R107 or R94 and R11 as necessary to
 *  prevent contention.
 */

#define PIN_LPUART2_RX      PIN_LPUART2_RX_1  /* PTA25 */
#define PIN_LPUART2_TX      PIN_LPUART2_TX_1  /* PTA24 */

/* I2C */

#ifdef CONFIG_KINETIS_I2C0
#ifdef CONFIG_FREEDOM_K28F_I2C_ALT_PINS
#  define PIN_I2C0_SCL      (PIN_I2C0_SCL_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C0_SDA      (PIN_I2C0_SDA_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#else
#  define PIN_I2C0_SCL      (PIN_I2C0_SCL_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C0_SDA      (PIN_I2C0_SDA_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#endif
#endif

/* REVISIT: Added only for clean compilation with I2C1 enabled. */

#ifdef CONFIG_KINETIS_I2C1
#ifdef CONFIG_FREEDOM_K28F_I2C_ALT_PINS
#  define PIN_I2C1_SCL      (PIN_I2C1_SCL_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C1_SDA      (PIN_I2C1_SDA_1 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#else
#  define PIN_I2C1_SCL      (PIN_I2C1_SCL_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#  define PIN_I2C1_SDA      (PIN_I2C1_SDA_2 | PIN_ALT2_OPENDRAIN | PIN_ALT2_SLOW)
#endif
#endif

/* SDHC */

#ifdef CONFIG_KINETIS_SDHC
#  define PIN_SDHC0_CMD     PIN_SDHC0_CMD_1
#  define PIN_SDHC0_D0      PIN_SDHC0_D0_1
#  define PIN_SDHC0_D1      PIN_SDHC0_D1_1
#  define PIN_SDHC0_D2      PIN_SDHC0_D2_1
#  define PIN_SDHC0_D3      PIN_SDHC0_D3_1
#  define PIN_SDHC0_DCLK    PIN_SDHC0_DCLK_1
#endif

/* LED definitions **********************************************************/

/* The Freedom K28F has a single RGB LED driven by the K28F as follows:
 *
 *   LED    K28
 *   ------ -------------------------------------------------------
 *   RED    PTE6
 *   BLUE   PTE7
 *   GREEN  PTE8
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Freedom K28F.  The following definitions describe how NuttX controls
 * the LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  -----------------
 */

#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* K28 is in sleep mode     (Not used)        */

/* Button definitions *******************************************************/

/* Two push buttons, SW2 and SW3, are available on FRDM-K28F board,
 * where SW2 is connected to PTA4 and SW3 is connected to PTD0.
 * Besides the general purpose input/output functions, SW2 and SW3 can be
 * low-power wake up signal. Also, only SW3 can be a non-maskable interrupt.
 *
 *   Switch    GPIO Function
 *   --------- --------------------------------------------------------------
 *   SW2       PTA4/NMI_B
 *   SW3       PTA4/NMI_B
 */

#define BUTTON_SW2        0
#define BUTTON_SW3        1
#define NUM_BUTTONS       2

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

#endif /* __BOARDS_ARM_KINETIS_FREEDOM_K28F_INCLUDE_BOARD_H */
