/************************************************************************************
 * configs/lpcxpresso-lpc1115/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_LPCXPRESSO_LPC1115_INCLUDE_BOARD_H
#define __CONFIG_LPCXPRESSO_LPC1115_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* NOTE:  The following definitions require lpc11_syscon.h.  It is not included here
 * because the including C file may not have that file in its include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (1200000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20, pre-divider=1
 *   MCLK = 480MHz / 6 = 80MHz               -> MCLK divider = 6
 */

#define LPC11_MCLK                 48000000 /* 48Mhz */

/* Select the internal RC oscillator as the frequency source.  SYSCLK is then the frequency
 * of the main oscillator.
 */

#undef CONFIG_LPC11_INTRCOSC
#define CONFIG_LPC11_INTRCOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKCFG_DIVIDER      6
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.  PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC11_PLL0
#define CONFIG_LPC11_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC11_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.  This divider is used when PLL1 is not enabled to get the
 * USB clock from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LPC11_FLASH
#define CONFIG_LPC11_FLASH         1
#define BOARD_FLASHCFG_VALUE       0x0000303a

/* LED definitions ******************************************************************/
/* The LPCXpresso LPC1115 board has a single red LED (there are additional LEDs on
 * the base board not considered here).
 */
                                     /* ON      OFF                 */
#define LED_STARTED                0 /* OFF     ON  (never happens) */
#define LED_HEAPALLOCATE           0 /* OFF     ON  (never happens) */
#define LED_IRQSENABLED            0 /* OFF     ON  (never happens) */
#define LED_STACKCREATED           1 /* ON      ON  (never happens) */
#define LED_INIRQ                  2 /* OFF     NC  (momentary) */
#define LED_SIGNAL                 2 /* OFF     NC  (momentary) */
#define LED_ASSERTION              2 /* OFF     NC  (momentary) */
#define LED_PANIC                  0 /* OFF     ON  (1Hz flashing) */

/* Alternate pin selections *********************************************************/
/* Pin Description                  Connector On Board       Base Board
 * -------------------------------- --------- -------------- ---------------------
 * P0[0]/RD1/TXD3/SDA1               J6-9     I2C E2PROM SDA TXD3/SDA1
 * P0[1]/TD1/RXD3/SCL                J6-10                   RXD3/SCL1
 * P0[2]/TXD0/AD0[7]                 J6-21
 * P0[3]/RXD0/AD0[6]                 J6-22
 * P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38                   CAN_RX2
 * P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39                   CAN_TX2
 * P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8                    SSEL1, OLED CS
 * P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7                    SCK1, OLED SCK
 * P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6                    MISO1
 * P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5                    MOSI1, OLED data in
 * P0[10]                            J6-40                   TXD2/SDA2
 * P0[11]                            J6-41                   RXD2/SCL2
 */

#define GPIO_UART3_TXD     GPIO_UART3_TXD_1
#define GPIO_I2C1_SDA      GPIO_I2C1_SDA_1
#define GPIO_UART3_RXD     GPIO_UART3_RXD_1
#define GPIO_I2C1_SCL      GPIO_I2C1_SCL_1
#define GPIO_SSP1_SCK      GPIO_SSP1_SCK_1
#define GPIO_UART2_TXD     GPIO_UART2_TXD_1
#define GPIO_UART2_RXD     GPIO_UART2_RXD_1
#define GPIO_UART1_TXD     GPIO_UART1_TXD_1
#define GPIO_SSP0_SCK      GPIO_SSP0_SCK_1
#define GPIO_UART1_RXD     GPIO_UART1_RXD_1
#define GPIO_SSP0_SSEL     GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO     GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI     GPIO_SSP0_MOSI_1

/* P1[0]/ENET-TXD0                   J6-34?  TXD0            TX-(Ethernet PHY)
 * P1[1]/ENET_TXD1                   J6-35?  TXD1            TX+(Ethernet PHY)
 * P1[4]/ENET_TX_EN                          TXEN            N/A
 * P1[8]/ENET_CRS                            CRS_DV/MODE2    N/A
 * P1[9]/ENET_RXD0                   J6-32?  RXD0/MODE0      RD-(Ethernet PHY)
 * P1[10]/ENET_RXD1                  J6-33?  RXD1/MODE1      RD+(Ethernet PHY)
 */

#define GPIO_ENET_MDC      GPIO_ENET_MDC_1
#define GPIO_ENET_MDIO     GPIO_ENET_MDIO_1

/* P2[0]/PWM1.1/TXD1                 J6-42                   PWM1.1 / RGB LED / RS422 RX
 * P2[1]/PWM1.2/RXD1                 J6-43                   PWM1.2 / OLED voltage / RGB LED / RS422 RX
 * P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44                   PWM1.3
 * P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45                   PWM1.4
 * P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46                   PWM1.5
 * P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47                   PWM1.6
 * P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48
 * P2[7]/RD2/RTS1                    J6-49                   OLED command/data
 * P2[8]/TD2/TXD2                    J6-50
 * P2[9]/USB_CONNECT/RXD2            PAD19   USB Pullup      N/A
 * P2[10]/EINT0/NMI                  J6-51
 * P2[11]/EINT1/I2STX_CLK            J6-52
 */

#define GPIO_PWM1p1        GPIO_PWM1p1_2
#define GPIO_PWM1p2        GPIO_PWM1p2_2
#define GPIO_PWM1p3        GPIO_PWM1p3_2
#define GPIO_PWM1p4        GPIO_PWM1p4_2
#define GPIO_PWM1p5        GPIO_PWM1p5_2
#define GPIO_PWM1p6        GPIO_PWM1p6_2

/* P3[25]/MAT0.0/PWM1.2              PAD13                   N/A
 * P3[26]/STCLK/MAT0.1/PWM1.3        PAD14                   N/A
 */

#endif  /* __CONFIG_LPCXPRESSO_LPC1115_INCLUDE_BOARD_H */
