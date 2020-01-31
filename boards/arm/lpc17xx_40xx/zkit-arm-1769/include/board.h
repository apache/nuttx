/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: BabuSubashChandar <code@zilogic.com>
 *
 * Based on boards/lpcxpresso-lpc1768/include/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_ZKIT_ARM_1769_INCLUDE_BOARD_H
#define __BOARDS_ARM_LPC17XX_40XX_ZKIT_ARM_1769_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC17_40_GPIOIRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* NOTE:  The following definitions require lpc17_40_syscon.h.
 * It is not included here because the including C file may not have that
 * file in its include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32768)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20, pre-divider=1
 *   CCLCK = 480MHz / 6 = 80MHz               -> CCLK divider = 6
 */

#define LPC17_40_CCLK                 80000000 /* 80Mhz */

/* Select the main oscillator as the frequency source.
 * SYSCLK is then the frequency of the main oscillator.
 */

#undef CONFIG_LPC17_40_MAINOSC
#define CONFIG_LPC17_40_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider.
 * The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKCFG_DIVIDER      6
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.
 * PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC17_40_PLL0
#define CONFIG_LPC17_40_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC17_40_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.
 * This divider is used when PLL1 is not enabled to get the USB clock
 * from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LP17_FLASH
#define CONFIG_LP17_FLASH          1
#define BOARD_FLASHCFG_VALUE       0x0000303a

/* Ethernet configuration */

//#define ETH_MCFG_CLKSEL_DIV ETH_MCFG_CLKSEL_DIV44
#define ETH_MCFG_CLKSEL_DIV ETH_MCFG_CLKSEL_DIV20

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 *
 * LED1            -- Connected to P0[19]
 * LED2            -- Connected to P0[21]
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1                0
#define BOARD_LED2                1
#define BOARD_NLEDS               2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT            (1 << BOARD_LED1)
#define BOARD_LED2_BIT            (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs
 * on board the ZKIT-ARM-1769.  The following definitions
 * describe how NuttX controls the LEDs:
 */

                                      /* LED1   LED2 */
#define LED_STARTED                0  /* OFF    OFF  */
#define LED_HEAPALLOCATE           1  /* ON     OFF  */
#define LED_IRQSENABLED            2  /* OFF    ON   */
#define LED_STACKCREATED           3  /* OFF    OFF  */

/* After the system is booted, this logic will no longer use LED 1.
 * LED 1 is available for use by application software using lpc17_40_led
 * (prototyped below)
 */
                                      /* LED1   LED2 */
#define LED_INIRQ                  4  /*  NC     ON  (momentary) */
#define LED_SIGNAL                 5  /*  NC     ON  (momentary) */
#define LED_ASSERTION              6  /*  NC     ON  (momentary) */
#define LED_PANIC                  7  /*  NC     ON  (1Hz flashing) */

/* Button definitions *******************************************************/

/* The ZKIT-ARM-1769 supports several buttons.
 * All will read "1" when open and "0" when closed
 *
 * KEY1            -- Connected to P1[19]
 * KEY2            -- Connected to P1[27]
 * KEY3            -- Connected to P1[28]
 * KEY4            -- Connected to P1[31]
 * KEY5            -- Connected to P2[13]
 *
 */

#define BOARD_BUTTON_1             0
#define BOARD_BUTTON_2             1
#define BOARD_BUTTON_3             2
#define BOARD_BUTTON_4             3
#define BOARD_BUTTON_5             4
#define NUM_BUTTONS                5

#define BOARD_BUTTON1_BIT          (1 << BOARD_BUTTON_1)
#define BOARD_BUTTON2_BIT          (1 << BOARD_BUTTON_2)
#define BOARD_BUTTON3_BIT          (1 << BOARD_BUTTON_3)
#define BOARD_BUTTON4_BIT          (1 << BOARD_BUTTON_4)
#define BOARD_BUTTON5_BIT          (1 << BOARD_BUTTON_5)

/* Alternate pin selections *************************************************/

/* Pin Description                      On Board       Connector
 * -------------------------------- ---------------- -------------
 * P0.0/RD1/TXD3/SDA1                   RD1            AUX-CON
 * P0.1/TD1/RXD3/SCL1                   TD1
 * P0.2/TXD0/AD0.7                      TXD0           COM0
 * P0.3/RXD0/AD0.6                      RXD0
 * P0.4/I2SRX_CLK/RD2/CAP2.0            GPIO0
 * P0.5/I2SRX_WS/TD2/CAP2.1             GPIO1
 * P0.6/I2SRX_SDA/SSEL1/MAT2.0          SSEL1          SPI
 * P0.7/I2STX_CLK/SCK1/MAT2.1           SCK1
 * P0.8/I2STX_WS/MISO1/MAT2.2           MISO1
 * P0.9/I2STX_SDA/MOSI1/MAT2.3          MOSI1
 * P0.10/TXD2/SDA2/MAT3.0               TXD2           AUX-CON
 * P0.11/RXD2/SCL2/MAT3.1               RXD2
 * P0.15/TXD1/SCK0/SCK                  SD-SCK
 * P0.16/RXD1/SSEL0/SSEL                SD-SSEL        SD-CARD
 * P0.17/CTS1/MISO0/MISO                SD-MISO
 * P0.18/DCD1/M0SI0/MOSI                SD-MOSI
 * P0.19/DSR1/SDA1                      LED1
 * P0.20/DTR1/SCL1                      DTR1           COM1
 * P0.21/RI1/RD1 N.C                    LED2
 * P0.22/RTS1/TD1                       RTS1           COM1
 * P0.23/AD0.0/I2SRX_CLK/CAP3.0         AD0
 * P0.24/AD0.1/I2SRX_WS/CAP3.1          AD1            AIN
 * P0.25/AD0.2/I2SRX_SDA/TXD3           AD2
 * P0.26/AD0.3/AOUT/RXD3                AD3
 * P0.27/SDA0/USB_SDA                   SDA0           I2C0
 * P0.28/SCL0/USB_SCL                   SCL0
 * P0.29/USB_D+                         USB-D+         USB
 * P0.30/USB_D-                         USB-D-
 */

#define GPIO_CAN1_RD       GPIO_CAN1_RD_1
#define GPIO_CAN1_TD       GPIO_CAN1_TD_1
#define GPIO_CAN2_RD       GPIO_CAN2_RD_2
#define GPIO_CAN2_TD       GPIO_CAN2_TD_2
#define GPIO_I2C1_SDA      GPIO_I2C0_SDA
#define GPIO_I2C1_SCL      GPIO_I2C0_SCL
#define GPIO_SSP1_SCK      GPIO_SSP1_SCK_1
#define GPIO_UART2_TXD     GPIO_UART2_TXD_1
#define GPIO_UART2_RXD     GPIO_UART2_RXD_1
#define GPIO_UART1_TXD     GPIO_UART1_TXD_2
#define GPIO_UART1_RXD     GPIO_UART1_RXD_2
#define GPIO_SSP0_SCK      GPIO_SSP0_SCK_2
#define GPIO_SSP0_SSEL     GPIO_SSP0_SSEL_2
#define GPIO_SSP0_MISO     GPIO_SSP0_MISO_2
#define GPIO_SSP0_MOSI     GPIO_SSP0_MOSI_2

/* P1.0/ENET_TXD0                       ETH-TXD0
 * P1.1/ENET_TXD1                       ETH-TXD1
 * P1.4/ENET_TX_EN                      ETH-TXEN
 * P1.8/ENET_CRS                        ETH-CRS
 * P1.9/ENET_RXD0                       ETH-RXD0       ETH
 * P1.10/ENET_RXD1                      ETH-RXD1
 * P1.14/ENET_RX_ER                     ETH-RXER
 * P1.15/ENET_REF_CLK                   ETH-REFCLK
 * P1.16/ENET_MDC                       ETH-MDC
 * P1.17/ENET_MDIO                      ETH-MDIO
 * P1.18/USB_UP_LED/PWM1.1/CAP1.0       USB-UP-LED
 * P1.19/MCOA0/nUSB_PPWR/CAP1.1         KEY1
 * P1.20/MCFB0/PWM1.2/SCK0              LCD-SCK
 * P1.21/MCABORT/PWM1.3/SSEL0           LCD-SSEL
 * P1.22/MCOB0/USB_PWRD/MAT1.0          LCD-A0         LCD
 * P1.23/MCFB1/PWM1.4/MISO0             NC
 * P1.24/MCFB2/PWM1.5/MOSI0             LCD_MOSI
 * P1.25/MCOA1/MAT1.1                   LCD-RST
 * P1.26/MCOB1/PWM1.6/CAP0.0            LCD-AO
 * P1.27/CLKOUT/nUSB_OVRCR/CAP0.1       KEY2
 * P1.28/MCOA2/MAT0.0                   KEY3
 * P1.29/MCOB2/PCAP1.1/MAT0.1           CAP1           PWM-CON
 * P1.30/VBUS/AD0.4                     VBUS           USB
 * P1.31/SCK1/AD0.5                     KEY4
 */

#define GPIO_ENET_MDC      GPIO_ENET_MDC_1
#define GPIO_ENET_MDIO     GPIO_ENET_MDIO_1

/* P2.0/PWM1.1/TXD1                     TXD1
 * P2.1/PWM1.2/RXD1                     RXD1           COM1
 * P2.2/PWM1.3/CTS1/TRACEDATA3          CTS1
 * P2.3/PWM1.4/DCD1/TRACEDATA2          PWM4
 * P2.4/PWM1.5/DSR1/TRACEDATA1          PWM5           PWM
 * P2.5/PWM1.6/DTR1/TRACEDATA0          PWM6
 * P2.6/PCAP1.0/RI1/TRACECLK            CAP0
 * P2.7/RD2/RTS1 RD2                    RD2            CAN2
 * P2.8/TD2/TXD2 TD2                    TD2
 * P2.9/USB_CONNECT/RXD2                USB_CONNECT    USB
 * P2.10/nEINT0/NMI                     ISP
 * P2.11/nEINT1/I2STX_CLK               INT1           I2C
 * P2.12/nEINT2/I2STX_WS                SD-DET         SD-CARD
 * P2.13/nEINT3/I2STX_SDA               KEY5
 */

#define GPIO_PWM1p2        GPIO_PWM1p2_3
#define GPIO_PWM1p3        GPIO_PWM1p3_3
#define GPIO_PWM1p4        GPIO_PWM1p4_2
#define GPIO_PWM1p5        GPIO_PWM1p5_2
#define GPIO_PWM1p6        GPIO_PWM1p6_2

/* P3.25/MAT0.0/PWM1.2                  PWM2           PWM
 * P3.26/STCLK/MAT0.1/PWM1.3            PWM3
 *
 * P4.28/RX_MCLK/MAT2.0/TXD3            GPIO2          SPI
 * P4.28/RX_MCLK/MAT2.0/TXD3            GPIO3
 */

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

/****************************************************************************
 * Name: lpc17_40_led
 *
 * Description:
 *   Once the system has booted, these functions can be used to control LEDs 1
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lpc17_40_led(int lednum, int state);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_ZKIT_ARM_1769_INCLUDE_BOARD_H */
