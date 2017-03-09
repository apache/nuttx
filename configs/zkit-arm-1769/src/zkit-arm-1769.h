/************************************************************************************
 * configs/zkit-arm-1769/src/zkit-arm-1769.h
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: BabuSubashChandar <code@zilogic.com>
 *
 * Based on configs/lpcxpresso-lpc1768/src/lpcxpresso.h
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
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

#ifndef _CONFIGS_ZKITARM_LPC1768_SRC_ZKITARM_H
#define _CONFIGS_ZKITARM_LPC1768_SRC_ZKITARM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/************************************************************************************
 * ZKit-ARM-1769 Pin Usage
 ************************************************************************************/
/*  Pin Description                      On Board       Connector
 *  -------------------------------- ---------------- -------------
 *  P0.0/RD1/TXD3/SDA1                   RD1            AUX-CON
 *  P0.1/TD1/RXD3/SCL1                   TD1
 *  P0.2/TXD0/AD0.7                      TXD0           COM0
 *  P0.3/RXD0/AD0.6                      RXD0
 *  P0.4/I2SRX_CLK/RD2/CAP2.0            GPIO0
 *  P0.5/I2SRX_WS/TD2/CAP2.1             GPIO1
 *  P0.6/I2SRX_SDA/SSEL1/MAT2.0          SSEL1          SPI
 *  P0.7/I2STX_CLK/SCK1/MAT2.1           SCK1
 *  P0.8/I2STX_WS/MISO1/MAT2.2           MISO1
 *  P0.9/I2STX_SDA/MOSI1/MAT2.3          MOSI1
 *  P0.10/TXD2/SDA2/MAT3.0               TXD2           AUX-CON
 *  P0.11/RXD2/SCL2/MAT3.1               RXD2
 *  P0.15/TXD1/SCK0/SCK                  SD-SCK
 *  P0.16/RXD1/SSEL0/SSEL                SD-SSEL        SD-CARD
 *  P0.17/CTS1/MISO0/MISO                SD-MISO
 *  P0.18/DCD1/M0SI0/MOSI                SD-MOSI
 *  P0.19/DSR1/SDA1                      LED1
 *  P0.20/DTR1/SCL1                      DTR1           COM1
 *  P0.21/RI1/RD1 N.C                    LED2
 *  P0.22/RTS1/TD1                       RTS1           COM1
 *  P0.23/AD0.0/I2SRX_CLK/CAP3.0         AD0
 *  P0.24/AD0.1/I2SRX_WS/CAP3.1          AD1            AIN
 *  P0.25/AD0.2/I2SRX_SDA/TXD3           AD2
 *  P0.26/AD0.3/AOUT/RXD3                AD3
 *  P0.27/SDA0/USB_SDA                   SDA0           I2C0
 *  P0.28/SCL0/USB_SCL                   SCL0
 *  P0.29/USB_D+                         USB-D+         USB
 *  P0.30/USB_D-                         USB-D-
 *
 *  P1.0/ENET_TXD0                       ETH-TXD0
 *  P1.1/ENET_TXD1                       ETH-TXD1
 *  P1.4/ENET_TX_EN                      ETH-TXEN
 *  P1.8/ENET_CRS                        ETH-CRS
 *  P1.9/ENET_RXD0                       ETH-RXD0       ETH
 *  P1.10/ENET_RXD1                      ETH-RXD1
 *  P1.14/ENET_RX_ER                     ETH-RXER
 *  P1.15/ENET_REF_CLK                   ETH-REFCLK
 *  P1.16/ENET_MDC                       ETH-MDC
 *  P1.17/ENET_MDIO                      ETH-MDIO
 *  P1.18/USB_UP_LED/PWM1.1/CAP1.0       USB-UP-LED
 *  P1.19/MCOA0/nUSB_PPWR/CAP1.1         KEY1
 *  P1.20/MCFB0/PWM1.2/SCK0              LCD-SCK
 *  P1.21/MCABORT/PWM1.3/SSEL0           LCD-SSEL
 *  P1.22/MCOB0/USB_PWRD/MAT1.0          LCD-A0         LCD
 *  P1.23/MCFB1/PWM1.4/MISO0             NC
 *  P1.24/MCFB2/PWM1.5/MOSI0             LCD_MOSI
 *  P1.25/MCOA1/MAT1.1                   LCD-RST
 *  P1.26/MCOB1/PWM1.6/CAP0.0            LCD-AO
 *  P1.27/CLKOUT/nUSB_OVRCR/CAP0.1       KEY2
 *  P1.28/MCOA2/MAT0.0                   KEY3
 *  P1.29/MCOB2/PCAP1.1/MAT0.1           CAP1           PWM-CON
 *  P1.30/VBUS/AD0.4                     VBUS           USB
 *  P1.31/SCK1/AD0.5                     KEY4
 *
 *  P2.0/PWM1.1/TXD1                     TXD1
 *  P2.1/PWM1.2/RXD1                     RXD1           COM1
 *  P2.2/PWM1.3/CTS1/TRACEDATA3          CTS1
 *  P2.3/PWM1.4/DCD1/TRACEDATA2          PWM4
 *  P2.4/PWM1.5/DSR1/TRACEDATA1          PWM5           PWM
 *  P2.5/PWM1.6/DTR1/TRACEDATA0          PWM6
 *  P2.6/PCAP1.0/RI1/TRACECLK            CAP0
 *  P2.7/RD2/RTS1 RD2                    RD2            CAN2
 *  P2.8/TD2/TXD2 TD2                    TD2
 *  P2.9/USB_CONNECT/RXD2                USB_CONNECT    USB
 *  P2.10/nEINT0/NMI                     ISP
 *  P2.11/nEINT1/I2STX_CLK               INT1           I2C
 *  P2.12/nEINT2/I2STX_WS                SD-DET         SD-CARD
 *  P2.13/nEINT3/I2STX_SDA               KEY5
 *
 *  P3.25/MAT0.0/PWM1.2                  PWM2           PWM
 *  P3.26/STCLK/MAT0.1/PWM1.3            PWM3
 *
 *  P4.28/RX_MCLK/MAT2.0/TXD3            GPIO2          SPI
 *  P4.28/RX_MCLK/MAT2.0/TXD3            GPIO3
 */

#define ZKITARM_I2C1_EPROM_SDA GPIO_I2C1_SDA
#define ZKITARM_I2C1_EPROM_SDL GPIO_I2C1_SCL

#define ZKITARM_LED1 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN19)
#define ZKITARM_LED2 (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN21)

#define ZKITARM_KEY1 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN19)
#define ZKITARM_KEY2 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN27)
#define ZKITARM_KEY3 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN28)
#define ZKITARM_KEY4 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT1 | GPIO_PIN31)
#define ZKITARM_KEY5 (GPIO_INPUT | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN13)

#define ZKITARM_INT_KEY5 (GPIO_INTFE | GPIO_FLOAT | GPIO_PORT2 | GPIO_PIN13)
#define ZKITARM_KEY5_IRQ LPC17_IRQ_P2p13

/* SD Slot
 *
 *       Board        LPC1768
 *  SD   Signal       Pin
 *  ---  -----------  ----------
 *  CS   SD-SSEL       P0.16
 *  DIN  SD-MOSI       P0.18 MOSI0
 *  DOUT SD-MISO       P0.17 MISO0
 *  CLK  SD-SCK        P0.15 SCK0
 *  CD   SD-DET        P2.12
 */

#define ZKITARM_SD_CS (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN16)
#ifdef CONFIG_LPC17_GPIOIRQ
#  define ZKITARM_SD_CD (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN12)
#else
#  define ZKITARM_SD_CD (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN12)
#endif

#define ZKITARM_SD_CDIRQ LPC17_IRQ_P2p12

/* USB:
 *
 *   Board               LPC1768
 *   Signal              Pin
 *   ------------------- --------
 *   USB_CONNECT         P2.9  USB_CONNECT
 *   USB_DM              P0.29 USB_D-
 *   USB_DP              P0.30 USB_D+
 *   USB_VBUS            P1.30 USB_VBUS
 *   USB_UPLED           P1.18 USB_UPLED
 *
 */

#define ZKITARM_USB_CONNECT (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN9)
#ifdef CONFIG_LPC17_GPIOIRQ
#  define ZKITARM_USB_VBUSSENSE (GPIO_INTBOTH | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN30)
#else
#  define ZKITARM_USB_VBUSSENSE (GPIO_INPUT   | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN30)
#endif

/* 128x64 LCD with SPI interface
 * ---------------------------------------
 * The LCD display is connected to the SPI-bus.
 *
 *   ZKit-ARM Signals
 *
 *     ----------------------------+---------------+--------------------------------------------
 *     LPC1758 Pin                 | Board Signal  |        Description
 *     ----------------------------+---------------+--------------------------------------------
 *     P1.20/MCFB0/PWM1.2/SCK0     |  LCD-SCK      | LCD Clock signal (D6)
 *     P1.21/MCABORT/PWM1.3/SSEL0  |  LCD-SSEL     | LCD Chip Select  (CSB)
 *     P1.22/MCOB0/USB_PWRD/MAT1.0 |  LCD-A0       | LCD-A0 (A0)
 *     P1.23/MCFB1/PWM1.4/MISO0    |  N.C          |
 *     P1.24/MCFB2/PWM1.5/MOSI0    |  LCD-MOSI     | LCD Data (D7)
 *     P1.25/MCOA1/MAT1.1          |  LCD-RST      | LCD Reset (RSTB) - Resets Everything in LCD
 *     ----------------------------+---------------+--------------------------------------------
 */

#if 0
#define ZKITARM_OLED_POWER (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN1)
#define ZKITARM_OLED_CS    (GPIO_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORT0 | GPIO_PIN6)
#define ZKITARM_OLED_DC    (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT2 | GPIO_PIN7)
#endif
#define ZKITARM_OLED_RST   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN25)
#define ZKITARM_OLED_CS    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN21)
#define ZKITARM_OLED_RS    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN22)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: zkit_sspdev_initialize
 *
 * Description:
 *   Called to configure SSP chip select GPIO pins for the ZKit-ARM-1769 board.
 *
 ************************************************************************************/

void weak_function zkit_sspdev_initialize(void);

/************************************************************************************
 * Name: zkit_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the ZKit-ARM-1769 board.
 *
 ************************************************************************************/

void weak_function zkit_spidev_initialize(void);

/************************************************************************************
 * Name: zkit_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int zkit_adc_setup(void);
#endif

/************************************************************************************
 * Name: zkit_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ************************************************************************************/

#ifdef CONFIG_CAN
int zkit_can_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_ZKITARM_LPC1768_SRC_ZKITARM_H */
