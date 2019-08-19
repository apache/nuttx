/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/pnev5180b.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Michael Jung <mijung@gmx.net>
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

#ifndef __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_PNEV5180B_H
#define __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_PNEV5180B_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * NXP PNEV5180B Pin Usage
 ****************************************************************************/

/*  Pin Description                      On Board       Connector
 *  -------------------------------- ---------------- -------------
 *  P0.2/TXD0/AD0.7                    TX               J201
 *  P0.3/RXD0/AD0.6                    RX
 *  P0.22/RTS1/TD1                     LD200            ORANGE LED
 *  P0.15/TXD1/SCK0/SCK                PN5180-SCK
 *  P0.16/RXD1/SSEL0/SSEL              PN5180-SSEL      PN5180
 *  P0.17/CTS1/MISO0/MISO              PN5180-MISO
 *  P0.18/DCD1/M0SI0/MOSI              PN5180-MOSI
 *  P0.19/DSR1/SDA1                    EEPROM           (Not Assembled)
 *  P0.20/DTR1/SCL1                    EEPROM
 *  P0.21/RI1/RD1                      PN5180-AUX2      PN5180
 *  P0.29/USB_D+                       USB-D+           USB
 *  P0.30/USB_D-                       USB-D-
 *  P2.0/PWM1.1/TXD1                   LD201            RED LED
 *  P2.5/PWM1.6/DTR1/TRACEDATA0        PN5180-nPN_RST
 *  P2.9/USB_CONNECT/RXD2              USB_CONNECT      USB
 *  P2.11/nEINT1/I2STX_CLK             PN5180-BUSY      PN5180
 *  P2.12/nEINT2/I2STX_WS              PN5180-IRQ
 *  P3.25/MAT0.0/PWM1.2                LD203            GREEN LED
 *  P3.26/STCLK/MAT0.1/PWM1.3          LD202            BLUE LED
 */

#define PNEV5180B_LED_BLUE   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | GPIO_PIN26)
#define PNEV5180B_LED_GREEN  (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT3 | GPIO_PIN25)
#define PNEV5180B_LED_ORANGE (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN22)
#define PNEV5180B_LED_RED    (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT2 | GPIO_PIN0)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pnev5180b_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int pnev5180b_bringup(void);

/****************************************************************************
 * Name: pnev5180b_autoled_initialize
 *
 * Description:
 *   Called early in power-up initialization to initialize the LED hardware.
 *
 ****************************************************************************/

void pnev5180b_autoled_initialize(void);

/****************************************************************************
 * Name: pnev5180b_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the NXP PNEV5180B
 *   board.
 *
 ****************************************************************************/

void weak_function pnev5180b_spidev_initialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC17XX_40XX_PNEV5180B_SRC_PNEV5180B_H */
