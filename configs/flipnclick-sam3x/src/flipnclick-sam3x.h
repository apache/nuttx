/************************************************************************************
 * configs/flipnclick-sam3x/src/flipnclick-sam3x.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H
#define __CONFIGS_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* There are four LEDs on the top, blue side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L - PB27 (PWM13)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A - PC6
 *   LED B - PC5
 *   LED C - PC7
 *   LED D - PC8
 *
 * A high output value illuminates the LEDs.
 */

#define GPIO_LED_L   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOB | GPIO_PIN27)
#define GPIO_LED_A   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN6)
#define GPIO_LED_B   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN5)
#define GPIO_LED_C   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN7)
#define GPIO_LED_D   (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                      GPIO_PORT_PIOC | GPIO_PIN8)

/* SPI chip select pins.
 *
 * SPI0 is available on the Arduino compatible SPI connector (but no SPI is
 * available on pins D10-D13 of the main Arduino Shield connectors where
 * you might expect then).  The SPI connector is configured as follows:
 *
 *   Pin Board Signal SAM3X  Pin Board Signal SAM3X
 *   --- ------------ -----  --- ------------ -----
 *    1  SPI0_MISO    PA25    2  VCC-5V       N/A
 *    3  SPI0_SCK     PA27    4  SPI0_MOSI    PA26
 *    5  MRST         NRSTB   6  GND          N/A
 *
 * SPI0 is also available on each of the mikroBUS Click connectors (in
 * addition to 5V and GND).  The connectivity differs only in the chip
 * select pin:
 *
 *   MikroBUS A:              MikroBUS B:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -----
 *   CS   SPI0_CS0     PA28   CS   PA29         PA29
 *   SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
 *   MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
 *   MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26
 *
 *   MikroBUS C:              MikroBUS D:
 *   Pin  Board Signal SAM3X  Pin  Board Signal SAM3X
 *   ---- ------------ -----  ---- ------------ -----
 *   CS   SPI0_CS2     PB21   CS   SPI0_CS3     PB23
 *   SCK  SPI0_SCK     PA27   SCK  SPI0_SCK     PA27
 *   MISO SPI0_MISO    PA25   MISO SPI0_MISO    PA25
 *   MOSI SPI0_MOSI    PA26   MOSI SPI0_MOSI    PA26
 */

#define GPIO_MBA_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN28)
#define MBA_CSNUM    0

#define GPIO_MBB_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOA | GPIO_PIN29)
#define MBB_CSNUM    1

#define GPIO_MBC_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOB | GPIO_PIN21)
#define MBC_CSNUM    2

#define GPIO_MBD_CS  (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_SET | \
                      GPIO_PORT_PIOB | GPIO_PIN23)
#define MBD_CSNUM    3

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

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int sam_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_FLIPNCLICK_SAM3X_SRC_FLIPNCLICK_SAM3X_H */

