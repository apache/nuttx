/****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max32660-evsys.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_MAX326XX_MAX32660_EVSYS_SRC_MAX32660_EVSYS_H
#define __BOARDS_ARM_MAX326XX_MAX32660_EVSYS_SRC_MAX32660_EVSYS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A single red LED is available driven by GPIO P0.13.
 * High illuminates, initial output is Low.
 */

#define GPIO_LED     (GPIO_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORT0 | GPIO_PIN13)

/* An single button is available on GPIO P0.12 for use by software.
 * Interrupts are available on both edges.
 */

#define GPIO_BUTTON  (GPIO_INTBOTH | GPIO_PORT0 | GPIO_PIN12)
#define BUTTON_IRQ   MAX326_IRQ_P0_12

/* SPI0: No alternative pin configurations:
 *
 *   PORT0  PIN  SPI FUNCTION ALT FUNCTION COMMENT
 *   ------ ---- ------------ ------------ ----------------------------
 *   P0.4   5    MISO         ALT1
 *   P0.5   6    MOSI         ALT1
 *   P0.6   8    SCK          ALT1
 *   P0.7   10   SS0          ALT1
 *
 * I have connected a Sparkfun microSD breakout board via SPI0 using the
 * SPI0 SS pin (P0.7) as a GPIO push-pull output.  We would like to pick
 * a pin for a card detect input the will not conflict with other
 * selections.  The CS pin needs a pull-up resistor so that also limits
 * the options (P0.0-1, P0.4-7, and P0.10-13).  Nothing is available:
 *
 *   PIN   HAS PULL_UP CONFLICTS
 *   ----- ----------- ------------------
 *   P0.0      Y       JTAG/SWD
 *   P0.1      Y       JTAG/SWD
 *   P0.2      N       I2C1
 *   P0.3      N       I2C1
 *   P0.4      Y       SPI0/UART0
 *   P0.5      Y       SPI0/UART0
 *   P0.6      Y       SPI0/UART0
 *   P0.7      Y       SPI0/UART0
 *   P0.8      N       I2C0/UART0
 *   P0.9      N       I2C0
 *   P0.10     Y       UART1 (serial console)
 *   P0.11     Y       UART1 (serial console)
 *   P0.12     Y       Button
 *   P0.13     Y       LED
 *
 * For now, we must either (1) configure with no card detect (2) add an
 * external pull-up to P0.2, P0.3, P0.8, or P0.9 (and don't use
 * corresponding I2C), or (3) get creative with pins associated with JTAG,
 * LEDs or Buttons.
 */

#define MICROSD_CS   (GPIO_OUTPUT | GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN7)
#undef  MICROSD_CD

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max326_bringup
 *
 * Description:
 *   Bring up board features.
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE=y, then this function will be called
 *   from board_late_initialize().
 *
 *   If CONFIG_BOARD_LATE_INITIALIZE is not selected,
 *   but CONFIG_LIB_BOARDCTL=y
 *   then this function will *probably* be called from application logic via
 *   boardctl().
 *
 *   Otherwise, this function will not be called (which is usually a bad
 *   thing)
 *
 ****************************************************************************/

int max326_bringup(void);

/****************************************************************************
 * Name: max326_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the MAX3660-EVSYS
 *   board.
 *
 *****************************************************************************/

#ifdef CONFIG_MAX326XX_HAVE_SPIM
void max326_spidev_initialize(void);
#endif

/*****************************************************************************
 * Name: max326_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int max326_mmcsd_initialize(int minor);
#endif

#endif /* __BOARDS_ARM_MAX326XX_MAX32660_EVSYS_SRC_MAX32660_EVSYS_H */
