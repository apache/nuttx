/****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max32660-evsys.h
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
 *   but CONFIG_BOARDCTL=y
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
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_HAVE_SPIM
void max326_spidev_initialize(void);
#endif

/****************************************************************************
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
