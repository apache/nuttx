/****************************************************************************
 * boards/arm/lpc31xx/ea3131/src/ea3131.h
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

#ifndef __BOARDS_ARM_LPC31XX_EA3131_SRC_EA3131_H
#define __BOARDS_ARM_LPC31XX_EA3131_SRC_EA3131_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "lpc31_ioconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EA3131L GPIOs ************************************************************/

/* LEDs -- interface through an I2C GPIO expander */

/* BUTTONS -- NOTE that some have EXTI interrupts configured */

/* SPI Chip Selects */

/* SPI NOR flash is the only device on SPI. SPI_CS_OUT0 is its chip select */

#define SPINOR_CS IOCONFIG_SPI_CSOUT0

/* USB Soft Connect Pullup -- NONE */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc31_meminitialize
 *
 * Description:
 *   Initialize external memory resources (sram, sdram, nand, nor, etc.)
 *
 ****************************************************************************/

#ifdef CONFIG_LPC31_EXTDRAM
void lpc31_meminitialize(void);
#endif

/****************************************************************************
 * Name: lpc31_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the EA3131 board.
 *
 ****************************************************************************/

void weak_function lpc31_spidev_initialize(void);

/****************************************************************************
 * Name: lpc31_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the EA3131 board.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBDEV)
void weak_function lpc31_usbdev_initialize(void);
#endif

/****************************************************************************
 * Name: lpc31_usbhost_bootinitialize
 *
 * Description:
 *   Called from lpc31_boardinitialize very early in inialization to setup
 *   USB host-related GPIO pins for the EA3131 board.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
void weak_function lpc31_usbhost_bootinitialize(void);
#endif

/****************************************************************************
 * Name: lpc31_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
int lpc31_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: lpc31_pginitialize
 *
 * Description:
 *   Set up mass storage device to support on demand paging.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
void weak_function lpc31_pginitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_LPC31XX_EA3131_SRC_EA3131_H */
