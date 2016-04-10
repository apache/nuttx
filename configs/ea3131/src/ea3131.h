/************************************************************************************
 * configs/ea3131/src/ea3131.h
 *
 *   Copyright (C) 2009-2010,2012 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_EA3131_SRC_EA3131_H
#define __CONFIGS_EA3131_SRC_EA3131_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "lpc31_ioconfig.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* EA3131L GPIOs ********************************************************************/

/* LEDs -- interface through an I2C GPIO expander */

/* BUTTONS -- NOTE that some have EXTI interrupts configured */

/* SPI Chip Selects */
/* SPI NOR flash is the only device on SPI. SPI_CS_OUT0 is its chip select */

#define SPINOR_CS IOCONFIG_SPI_CSOUT0

/* USB Soft Connect Pullup -- NONE */

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
 * Name: lpc31_meminitialize
 *
 * Description:
 *   Initialize external memory resources (sram, sdram, nand, nor, etc.)
 *
 ************************************************************************************/

#ifdef CONFIG_LPC31_EXTDRAM
void lpc31_meminitialize(void);
#endif

/************************************************************************************
 * Name: lpc31_spidev_intialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the EA3131 board.
 *
 ************************************************************************************/

void weak_function lpc31_spidev_intialize(void);

/************************************************************************************
 * Name: lpc31_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the EA3131 board.
 *
 ************************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBDEV)
void weak_function lpc31_usbdev_initialize(void);
#endif

/************************************************************************************
 * Name: lpc31_usbhost_bootinitialize
 *
 * Description:
 *   Called from lpc31_boardinitialize very early in inialization to setup USB
 *   host-related GPIO pins for the EA3131 board.
 *
 ************************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
void weak_function lpc31_usbhost_bootinitialize(void);
#endif

/***********************************************************************************
 * Name: lpc31_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

#if defined(CONFIG_LPC31_USBOTG) && defined(CONFIG_USBHOST)
int lpc31_usbhost_initialize(void);
#endif

/************************************************************************************
 * Name: lpc31_pginitialize
 *
 * Description:
 *   Set up mass storage device to support on demand paging.
 *
 ************************************************************************************/

#ifdef CONFIG_PAGING
void weak_function lpc31_pginitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_EA3131_SRC_EA3131_H */
