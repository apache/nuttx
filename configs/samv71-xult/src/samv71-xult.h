/************************************************************************************
 * configs/samv71-xult/src/samv71-xult.h
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

#ifndef __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H
#define __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

//#include "chip/sam_pinmap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#define HAVE_HSMCI      1
#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1
#define HAVE_NETWORK    1

/* HSMCI */
/* Can't support MMC/SD if the card interface is not enabled */

#if !defined(CONFIG_SAMV7_HSMCI)
#  undef HAVE_HSMCI
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(HAVE_HSMCI) && defined(CONFIG_DISABLE_MOUNTPOINT)
#  warning Mountpoints disabled.  No MMC/SD support
#  undef HAVE_HSMCI
#endif

/* We need PIO interrupts on GPIOA to support card detect interrupts */

#if defined(HAVE_HSMCI) && !defined(CONFIG_SAMV7_GPIOA_IRQ)
#  warning PIOA interrupts not enabled.  No MMC/SD support.
#  undef HAVE_HSMCI
#endif

/* USB Device */
/* CONFIG_SAMV7_UDP and CONFIG_USBDEV must be defined, or there is no USB
 * device.
 */

#if !defined(CONFIG_SAMV7_UDP) || !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#if !defined(CONFIG_SYSTEM_USBMONITOR) || !defined(CONFIG_USBDEV_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* Networking */

#if !defined(CONFIG_NET) || !defined(CONFIG_SAMV7_EMAC)
#  undef HAVE_NETWORK
#endif

/* SAMV71-XULT GPIO Pin Definitions *************************************************/

/* LCD:
 * To be provided
 */

/* Ethernet MAC.
 * to be provided
 */

/* LEDs
 *
 * There are two yellow LED available on the SAM V71 Xplained Ultra board that
 * can be turned on and off.  The LEDs can be activated by driving the
 * connected I/O line to GND.
 *
 *   ------ ----------- ---------------------
 *   SAMV71 Function    Shared functionality
 *   PIO
 *   ------ ----------- ---------------------
 *   PA23   Yellow LED0 EDBG GPIO
 *   PC09   Yellow LED1 LCD, and Shield
 *   ------ ----------- ---------------------
 */

#define GPIO_LED0     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                       GPIO_PORT_PIOA | GPIO_PIN23)
#define GPIO_LED1     (GPIO_OUTPUT | GPIO_CFG_DEFAULT | GPIO_OUTPUT_CLEAR | \
                       GPIO_PORT_PIOC | GPIO_PIN9)

/* Buttons
 *
 * SAM V71 Xplained Ultra contains three mechanical buttons. One button is the
 * RESET button connected to the SAM V71 reset line and the others are generic
 * user configurable buttons. When a button is pressed it will drive the I/O
 * line to GND.
 *
 *   ------ ----------- ---------------------
 *   SAMV71 Function    Shared functionality
 *   PIO
 *   ------ ----------- ---------------------
 *   RESET  RESET       Trace, Shield, and EDBG
 *   PA09   SW0         EDBG GPIO and Camera
 *   PB12   SW1         EDBG SWD and Chip Erase
 *   ------ ----------- ---------------------
 *
 * NOTES:
 *
 *   - There are no pull-up resistors connected to the generic user buttons so
 *     it is necessary to enable the internal pull-up in the SAM V71 to use the
 *     button.
 *   - PB12 is set up as a system flash ERASE pin when the firmware boots. To
 *     use the SW1, PB12 has to be configured as a normal regular I/O pin in
 *     the MATRIX module. For more information see the SAM V71 datasheet.
 */

#define GPIO_SW0      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOA | GPIO_PIN9)
#define GPIO_SW1      (GPIO_INPUT | GPIO_CFG_PULLUP | GPIO_CFG_DEGLITCH | \
                       GPIO_INT_BOTHEDGES | GPIO_PORT_PIOB | GPIO_PIN12)

#define IRQ_SW0       SAM_IRQ_PA9
#define IRQ_SW1       SAM_IRQ_PB12

/* HSMCI SD Card Detect
 * To be provided
 */

/* SPI Chip Selects
 * to be provided
 */

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
 * Name: sam_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM4E-EK board.
 *
 ************************************************************************************/

void weak_function sam_spiinitialize(void);

/************************************************************************************
 * Name: sam_hsmci_initialize
 *
 * Description:
 *   Initialize HSMCI support
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
int sam_hsmci_initialize(int minor);
#else
# define sam_hsmci_initialize(minor) (-ENOSYS)
#endif

/************************************************************************************
 * Name: sam_netinitialize
 *
 * Description:
 *   Configure board resources to support networking.
 *
 ************************************************************************************/

#ifdef HAVE_NETWORK
void weak_function sam_netinitialize(void);
#endif

/************************************************************************************
 * Name: sam_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected HSMCI slot
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_cardinserted(int slotno);
#else
#  define sam_cardinserted(slotno) (false)
#endif

/************************************************************************************
 * Name: sam_writeprotected
 *
 * Description:
 *   Check if the card in the MMCSD slot is write protected
 *
 ************************************************************************************/

#ifdef HAVE_HSMCI
bool sam_writeprotected(int slotno);
#else
#  define sam_writeprotected(slotno) (false)
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SAMV71_XULT_SRC_SAMV71_XULT_H */
