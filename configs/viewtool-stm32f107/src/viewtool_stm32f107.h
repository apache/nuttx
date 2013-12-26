/******************************************************************************
 * configs/viewtool-stm32f107/src/viewtool_stm32f107.h
 *
 *   Copyright (C) 2013 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mholtzberg@uvc-ingenieure.de>
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
 ******************************************************************************/
#ifndef __CONFIGS_VIEWTOOL_STM32F107_SRC_INTERNAL_H
#define __CONFIGS_VIEWTOOL_STM32F107_SRC_INTERNAL_H

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/
/* Configuration **************************************************************/
/* Assume that everything is supported */

#define HAVE_USBDEV   1
#define HAVE_MMCSD    1

/* Handle chip differences */

#if defined(CONFIG_ARCH_CHIP_STM32F103VCT6)
#  undef CONFIG_STM32_OTGFS
#elif defined(CONFIG_ARCH_CHIP_STM32F107VC)
#  undef CONFIG_STM32_USB
#  undef CONFIG_STM32_SDIO
#else
#  error Unknown chip on Viewtool board
#  undef HAVE_USBDEV
#  undef HAVE_MMCSD
#endif

/* Check if USB is enabled */

#if !defined(CONFIG_STM32_OTGFS) && !defined(CONFIG_STM32_USB)
#  undef HAVE_USBDEV
#elif !defined(CONFIG_USBDEV)
#  warning CONFIG_STM32_OTGFS (F107) or CONFIG_STM32_USB (F103) is enabled but CONFIG_USBDEV is not
#  undef HAVE_USB
#endif

/* Can't support MMC/SD features if the SDIO peripheral is disabled */

#ifndef CONFIG_STM32_SDIO
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD slot number/device minor number */

#define VIEWTOOL_MMCSD_SLOTNO 0

/* GPIO Configuration *********************************************************/
/* LEDs
 *
 * There are four LEDs on the ViewTool STM32F103/F107 board that can be controlled
 * by software:  LED1 through LED4.  All pulled high and can be illuminated by
 * driving the output to low
 *
 *   LED1 PA6
 *   LED2 PA7
 *   LED3 PB12
 *   LED4 PB13
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN6)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN7)
#define GPIO_LED3       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LED4       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN13)


/* Buttons **************************************************************************/
/* All pulled high and will be sensed low when depressed.
 *
 *   SW2 PC11  Needs J42 closed
 *   SW3 PC12  Needs J43 closed
 *   SW4 PA0   Needs J44 closed
 */

#define MIN_IRQBUTTON   BUTTON_SW2
#define MAX_IRQBUTTON   BUTTON_SW4
#define NUM_IRQBUTTONS  (BUTTON_SW4 - BUTTON_SW2 + 1)

#define GPIO_SW2        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTC | GPIO_PIN11)
#define GPIO_SW3        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTC | GPIO_PIN12)
#define GPIO_SW4        (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | \
                         GPIO_EXTI | GPIO_PORTA | GPIO_PIN10)

/* microSD Card Interface
 *
 * microSD Connector
 * -----------------
 *
 *   ----------------------------- ------------------------- --------------------------------
 *          Connector J17            GPIO CONFIGURATION(s)
 *   PIN SIGNAL        LEGEND          (no remapping)                 DP83848C Board
 *   --- ------------- ----------- ------------------------- --------------------------------
 *   1   VDD 3.3       N/A         N/A                       3.3
 *   2   GND           N/A         N/A                       GND
 *   3   PC8           SDIO_D0     GPIO_SDIO_D0              D0
 *   4   PD2           SDIO_CMD    GPIO_SDIO_CMD             CMD
 *   5   PC12          SDIO_CLK    GPIO_SDIO_CK              CLK
 *   6   PC11          SDIO_D3     GPIO_SDIO_D3              D3
 *   7   PC10          SDIO_D2     GPIO_SDIO_D2              D2
 *   8   PC9           SDIO_D1     GPIO_SDIO_D1              D1
 *   9   PA8           CD          Board-specific GPIO input CD
 *   --- ------------- ----------- ------------------------- --------------------------------
 *
 *   NOTES:
 *   1. The STM32F107 does not support the SDIO/memory card interface.  So the SD card
 *      cannot be used with the STM32F107 (unless the pin-out just happens to match up
 *      with an SPI-based card interface???)
 */

#ifdef CONFIG_ARCH_CHIP_STM32F103VCT6
#  define GPIO_SD_CD      (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN8)
#endif

/* USB
 *
 * The Viewtool base board has a USB Mini-B connector.  Only USB device can
 * be supported with this connector.
 *
 *  ------------------------- ------------------------------------
 *        USB Connector
 *        J10 mini-USB       GPIO CONFIGURATION(s)
 * --- --------- ----------- ------------------------------------
 * Pin Signal
 * --- --------- ----------- ------------------------------------
 *  1  USB_VBUS  VDD_USB     (No sensing available)
 *  2  OTG_DM    PA11        GPIO_OTGFS_DM (F107) GPIO_USB_DM (F103)
 *  3  OTG_DP    PA12        GPIO_OTGFS_DP (F107) GPIO_USB_DP (F103)
 *  4  OTG_ID    PA10        GPIO_OTGFS_ID (F107)
 *  5  Shield    N/A         N/A
 *  6  Shield    N/A         N/A
 *  7  Shield    N/A         N/A
 *  8  Shield    N/A         N/A
 *  9  Shield    N/A         N/A
 *               PE11 USB_EN   GPIO controlled soft pull-up (if J51 closed)
 *
 *  NOTES:
 *  1. GPIO_OTGFS_VBUS (F107) should not be configured.  No VBUS sensing
 *  2. GPIO_OTGFS_SOF (F107) is not used
 *  3. The OTG FS module has is own, internal soft pull-up logic.  J51 should
 *     be open so that PE11 activity does effect USB.
 */

#ifdef CONFIG_ARCH_CHIP_STM32F103VCT6
#  define GPIO_USB_PULLUP (GPIO_OUTPUT|GPIO_CNF_OUTOD|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN11)
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the M3 Wildfire board.
 *
 ************************************************************************************/

void weak_function stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbdev_initialize
 *
 * Description:
 *   Called from stm32_usbdev_initialize very early in initialization to setup USB-related
 *   GPIO pins for the Viewtool STM32F107 board.
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBDEV)
void weak_function stm32_usbdev_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ************************************************************************************/

int stm32_sdinitialize(int minor);

/************************************************************************************
 * Name: up_ledinit
 *
 * Description:
 *   Configure LEDs.  LEDs are left in the OFF state.
 *
 ************************************************************************************/

void stm32_ledinit(void);

#endif  /* __ASSEMBLY__ */
#endif /* __CONFIGS_VIEWTOOL_STM32F107_SRC_INTERNAL_H */
