/****************************************************************************
 * configs/olimex-stm32-p407/src/olimex-stm32-p407.h
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from a file of a similar name for the Olimex STM32 P207:
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
 ****************************************************************************/

#ifndef __CONFIGS_OLIMEX_STM32_P407_SRC_H
#define __CONFIGS_OLIMEX_STM32_P407_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Assume that we support everything until convinced otherwise */

#define HAVE_MMCSD      1
#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_ELF        1
#define HAVE_MODSYMS    1

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO support
 * is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO) || \
   !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_MMCSD
#endif

/* Default MMC/SD minor number */

#ifdef HAVE_MMCSD

/* Default MMC/SD SLOT number */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error Only one MMC/SD slot
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif

#  ifdef CONFIG_NSH_MMCSDSLOTNO
#    define MMCSD_SLOTNO CONFIG_NSH_MMCSDSLOTNO
#  else
#    define MMCSD_SLOTNO 0
#  endif

/* Default minor device number */

#  ifdef CONFIG_NSH_MMCSDMINOR
#    define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define MMCSD_MINOR 0
#  endif
#endif

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif

/* ELF */

#if defined(CONFIG_BINFMT_DISABLE) || !defined(CONFIG_ELF)
#  undef HAVE_ELF
#endif

/* Module symbol table */

#if !defined(CONFIG_EXAMPLES_MODULE) || defined(CONFIG_BUILD_FLAT)
#  undef HAVE_MODSYMS
#endif

#ifdef HAVE_MODSYMS
#  define MODSYMS_NSYMBOLS_VAR g_mod_nexports
#  define MODSYMS_SYMTAB_ARRAY g_mod_exports
#endif

/* Olimex-STM32-P407 GPIOs **************************************************/
/* LEDs */

#define GPIO_LED1         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN6)
#define GPIO_LED2         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN7)
#define GPIO_LED3         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN8)
#define GPIO_LED4         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTF|GPIO_PIN9)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON     BUTTON_TAMPER
#define MAX_IRQBUTTON     BUTTON_CENTER
#define NUM_IRQBUTTONS    7

#define GPIO_BTN_TAMPER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_BTN_WKUP     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_RIGHT    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN6)
#define GPIO_BTN_UP       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN7)
#define GPIO_BTN_LEFT     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN11)
#define GPIO_BTN_DOWN     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN8)
#define GPIO_BTN_CENTER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN15)

/* DHTxx pin configuration */

#define GPIO_DHTXX_PIN          (GPIO_PORTG|GPIO_PIN9)
#define GPIO_DHTXX_PIN_OUTPUT   (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_DHTXX_PIN)
#define GPIO_DHTXX_PIN_INPUT    (GPIO_INPUT|GPIO_FLOAT|GPIO_DHTXX_PIN)

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC2  OTG_FS_PowerSwitchOn
 * PB10 OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN2)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN10)
#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_PORTB|GPIO_PIN10)
#endif

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_bringup
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
 ************************************************************************************/

int stm32_bringup(void);

/************************************************************************************
 * Name: stm32_stram_configure
 *
 * Description:
 *   Initialize to access external SRAM.  SRAM will be visible at the FSMC Bank
 *   NOR/SRAM2 base address (0x64000000)
 *
 *   General transaction rules.  The requested AHB transaction data size can be 8-,
 *   16- or 32-bit wide whereas the SRAM has a fixed 16-bit data width. Some simple
 *   transaction rules must be followed:
 *
 *   Case 1: AHB transaction width and SRAM data width are equal
 *     There is no issue in this case.
 *   Case 2: AHB transaction size is greater than the memory size
 *     In this case, the FSMC splits the AHB transaction into smaller consecutive
 *     memory accesses in order to meet the external data width.
 *   Case 3: AHB transaction size is smaller than the memory size.
 *     SRAM supports the byte select feature.
 *     a) FSMC allows write transactions accessing the right data through its
 *        byte lanes (NBL[1:0])
 *     b) Read transactions are allowed (the controller reads the entire memory
 *        word and uses the needed byte only). The NBL[1:0] are always kept low
 *        during read transactions.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_stram_configure(void);
#endif

/************************************************************************************
 * Name: stm32_usb_configure
 *
 * Description:
 *   Called from stm32_boardinitialize very early in inialization to setup USB-related
 *   GPIO pins for the Olimex STM32 P407 board.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usb_configure(void);
#endif

/************************************************************************************
 * Name: stm32_usbhost_setup
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device connection/
 *   disconnection events.
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_setup(void);
#endif

/************************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ************************************************************************************/

#ifdef CONFIG_ADC
int stm32_adc_setup(void);
#endif

/****************************************************************************
 * Name: stm32_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_CAN
int stm32_can_setup(void);
#endif

/************************************************************************************
 * Name: stm32_dhtxx_initialize
 *
 * Description:
 *   Called to initialize the DHTxx sensor
 *
 ************************************************************************************/

#ifdef CONFIG_SENSORS_DHTXX
int stm32_dhtxx_initialize(FAR const char *devpath);
#endif

#endif  /* __ASSEMBLY__ */
#endif /* __CONFIGS_OLIMEX_STM32_P407_SRC_H */
