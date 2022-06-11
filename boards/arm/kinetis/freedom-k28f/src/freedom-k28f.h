/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/freedom-k28f.h
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

#ifndef __BOARDS_ARM_KINETIS_FREEDOM_K28F_SRC_FREEDOM_K28F_H
#define __BOARDS_ARM_KINETIS_FREEDOM_K28F_SRC_FREEDOM_K28F_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Application Configuration ************************************************/

/* Assume we have everything */

#define HAVE_MMCSD       1
#define HAVE_USB_MSC     1
#ifdef CONFIG_FRDMK28F_SDHC_AUTOMOUNT
#  define HAVE_SDHC_AUTOMOUNTER 1
#endif
#ifdef CONFIG_FRDMK28F_USB_AUTOMOUNT
#  define HAVE_USB_AUTOMOUNTER 1
#endif

/* SD card support */

#define MMCSD_SLOTNO 0

/* Can't support MMC/SD features if mountpoints are disabled or if SDHC
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_KINETIS_SDHC)
#  undef HAVE_MMCSD
#endif

#ifdef HAVE_MMCSD
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error Only one MMC/SD slot, slot 0
#  endif

#  ifdef CONFIG_NSH_MMCSDMINOR
#    define MMSCD_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define MMSCD_MINOR 0
#  endif

/* Check for USB_HOST and USB_MSC */
#if defined(CONFIG_DISABLE_MOUNTPOINT) || \
  !defined(CONFIG_KINETIS_USBHS) || !defined(CONFIG_USBHOST_MSC)
#  undef HAVE_USB_MSC
#endif

/* We expect to receive GPIO interrupts for card insertion events */

#  ifndef CONFIG_KINETIS_GPIOIRQ
#    error "CONFIG_KINETIS_GPIOIRQ required for card detect interrupt"
#  endif

#  ifndef CONFIG_KINETIS_PORTBINTS
#    error "CONFIG_KINETIS_PORTBINTS required for card detect interrupt"
#  endif

#endif

/* Automounter */

#if !defined(CONFIG_FS_AUTOMOUNTER) || !defined(HAVE_MMCSD)
#  undef HAVE_SDHC_AUTOMOUNTER
#endif

#if !defined(HAVE_USB_MSC) || !defined(CONFIG_USBHOST_MSC_NOTIFIER)
#  undef HAVE_USB_AUTOMOUNTER
#endif

/* Automounter defaults */

#ifdef HAVE_SDHC_AUTOMOUNTER

#  ifndef CONFIG_FRDMK28F_SDHC_AUTOMOUNT_FSTYPE
#    define CONFIG_FRDMK28F_SDHC_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_FRDMK28F_SDHC_AUTOMOUNT_BLKDEV
#    define CONFIG_FRDMK28F_SDHC_AUTOMOUNT_BLKDEV "/dev/mmcds0"
#  endif

#  ifndef CONFIG_FRDMK28F_SDHC_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_FRDMK28F_SDHC_AUTOMOUNT_MOUNTPOINT "/mnt/sdcard"
#  endif

#  ifndef CONFIG_FRDMK28F_SDHC_AUTOMOUNT_DDELAY
#    define CONFIG_FRDMK28F_SDHC_AUTOMOUNT_DDELAY 1000
#  endif

#  ifndef CONFIG_FRDMK28F_SDHC_AUTOMOUNT_UDELAY
#    define CONFIG_FRDMK28F_SDHC_AUTOMOUNT_UDELAY 2000
#  endif
#endif /* HAVE_SDHC_AUTOMOUNTER */

#ifdef HAVE_USB_AUTOMOUNTER

#  ifndef CONFIG_FRDMK28F_USB_AUTOMOUNT_FSTYPE
#    define CONFIG_FRDMK28F_USB_AUTOMOUNT_FSTYPE "vfat"
#  endif

#  ifndef CONFIG_FRDMK28F_USB_AUTOMOUNT_BLKDEV
#    define CONFIG_FRDMK28F_USB_AUTOMOUNT_BLKDEV "/dev/sd"
#  endif

#  ifndef CONFIG_FRDMK28F_USB_AUTOMOUNT_MOUNTPOINT
#    define CONFIG_FRDMK28F_USB_AUTOMOUNT_MOUNTPOINT "/mnt/usb"
#  endif

#  ifndef CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV
#    define CONFIG_FRDMK28F_USB_AUTOMOUNT_NUM_BLKDEV 4
#  endif

#  ifndef CONFIG_FRDMK28F_USB_AUTOMOUNT_UDELAY
#    define CONFIG_FRDMK28F_USB_AUTOMOUNT_UDELAY 2000
#  endif
#endif /* HAVE_USB_AUTOMOUNTER */

/* Freedom-K28F GPIOs *******************************************************/

/* A micro Secure Digital (SD) card slot is available on the FRDM-K28F
 * connected to the SD Host Controller (SDHC) signals of the MCU.
 * This slot will accept micro format SD memory cards.
 * The SD card detect pin (PTB5) is an open switch that shorts with VDD when
 * card is inserted.
 */

#define GPIO_SD_CARDDETECT (GPIO_INPUT | PIN_INT_BOTH | PIN_PORTB | PIN5)

/* An RGB LED is connected through GPIO as shown below:
 *
 *   LED    K28
 *   ------ -------------------------------------------------------
 *   RED    PTE6
 *   GREEN  PTE7
 *   BLUE   PTE8
 */

#define GPIO_LED_R (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN6)
#define GPIO_LED_G (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN7)
#define GPIO_LED_B (GPIO_LOWDRIVE | GPIO_OUTPUT_ZERO | PIN_PORTE | PIN8)

/* Two push buttons, SW2 and SW3, are available on FRDM-K28F board, where SW2
 * is connected to PTA4 and SW3 is connected to PTD0.
 * Besides the general purpose input/output functions, SW2 and SW3 can be
 * low-power wake up signal.
 * Also, only SW3 can be a non-maskable interrupt.
 *
 *   Switch    GPIO Function
 *   --------- --------------------------------------------------------------
 *   SW2       PTA4/NMI_B
 *   SW3       PTD0/LLWU_P12
 */

#define GPIO_SW2           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTA | PIN4)
#define GPIO_SW3           (GPIO_PULLUP | PIN_INT_BOTH | PIN_PORTD | PIN0)

/* A micro Secure Digital (SD) card slot is available on the FRDM-K28F
 * connected to the SD Host Controller (SDHC) signals of the MCU.
 * This slot will accept micro format SD memory cards.
 * The SD card detect pin (PTB5) is an open switch that shorts with VDD when
 * card is inserted.
 */

#define GPIO_SD_CARDDETECT (GPIO_INPUT | PIN_INT_BOTH | PIN_PORTB | PIN5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct i2c_master_s;  /* Forward reference */

#ifdef CONFIG_KINETIS_I2C0
extern struct i2c_master_s *g_i2c0_dev;
#endif
#ifdef CONFIG_KINETIS_I2C1
extern struct i2c_master_s *g_i2c1_dev;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: k28_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int k28_bringup(void);

/****************************************************************************
 * Name: k28_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the KwikStik-K40
 *   board.
 *
 ****************************************************************************/

void weak_function k28_spidev_initialize(void);

/****************************************************************************
 * Name: k28_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

void k28_i2cdev_initialize(void);

/****************************************************************************
 * Name: k28_usbdev_initialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the Freedom K28F board.
 *
 ****************************************************************************/

extern void weak_function k28_usbdev_initialize(void);

/****************************************************************************
 * Name: k28_usbhost_initialize
 *
 * Description:
 *   Inititialize USB High Speed Host
 *
 ****************************************************************************/

#if defined(CONFIG_KINETIS_USBHS) && defined(CONFIG_USBHOST)
int k28_usbhost_initialize(void);
#else
#  define k28_usbhost_initialize() (OK)
#endif

/****************************************************************************
 * Name: k28_sdhc_initialize
 *
 * Description:
 *   Inititialize the SDHC SD card slot
 *
 ****************************************************************************/

#ifdef HAVE_MMCSD
int k28_sdhc_initialize(void);
#else
#  define k28_sdhc_initialize() (OK)
#endif

/****************************************************************************
 * Name: k28_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the SDHC slot
 *
 ****************************************************************************/

#ifdef HAVE_SDHC_AUTOMOUNTER
bool k28_cardinserted(void);
#else
#  define k28_cardinserted() (false)
#endif

/****************************************************************************
 * Name: k28_writeprotected
 *
 * Description:
 *   Check if the card in the MMC/SD slot is write protected
 *
 ****************************************************************************/

#ifdef HAVE_SDHC_AUTOMOUNTER
bool k28_writeprotected(void);
#else
#  define k28_writeprotected() (false)
#endif

/****************************************************************************
 * Name:  k28_sdhc_automount_event
 *
 * Description:
 *   The SDHC card detection logic has detected an insertion or removal
 *   event.
 *   It has already scheduled the MMC/SD block driver operations.
 *   Now we need to schedule the auto-mount event which will occur with a
 *   substantial delay to make sure that everything has settle down.
 *
 * Input Parameters:
 *   inserted - True if the card is inserted in the slot.  False otherwise.
 *
 *  Returned Value:
 *    None
 *
 *  Assumptions:
 *    Interrupts are disabled.
 *
 ****************************************************************************/

#ifdef HAVE_SDHC_AUTOMOUNTER
void k28_sdhc_automount_event(bool inserted);
#endif

/****************************************************************************
 * Name:  k28_automount_initialize
 *
 * Description:
 *   Configure auto-mounter for the configured SDHC slot
 *
 * Input Parameters:
 *   None
 *
 *  Returned Value:
 *    None
 *
 ****************************************************************************/

#ifdef HAVE_SDHC_AUTOMOUNTER
void k28_automount_initialize(void);
#endif

/****************************************************************************
 * Name: k28_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int k28_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_FREEDOM_K28F_SRC_FREEDOM_K28F_H */
