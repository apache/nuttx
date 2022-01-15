/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/src/olimex-stm32-p407.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_P407_SRC_OLIMEX_STM32_P407_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_P407_SRC_OLIMEX_STM32_P407_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

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

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
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
#endif

/* Can't support USB device if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

/* ELF */

#if defined(CONFIG_BINFMT_DISABLE) || !defined(CONFIG_ELF)
#  undef HAVE_ELF
#endif

/* Module symbol table */

#if !defined(CONFIG_EXAMPLES_MODULE) || !defined(CONFIG_BUILD_FLAT)
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

#ifdef CONFIG_INPUT_DJOYSTICK
#  define MIN_IRQBUTTON     BUTTON_TAMPER
#  define MAX_IRQBUTTON     BUTTON_WKUP
#  define NUM_IRQBUTTONS    2
#else
#  define MIN_IRQBUTTON     BUTTON_TAMPER
#  define MAX_IRQBUTTON     JOYSTICK_CENTER
#  define NUM_IRQBUTTONS    7
#endif

#define GPIO_BTN_TAMPER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_BTN_WKUP     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

#define GPIO_JOY_RIGHT    (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN6)
#define GPIO_JOY_UP       (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN7)
#define GPIO_JOY_LEFT     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN11)
#define GPIO_JOY_DOWN     (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN8)
#define GPIO_JOY_CENTER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN15)

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

/* The CS4344 depends on the CS4344 driver and I2S3 support */

#if defined(CONFIG_AUDIO_CS4344) && defined(CONFIG_STM32_I2S3)
#  define HAVE_CS4344
#endif

#ifdef HAVE_CS4344
  /* The CS4344 transfers data on I2S3 */

#  define CS4344_I2S_BUS      3
#endif

/* External ST7735 Pins */

#define GPIO_ST7735_CS     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)
#define GPIO_ST7735_AO     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN10)
#define GPIO_ST7735_RST    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                            GPIO_OUTPUT_SET|GPIO_PORTG|GPIO_PIN12)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
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

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the olimex-stm32-p407
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_stram_configure
 *
 * Description:
 *   Initialize to access external SRAM.  SRAM will be visible at the FSMC
 *   Bank NOR/SRAM2 base address (0x64000000)
 *
 *   General transaction rules.  The requested AHB transaction data size can
 *   be 8-, 16- or 32-bit wide whereas the SRAM has a fixed 16-bit data
 *   width. Some simple transaction rules must be followed:
 *
 *   Case 1: AHB transaction width and SRAM data width are equal
 *     There is no issue in this case.
 *   Case 2: AHB transaction size is greater than the memory size
 *     In this case, the FSMC splits the AHB transaction into smaller
 *     consecutive memory accesses in order to meet the external data width.
 *   Case 3: AHB transaction size is smaller than the memory size.
 *     SRAM supports the byte select feature.
 *     a) FSMC allows write transactions accessing the right data through its
 *        byte lanes (NBL[1:0])
 *     b) Read transactions are allowed (the controller reads the entire
 *        memory word and uses the needed byte only). The NBL[1:0] are always
 *        kept low during read transactions.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_stram_configure(void);
#endif

/****************************************************************************
 * Name: stm32_usb_configure
 *
 * Description:
 *   Called from stm32_boardinitialize very early in inialization to setup
 *   USB-related GPIO pins for the Olimex STM32 P407 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usb_configure(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_setup
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_setup(void);
#endif

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

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

#ifdef CONFIG_STM32_CAN_CHARDRIVER
int stm32_can_setup(void);
#endif

/****************************************************************************
 * Name: stm32_cs4344_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the CS4344 device.  This function will register the driver
 *   as /dev/audio/pcm[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_CS4344
int stm32_cs4344_initialize(int minor);
#endif

/****************************************************************************
 * Name: stm32_djoy_initialize
 *
 * Description:
 *   Initialize and register the discrete joystick driver
 *
 ****************************************************************************/

#ifdef CONFIG_INPUT_DJOYSTICK
int stm32_djoy_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_P407_SRC_OLIMEX_STM32_P407_H */
