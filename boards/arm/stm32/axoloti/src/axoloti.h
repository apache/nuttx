/****************************************************************************
 * boards/arm/stm32/axoloti/src/axoloti.h
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

#ifndef __BOARDS_ARM_STM32_AXOLOTI_SRC_AXOLOTI_H
#define __BOARDS_ARM_STM32_AXOLOTI_SRC_AXOLOTI_H

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

/****************************************************************************
 * configuration
 */

/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_SDIO       1
#define HAVE_ADAU1961   1
#define HAVE_SDRAM      1

/* Can't support USB host if USB OTG HS is not enabled */

#if !defined(CONFIG_STM32_OTGHS) || !defined(CONFIG_USBHOST)
#  undef HAVE_USBHOST
#endif

/* Can't support USB device if USB OTG FS is not enabled */

#if !defined(CONFIG_STM32_OTGFS) || !defined(CONFIG_USBDEV)
#  undef HAVE_USBDEV
#endif

/* Can't support MMC/SD features if mountpoints or SDIO support are
 * disabled
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef HAVE_SDIO
#endif

/* The ADAU1961 depends on the ADAU1961 driver, I2C3, and SAI1 support */

#if !defined(CONFIG_AUDIO_ADAU1961) || !defined(CONFIG_STM32_I2C3) || \
    !defined(CONFIG_STM32_SAI1)
#  undef HAVE_ADAU1961
#endif

/* Can't support SDRAM if the memory controller is disabled */

#if !defined(CONFIG_STM32_FMC)
#  undef HAVE_SDRAM
#endif

/****************************************************************************
 * Audio Configuration
 */

#define ADAU1961_I2C_BUS 3      /* i2c3 */
#define ADAU1961_I2C_ADDRESS 0x38
#define ADAU1961_SAI_BUS SAI1_BLOCK_A

/****************************************************************************
 * SDIO Configuration
 */

#define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#define SDIO_SLOTNO CONFIG_NSH_MMCSDSLOTNO

/* SD Slot Card detect */

#define GPIO_SDIO_NCD (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN13)

/****************************************************************************
 * PROC File System Configuration
 */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * LEDs
 */

#define GPIO_LED1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                   GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)

#define GPIO_LED2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                   GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)

/****************************************************************************
 * Buttons
 */

#define MIN_IRQBUTTON BUTTON_USER
#define MAX_IRQBUTTON BUTTON_USER
#define NUM_IRQBUTTONS  1
#define GPIO_BTN_USER (GPIO_INPUT|GPIO_PULLDOWN|GPIO_EXTI|GPIO_PORTA|GPIO_PIN10)

/****************************************************************************
 * USB Host (OTG High Speed)
 */

#define GPIO_OTGHS_PWRON (GPIO_OUTPUT|GPIO_OUTPUT_SET|GPIO_FLOAT| \
                          GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN7)

#define GPIO_OTGHS_OVER  (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT| \
                          GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN13)

/* #define GPIO_OTGHS_VBUS no vbus monitoring.... */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_STM32_SDIO)
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the STM32F4Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_OTGHS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_OTGHS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_adau1961_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize the adau1961 audio driver.
 *
 ****************************************************************************/

#if defined(HAVE_ADAU1961)
int stm32_adau1961_initialize(int minor);
#endif

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *
 ****************************************************************************/

#if defined(HAVE_SDRAM)
int stm32_sdram_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_AXOLOTI_SRC_AXOLOTI_H */
