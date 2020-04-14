/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/src/stm32h747i-disco.h
 *
#   Copyright (C) 2017, 2019 Gwenhael Goavec-Merou. All rights reserved.
#   Authors: Gwenhael Goavec-Merou<gwenhael.goavec-merou@trabucayre.com>
 *           David Sidrane <david.sidrane@nscdg.com>
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

#ifndef __BOARDS_ARM_STM32H7_STM32H747I_DISCO_SRC_STM32H747I_DISCO_H
#define __BOARDS_ARM_STM32H7_STM32H747I_DISCO_SRC_STM32H747I_DISCO_H

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

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Check if we can support the RTC driver */

#define HAVE_RTC_DRIVER 1
#if !defined(CONFIG_RTC) || !defined(CONFIG_RTC_DRIVER)
#  undef HAVE_RTC_DRIVER
#endif

/* LED
 *
 * LD1 Green   PI12
 * LD2 Orange  PI13
 * LD3 Red     PI14
 * LD4 Blue    PI15
 */

#define GPIO_LD1        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                         GPIO_PORTI | GPIO_PIN12)
#define GPIO_LD2        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                         GPIO_PORTI | GPIO_PIN13)
#define GPIO_LD3        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                         GPIO_PORTI | GPIO_PIN14)
#define GPIO_LD4        (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_SET | \
                         GPIO_PORTI | GPIO_PIN15)

#define GPIO_LED_GREEN  GPIO_LD1
#define GPIO_LED_ORANGE GPIO_LD2
#define GPIO_LED_RED    GPIO_LD3
#define GPIO_LED_BLUE   GPIO_LD4

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "Wakeup", is connected to GPIO PC13.  A
 * high value will be sensed when the button is depressed.
 *
 * Notes:
 *    1) That the EXTI is included in the definition to enable an interrupt
 *       on this IO.
 *    2) The following definitions assume the default Solder Bridges are
 *       installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* SD/TF Card'detected pin */

#if defined(CONFIG_STM32H7_SDMMC1)
#  define HAVE_SDIO
#endif

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define GPIO_SDIO_NCD      (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTI|GPIO_PIN8)

#define SDIO_SLOTNO        0
#define SDIO_MINOR         0

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32H7_SPI
void stm32_spidev_initialize(void);
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
 * Name: stm32_dma_alloc_init
 *
 * Description:
 *   Called to create a FAT DMA allocator
 *
 * Returned Value:
 *   0 on success or -ENOMEM
 *
 ****************************************************************************/

#if defined (CONFIG_FAT_DMAMEMORY)
int stm32_dma_alloc_init(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

#ifdef HAVE_SDIO
int stm32_sdio_initialize(void);
#endif

#endif /* __BOARDS_ARM_STM32H7_STM32H747I_DISCO_SRC_STM32H747I_DISCO_H */
