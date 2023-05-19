/****************************************************************************
 * boards/arm/stm32l4/nucleo-l496zg/src/nucleo-144.h
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

#ifndef __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_SRC_NUCLEO_144_H
#define __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_SRC_NUCLEO_144_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include "stm32l4_gpio.h"

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

/* Nucleo-144 GPIO Pin Definitions ******************************************/

/* LED
 *
 * The Nucleo-144 board has numerous LEDs but only three, LD1 a Green LED,
 * LD2 a Blue LED and LD3 a Red LED, that can be controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LD1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTC | GPIO_PIN7)
#define GPIO_LD2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN7)
#define GPIO_LD3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | \
                        GPIO_PORTB | GPIO_PIN14)

#define GPIO_LED_GREEN GPIO_LD1
#define GPIO_LED_BLUE  GPIO_LD2
#define GPIO_LED_RED   GPIO_LD3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The Blue pushbutton B1, labeled "User", is connected to GPIO PC13.
 * A high value will be sensed when the button is depressed.
 * Note:
 *    1) That the EXTI is included in the definition to enable an interrupt
 *       on this IO.
 *    2) The following definitions assume the default Solder Bridges are
 *       installed.
 */

#define GPIO_BTN_USER  (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTC | GPIO_PIN13)

/* SPI **********************************************************************/

#define GPIO_SPI_CS    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                        GPIO_OUTPUT_SET)

#define GPIO_SPI1_CS0  (GPIO_SPI_CS | GPIO_PORTD | GPIO_PIN14)
#define GPIO_SPI1_CS1  (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN15)
#define GPIO_SPI1_CS2  (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN14)
#define GPIO_SPI1_CS3  (GPIO_SPI_CS | GPIO_PORTC | GPIO_PIN2)
#define GPIO_SPI2_CS0  (GPIO_SPI_CS | GPIO_PORTD | GPIO_PIN7)
#define GPIO_SPI2_CS1  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN1)
#define GPIO_SPI2_CS2  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN2)
#define GPIO_SPI2_CS3  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN3)
#define GPIO_SPI3_CS0  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN4)
#define GPIO_SPI3_CS1  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN5)
#define GPIO_SPI3_CS2  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN6)
#define GPIO_SPI3_CS3  (GPIO_SPI_CS | GPIO_PORTG | GPIO_PIN7)

#if defined(CONFIG_STM32L4_SDMMC1) || defined(CONFIG_STM32L4_SDMMC2)
#  define HAVE_SDIO
#endif

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_MMCSD_SDIO)
#  undef HAVE_SDIO
#endif

#define SDIO_SLOTNO 0  /* Only one slot */

#ifdef HAVE_SDIO
#  if defined(CONFIG_STM32L4_SDMMC1)
#    define GPIO_SDMMC1_NCD (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTC | GPIO_PIN6)
#  endif

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && (CONFIG_NSH_MMCSDSLOTNO != 0)
#    warning "Only one MMC/SD slot, slot 0"
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/* USB OTG FS
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PG6  OTG_FS_PowerSwitchOn
 * PG5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS   (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)

#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN6)

#ifdef CONFIG_USBHOST
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_EXTI|GPIO_FLOAT|\
                           GPIO_SPEED_100MHz|GPIO_PUSHPULL|\
                           GPIO_PORTG|GPIO_PIN5)

#else
#  define GPIO_OTGFS_OVER (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTG|GPIO_PIN5)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI)
void stm32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_spidev_bus_test
 *
 * Description:
 *   Called to create the defined SPI buses and test them by initializing
 *   them and sending the NUCLEO_SPI_TEST (no chip select).
 *
 ****************************************************************************/

#if defined(CONFIG_NUCLEO_SPI_TEST)
int stm32_spidev_bus_test(void);
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

void stm32_dma_alloc_init(void);

#if defined (CONFIG_FAT_DMAMEMORY)
int stm32_dma_alloc_init(void);
#endif

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Called at application startup time to initialize the SCMMC
 *   functionality.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int stm32_sdio_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the nucleo-144 board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32L4_OTGFS
void stm32_usbinitialize(void);
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
 * Name: stm32_dac_setup
 *
 * Description:
 *   Initialize DAC and register the DAC driver.
 *
 ****************************************************************************/

#ifdef CONFIG_DAC
int stm32_dac_setup(void);
#endif

/****************************************************************************
 * Name: stm32_dfsdm_setup
 *
 * Description:
 *   Initialize DFSDM and register the ADC drivers for DFSDM filters.
 *
 ****************************************************************************/

#if defined(CONFIG_ADC) && defined(CONFIG_STM32L4_DFSDM)
int stm32_dfsdm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32L4_NUCLEO_L496ZG_SRC_NUCLEO_144_H */
