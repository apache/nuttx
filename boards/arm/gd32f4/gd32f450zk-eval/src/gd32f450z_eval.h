/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f450z_eval.h
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

#ifndef __BOARDS_ARM_GD32F4_GD32F450ZK_EVAL_SRC_GD32F450Z_EVAL_H
#define __BOARDS_ARM_GD32F4_GD32F450ZK_EVAL_SRC_GD32F450Z_EVAL_H

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
#    define GD32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define GD32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

#ifdef CONFIG_FS_NXFFS
#  ifndef CONFIG_GD32F4_NXFFS_MOUNTPT
#    define CONFIG_GD32F4_NXFFS_MOUNTPT "/mnt/gd32nxffs"
#  endif
#endif

/* GD32F450ZK GPIO Pin Definitions ******************************************/

/* LED
 *
 * The GD32F450ZK-EVAL board has three LEDs, LED1, LED2 and LED3, that can be
 * controlled by software.
 * The following definitions assume the default Solder Bridges are installed.
 */

#define GPIO_LED1       (GPIO_CFG_PORT_D | GPIO_CFG_OUTPUT_RESET | GPIO_PIN4_OUTPUT)
#define GPIO_LED2       (GPIO_CFG_PORT_D | GPIO_CFG_OUTPUT_RESET | GPIO_PIN5_OUTPUT)
#define GPIO_LED3       (GPIO_CFG_PORT_G | GPIO_CFG_OUTPUT_RESET | GPIO_PIN3_OUTPUT)

#define LED1            GPIO_LED1
#define LED2            GPIO_LED2
#define LED3            GPIO_LED3

#define LED_DRIVER_PATH "/dev/userleds"

/* BUTTONS
 *
 * The GD32F450Z Eval board has User, Tamper, and Wakeup key, the are
 * connected to GPIO PB14, PC13 and PA0.
 * A low value will be sensed when the button is depressed.
 *
 * Note:
 *   That the EXTI is included in the definition to enable
 *       an interrupt on this IO.
 */

#define GPIO_BTN_USER      (GPIO_CFG_PORT_B | GPIO_CFG_EXTI | GPIO_PIN14_INPUT)
#define GPIO_BTN_TAMPER    (GPIO_CFG_PORT_C | GPIO_CFG_EXTI | GPIO_PIN13_INPUT)
#define GPIO_BTN_WAKEUP    (GPIO_CFG_PORT_A | GPIO_CFG_EXTI | GPIO_PIN0_INPUT)

/* SPI Flash ****************************************************************/

/* SPI FLASH (GD25Q40)
 *
 *  PG12  SPI5_MISO
 *  PG14  SPI5_MOSI
 *  PG13  SPI5_SCK
 *
 *  PG9   SPI5_CS
 */

#define GPIO_SPI5_CS      (GPIO_CFG_PORT_G | GPIO_PIN9_OUTPUT)
#define SPI_FLASH_CSNUM   5

#define HAVE_GD25         1

#if !defined(CONFIG_MTD_GD25) || !defined(CONFIG_GD32F4_SPI5)
#  undef HAVE_GD25
#endif

/* Can't support AT24 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || \
   !defined(CONFIG_GD32F450ZK_EVAL_GD25_BLOCKMOUNT)
#  undef HAVE_GD25
#endif

#define HAVE_AT24         1

/* AT24 Serial EEPROM
 *
 * A AT24C02C Serial EEPPROM was used for tested I2C0.
 */

#define AT24_BUS          0
#define AT24_MINOR        0

#if !defined(CONFIG_MTD_AT24XX) || !defined(CONFIG_GD32F4_I2C0)
#  undef HAVE_AT24
#endif

/* Can't support AT24 features if mountpoints are disabled or if we were not
 * asked to mount the AT25 part
 */

#ifndef CONFIG_GD32F450ZK_EVAL_AT24_TEST
#  undef HAVE_AT24
#endif

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_OUT1         (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_50MHZ | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the GD32F450Z
 *   Eval board.
 *
 ****************************************************************************/

#if defined(CONFIG_SPI)
void gd32_spidev_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_gd25_automount
 *
 * Description:
 *   Initialize, configure, and mount the GD25 SPI FLASH.  The FLASH will
 *   be mounted at /dev/gd25.
 *
 ****************************************************************************/

#ifdef HAVE_GD25
int gd32_gd25_automount(int minor);
#endif

/****************************************************************************
 * Name: gd32_at24_wr_test
 *
 * Description:
 *   Write and read the AT24 serial EEPROM test.
 *
 ****************************************************************************/

#ifdef HAVE_AT24
int gd32_at24_wr_test(int minor);
#endif

/****************************************************************************
 * Name: gd32_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int gd32_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: gd32_dma_alloc_init
 *
 * Description:
 *   Called to create a FAT DMA allocator
 *
 * Returned Value:
 *   0 on success or -ENOMEM
 *
 ****************************************************************************/

void gd32_dma_alloc_init(void);

#if defined (CONFIG_FAT_DMAMEMORY)
int gd32_dma_alloc_init(void);
#endif

/****************************************************************************
 * Name: gd32_sdio_initialize
 *
 * Description:
 *   Called at application startup time to initialize the SCMMC
 *   functionality.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD
int gd32_sdio_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_GD32F4_GD32F450ZK_EVAL_SRC_GD32F450Z_EVAL_H */
