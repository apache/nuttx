/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-aiotbox/src/gd32f450z_aiotbox.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H
#define __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H

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
/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_OUT1         (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_50MHZ | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_GD32_SDIO)
#  undef HAVE_SDIO
#endif

#define  SDIO_MINOR   0 /* Any minor number, default 0 */
#define  SDIO_SLOTNO  0 /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINIT=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int gd32_bringup(void);

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
#endif /* __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H */

#ifdef CONFIG_I2C
void gd32_i2c_initialize(void);
#endif
/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-aiotbox/src/gd32f450z_aiotbox.h
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

#ifndef __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H
#define __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H

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

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_IN1          (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#define GPIO_OUT1         (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_OUTPUT_SET | GPIO_CFG_SPEED_50MHZ | \
                           GPIO_CFG_PORT_B | GPIO_CFG_PIN_1)
#define GPIO_INT1         (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_GD32_SDIO)
#  undef HAVE_SDIO
#endif

#define  SDIO_MINOR   0 /* Any minor number, default 0 */
#define  SDIO_SLOTNO  0 /* Only one slot */

#ifdef HAVE_SDIO
#  if !defined(CONFIG_NSH_MMCSDSLOTNO)
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  elif CONFIG_NSH_MMCSDSLOTNO != 0
#    warning "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO SDIO_SLOTNO
#  endif

#  if defined(CONFIG_NSH_MMCSDMINOR)
#    define SDIO_MINOR CONFIG_NSH_MMCSDMINOR
#  else
#    define SDIO_MINOR 0
#  endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: gd32_bringup
 *
 * Description:
 *   Perform architecture specific initialization
 *
 *   CONFIG_BOARDCTL=y:
 *     If CONFIG_NSH_ARCHINIT=y:
 *       Called from the NSH library (or other application)
 *     Otherwise, assumed to be called from some other application.
 *
 *   Otherwise CONFIG_BOARD_LATE_INITIALIZE=y:
 *     Called from board_late_initialize().
 *
 *   Otherwise, bad news:  Never called
 *
 ****************************************************************************/

int gd32_bringup(void);

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
#endif /* __BOARDS_ARM_GD32F4_GD32F450ZK_AIOTBOX_SRC_GD32F450Z_AIOTBOX_H */

#ifdef CONFIG_I2C
void gd32_i2c_initialize(void);
#endif