/****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/src/imxrt1050-evk.h
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_IMXRT_IMXRT1050_EVK_SRC_IMXRT1050_EVK_H
#define __BOARDS_ARM_IMXRT_IMXRT1050_EVK_SRC_IMXRT1050_EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs */

#define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN9)       /* AD_BO_09 */

/* Buttons ****************************************************************/

#define GPIO_SW8       (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                        IOMUX_SW_DEFAULT | \
                        GPIO_PORT5 | GPIO_PIN0 | )              /* WAKEUP */

/* Test Pins **************************************************************/

#define BOARD_NGPIOIN   0 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT  4 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT  0 /* Amount of GPIO Input w/ Interruption pins */

#define GPIO_GOUT1      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PORT1 | GPIO_PIN19)

#define GPIO_GOUT2      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN18 | GPIO_PORT1)

#define GPIO_GOUT3      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN10 | GPIO_PORT1)

#define GPIO_GOUT4      (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | IOMUX_GOUT_DEFAULT | \
                         GPIO_PIN9 | GPIO_PORT1)

/* Backlight */

#define GPIO_LCD_BL     (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT2 | \
                         GPIO_PIN31 | IOMUX_LCD_BL_DEFAULT)

/* Ethernet */

#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | \
                         GPIO_PORT1 | GPIO_PIN10)                    /* AD_B0_10 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO1_10
#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT1 | GPIO_PIN9 | IOMUX_ENET_RST_DEFAULT)

#ifdef CONFIG_ETH0_PHY_KSZ8081
#ifdef GPIO_LED
#warning LED interferes with ETH reset unless R323 is removed.
#endif
#endif

/* LPSPI CS: */

#define IOMUX_LPSPI3_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI3_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT1 | GPIO_PIN3 | IOMUX_LPSPI3_CS) /* GPIO_AD_B0_03 */

/* LPSPI1 CS:  GPIO_SD_B0_01 */

#define IOMUX_LPSPI1_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_LPSPI1_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT3 | GPIO_PIN13 | IOMUX_LPSPI1_CS)

#define IOMUX_MMCSD_EN  (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#define GPIO_MMCSD_EN   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT3 | GPIO_PIN2 | IOMUX_MMCSD_EN)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the i.MXRT1050 EVK.
 *
 ****************************************************************************/

void imxrt_spidev_initialize(void);

/*****************************************************************************
 * Name: imxrt_mmcsd_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int imxrt_mmcsd_spi_initialize(int minor);
#endif

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void imxrt_autoled_initialize(void);
#endif

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int imxrt_gpio_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1050_EVK_SRC_IMXRT1050_EVK_H */
