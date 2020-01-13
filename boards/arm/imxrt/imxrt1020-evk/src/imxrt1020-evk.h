/****************************************************************************
 * boards/arm/imxrt/imxrt1020-evk/src/imxrt1020-evk.h
 *
 *   Copyright (C) 2018-2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Dave Marples <dave@marples.net>
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

#ifndef __BOARDS_ARM_IMXRT_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H
#define __BOARDS_ARM_IMXRT_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H

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

/* i.MX RT 1020 GPIO Pin Definitions ****************************************/

/* LEDs *********************************************************************/

/* There is one user accessible LED status indicator located on the 1020-EVK.
 * The function of the LEDs include:
 *
 * D3: Power (Green) & Overpower (Red)
 * D5: User LED (Green) GPIO_AD_B0_05
 * D15: RST LED (Red)
 */

#define GPIO_USERLED    (IOMUX_LED_DEFAULT | GPIO_OUTPUT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN5)  /* AD_B0_05 */

/* Buttons ******************************************************************/

/* The IMXRT board has three external buttons
 *
 * 1. SW2 (IRQ88, ONOFF)  Not on a GPIO, No muxing
 * 2. SW3 (IRQ88, POR)    Not on a GPIO, No muxing
 * 2. SW4 (IRQ88, USER)   Wakeup, GPIO5-0
 */


#define GPIO_SWWAKE     (GPIO_INTERRUPT | GPIO_INT_FALLINGEDGE | \
                         IOMUX_SWWAKE_DEFAULT | GPIO_PORT5 | GPIO_PIN0)  /* WAKE */


/* ETH Disambiguation ********************************************************/

#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | GPIO_INTERRUPT | \
                         GPIO_INT_FALLINGEDGE |	GPIO_PORT1 | GPIO_PIN22) /* AD_B1_06 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO1_12
#define GPIO_ENET_RST   (GPIO_OUTPUT | IOMUX_ENET_RST_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT1 | GPIO_PIN4 )  /* AD_B0_04, Inverted logic */

/* USBOTG *********************************************************************/

#define GPIO_USBOTG_ID  (GPIO_USB_OTG_ID_1 | IOMUX_USBOTG_ID_DEFAULT)      /* AD_B1_11 */
#define GPIO_USBOTG_PWR (GPIO_USB_OTG_PWR_1 | IOMUX_USBOTG_PWR_DEFAULT)    /* AD_B1_10 */
#define GPIO_USBOTG_OC  (GPIO_USB_OTG_OC_1 | IOMUX_USBOTG_OC_DEFAULT)      /* AD_B1_12 */

/* USDHC **********************************************************************/

#define PIN_USDHC1_CD   (IOMUX_VSD_DEFAULT | \
                         GPIO_PORT3 | GPIO_PIN19 )                         /* SD_B0_06 */
#define GPIO_VSDHIGH    (GPIO_OUTPUT | IOMUX_VSD_DEFAULT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT1 | GPIO_PIN22)                          /* AD_B1_07 */
#define PIN_USDHC1_PWREN (GPIO_OUTPUT | IOMUX_VSD_DEFAULT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT3 | GPIO_PIN24  )                        /* SD_B1_04 */

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
 *   Called to configure SPI chip select GPIO pins for the versiboard2
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

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

#ifdef CONFIG_DEV_GPIO
int imxrt_gpio_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_usbhost_initialize
 *
 * Description:
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int imxrt_usbhost_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_IMXRT_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H */
