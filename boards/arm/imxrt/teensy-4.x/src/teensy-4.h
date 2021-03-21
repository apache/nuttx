/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/teensy-4.h
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

#ifndef __BOARDS_ARM_IMXRT_TEENSY_4X_SRC_TEENSY_4_H
#  define __BOARDS_ARM_IMXRT_TEENSY_4X_SRC_TEENSY_4_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#  include <nuttx/config.h>

#  include <stdint.h>
#  include <stdbool.h>

#  include <arch/irq.h>
#  include <nuttx/irq.h>

#  include "imxrt_gpio.h"
#  include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* i.MX RT 1060 GPIO Pin Definitions ****************************************/

/* LEDs */

/* There are two LED status indicators located on the Teensy 4.x board.
 * The functions of these LEDs include:
 *
 *   - RED LED (loading status)
 *      - dim:    ready
 *      - bright: writing
 *      - blink:  no USB
 *   - USER LED (D3)
 *
 * Only a single LED, D3, is under software control.
 */

#  define GPIO_LED        (GPIO_OUTPUT | IOMUX_LED_DEFAULT | \
                         GPIO_OUTPUT_ZERO | GPIO_PORT2 | GPIO_PIN3)  /* BO_03 */

#  define LED_DRIVER_PATH "/dev/userleds"

/* LPSPI3 CS:  GPIO_AD_B1_12 */

#  define IOMUX_LPSPI3_CS (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                         IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                         _IOMUX_PULL_ENABLE)
#  define GPIO_LPSPI3_CS  (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                         GPIO_PORT1 | GPIO_PIN28 | IOMUX_LPSPI3_CS)

/* LPSPI4 CS:  GPIO_B0_00  */

#  define IOMUX_LPSPI4_CS      (IOMUX_SLEW_FAST | IOMUX_DRIVE_50OHM | \
                              IOMUX_SPEED_MEDIUM | IOMUX_PULL_UP_100K | \
                              _IOMUX_PULL_ENABLE)
#  define GPIO_LPSPI4_CS       (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT2 | GPIO_PIN0 | IOMUX_LPSPI4_CS)

/* LCD dispay */

#  define GPIO_LCD_RST        (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT2 | GPIO_PIN18 | IOMUX_LPSPI4_CS)    /* B1_02 */

#  define GPIO_LCD_CD         (GPIO_OUTPUT | GPIO_OUTPUT_ONE | \
                              GPIO_PORT2 | GPIO_PIN19 | IOMUX_LPSPI4_CS)    /* B1_03 */

/* USB OTG ID Pin： GPIO_AD_B1_02 */

#define GPIO_USBOTG_ID  (GPIO_USB_OTG1_ID_1 | IOMUX_USBOTG_ID_DEFAULT)      /* AD_B1_02 */

/* Ethernet */

/* Ethernet Interrupt: GPIO_B0_15
 *
 * This pin has a week pull-up within the PHY, is open-drain, and requires
 * an external 1k ohm pull-up resistor (present on the EVK).  A falling
 * edge then indicates a change in state of the PHY.
 */

#define GPIO_ENET_INT   (IOMUX_ENET_INT_DEFAULT | \
                         GPIO_PORT2 | GPIO_PIN15)    /* B0_15 */
#define GPIO_ENET_IRQ   IMXRT_IRQ_GPIO2_15

/* Ethernet Reset:  GPIO_B0_14
 *
 * The #RST uses inverted logic.  The initial value of zero will put the
 * PHY into the reset state.
 */

#define GPIO_ENET_RST   (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | \
                         GPIO_PORT2 | GPIO_PIN14 | IOMUX_ENET_RST_DEFAULT) /* B0_14 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#  ifndef __ASSEMBLY__

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#    if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#    endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the i.MXRT1050 EVK.
 *
 ****************************************************************************/

void imxrt_spidev_initialize(void);

/****************************************************************************
 * Name: imxrt_can_setup
 *
 * Description:
 *  Initialize CAN and register the CAN device
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXCAN
int imxrt_can_setup(void);
#endif

/****************************************************************************
 * Name: imxrt_adc_initialize
 *
 * Description:
 *   Initialize ADC drivers
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ADC
int imxrt_adc_initialize(void);
#endif

/****************************************************************************
 * Name: imxrt_i2c_setup
 *
 * Description:
 *  Choose which I2C driver should be initialize
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LPI2C
void imxrt_i2c_setup(void);
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

#endif  /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_TEENSY_4X_SRC_TEENSY_4_H */
