/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_gpio.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_GPIO_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#include "chip.h"
#include "hardware/nrf54l_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bit-encoded input to nrf54l_gpio_config() ********************************/

/* 32-bit encoding:
 *
 *   Bits  0-4:  Pin number (0-31)
 *   Bits  5-6:  Port number (0-3)
 *   Bit     7:  Initial output value
 *   Bits  8-9:  Pull configuration
 *   Bits 10-13: Drive configuration
 *   Bits 14-15: Sense configuration
 *   Bits 16-17: Input/output function
 */

#define GPIO_FUNC_SHIFT         (16)    /* Bits 16-17: GPIO mode */
#define GPIO_FUNC_MASK          (0x03 << GPIO_FUNC_SHIFT)
#  define GPIO_INPUT            (0x00 << GPIO_FUNC_SHIFT)  /* GPIO input pin */
#  define GPIO_OUTPUT           (0x01 << GPIO_FUNC_SHIFT)  /* GPIO output pin */

#define GPIO_SENSE_SHIFT        (14)     /* Bits 14-15: Pin sense mode */
#define GPIO_SENSE_MASK         (0x3 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_NONE       (0 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_HIGH       (2 << GPIO_SENSE_SHIFT)
#  define GPIO_SENSE_LOW        (3 << GPIO_SENSE_SHIFT)

#define GPIO_DRIVE_SHIFT        (10)     /* Bits 10-13: Pin drive mode */
#define GPIO_DRIVE_MASK         (0xf << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_S0S1       (0 << GPIO_DRIVE_SHIFT)  /* Standard '0', standard '1' */
#  define GPIO_DRIVE_H0S1       (1 << GPIO_DRIVE_SHIFT)  /* High drive '0', standard '1' */
#  define GPIO_DRIVE_S0H1       (2 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_H0H1       (3 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_D0S1       (4 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_D0H1       (5 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_S0D1       (6 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_H0D1       (7 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_EOS1       (8 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_SOE1       (9 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_EOE1       (10 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_DOE1       (11 << GPIO_DRIVE_SHIFT)
#  define GPIO_DRIVE_EOD1       (12 << GPIO_DRIVE_SHIFT)

#define GPIO_MODE_SHIFT         (8)      /* Bits 8-9: Pin pull-up mode */
#define GPIO_MODE_MASK          (0x3 << GPIO_MODE_SHIFT)
#  define GPIO_FLOAT            (0 << GPIO_MODE_SHIFT) /* Neither pull-up nor -down */
#  define GPIO_PULLDOWN         (1 << GPIO_MODE_SHIFT) /* Pull-down resistor enabled */
#  define GPIO_PULLUP           (2 << GPIO_MODE_SHIFT) /* Pull-up resistor enabled */

#define GPIO_VALUE              (1 << 7)  /* Bit 7: Initial GPIO output value */
#  define GPIO_VALUE_ONE        GPIO_VALUE
#  define GPIO_VALUE_ZERO       (0)

#define GPIO_PORT_SHIFT         (5)       /* Bits 5-6: Port number */
#define GPIO_PORT_MASK          (0x3 << GPIO_PORT_SHIFT)
#  define GPIO_PORT0            (0 << GPIO_PORT_SHIFT)
#  define GPIO_PORT1            (1 << GPIO_PORT_SHIFT)
#  define GPIO_PORT2            (2 << GPIO_PORT_SHIFT)
#  define GPIO_PORT3            (3 << GPIO_PORT_SHIFT)

#define GPIO_PIN_SHIFT          0         /* Bits 0-4: GPIO number: 0-31 */
#define GPIO_PIN_MASK           (0x1f << GPIO_PIN_SHIFT)
#  define GPIO_PIN0             (0  << GPIO_PIN_SHIFT)
#  define GPIO_PIN1             (1  << GPIO_PIN_SHIFT)
#  define GPIO_PIN2             (2  << GPIO_PIN_SHIFT)
#  define GPIO_PIN3             (3  << GPIO_PIN_SHIFT)
#  define GPIO_PIN4             (4  << GPIO_PIN_SHIFT)
#  define GPIO_PIN5             (5  << GPIO_PIN_SHIFT)
#  define GPIO_PIN6             (6  << GPIO_PIN_SHIFT)
#  define GPIO_PIN7             (7  << GPIO_PIN_SHIFT)
#  define GPIO_PIN8             (8  << GPIO_PIN_SHIFT)
#  define GPIO_PIN9             (9  << GPIO_PIN_SHIFT)
#  define GPIO_PIN10            (10 << GPIO_PIN_SHIFT)
#  define GPIO_PIN11            (11 << GPIO_PIN_SHIFT)
#  define GPIO_PIN12            (12 << GPIO_PIN_SHIFT)
#  define GPIO_PIN13            (13 << GPIO_PIN_SHIFT)
#  define GPIO_PIN14            (14 << GPIO_PIN_SHIFT)
#  define GPIO_PIN15            (15 << GPIO_PIN_SHIFT)
#  define GPIO_PIN16            (16 << GPIO_PIN_SHIFT)
#  define GPIO_PIN17            (17 << GPIO_PIN_SHIFT)
#  define GPIO_PIN18            (18 << GPIO_PIN_SHIFT)
#  define GPIO_PIN19            (19 << GPIO_PIN_SHIFT)
#  define GPIO_PIN20            (20 << GPIO_PIN_SHIFT)
#  define GPIO_PIN21            (21 << GPIO_PIN_SHIFT)
#  define GPIO_PIN22            (22 << GPIO_PIN_SHIFT)
#  define GPIO_PIN23            (23 << GPIO_PIN_SHIFT)
#  define GPIO_PIN24            (24 << GPIO_PIN_SHIFT)
#  define GPIO_PIN25            (25 << GPIO_PIN_SHIFT)
#  define GPIO_PIN26            (26 << GPIO_PIN_SHIFT)
#  define GPIO_PIN27            (27 << GPIO_PIN_SHIFT)
#  define GPIO_PIN28            (28 << GPIO_PIN_SHIFT)
#  define GPIO_PIN29            (29 << GPIO_PIN_SHIFT)
#  define GPIO_PIN30            (30 << GPIO_PIN_SHIFT)
#  define GPIO_PIN31            (31 << GPIO_PIN_SHIFT)
#  define GPIO_PIN(n)           ((n) << GPIO_PIN_SHIFT)

/* Helper macros */

#define GPIO_PIN_DECODE(p)  (((p) & GPIO_PIN_MASK)  >> GPIO_PIN_SHIFT)

#define GPIO_PORT_DECODE(p) (((p) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef uint32_t nrf54l_pinset_t;

enum nrf54l_gpio_detectmode_e
{
  NRF54L_GPIO_DETECTMODE_DETECT,
  NRF54L_GPIO_DETECTMODE_LDETECT,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf54l_gpio_config(nrf54l_pinset_t cfgset);

/****************************************************************************
 * Name: nrf54l_gpio_unconfig
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int nrf54l_gpio_unconfig(nrf54l_pinset_t cfgset);

/****************************************************************************
 * Name: nrf54l_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void nrf54l_gpio_write(nrf54l_pinset_t pinset, bool value);

/****************************************************************************
 * Name: nrf54l_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool nrf54l_gpio_read(nrf54l_pinset_t pinset);

/****************************************************************************
 * Name: nrf54l_gpio_detectmode
 *
 * Description:
 *   Select normal or latched pin detection for a GPIO port.
 *
 ****************************************************************************/

void nrf54l_gpio_detectmode(int port, enum nrf54l_gpio_detectmode_e mode);

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_GPIO_H */
