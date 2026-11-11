/****************************************************************************
 * arch/arm/src/rzv2h/rzv2h_gpio.h
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

#ifndef __ARCH_ARM_SRC_RZV2H_RZV2H_GPIO_H
#define __ARCH_ARM_SRC_RZV2H_RZV2H_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/rzv2h_gpio.h"
#include "hardware/rzv2h_pinmap.h"

/* Include external hal fsp renesas */
#include "r_ioport.h"
#include "bsp_io.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Must be big enough to hold the 32-bit encoding */

typedef struct gpio_pinset
{
  bsp_io_port_pin_t port_pin; /* Physical pin */
  uint32_t cfg;               /* FSP configuration bits */
} gpio_pinset_t;

extern const ioport_instance_t g_ioport;

/****************************************************************************
 * Name: rzv2h_configgpio
 *
 * Description:
 *   Configure a GPIO pin using the bit-encoded pin configuration supplied
 *   by the caller.
 ****************************************************************************/

int rzv2h_configgpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: rzv2h_gpiowrite
 *
 * Description:
 *   Write a logic high or low value to the selected GPIO pin.
 ****************************************************************************/

int rzv2h_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: rzv2h_gpioread
 *
 * Description:
 *   Read the current logic level from the selected GPIO pin.
 ****************************************************************************/

bool rzv2h_gpioread(gpio_pinset_t pinset);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RZV2H_RZV2H_GPIO_H */
