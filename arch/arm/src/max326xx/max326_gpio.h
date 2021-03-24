/****************************************************************************
 * arch/arm/src/max326xx/max326_gpio.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_GPIO_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/max326_gpio.h"

/* Bit-encoded input to max326_gpio_config() ********************************/

#if defined(CONFIG_ARCH_FAMILY_MAX32620) || defined(CONFIG_ARCH_FAMILY_MAX32630)
#  include "max32620_30/max32620_30_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_MAX32660)
#  include "max32660/max32660_gpio.h"
#else
#  error "Unsupported MAX326XX family"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: max326_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is
 *   called by the OS initialization logic and is not a user interface.
 *
 * Assumptions:
 *   Called early in the boot-up sequence
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqinitialize(void);
#endif

/****************************************************************************
 * Name: max326_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 * Assumptions:
 *   - The pin interrupt has been disabled and all interrupt related bits
 *     have been set to zero by max436_gpio_config().
 *   - We are called in a critical section.
 *
 ****************************************************************************/

int max326_gpio_config(max326_pinset_t pinset);

/****************************************************************************
 * Name: max326_gpio_irqconfig
 *
 * Description:
 *   Configure a pin for interrupt operation.  This function should not be
 *   called directory but, rather, indirectly through max326_gpio_config().
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqconfig(max326_pinset_t pinset);
#endif

/****************************************************************************
 * Name: max326_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void max326_gpio_write(max326_pinset_t pinset, bool value);

/****************************************************************************
 * Name: max326_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool max326_gpio_read(max326_pinset_t pinset);

/****************************************************************************
 * Name: max326_gpio_irqdisable
 *
 * Description:
 *   Disable a GPIO pin interrupt.  This function should not be called
 *   directly but, rather through up_disable_irq();
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqdisable(int irq);
#endif

/****************************************************************************
 * Name: max326_gpio_irqenable
 *
 * Description:
 *   Enable a GPIO pin interrupt.  This function should not be called
 *   directly but, rather through up_enable_irq();
 *
 ****************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqenable(int irq);
#endif

/****************************************************************************
 * Function:  max326_gpio_dump
 *
 * Description:
 *   Decode and dump all GPIO registers associated with the port and pin
 *   numbers in the provided pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int max326_gpio_dump(max326_pinset_t pinset, const char *msg);
#else
#  define max326_gpio_dump(p,m)
#endif

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_MAX326XX_MAX326_GPIO_H */
