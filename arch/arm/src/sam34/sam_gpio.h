/****************************************************************************
 * arch/arm/src/sam34/sam_gpio.h
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

#ifndef __ARCH_ARM_SRC_SAM34_SAM_GPIO_H
#define __ARCH_ARM_SRC_SAM34_SAM_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  include "sam3u_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  include "sam3x_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4CM)
#  include "sam4cm_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4E)
#  include "sam4e_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  include "sam4l_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4S)
#  include "sam4s_gpio.h"
#else
#  error Unrecognized SAM architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(CONFIG_SAM34_GPIOA_IRQ) || defined(CONFIG_SAM34_GPIOB_IRQ) || \
    defined(CONFIG_SAM34_GPIOC_IRQ) || defined(CONFIG_SAM34_GPIOD_IRQ) || \
    defined(CONFIG_SAM34_GPIOE_IRQ) || defined(CONFIG_SAM34_GPIOF_IRQ)
#  define CONFIG_SAM34_GPIO_IRQ 1
#else
#  undef CONFIG_SAM34_GPIO_IRQ
#endif

/****************************************************************************
 * Public Function Prototypes
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
 * Name: sam_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqinitialize(void);
#else
#  define sam_gpioirqinitialize()
#endif

/****************************************************************************
 * Name: sam_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int sam_configgpio(gpio_pinset_t cfgset);

/****************************************************************************
 * Name: sam_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void sam_gpiowrite(gpio_pinset_t pinset, bool value);

/****************************************************************************
 * Name: sam_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool sam_gpioread(gpio_pinset_t pinset);

/****************************************************************************
 * Name: sam_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirq(gpio_pinset_t pinset);
#else
#  define sam_gpioirq(pinset)
#endif

/****************************************************************************
 * Name: sam_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqenable(int irq);
#else
#  define sam_gpioirqenable(irq)
#endif

/****************************************************************************
 * Name: sam_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqdisable(int irq);
#else
#  define sam_gpioirqdisable(irq)
#endif

/****************************************************************************
 * Function:  sam_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided
 *   pinset.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
int sam_dumpgpio(uint32_t pinset, const char *msg);
#else
#  define sam_dumpgpio(p,m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_SAM34_SAM_GPIO_H */
