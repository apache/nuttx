/****************************************************************************
 * arch/arm/src/tiva/tiva_gpio.h
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

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H
#define __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>

/* Include chip specific definitions */

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include "lm/lm3s_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include "lm/lm4f_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_TM4C)
#  include "tm4c/tm4c_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_CC13X0) || defined(CONFIG_ARCH_CHIP_CC13X2)
#  include "cc13xx/cc13xx_gpio.h"
#else
#  error "Unsupported Tiva/Stellaris/SimpleLink GPIO"
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: tiva_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int tiva_configgpio(pinconfig_t pinconfig);

/****************************************************************************
 * Name: tiva_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void tiva_gpiowrite(pinconfig_t pinconfig, bool value);

/****************************************************************************
 * Name: tiva_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

bool tiva_gpioread(pinconfig_t pinconfig);

/****************************************************************************
 * Function:  tiva_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

int tiva_dumpgpio(pinconfig_t pinconfig, const char *msg);

/****************************************************************************
 * Name: tiva_gpio_lockport
 *
 * Description:
 *   Certain pins require to be unlocked from the NMI to use for normal GPIO
 *   use. See table 10-10 in datasheet for pins with special considerations.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_LM) || defined(CONFIG_ARCH_CHIP_TM4C)
void tiva_gpio_lockport(pinconfig_t pinconfig, bool lock);
#endif

/****************************************************************************
 * Function:  tiva_gpio_dumpconfig
 *
 * Description:
 *   Dump all GPIO registers associated with the provided base address
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_GPIO_INFO
void tiva_gpio_dumpconfig(pinconfig_t pinconfig);
#else
# define tiva_gpio_dumpconfig(p)
#endif

#ifdef CONFIG_TIVA_GPIO_IRQS
/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 ****************************************************************************/

int weak_function tiva_gpioirqinitialize(void);

/****************************************************************************
 * Name: tiva_gpioirqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provided 'isr'. If isr==NULL, then the
 *   irq_unexpected_isr handler is assigned and the pin's interrupt mask is
 *   disabled to stop further interrupts. Otherwise, the new isr is linked
 *   and the pin's interrupt mask is set.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   return to indicate the nature of the failure.
 *
 ****************************************************************************/

int tiva_gpioirqattach(pinconfig_t pinconfig, xcpt_t isr, void *arg);
#  define tiva_gpioirqdetach(p) tiva_gpioirqattach((p),NULL,NULL)

/****************************************************************************
 * Name: tiva_gpioirqenable
 *
 * Description:
 *   Enable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqenable(pinconfig_t pinconfig);

/****************************************************************************
 * Name: tiva_gpioirqdisable
 *
 * Description:
 *   Disable the GPIO port IRQ
 *
 ****************************************************************************/

void tiva_gpioirqdisable(pinconfig_t pinconfig);

/****************************************************************************
 * Name: tiva_gpioirqclear
 *
 * Description:
 *   Clears the interrupt status of the input base
 *
 ****************************************************************************/

void tiva_gpioirqclear(pinconfig_t pinconfig);

#endif /* CONFIG_TIVA_GPIO_IRQS */

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_GPIO_H */
