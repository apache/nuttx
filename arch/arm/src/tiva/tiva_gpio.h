/****************************************************************************
 * arch/arm/src/tiva/tiva_gpio.h
 *
 *   Copyright (C) 2009-2010, 2013-2015, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * With modifications from Calvin Maguranis <calvin.maguranis@trd2inc.com>
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
