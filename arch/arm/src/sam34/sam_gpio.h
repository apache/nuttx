/************************************************************************************
 * arch/arm/src/sam34/sam_gpio.h
 *
 *   Copyright (C) 2009-2011, 2013-2014 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM34_SAM_GPIO_H
#define __ARCH_ARM_SRC_SAM34_SAM_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_SAM3U)
#  include "sam3u_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
#  include "sam3x_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4E)
#  include "sam4e_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  include "sam4l_gpio.h"
#elif defined(CONFIG_ARCH_CHIP_SAM4S)
#  include "sam4s_gpio.h"
#else
#  error Unrecognized SAM architecture
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#if defined(CONFIG_SAM34_GPIOA_IRQ) || defined(CONFIG_SAM34_GPIOB_IRQ) || \
    defined(CONFIG_SAM34_GPIOC_IRQ)
#  define CONFIG_SAM34_GPIO_IRQ 1
#else
#  undef CONFIG_SAM34_GPIO_IRQ
#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_GPIO
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/************************************************************************************
 * Name: sam_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for GPIO pins.
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqinitialize(void);
#else
#  define sam_gpioirqinitialize()
#endif

/************************************************************************************
 * Name: sam_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ************************************************************************************/

int sam_configgpio(gpio_pinset_t cfgset);

/************************************************************************************
 * Name: sam_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void sam_gpiowrite(gpio_pinset_t pinset, bool value);

/************************************************************************************
 * Name: sam_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool sam_gpioread(gpio_pinset_t pinset);

/************************************************************************************
 * Name: sam_gpioirq
 *
 * Description:
 *   Configure an interrupt for the specified GPIO pin.
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirq(gpio_pinset_t pinset);
#else
#  define sam_gpioirq(pinset)
#endif

/************************************************************************************
 * Name: sam_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqenable(int irq);
#else
#  define sam_gpioirqenable(irq)
#endif

/************************************************************************************
 * Name: sam_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 ************************************************************************************/

#ifdef CONFIG_SAM34_GPIO_IRQ
void sam_gpioirqdisable(int irq);
#else
#  define sam_gpioirqdisable(irq)
#endif

/************************************************************************************
 * Function:  sam_dumpgpio
 *
 * Description:
 *   Dump all GPIO registers associated with the base address of the provided pinset.
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_GPIO
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
