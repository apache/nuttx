/************************************************************************************
 * arch/arm/src/max326xx/max326_gpio.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_MAX326XX_MAX326_GPIO_H
#define __ARCH_ARM_SRC_MAX326XX_MAX326_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "hardware/max326_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Bit-encoded input to max326_gpio_config() ****************************************/

#if defined(CONFIG_ARCH_FAMILY_MAX32620) || defined(CONFIG_ARCH_FAMILY_MAX32630)
#  include "max32620_30/max32620_30_gpio.h"
#elif defined(CONFIG_ARCH_FAMILY_MAX32660)
#  include "max32660/max32660_gpio.h"
#else
#  error "Unsupported MAX326XX family"
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: max326_gpio_irqinitialize
 *
 * Description:
 *   Initialize logic to support interrupting GPIO pins.  This function is called by
 *   the OS initialization logic and is not a user interface.
 *
 * Assumptions:
 *   Called early in the boot-up sequence
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqinitialize(void);
#endif

/************************************************************************************
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
 ************************************************************************************/

int max326_gpio_config(max326_pinset_t pinset);

/************************************************************************************
 * Name: max326_gpio_irqconfig
 *
 * Description:
 *   Configure a pin for interrupt operation.  This function should not be called
 *   directory but, rather, indirectly through max326_gpio_config().
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqconfig(max326_pinset_t pinset);
#endif

/************************************************************************************
 * Name: max326_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/

void max326_gpio_write(max326_pinset_t pinset, bool value);

/************************************************************************************
 * Name: max326_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/

bool max326_gpio_read(max326_pinset_t pinset);

/************************************************************************************
 * Name: max326_gpio_irqdisable
 *
 * Description:
 *   Disable a GPIO pin interrupt.  This function should not be called directly but,
 *   rather through up_disable_irq();
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqdisable(int irq);
#endif

/************************************************************************************
 * Name: max326_gpio_irqenable
 *
 * Description:
 *   Enable a GPIO pin interrupt.  This function should not be called directly but,
 *   rather through up_enable_irq();
 *
 ************************************************************************************/

#ifdef CONFIG_MAX326XX_GPIOIRQ
void max326_gpio_irqenable(int irq);
#endif

/************************************************************************************
 * Function:  max326_gpio_dump
 *
 * Description:
 *   Decode and dump all GPIO registers associated with the port and pin
 *   numbers in the provided pinset.
 *
 ************************************************************************************/

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
