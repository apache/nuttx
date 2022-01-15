/****************************************************************************
 * arch/avr/src/at90usb/at90usb.h
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

#ifndef __ARCH_AVR_SRC_AT90USB_AT90USB_H
#define __ARCH_AVR_SRC_AT90USB_AT90USB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "at90usb_config.h"

#ifdef CONFIG_AVR_GPIOIRQ
#  include <nuttx/irq.h>
#endif

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: up_clkinit
 *
 * Description:
 *   Initialize clock/PLL settings per the definitions in the board.h file.
 *
 ****************************************************************************/

void up_clkinitialize(void);

/****************************************************************************
 * Name: usart1_reset
 *
 * Description:
 *   Reset USART1.
 *
 ****************************************************************************/

void usart1_reset(void);

/****************************************************************************
 * Name: usart1_configure
 *
 * Description:
 *   Configure USART1.
 *
 ****************************************************************************/

void usart1_configure(void);

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Initialize a console for debug output.  This function is called very
 *   early in the initialization sequence to configure the serial console
 *   UART (only).
 *
 ****************************************************************************/

void up_consoleinit(void);

/****************************************************************************
 * Name: at90usb_boardinitialize
 *
 * Description:
 *   This function must be provided by the board-specific logic in the
 *   directory boards/avr/at90usb/<board-name>/src.
 *
 ****************************************************************************/

void at90usb_boardinitialize(void);

/****************************************************************************
 * Name: gpio_irqinitialize
 *
 * Description:
 *   Initialize all vectors to the unexpected interrupt handler
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 * Assumptions:
 *   Called during the early boot sequence before global interrupts have
 *   been enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
void weak_function gpio_irqinitialize(void);
#endif

/****************************************************************************
 * Name: gpio_irqattach
 *
 * Description:
 *   Attach in GPIO interrupt to the provide 'isr'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
int gpio_irqattach(int irq, xcpt_t newisr, xcpt_t *oldisr);
#endif

/****************************************************************************
 * Name: gpio_irqenable
 *
 * Description:
 *   Enable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
void gpio_irqenable(int irq);
#endif

/****************************************************************************
 * Name: gpio_irqdisable
 *
 * Description:
 *   Disable the GPIO IRQ specified by 'irq'
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_GPIOIRQ
void gpio_irqdisable(int irq);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AT90USB_AT90USB_H */
