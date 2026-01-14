/****************************************************************************
 * arch/avr/src/avrdx/avrdx_gpio.h
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

#ifndef __ARCH_AVR_SRC_AVRDX_AVRDX_GPIO_H
#define __ARCH_AVR_SRC_AVRDX_AVRDX_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Mapping of port name to numeric index alphabetically.
 * Ports missing on a specific device are not skipped.
 */

#define AVRDX_GPIO_PORTA_IDX 0
#define AVRDX_GPIO_PORTC_IDX 2
#define AVRDX_GPIO_PORTD_IDX 3
#define AVRDX_GPIO_PORTF_IDX 5

#ifdef CONFIG_AVR_HAS_PORTB
#  define AVRDX_GPIO_PORTB_IDX 1
#endif

#ifdef CONFIG_AVR_HAS_PORTE
#  define AVRDX_GPIO_PORTE_IDX 4
#endif

#ifdef CONFIG_AVR_HAS_PORTG
#  define AVRDX_GPIO_PORTG_IDX 6
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN const IOBJ uint8_t avrdx_gpio_irq_vectors[];

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: avrdx_gpio_isr_mux_init
 *
 * Description:
 *   Initialize global variable holding ISR data
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_AVRDX_GPIO_ISR_MUX
void avrdx_gpio_isr_mux_init(void);
#endif

/****************************************************************************
 * Name: avrdx_irq_attach_gpio_mux
 *
 * Description:
 *   Attaches given handler and arg to the GPIO IRQ multiplexer.
 *
 * Input Parameters:
 *   Requested port
 *   Pins handled by this handler
 *   Handler itself
 *   Opaque arg argument
 *   ISC - input sense configuration (ie. what pin change triggers
 *     the interrupt. See eg. AVR128DA28 datasheet, chapter PORT
 *     I/O pin configuration, section Register Description, subsection
 *     Pin n Control (17.5.16 in rev C of the document)
 *
 * Returned Value:
 *   OK or EBUSY if another handler already uses the pin. EINVAL for port
 *   that does not exist.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_AVRDX_GPIO_ISR_MUX
int avrdx_irq_attach_gpio_mux(uint8_t port_idx, uint8_t pins,
                              xcpt_t handler, void *arg,
                              uint8_t isc);
#endif

/****************************************************************************
 * Name: avrdx_irq_detach_gpio_mux
 *
 * Description:
 *   Detaches handler from GPIO IRQ multiplexer.
 *
 * Input Parameters:
 *   Requested port
 *   Pins to be detached. Removes handler completely if it no longer
 *     services any pin. (Except if keep_handler flag is set)
 *   Keep handler flag. See above.
 *
 * Returned Value:
 *   OK. EINVAL when detaching port that was not in use or other
 *   incorrect input.
 *
 ****************************************************************************/

#ifdef CONFIG_AVR_AVRDX_GPIO_ISR_MUX
int avrdx_irq_detach_gpio_mux(uint8_t port_idx, uint8_t pins,
                              bool keep_handler);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_AVR_SRC_AVRDX_AVRDX_GPIO_H */
