/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_exti.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXTI_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXTI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>

#include "chip.h"
#include "hardware/gd32f4xx_exti.h"

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
 * Name: gd32_exti_gpioirq_init
 *
 * Description:
 *   Initialize the EXTI gpio irq.
 *
 * Input Parameters:
 *  - cfgset: GPIO pin
 *  - exti_mode: interrupt or event mode
 *  - trig_type: interrupt trigger type
 *  - irqnum: pointer to GPIO pin irq number
 *
 ****************************************************************************/

int gd32_exti_gpioirq_init(uint32_t cfgset, uint8_t exti_mode,
                           uint8_t trig_type, uint8_t *irqnum);

/****************************************************************************
 * Name: gd32_exti_gpio_irq_attach
 *
 * Description:
 *   Attach the EXTI gpio irq handler.
 *
 * Input Parameters:
 *  - irqpin: GPIO irq pin
 *  - irqhandler: irq handler
 *  - arg: Argument passed to the interrupt callback
 *
 ****************************************************************************/

int gd32_exti_gpio_irq_attach(uint8_t irqpin, xcpt_t irqhandler,
                              void *arg);

/****************************************************************************
 * Name: gd32_exti_init
 *
 * Description:
 *   Initialize the EXTI.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *  - exti_mode: interrupt or event mode
 *  - trig_type: interrupt trigger type
 *
 ****************************************************************************/

void gd32_exti_init(uint32_t linex, uint8_t exti_mode, uint8_t trig_type);

/****************************************************************************
 * Name: gd32_gpio_exti_linex_get
 *
 * Description:
 *   Get EXTI GPIO port and linex from GPIO pin.
 *
 ****************************************************************************/

int gd32_gpio_exti_linex_get(uint32_t cfgset, uint32_t *linex);

/****************************************************************************
 * Name: gd32_gpio_exti_linex_get
 *
 * Description:
 *   Get EXTI GPIO port and linex from GPIO pin.
 *
 ****************************************************************************/

int gd32_gpio_exti_irqnum_get(uint32_t cfgset, uint8_t *irqnum);

/****************************************************************************
 * Name: gd32_exti_interrupt_enable
 *
 * Description:
 *   Enable the interrupts from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_disable
 *
 * Description:
 *   Disable the interrupts from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_event_enable
 *
 * Description:
 *   Enable the events from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_event_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_event_disable
 *
 * Description:
 *   Disable the events from EXTI line x.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_event_disable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_software_interrupt_enable
 *
 * Description:
 *   Enable EXTI software interrupt event.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_enable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_software_interrupt_disable
 *
 * Description:
 *   Disable EXTI software interrupt event.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_software_interrupt_disable(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_get
 *
 * Description:
 *   Get EXTI lines flag when the interrupt flag is set.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 * Returned Value:
 *   status of flag (false or true)
 *
 ****************************************************************************/

bool gd32_exti_interrupt_flag_get(uint32_t linex);

/****************************************************************************
 * Name: gd32_exti_interrupt_flag_clear
 *
 * Description:
 *   Clear EXTI lines pending flag.
 *
 * Input Parameters:
 *  - linex: EXTI line number
 *
 ****************************************************************************/

void gd32_exti_interrupt_flag_clear(uint32_t linex);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_EXTI_H */
