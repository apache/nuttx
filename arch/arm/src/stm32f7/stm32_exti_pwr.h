/****************************************************************************
 * arch/arm/src/stm32f7/stm32_exti_pwr.h
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

#ifndef __ARCH_ARM_SRC_STM32F7_STM32_EXTI_PWR_H
#define __ARCH_ARM_SRC_STM32F7_STM32_EXTI_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_exti_pvd
 *
 * Description:
 *   Sets/clears EXTI PVD interrupt.
 *
 * Input Parameters:
 *  - rising/falling edge: enables interrupt on rising/falling edge
 *  - event:  generate event when set
 *  - func:   when non-NULL, generate interrupt
 *  - arg:    Argument passed to the interrupt callback
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int stm32_exti_pvd(bool risingedge, bool fallingedge, bool event,
                   xcpt_t func, void *arg);

#endif /* __ARCH_ARM_SRC_STM32F7_STM32_EXTI_PWR_H */
