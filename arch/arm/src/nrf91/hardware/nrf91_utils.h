/****************************************************************************
 * arch/arm/src/nrf91/hardware/nrf91_utils.h
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

#ifndef __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UTILS_H
#define __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes / Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.
 *
 ****************************************************************************/

void nrf91_clrpend(int irq);

/****************************************************************************
 * Name: nrf91_easydma_valid
 *
 * Description:
 *   Validate if easyDMA transfer is possible.
 *
 ****************************************************************************/

bool nrf91_easydma_valid(uint32_t addr);

/****************************************************************************
 * Name: nrf91_task_trigger
 *
 * Description:
 *   trigger the special task which is passed from task parameter
 *
 ****************************************************************************/

static inline void nrf91_task_trigger(uint32_t task)
{
  putreg32(1, task);
}

/****************************************************************************
 * Name: nrf91_event_clear
 *
 * Description:
 *   clear the Event
 *
 ****************************************************************************/

static inline void nrf91_event_clear(uint32_t event)
{
  putreg32(0, event);
}

/****************************************************************************
 * Name: nrf91_interrupt_enable
 *
 * Description:
 *   Enable the bitfield interrupt
 *
 ****************************************************************************/

static inline void nrf91_interrupt_enable(uint32_t reg_intenset,
                                          uint32_t bitfield)
{
  putreg32(bitfield, reg_intenset);
}

/****************************************************************************
 * Name: nrf91_interrupt_disable
 *
 * Description:
 *   Disable the bitfield interrupt
 *
 ****************************************************************************/

static inline void nrf91_interrupt_disable(uint32_t reg_intenclr,
                                           uint32_t bitfield)
{
  putreg32(bitfield, reg_intenclr);
}

#endif /* __ARCH_ARM_SRC_NRF91_HARDWARE_NRF91_UTILS_H */
