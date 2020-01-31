/****************************************************************************
 * arch/arm/src/nrf52/hardware/nrf52_utils.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Zhiqiang Li <levin.li@outlook.com>
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

#ifndef __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UTILS_H
#define __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes / Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_clrpend
 *
 * Description:
 *   Clear a pending interrupt at the NVIC.  This does not seem to be
 *   required for most interrupts.
 *
 ****************************************************************************/

void nrf52_clrpend(int irq);

/****************************************************************************
 * Name: nrf52832_errdata_init
 *
 * Description:
 *   ErrData correction for 52832
 *   required for most interrupts.
 *
 ****************************************************************************/

void nrf52832_errdata_init(void);

/****************************************************************************
 * Name: nrf52_task_trigger
 *
 * Description:
 *   trigger the special task which is passed from task parameter
 *
 ****************************************************************************/

static inline void nrf52_task_trigger(uint32_t task)
{
  putreg32(1, task);
}

/****************************************************************************
 * Name: nrf52_event_clear
 *
 * Description:
 *   clear the Event
 *
 ****************************************************************************/

static inline void nrf52_event_clear(uint32_t event)
{
  putreg32(0, event);
}

/****************************************************************************
 * Name: nrf52_interrupt_enable
 *
 * Description:
 *   Enable the bitfield interrupt
 *
 ****************************************************************************/

static inline void nrf52_interrupt_enable(uint32_t reg_intenset,
                                          uint32_t bitfield)
{
  putreg32(bitfield, reg_intenset);
}

/****************************************************************************
 * Name: nrf52_interrupt_disable
 *
 * Description:
 *   Disable the bitfield interrupt
 *
 ****************************************************************************/

static inline void nrf52_interrupt_disable(uint32_t reg_intenclr,
                                           uint32_t bitfield)
{
  putreg32(bitfield, reg_intenclr);
}

#endif /* __ARCH_ARM_SRC_NRF52_HARDWARE_NRF52_UTILS_H */
