/****************************************************************************
 * arch/risc-v/src/gap8/gap8_tim.h
 * PIN driver for GAP8
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: hhuysqt <1020988872@qq.com>
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

/****************************************************************************
 *  GAP8 features a 64-bit basic timer, able to split into 2 32-bit timers,
 *  with identicle memory map and 2 IRQ channels, for both FC core and
 *  cluster. We would use it as system timer.
 ****************************************************************************/

#ifndef __ARCH_RISC_V_SRC_GAP8_TIM_H
#define __ARCH_RISC_V_SRC_GAP8_TIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gap8.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_timer_initialize
 *
 * Description:
 *   Initialize the timer based on the frequency of source clock and ticks
 *   per second.
 *
 ****************************************************************************/

void gap8_timer_initialize(uint32_t source_clock, uint32_t tick_per_second);

/****************************************************************************
 * Name: gap8_register_timercallback
 *
 * Description:
 *   Register a callback function called on timer IRQ
 *
 ****************************************************************************/

void gap8_register_timercallback(void (*on_timer)(void*arg), void *arg);

/****************************************************************************
 * Name: gap8_timer_isr
 *
 * Description:
 *   ISR for timer
 *
 ****************************************************************************/

void gap8_timer_isr(void);

#endif /* __ARCH_RISC_V_SRC_GAP8_TIM_H */
