/****************************************************************************
 * arch/risc-v/src/gap8/gap8_fll.h
 * GAP8 FLL clock generator
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
 *  FC can run up to 250MHz@1.2V, and 150MHz@1.0V. While the default voltage
 *  of PMU is 1.2V, it's okay to boost up without considering PMU.
 ****************************************************************************/

#ifndef __ARCH_RISC_V_SRC_GAP8_FLL_H
#define __ARCH_RISC_V_SRC_GAP8_FLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gap8.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gap8_setfreq
 *
 * Description:
 *   Set frequency up to 250MHz. Input frequency in Hz.
 *
 ****************************************************************************/

void gap8_setfreq(uint32_t frequency);

/****************************************************************************
 * Name: gap8_getfreq
 *
 * Description:
 *   Get current system clock frequency in Hz.
 *
 ****************************************************************************/

uint32_t gap8_getfreq(void);

#endif /* __ARCH_RISC_V_SRC_GAP8_FLL_H */
