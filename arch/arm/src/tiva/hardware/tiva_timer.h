/************************************************************************************
 * arch/arm/src/tiva/hardware/tiva_timer.h
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_TIMER_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/* Include the timer header file for the specific Tiva/Stellaris/SimpleLink chip */

#if defined(CONFIG_ARCH_CHIP_LM3S)
#  include "hardware/lm/lm3s_timer.h"
#elif defined(CONFIG_ARCH_CHIP_LM4F)
#  include "hardware/lm/lm4f_timer.h"
#elif defined(CONFIG_ARCH_CHIP_TM4C123)
#  include "hardware/tm4c/tm4c123_timer.h"
#elif defined(CONFIG_ARCH_CHIP_TM4C129)
#  include "hardware/tm4c/tm4c129_timer.h"
#elif defined(CONFIG_ARCH_CHIP_CC13X0)
#  include "hardware/cc13x0/cc13x0_timer.h"
#elif defined(CONFIG_ARCH_CHIP_CC13X2)
#  include "hardware/cc13x2_cc26x2/cc13x2_cc26x2_timer.h"
#else
#  error "Unsupported Tiva/Stellaris system control module"
#endif

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_TIMER_H */
