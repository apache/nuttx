/************************************************************************************
 * arch/arm/src/stm32/hardware/stm32_i2c.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/* There are 2 main types of I2C IP cores among STM32 chips:
 *   1. STM32 I2C IPv1 - F1, F2, F4 and L1
 *   2. STM32 I2C IPv2 - F0, F3, F7, G0, G4, H7, L0 and L4
 */

#if defined(CONFIG_STM32_HAVE_IP_I2C_V1)
#  include "stm32_i2c_v1.h"
#elif defined(CONFIG_STM32_HAVE_IP_I2C_V2)
#  include "stm32_i2c_v2.h"
#else
#  error STM32 I2C IP version not specified
#endif

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_I2C_H */
