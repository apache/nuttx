/****************************************************************************
 * arch/arm/src/efm32/efm32_rmu.h
 *
 *   Copyright (C) 2015 Pierre-noel Bouteville . All rights reserved.
 *   Authors: Pierre-noel Bouteville <pnb990@gmail.com>
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

#ifndef __ARCH_ARM_SRC_EFM32_EFM32_RMU_H
#define __ARCH_ARM_SRC_EFM32_EFM32_RMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip/efm32_rmu.h"

#ifdef CONFIG_EFM32_RMU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_ERROR
#  undef CONFIG_EFM32_RMU_DEBUG
#endif

#ifdef CONFIG_EFM32_RMU_DEBUG
#  define rmuerr   _err
#  define rmuwarn  _warn
#  define rmuinfo  _info
#else
#  define rmuerr(x...)
#  define rmuwarn(x...)
#  define rmuinfo(x...)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t g_efm32_rstcause;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/************************************************************************************
 * Name: efm32_rmu_initialize
 *
 * Description:
 *    Store reset cause into g_efm32_rstcause then clear reset cause register.
 *
 ************************************************************************************/

void efm32_rmu_initialize(void);
const char* efm32_reset_cause_list_str(uint32_t reg, unsigned int *idx);

/************************************************************************************
 * Name: efm32_reset_cause_list_str
 *
 * Description:
 *    Return next reset cause string, NULL if no more reset cause.
 *
 * Input Parameters:
 *   reg: reset cause register to decode (like g_efm32_rstcause)
 *   idx: Use to keep in maind reset cause decoding position.
 *        set *idx to zero before first call.
 *
 ************************************************************************************/

#ifdef CONFIG_EFM32_RMU_DEBUG
const char *efm32_reset_cause_list_str(uint32_t reg, unsigned int *idx);
#endif

#endif /* CONFIG_EFM32_RMU */
#endif /* __ARCH_ARM_SRC_EFM32_EFM32_RMU_H */
