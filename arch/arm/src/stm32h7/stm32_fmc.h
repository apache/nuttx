/****************************************************************************
 * arch/arm/src/stm32/stm32_fmc.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_FMC_H
#define __ARCH_ARM_SRC_STM32H7_STM32_FMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_fmc.h"

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
 * Name: stm32_fmc_init
 *
 * Description:
 *   Initialize the FMC peripherial. Because FMC initialization is highly
 *   dependent on the used parts, definition of the initial values for FMC
 *   registers is mostly left to board designer.
 *
 *   Typically called from arm_addregion().
 *
 ****************************************************************************/

void stm32_fmc_init(void);

/****************************************************************************
 * Name: stm32_fmc_enable
 *
 * Description:
 *   Enable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_enable_clk(void);

/****************************************************************************
 * Name: stm32_fmc_disable
 *
 * Description:
 *   Disable clocking to the FMC.
 *
 ****************************************************************************/

void stm32_fmc_disable(void);

/****************************************************************************
 * Name: stm32_fmc_sdram_write_protect
 *
 * Description:
 *   Enable/Disable writes to an SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_write_protect(int bank, bool state);

/****************************************************************************
 * Name: stm32_fmc_sdram_set_refresh_rate
 *
 * Description:
 *   Set the SDRAM refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_refresh_rate(int count);

/****************************************************************************
 * Name: stm32_fmc_sdram_enable
 *
 * Description:
 *   Enable FMC SDRAM. Do this after issue refresh rate.
 *
 ****************************************************************************/

void stm32_fmc_sdram_enable(void);

/****************************************************************************
 * Name: stm32_fmc_sdram_set_timing
 *
 * Description:
 *   Set the SDRAM timing parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_timing(int bank, uint32_t timing);

/****************************************************************************
 * Name: stm32_fmc_sdram_set_control
 *
 * Description:
 *   Set the SDRAM control parameters.
 *
 ****************************************************************************/

void stm32_fmc_sdram_set_control(int bank, uint32_t ctrl);

/****************************************************************************
 * Name: stm32_fmc_sdram_command
 *
 * Description:
 *   Send a command to the SDRAM.
 *
 ****************************************************************************/

void stm32_fmc_sdram_command(uint32_t cmd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_FMC_H */
