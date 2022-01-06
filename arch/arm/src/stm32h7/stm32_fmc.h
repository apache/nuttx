/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fmc.h
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
 *   Initialize the FMC peripheral. Because FMC initialization is highly
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
