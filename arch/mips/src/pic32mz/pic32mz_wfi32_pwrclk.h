/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_wfi32_pwrclk.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_WFI32_PWRCLK_H
#define __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_WFI32_PWRCLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_ARCH_CHIP_WFI32E01

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_wfi32_pmu_initialize
 *
 * Description:
 *   Bring the buck/MLDO regulator from its power-on state into its
 *   run-mode configuration, applying the factory trim when present.
 *   Must be called before pic32mz_wfi32_clk_initialize().
 *
 ****************************************************************************/

void pic32mz_wfi32_pmu_initialize(void);

/****************************************************************************
 * Name: pic32mz_wfi32_clk_initialize
 *
 * Description:
 *   Start the 40 MHz primary oscillator, run SYSCLK at 200 MHz from the
 *   system PLL, start the Ethernet/Wi-Fi PLL and power down the unused
 *   USB and Bluetooth PLLs.
 *
 ****************************************************************************/

void pic32mz_wfi32_clk_initialize(void);

#ifdef CONFIG_PIC32MZ_W1_BOOTTRACE
void pic32mz_wfi32_trace_init(void);
void pic32mz_wfi32_trace(int ch);
void pic32mz_wfi32_trace_hex(uint32_t value);
void pic32mz_wfi32_trace_end(void);
#endif

#endif /* CONFIG_ARCH_CHIP_WFI32E01 */
#endif /* __ARCH_MIPS_SRC_PIC32MZ_PIC32MZ_WFI32_PWRCLK_H */
