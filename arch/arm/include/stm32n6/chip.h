/****************************************************************************
 * arch/arm/include/stm32n6/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32N6_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32N6_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Memory sizes - STM32N6 has no internal flash.  Code runs from the
 * on-chip AXI SRAM (loaded by the debugger in DEV mode, or by an FSBL
 * from external XSPI flash in normal boot).
 *
 * AXI SRAM layout (from CMSIS stm32n657xx.h):
 *   AXISRAM1:     0x34000000  1 MB
 *   AXISRAM2:     0x34100000  1 MB
 *   AXISRAM3:     0x34200000  448 KB
 *   AXISRAM4:     0x34270000  448 KB
 *   AXISRAM5:     0x342E0000  448 KB
 *   AXISRAM6:     0x34350000  448 KB
 *   CACHEAXIRAM:  0x343C0000  256 KB
 *   -------------------------------------------
 *   Total:                    4 MB    (end = 0x34400000)
 *
 * VENCRAM (128 KB at 0x34400000) is excluded -- reserved for video encoder.
 * Each bank requires its RCC MEMENR clock enable bit to be set.
 */

#define STM32_SRAM_SIZE         (4 * 1024 * 1024)  /* 4194304 bytes (4 MiB) */

#define STM32_NPORTS                  (12)   /* GPIO ports A-H (8) + N, O, P, Q (4) */
#define STM32_NUSART                   (1)   /* USART1 */

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels (4-bit priority) */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32N6_CHIP_H */
