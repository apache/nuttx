/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_wdg.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_WDG_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_WDG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_IWDG_KR_OFFSET     0x0000  /* Key register (32-bit) */
#define STM32_IWDG_PR_OFFSET     0x0004  /* Prescaler register (32-bit) */
#define STM32_IWDG_RLR_OFFSET    0x0008  /* Reload register (32-bit) */
#define STM32_IWDG_SR_OFFSET     0x000c  /* Status register (32-bit) */
#if defined(CONFIG_STM32_STM32F30XX)
#  define STM32_IWDG_WINR_OFFSET 0x000c  /* Window register (32-bit) */
#endif

#define STM32_WWDG_CR_OFFSET     0x0000  /* Control Register (32-bit) */
#define STM32_WWDG_CFR_OFFSET    0x0004  /* Configuration register (32-bit) */
#define STM32_WWDG_SR_OFFSET     0x0008  /* Status register (32-bit) */

/* Register Addresses *******************************************************/

#define STM32_IWDG_KR            (STM32_IWDG_BASE+STM32_IWDG_KR_OFFSET)
#define STM32_IWDG_PR            (STM32_IWDG_BASE+STM32_IWDG_PR_OFFSET)
#define STM32_IWDG_RLR           (STM32_IWDG_BASE+STM32_IWDG_RLR_OFFSET)
#define STM32_IWDG_SR            (STM32_IWDG_BASE+STM32_IWDG_SR_OFFSET)
#if defined(CONFIG_STM32_STM32F30XX)
#  define STM32_IWDG_WINR        (STM32_IWDG_BASE+STM32_IWDG_WINR_OFFSET)
#endif

#define STM32_WWDG_CR            (STM32_WWDG_BASE+STM32_WWDG_CR_OFFSET)
#define STM32_WWDG_CFR           (STM32_WWDG_BASE+STM32_WWDG_CFR_OFFSET)
#define STM32_WWDG_SR            (STM32_WWDG_BASE+STM32_WWDG_SR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Key register (32-bit) */

#define IWDG_KR_KEY_SHIFT        (0)       /* Bits 15-0: Key value (write only, read 0000h) */
#define IWDG_KR_KEY_MASK         (0xffff << IWDG_KR_KEY_SHIFT)

#define IWDG_KR_KEY_ENABLE       (0x5555)  /* Enable register access */
#define IWDG_KR_KEY_DISABLE      (0x0000)  /* Disable register access */
#define IWDG_KR_KEY_RELOAD       (0xaaaa)  /* Reload the counter */
#define IWDG_KR_KEY_START        (0xcccc)  /* Start the watchdog */

/* Prescaler register (32-bit) */

#define IWDG_PR_SHIFT            (0)       /* Bits 2-0: Prescaler divider */
#define IWDG_PR_MASK             (7 << IWDG_PR_SHIFT)
#  define IWDG_PR_DIV4           (0 << IWDG_PR_SHIFT) /* 000: divider /4 */
#  define IWDG_PR_DIV8           (1 << IWDG_PR_SHIFT) /* 001: divider /8 */
#  define IWDG_PR_DIV16          (2 << IWDG_PR_SHIFT) /* 010: divider /16 */
#  define IWDG_PR_DIV32          (3 << IWDG_PR_SHIFT) /* 011: divider /32 */
#  define IWDG_PR_DIV64          (4 << IWDG_PR_SHIFT) /* 100: divider /64 */
#  define IWDG_PR_DIV128         (5 << IWDG_PR_SHIFT) /* 101: divider /128 */
#  define IWDG_PR_DIV256         (6 << IWDG_PR_SHIFT) /* 11x: divider /256 */

/* Reload register (32-bit) */

#define IWDG_RLR_RL_SHIFT        (0)       /* Bits11:0 RL[11:0]: Watchdog counter reload value */
#define IWDG_RLR_RL_MASK         (0x0fff << IWDG_RLR_RL_SHIFT)

#define IWDG_RLR_MAX             (0xfff)

/* Status register (32-bit) */

#define IWDG_SR_PVU              (1 << 0)  /* Bit 0: Watchdog prescaler value update */
#define IWDG_SR_RVU              (1 << 1)  /* Bit 1: Watchdog counter reload value update */

#if defined(CONFIG_STM32_STM32F30XX)
#  define IWDG_SR_WVU            (1 << 2)  /* Bit 2:  */
#endif

/* Window register (32-bit) */

#if defined(CONFIG_STM32_STM32F30XX)
#  define IWDG_WINR_SHIFT        (0)
#  define IWDG_WINR_MASK         (0x0fff << IWDG_WINR_SHIFT)
#endif

/* Control Register (32-bit) */

#define WWDG_CR_T_SHIFT          (0)       /* Bits 6:0 T[6:0]: 7-bit counter (MSB to LSB) */
#define WWDG_CR_T_MASK           (0x7f << WWDG_CR_T_SHIFT)
#  define WWDG_CR_T_MAX          (0x3f << WWDG_CR_T_SHIFT)
#  define WWDG_CR_T_RESET        (0x40 << WWDG_CR_T_SHIFT)
#define WWDG_CR_WDGA             (1 << 7)  /* Bit 7: Activation bit */

/* Configuration register (32-bit) */

#define WWDG_CFR_W_SHIFT         (0)       /* Bits 6:0 W[6:0] 7-bit window value */
#define WWDG_CFR_W_MASK          (0x7f << WWDG_CFR_W_SHIFT)
#define WWDG_CFR_WDGTB_SHIFT     (7)       /* Bits 8:7 [1:0]: Timer Base */
#define WWDG_CFR_WDGTB_MASK      (3 << WWDG_CFR_WDGTB_SHIFT)
#  define WWDG_CFR_PCLK1         (0 << WWDG_CFR_WDGTB_SHIFT) /* 00: CK Counter Clock (PCLK1 div 4096) div 1 */
#  define WWDG_CFR_PCLK1d2       (1 << WWDG_CFR_WDGTB_SHIFT) /* 01: CK Counter Clock (PCLK1 div 4096) div 2 */
#  define WWDG_CFR_PCLK1d4       (2 << WWDG_CFR_WDGTB_SHIFT) /* 10: CK Counter Clock (PCLK1 div 4096) div 4 */
#  define WWDG_CFR_PCLK1d8       (3 << WWDG_CFR_WDGTB_SHIFT) /* 11: CK Counter Clock (PCLK1 div 4096) div 8 */

#define WWDG_CFR_EWI             (1 << 9)  /* Bit 9: Early Wakeup Interrupt */

/* Status register (32-bit) */

#define WWDG_SR_EWIF             (1 << 0)  /* Bit 0: Early Wakeup Interrupt Flag */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_WDG_H */
