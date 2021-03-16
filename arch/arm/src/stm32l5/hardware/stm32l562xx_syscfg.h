/****************************************************************************
 * arch/arm/src/stm32l5/hardware/stm32l562xx_syscfg.h
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

#ifndef __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L562XX_SYSCFG_H
#define __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L562XX_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

#if defined(CONFIG_STM32L5_STM32L562XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32L5_SYSCFG_SECCFGR_OFFSET   0x0000 /* SYSCFG secure configuration register */
#define STM32L5_SYSCFG_CFGR1_OFFSET     0x0004 /* SYSCFG configuration register 1 */
#define STM32L5_SYSCFG_FPUIMR_OFFSET    0x0008 /* SYSCFG FPU interrupt mask register */
#define STM32L5_SYSCFG_CNSLCKR_OFFSET   0x000c /* SYSCFG CPU non-secure lock register */
#define STM32L5_SYSCFG_CSLCKR_OFFSET    0x0010 /* SYSCFG CPU secure lock register */
#define STM32L5_SYSCFG_CFGR2_OFFSET     0x0014 /* SYSCFG configuration register 2 */
#define STM32L5_SYSCFG_SCSR_OFFSET      0x0018 /* SYSCFG SRAM2 control and status register */
#define STM32L5_SYSCFG_SKR_OFFSET       0x001c /* SYSCFG SRAM2 key register */
#define STM32L5_SYSCFG_SWPR_OFFSET      0x0020 /* SYSCFG SRAM2 write protection register */
#define STM32L5_SYSCFG_SWPR2_OFFSET     0x0024 /* SYSCFG SRAM2 write protection register 2 */
#define STM32L5_SYSCFG_RSSCMDR_OFFSET   0x002c /* SYSCFG RSS command register */

/* Register Addresses *******************************************************/

#define STM32L5_SYSCFG_SECCFGR          (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_SECCFGR_OFFSET)
#define STM32L5_SYSCFG_CFGR1            (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_CFGR1_OFFSET)
#define STM32L5_SYSCFG_FPUIMR           (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_FPUIMR_OFFSET)
#define STM32L5_SYSCFG_CNSLCKR          (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_CNSLCKR_OFFSET)
#define STM32L5_SYSCFG_CSLCKR           (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_CSLCKR_OFFSET)
#define STM32L5_SYSCFG_CFGR2            (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_CFGR2_OFFSET)
#define STM32L5_SYSCFG_SCSR             (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_SCSR_OFFSET)
#define STM32L5_SYSCFG_SKR              (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_SKR_OFFSET)
#define STM32L5_SYSCFG_SWPR             (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_SWPR_OFFSET)
#define STM32L5_SYSCFG_SWPR2            (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_SWPR2_OFFSET)
#define STM32L5_SYSCFG_RSSCMDR          (STM32L5_SYSCFG_BASE + STM32L5_SYSCFG_RSSCMDR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* SYSCFG secure configuration register */

#define SYSCFG_SECCFGR_SYSCFGSEC      (1 << 0) /* SYSCFG clock control security */
#define SYSCFG_SECCFGR_CLASSBSEC      (1 << 1) /* ClassB security */
#define SYSCFG_SECCFGR_SRAM2SEC       (1 << 2) /* SRAM2 security */
#define SYSCFG_SECCFGR_FPUSEC         (1 << 3) /* FPU security */

/* SYSCFG configuration register 1 */

#define SYSCFG_CFGR1_BOOSTEN          (1 <<  8) /* Bit  8: I/O analog switch voltage booster enable (use when vdd is low) */
#define SYSCFG_CFGR1_ANASWVDD         (1 <<  9) /* Bit  9: GPIO analog switch control voltage selection */
#define SYSCFG_CFGR1_I2C_PB6_FMP      (1 << 16) /* Bit 16: Fast-mode Plus (Fm+) driving capability activation on PB6 */
#define SYSCFG_CFGR1_I2C_PB7_FMP      (1 << 17) /* Bit 17: Fast-mode Plus (Fm+) driving capability activation on PB7 */
#define SYSCFG_CFGR1_I2C_PB8_FMP      (1 << 18) /* Bit 18: Fast-mode Plus (Fm+) driving capability activation on PB8 */
#define SYSCFG_CFGR1_I2C_PB9_FMP      (1 << 19) /* Bit 19: Fast-mode Plus (Fm+) driving capability activation on PB9 */
#define SYSCFG_CFGR1_I2C1_FMP         (1 << 20) /* Bit 20: I2C1 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C2_FMP         (1 << 21) /* Bit 21: I2C2 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C3_FMP         (1 << 22) /* Bit 22: I2C3 Fast-mode Plus (Fm+) driving capability activation */
#define SYSCFG_CFGR1_I2C4_FMP         (1 << 23) /* Bit 23: I2C4 Fast-mode Plus (Fm+) driving capability activation */

/* SYSCFG FPU interrupt mask register */

#define SYSCFG_FPUIMR_IE0             (1 << 0) /* Bit 0: FPU Invalid operation interrupt enable */
#define SYSCFG_FPUIMR_IE1             (1 << 1) /* Bit 1: FPU Divide-by-zero interrupt enable */
#define SYSCFG_FPUIMR_IE2             (1 << 2) /* Bit 2: FPU Underflow interrupt enable */
#define SYSCFG_FPUIMR_IE3             (1 << 3) /* Bit 3: FPU Overflow interrupt enable */
#define SYSCFG_FPUIMR_IE4             (1 << 4) /* Bit 4: FPU Input abnormal interrupt enable */
#define SYSCFG_FPUIMR_IE5             (1 << 5) /* Bit 5: FPU Inexact interrupt enable */

/* SYSCFG CPU non-secure lock register */

#define SYSCFG_CNSLCKR_LOCKNSVTOR     (1 << 0) /* Bit 0: VTOR_NS register lock */
#define SYSCFG_CNSLCKR_LOCKNSMPU      (1 << 1) /* Bit 1: Non-seucre MPU registers lock */

/* SYSCFG CPU secure lock register */

#define SYSCFG_CSLCKR_LOCKSVTAIRCR     (1 << 0) /* Bit 0: VTOR_S register and AIRCR register bits lock */
#define SYSCFG_CSLCKR_LOCKSMPU         (1 << 1) /* Bit 1: Secure MPU registers lock */
#define SYSCFG_CSLCKR_LOCKSAU          (1 << 2) /* Bit 2: SAU registers lock */

/* SYSCFG configuration register 2 */

#define SYSCFG_CFGR2_CLL               (1 <<  0) /* Bit  0: Cortex-M33 LOCKUP (hardfault) output enable */
#define SYSCFG_CFGR2_SPL               (1 <<  1) /* Bit  1: SRAM2 parity lock */
#define SYSCFG_CFGR2_PVDL              (1 <<  2) /* Bit  2: PVD lock enable */
#define SYSCFG_CFGR2_ECCL              (1 <<  3) /* Bit  3: ECC lock */
#define SYSCFG_CFGR2_SPF               (1 <<  8) /* Bit  8: SRAM2 parity error flag */

/* SYSCFG SRAM2 control and status register */

#define SYSCFG_SCSR_SRAM2ER           (1 <<  0) /* Bit  0: SRAM2 Erase */
#define SYSCFG_SCSR_SRAM2BSY          (1 <<  1) /* Bit  1: SRAM2 busy in erase operation */

/* SYSCFG SRAM2 key register */

#define SYSCFG_SKR_SHIFT              0
#define SYSCFG_SKR_MASK               (0xFF << SYSCFG_SKR_SHIFT)

/* SYSCFG SRAM2 write protection register 1 and 2: There is one bit per SRAM2
 * page (0 to 31 and 32 to 63, respectively).
 */

/* SYSCFG RSS command register */

#define SYSCFG_RSSCMDR_SHIFT              0
#define SYSCFG_RSSCMDR_MASK               (0xFFFF << SYSCFG_RSSCMDR_SHIFT)

#endif /* CONFIG_STM32L5_STM32L562XX */
#endif /* __ARCH_ARM_SRC_STM32L5_HARDWARE_STM32L562XX_SYSCFG_H */
