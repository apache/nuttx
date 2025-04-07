/****************************************************************************
 * arch/arm/src/stm32f0l0g0/hardware/stm32_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DBGMCU_H
#define __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_DBGMCU_IDCODE       0x40015800  /* MCU identifier */
#define STM32_DBGMCU_CR           0x40015804  /* MCU debug */
#define STM32_DBGMCU_APB1_FZ      0x40015808  /* Debug MCU APB1 freeze register */
#define STM32_DBGMCU_APB2_FZ      0x4001580c  /* Debug MCU APB2 freeze register */

/* Register Bitfield Definitions ********************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK  (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK  (0xffff << DBGMCU_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DBGMCU_CR_SLEEP           (1 << 0)  /* Bit 0: Debug Sleep Mode */
#define DBGMCU_CR_STOP            (1 << 1)  /* Bit 1: Debug Stop Mode */
#define DBGMCU_CR_STANDBY         (1 << 2)  /* Bit 2: Debug Standby mode */

/* Debug MCU APB freeze register 1 */

#ifdef CONFIG_ARCH_CHIP_STM32C0
#  define DBGMCU_APB1_TIM2STOP    (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#  define DBGMCU_APB1_TIM3STOP    (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#  define DBGMCU_APB1_RTCSTOP     (1 << 10)  /* Bit 10: RTC stopped when core is halted */
#  define DBGMCU_APB1_WWDGSTOP    (1 << 11)  /* Bit 11: WWDG stopped when core is halted */
#  define DBGMCU_APB1_IWDGSTOP    (1 << 12)  /* Bit 12: IWDG stopped when core is halted */
#  define DBGMCU_APB1_I2C1STOP    (1 << 21)  /* Bit 21: SMBUS timeout mode stopped when Core is halted  */
#endif

/* Debug MCU APB freeze register 2 */

#ifdef CONFIG_ARCH_CHIP_STM32C0
#  define DBGMCU_APB1_TIM1STOP    (1 << 11)   /* Bit 1: TIM1 stopped when core is halted */
#  define DBGMCU_APB1_TIM14STOP   (1 << 15)   /* Bit 15: TIM14 stopped when core is halted */
#  define DBGMCU_APB1_TIM15STOP   (1 << 16)   /* Bit 16: TIM15 stopped when core is halted */
#  define DBGMCU_APB1_TIM16STOP   (1 << 17)   /* Bit 16: TIM16 stopped when core is halted */
#  define DBGMCU_APB1_TIM17STOP   (1 << 18)   /* Bit 16: TIM17 stopped when core is halted */
#endif

#endif /* __ARCH_ARM_SRC_STM32F0L0G0_HARDWARE_STM32_DBGMCU_H */
