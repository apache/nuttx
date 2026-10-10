/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DBGMCU_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The freeze registers and the per-peripheral freeze bits are described by
 * STM32_DBGMCU_<periph>_FZ (register) and DBGMCU_<periph>_STOP (bit), so
 * drivers do not depend on the DBGMCU register layout of the family.
 */

#ifndef STM32_DBGMCU_BASE
#  define STM32_DBGMCU_BASE             STM32_DEBUGMCU_BASE
#endif

/* Register Offsets *********************************************************/

#define STM32_DBGMCU_IDCODE_OFFSET      0x0000  /* MCU identifier */
#define STM32_DBGMCU_CR_OFFSET          0x0004  /* MCU debug */

/* Register Addresses *******************************************************/

#define STM32_DBGMCU_IDCODE             (STM32_DBGMCU_BASE + STM32_DBGMCU_IDCODE_OFFSET)
#define STM32_DBGMCU_CR                 (STM32_DBGMCU_BASE + STM32_DBGMCU_CR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT       (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK        (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT       (16)      /* Bits 31-16: Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK        (0xffff << DBGMCU_IDCODE_REVID_SHIFT)

#ifdef CONFIG_ARCH_CHIP_STM32H7
#  define STM32_IDCODE_DEVID            (0x0450 << DBGMCU_IDCODE_DEVID_SHIFT)
#  define STM32_IDCODE_REVID_Z          (0x1001 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_Y          (0x1003 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_X          (0x2001 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_V          (0x2003 << DBGMCU_IDCODE_REVID_SHIFT)
#endif

/* MCU debug */

#if defined(CONFIG_STM32_HAVE_IP_DBGMCU_V1) || \
    defined(CONFIG_STM32_HAVE_IP_DBGMCU_V2)
#  define DBGMCU_CR_SLEEP               (1 << 0)  /* Bit 0: Debug Sleep Mode */
#  define DBGMCU_CR_STOP                (1 << 1)  /* Bit 1: Debug Stop Mode */
#  define DBGMCU_CR_STANDBY             (1 << 2)  /* Bit 2: Debug Standby mode */
#  define DBGMCU_CR_TRACEIOEN           (1 << 5)  /* Bit 5: Trace enabled */
#  define DBGMCU_CR_TRACEMODE_SHIFT     (6)       /* Bits 7-6: Trace mode pin assignment */
#  define DBGMCU_CR_TRACEMODE_MASK      (3 << DBGMCU_CR_TRACEMODE_SHIFT)
#  define DBGMCU_CR_ASYNCH              (0 << DBGMCU_CR_TRACEMODE_SHIFT) /* Asynchronous Mode */
#  define DBGMCU_CR_SYNCH1              (1 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=1 */
#  define DBGMCU_CR_SYNCH2              (2 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=2 */
#  define DBGMCU_CR_SYNCH4              (3 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=4 */
#endif

/* Freeze Registers and Bits ************************************************/

#if defined(CONFIG_STM32_HAVE_IP_DBGMCU_V1)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM1_STOP              (1 << 10)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM2_STOP              (1 << 11)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM3_STOP              (1 << 12)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM4_STOP              (1 << 13)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM5_STOP              (1 << 18)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM6_STOP              (1 << 19)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM7_STOP              (1 << 20)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_CR
#  define DBGMCU_TIM8_STOP              (1 << 17)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM12_STOP             (1 << 25)
#  define STM32_DBGMCU_TIM13_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM13_STOP             (1 << 26)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM14_STOP             (1 << 27)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM15_STOP             (1 << 22)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM16_STOP             (1 << 23)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_CR
#  define DBGMCU_TIM17_STOP             (1 << 24)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_CR
#  define DBGMCU_IWDG_STOP              (1 << 8)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_CR
#  define DBGMCU_WWDG_STOP              (1 << 9)

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V2)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x0008
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x000c
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM5_STOP              (1 << 3)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM8_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM12_STOP             (1 << 6)
#  define STM32_DBGMCU_TIM13_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM13_STOP             (1 << 7)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM14_STOP             (1 << 8)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 2)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 3)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 4)
#  define STM32_DBGMCU_TIM18_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM18_STOP             (1 << 9)
#  define STM32_DBGMCU_TIM19_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM19_STOP             (1 << 5)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 9)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_IWDG_STOP              (1 << 12)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#if defined(CONFIG_ARCH_CHIP_STM32F2) || \
    defined(CONFIG_ARCH_CHIP_STM32F4) || \
    defined(CONFIG_ARCH_CHIP_STM32F7)
#  define STM32_DBGMCU_TIM9_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM9_STOP              (1 << 16)
#  define STM32_DBGMCU_TIM10_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM10_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM11_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM11_STOP             (1 << 18)
#elif defined(CONFIG_ARCH_CHIP_STM32L1)
#  define STM32_DBGMCU_TIM9_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM9_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM10_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM10_STOP             (1 << 3)
#  define STM32_DBGMCU_TIM11_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM11_STOP             (1 << 4)
#endif

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V3)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x0008
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x000c
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 11)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 16)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_TIM21_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM21_STOP             (1 << 2)
#  define STM32_DBGMCU_TIM22_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM22_STOP             (1 << 6)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 31)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM2_STOP            (1 << 30)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB2FZ
#  define DBGMCU_LPTIM3_STOP            (1 << 18)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_IWDG_STOP              (1 << 12)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#if defined(CONFIG_ARCH_CHIP_STM32C0) || \
    defined(CONFIG_ARCH_CHIP_STM32G0) || \
    defined(CONFIG_ARCH_CHIP_STM32U0)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM14_STOP             (1 << 15)
#elif defined(CONFIG_ARCH_CHIP_STM32F0)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM14_STOP             (1 << 8)
#endif

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V4)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x0008
#  define STM32_DBGMCU_APB1FZ2_OFFSET   0x000c
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x0010
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB1FZ2          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ2_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 11)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM5_STOP              (1 << 3)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM8_STOP              (1 << 13)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 16)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_TIM20_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM20_STOP             (1 << 20)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 31)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB1FZ2
#  define DBGMCU_LPTIM2_STOP            (1 << 5)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB1FZ2
#  define DBGMCU_LPTIM3_STOP            (1 << 6)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_IWDG_STOP              (1 << 12)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V5)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x0008
#  define STM32_DBGMCU_APB1FZ2_OFFSET   0x000c
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x0010
#  define STM32_DBGMCU_APB3FZ_OFFSET    0x0014
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB1FZ2          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ2_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)
#  define STM32_DBGMCU_APB3FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB3FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 11)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM5_STOP              (1 << 3)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM8_STOP              (1 << 13)
#  define STM32_DBGMCU_TIM13_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM13_STOP             (1 << 7)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM14_STOP             (1 << 8)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 16)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB3FZ
#  define DBGMCU_LPTIM1_STOP            (1 << 17)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB1FZ2
#  define DBGMCU_LPTIM2_STOP            (1 << 5)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB3FZ
#  define DBGMCU_LPTIM3_STOP            (1 << 18)
#  define STM32_DBGMCU_LPTIM4_FZ        STM32_DBGMCU_APB3FZ
#  define DBGMCU_LPTIM4_STOP            (1 << 19)
#  define STM32_DBGMCU_LPTIM5_FZ        STM32_DBGMCU_APB3FZ
#  define DBGMCU_LPTIM5_STOP            (1 << 20)
#  define STM32_DBGMCU_LPTIM6_FZ        STM32_DBGMCU_APB3FZ
#  define DBGMCU_LPTIM6_STOP            (1 << 21)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_IWDG_STOP              (1 << 12)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#if defined(CONFIG_ARCH_CHIP_STM32C5) || \
    defined(CONFIG_ARCH_CHIP_STM32H5)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM12_STOP             (1 << 6)
#elif defined(CONFIG_ARCH_CHIP_STM32U3)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM12_STOP             (1 << 15)
#endif

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V6)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x003c
#  define STM32_DBGMCU_APB1FZ2_OFFSET   0x0044
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x004c
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB1FZ2          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ2_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 11)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 31)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB1FZ2
#  define DBGMCU_LPTIM2_STOP            (1 << 5)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB1FZ2
#  define DBGMCU_LPTIM3_STOP            (1 << 6)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_IWDG_STOP              (1 << 12)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V7)

/* Freeze registers */

#  define STM32_DBGMCU_APB3FZ_OFFSET    0x0034
#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x003c
#  define STM32_DBGMCU_APB1FZ2_OFFSET   0x0044
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x004c
#  define STM32_DBGMCU_APB4FZ_OFFSET    0x0054
#  define STM32_DBGMCU_APB3FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB3FZ_OFFSET)
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB1FZ2          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ2_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)
#  define STM32_DBGMCU_APB4FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB4FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM5_STOP              (1 << 3)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM8_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM9_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM9_STOP              (1 << 19)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM12_STOP             (1 << 6)
#  define STM32_DBGMCU_TIM13_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM13_STOP             (1 << 7)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM14_STOP             (1 << 8)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 16)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_TIM23_FZ         STM32_DBGMCU_APB1FZ2
#  define DBGMCU_TIM23_STOP             (1 << 24)
#  define STM32_DBGMCU_TIM24_FZ         STM32_DBGMCU_APB1FZ2
#  define DBGMCU_TIM24_STOP             (1 << 25)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 9)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM2_STOP            (1 << 9)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM3_STOP            (1 << 10)
#  define STM32_DBGMCU_LPTIM4_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM4_STOP            (1 << 11)
#  define STM32_DBGMCU_LPTIM5_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM5_STOP            (1 << 12)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB4FZ
#  define DBGMCU_IWDG_STOP              (1 << 18)
#  define STM32_DBGMCU_RTC_FZ           STM32_DBGMCU_APB4FZ
#  define DBGMCU_RTC_STOP               (1 << 16)
#  define STM32_DBGMCU_I2C1_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_I2C1_STOP              (1 << 21)
#  define STM32_DBGMCU_I2C2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_I2C2_STOP              (1 << 22)
#  define STM32_DBGMCU_I2C3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_I2C3_STOP              (1 << 23)

#if defined(CONFIG_STM32_H7_LEGACY)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB3FZ
#  define DBGMCU_WWDG_STOP              (1 << 6)
#elif defined(CONFIG_STM32_STM32H7RSXX)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)
#endif

#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V8)

/* Freeze registers */

#  define STM32_DBGMCU_APB1FZ1_OFFSET   0x0010
#  define STM32_DBGMCU_APB2FZ_OFFSET    0x0018
#  define STM32_DBGMCU_APB4FZ_OFFSET    0x001c
#  define STM32_DBGMCU_APB1FZ1          (STM32_DBGMCU_BASE + STM32_DBGMCU_APB1FZ1_OFFSET)
#  define STM32_DBGMCU_APB2FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB2FZ_OFFSET)
#  define STM32_DBGMCU_APB4FZ           (STM32_DBGMCU_BASE + STM32_DBGMCU_APB4FZ_OFFSET)

/* Peripherals stopped when the core is halted */

#  define STM32_DBGMCU_TIM1_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM1_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM2_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM2_STOP              (1 << 0)
#  define STM32_DBGMCU_TIM3_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM3_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM4_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM4_STOP              (1 << 2)
#  define STM32_DBGMCU_TIM5_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM5_STOP              (1 << 3)
#  define STM32_DBGMCU_TIM6_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM6_STOP              (1 << 4)
#  define STM32_DBGMCU_TIM7_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM7_STOP              (1 << 5)
#  define STM32_DBGMCU_TIM8_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM8_STOP              (1 << 1)
#  define STM32_DBGMCU_TIM9_FZ          STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM9_STOP              (1 << 19)
#  define STM32_DBGMCU_TIM10_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM10_STOP             (1 << 12)
#  define STM32_DBGMCU_TIM11_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM11_STOP             (1 << 13)
#  define STM32_DBGMCU_TIM12_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM12_STOP             (1 << 6)
#  define STM32_DBGMCU_TIM13_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM13_STOP             (1 << 7)
#  define STM32_DBGMCU_TIM14_FZ         STM32_DBGMCU_APB1FZ1
#  define DBGMCU_TIM14_STOP             (1 << 8)
#  define STM32_DBGMCU_TIM15_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM15_STOP             (1 << 16)
#  define STM32_DBGMCU_TIM16_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM16_STOP             (1 << 17)
#  define STM32_DBGMCU_TIM17_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM17_STOP             (1 << 18)
#  define STM32_DBGMCU_TIM18_FZ         STM32_DBGMCU_APB2FZ
#  define DBGMCU_TIM18_STOP             (1 << 15)
#  define STM32_DBGMCU_LPTIM1_FZ        STM32_DBGMCU_APB1FZ1
#  define DBGMCU_LPTIM1_STOP            (1 << 9)
#  define STM32_DBGMCU_LPTIM2_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM2_STOP            (1 << 9)
#  define STM32_DBGMCU_LPTIM3_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM3_STOP            (1 << 10)
#  define STM32_DBGMCU_LPTIM4_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM4_STOP            (1 << 11)
#  define STM32_DBGMCU_LPTIM5_FZ        STM32_DBGMCU_APB4FZ
#  define DBGMCU_LPTIM5_STOP            (1 << 12)
#  define STM32_DBGMCU_IWDG_FZ          STM32_DBGMCU_APB4FZ
#  define DBGMCU_IWDG_STOP              (1 << 18)
#  define STM32_DBGMCU_WWDG_FZ          STM32_DBGMCU_APB1FZ1
#  define DBGMCU_WWDG_STOP              (1 << 11)

#else
#  error "Unsupported STM32 DBGMCU"
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_DBGMCU_H */
