/***************************************************************************************************
 * arch/arm/src/stm32f7/hardware/stm32f76xx77xx_dbgmcu.h
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
 ***************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXDBGMCU_H
#define __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXDBGMCU_H

/***************************************************************************************************
 * Included Files
 ***************************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/***************************************************************************************************
 * Pre-processor Definitions
 ***************************************************************************************************/

/* Register Offsets  *******************************************************************************/

#define STM32_DBGMCU_IDC_OFFSET      0x0000  /* DBGMCU identity code register */
#define STM32_DBGMCU_CR_OFFSET       0x0004  /* MCU debug */
#define STM32_DBGMCU_APB3FZ1_OFFSET  0x0034  /* APB3 peripheral freeze register */
#define STM32_DBGMCU_APB1LFZ1_OFFSET 0x003c  /* APB1L peripheral freeze register */
#define STM32_DBGMCU_APB2FZ1_OFFSET  0x004c  /* APB2 peripheral freeze register */
#define STM32_DBGMCU_APB4LFZ1_OFFSET 0x0054  /* APB4 peripheral freeze register */

/* Register Addresses ******************************************************************************/

#define STM32_DBGMCU_IDC      (STM32_DEBUGMCU_BASE + STM32_DBGMCU_IDC_OFFSET)
#define STM32_DBGMCU_CR       (STM32_DEBUGMCU_BASE + STM32_DBGMCU_CR_OFFSET)
#define STM32_DBGMCU_APB3FZ1  (STM32_DEBUGMCU_BASE + STM32_DBGMCU_APB3FZ1_OFFSET)
#define STM32_DBGMCU_APB1LFZ1 (STM32_DEBUGMCU_BASE + STM32_DBGMCU_APB1LFZ1_OFFSET)
#define STM32_DBGMCU_APB2FZ1  (STM32_DEBUGMCU_BASE + STM32_DBGMCU_APB2FZ1_OFFSET)
#define STM32_DBGMCU_APB4LFZ1 (STM32_DEBUGMCU_BASE + STM32_DBGMCU_APB4LFZ1_OFFSET)

/* Register Bitfield Definitions *******************************************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK  (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#  define STM32_IDCODE_DEVID      (0x0450 << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK  (0xffff << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_Z     (0x1001 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_Y     (0x1003 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_X     (0x2001 << DBGMCU_IDCODE_REVID_SHIFT)
#  define STM32_IDCODE_REVID_V     (0x2003 << DBGMCU_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DBGMCU_CR_SLEEP           (1 << 0)  /* Bit 0: Debug Sleep Mode */
#define DBGMCU_CR_STOP            (1 << 1)  /* Bit 1: Debug Stop Mode */
#define DBGMCU_CR_STANDBY         (1 << 2)  /* Bit 2: Debug Standby mode */

#define DBGMCU_CR_TRACECLKEN      (1 << 20)  /* Bit 20: Trace port clock enable */
#define DBGMCU_CR_D1DBGCKEN       (1 << 21)  /* Bit 21: D1 debug clock enable */
#define DBGMCU_CR_D3DBGCKEN       (1 << 22)  /* Bit 22: D3 debug clock enable */

#define DBGMCU_CR_TRGOEN          (1 << 28)  /* Bit 28: External trigger output enable */

/* Debug MCU APB3 freeze register */

#define DBGMCU_APB3_WWDOG          (1 << 6)   /* Bit 6: WWDG1 stop in debug */

/* Debug MCU APB1L freeze register */

#define DBGMCU_APB1L_TIM2STOP      (1 << 0)   /* Bit 0: TIM2 stopped when halted */
#define DBGMCU_APB1L_TIM3STOP      (1 << 1)   /* Bit 1: TIM3 stopped when halted */
#define DBGMCU_APB1L_TIM4STOP      (1 << 2)   /* Bit 2: TIM4 stopped when halted */
#define DBGMCU_APB1L_TIM5STOP      (1 << 3)   /* Bit 3: TIM5 stopped when halted */
#define DBGMCU_APB1L_TIM6STOP      (1 << 4)   /* Bit 4: TIM6 stopped when halted */
#define DBGMCU_APB1L_TIM7STOP      (1 << 5)   /* Bit 5: TIM7 stopped when halted */
#define DBGMCU_APB1L_TIM12STOP     (1 << 6)   /* Bit 6: TIM12 stopped when halted */
#define DBGMCU_APB1L_TIM13STOP     (1 << 7)   /* Bit 7: TIM13 stopped when halted */
#define DBGMCU_APB1L_TIM14STOP     (1 << 8)   /* Bit 8: TIM14 stopped when halted */
#define DBGMCU_APB1L_LPTIM1STOP    (1 << 9)   /* Bit 9: LPTIM1 stopped when halted */

#define DBGMCU_APB1L_I2C1STOP      (1 << 21)  /* Bit 21: I2C1 SMBUS timeout stopped when halted */
#define DBGMCU_APB1L_I2C2STOP      (1 << 22)  /* Bit 22: I2C2 SMBUS timeout stopped when halted */
#define DBGMCU_APB1L_I2C3STOP      (1 << 23)  /* Bit 23: I2C3 SMBUS timeout stopped when halted */

/* Debug MCU APB2 freeze register */

#define DBGMCU_APB2Z1_TIM1STOP     (1 << 0)   /* Bit 0:  TIM1 stopped when halted */
#define DBGMCU_APB2Z1_TIM8STOP     (1 << 1)   /* Bit 1:  TIM8 stopped when halted */
#define DBGMCU_APB2Z1_TIM15STOP    (1 << 16)  /* Bit 16: TIM15 stopped when halted */
#define DBGMCU_APB2Z1_TIM16STOP    (1 << 17)  /* Bit 17: TIM16 stopped when halted */
#define DBGMCU_APB2Z1_TIM17STOP    (1 << 18)  /* Bit 18: TIM17 stopped when halted */
#define DBGMCU_APB2Z1_TIM17STOP    (1 << 18)  /* Bit 18: TIM17 stopped when halted */
#define DBGMCU_APB2Z1_HRTIMSTOP    (1 << 29)  /* Bit 29: HRTIM stopped when halted */

/* Debug MCU APB4 freeze register */

#define DBGMCU_APB4FZ1_I2C4STOP     (1 << 7)    /* Bit 7: I2C4 stopped when halted */
#define DBGMCU_APB4FZ1_LPTIM2STOP   (1 << 9)    /* Bit 9: LPTIM2 stopped when halted */
#define DBGMCU_APB4FZ1_LPTIM3STOP   (1 << 10)   /* Bit 10:LPTIM3 stopped when halted */
#define DBGMCU_APB4FZ1_LPTIM4STOP   (1 << 11)   /* Bit 11:LPTIM4 stopped when halted */
#define DBGMCU_APB4FZ1_LPTIM5STOP   (1 << 12)   /* Bit 12:LPTIM5 stopped when halted */
#define DBGMCU_APB4FZ1_RTCSTOP      (1 << 16)   /* Bit 16:RTC stopped when halted */
#define DBGMCU_APB4FZ1_IIWDG1STOP   (1 << 18)   /* Bit 18:IIWDG1 stopped when halted */

#endif /* __ARCH_ARM_SRC_STM32H7_HARDWARE_STM32H7XXXDBGMCU_H */
