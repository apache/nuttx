/****************************************************************************
 * arch/arm/src/stm32/hardware/stm32_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DBGMCU_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define STM32_DBGMCU_IDCODE       0xe0042000  /* MCU identifier */
#define STM32_DBGMCU_CR           0xe0042004  /* MCU debug */
#ifdef CONFIG_STM32_HAVE_IP_DBGMCU_V2
#  define STM32_DBGMCU_APB1_FZ    0xe0042008  /* Debug MCU APB1 freeze register */
#  define STM32_DBGMCU_APB2_FZ    0xe004200c  /* Debug MCU APB2 freeze register */
#endif
#ifdef CONFIG_STM32_HAVE_IP_DBGMCU_V3
#  define STM32_DBGMCU_APB1_FZ1    0xe0042008  /* Debug MCU APB1 freeze 1 register */
#  define STM32_DBGMCU_APB1_FZ2    0xe004200c  /* Debug MCU APB1 freeze 2 register */
#  define STM32_DBGMCU_APB2_FZ     0xe0042010  /* Debug MCU APB2 freeze register */
#endif

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
#define DBGMCU_CR_TRACEIOEN       (1 << 5)  /* Bit 5: Trace enabled */

#define DBGMCU_CR_TRACEMODE_SHIFT (6)        /* Bits 7-6: Trace mode pin assignment */
#define DBGMCU_CR_TRACEMODE_MASK  (3 << DBGMCU_CR_TRACEMODE_SHIFT)
#  define DBGMCU_CR_ASYNCH        (0 << DBGMCU_CR_TRACEMODE_SHIFT) /* Asynchronous Mode */
#  define DBGMCU_CR_SYNCH1        (1 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=1 */
#  define DBGMCU_CR_SYNCH2        (2 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=2 */
#  define DBGMCU_CR_SYNCH4        (3 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=4 */

#ifdef CONFIG_STM32_HAVE_IP_DBGMCU_V1
#  define DBGMCU_CR_IWDGSTOP      (1 << 8)   /* Bit 8: Independent Watchdog stopped when core is halted */
#  define DBGMCU_CR_WWDGSTOP      (1 << 9)   /* Bit 9: Window Watchdog stopped when core is halted */
#  define DBGMCU_CR_TIM1STOP      (1 << 10)  /* Bit 10: TIM1 stopped when core is halted */
#  define DBGMCU_CR_TIM2STOP      (1 << 11)  /* Bit 11: TIM2 stopped when core is halted */
#  define DBGMCU_CR_TIM3STOP      (1 << 12)  /* Bit 12: TIM3 stopped when core is halted */
#  define DBGMCU_CR_TIM4STOP      (1 << 13)  /* Bit 13: TIM4 stopped when core is halted */
#  define DBGMCU_CR_CAN1STOP      (1 << 14)  /* Bit 14: CAN1 stopped when core is halted */
#  define DBGMCU_CR_SMBUS1STOP    (1 << 15)  /* Bit 15: I2C1 SMBUS timeout mode stopped when core is halted */
#  define DBGMCU_CR_SMBUS2STOP    (1 << 16)  /* Bit 16: I2C2 SMBUS timeout mode stopped when core is halted */
#  define DBGMCU_CR_TIM8STOP      (1 << 17)  /* Bit 17: TIM8 stopped when core is halted */
#  define DBGMCU_CR_TIM5STOP      (1 << 18)  /* Bit 18: TIM5 stopped when core is halted */
#  define DBGMCU_CR_TIM6STOP      (1 << 19)  /* Bit 19: TIM6 stopped when core is halted */
#  define DBGMCU_CR_TIM7STOP      (1 << 20)  /* Bit 20: TIM7 stopped when core is halted */
#  define DBGMCU_CR_CAN2STOP      (1 << 21)  /* Bit 21: CAN2 stopped when core is halted */
#endif /* CONFIG_STM32_HAVE_IP_DBGMCU_V1 */

#ifdef CONFIG_STM32_HAVE_IP_DBGMCU_V2

/* Debug MCU APB1 freeze register */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define DBGMCU_APB1_TIM2STOP    (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#  define DBGMCU_APB1_TIM3STOP    (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#  define DBGMCU_APB1_TIM4STOP    (1 << 2)   /* Bit 2: TIM4 stopped when core is halted */
#  define DBGMCU_APB1_TIM5STOP    (1 << 3)   /* Bit 3: TIM5 stopped when core is halted */
#  define DBGMCU_APB1_TIM6STOP    (1 << 4)   /* Bit 4: TIM6 stopped when core is halted */
#  define DBGMCU_APB1_TIM7STOP    (1 << 5)   /* Bit 5: TIM7 stopped when core is halted */
#  define DBGMCU_APB1_TIM12STOP   (1 << 6)   /* Bit 6: TIM12 stopped when core is halted */
#  define DBGMCU_APB1_TIM13STOP   (1 << 7)   /* Bit 7: TIM13 stopped when core is halted */
#  define DBGMCU_APB1_TIM14STOP   (1 << 8)   /* Bit 7: TIM14 stopped when core is halted */
#  define DBGMCU_APB1_RTCSTOP     (1 << 10)  /* Bit 10: RTC stopped when Core is halted */
#  define DBGMCU_APB1_WWDGSTOP    (1 << 11)  /* Bit 11: Window Watchdog stopped when core is halted */
#  define DBGMCU_APB1_IWDGSTOP    (1 << 12)  /* Bit 12: Independent Watchdog stopped when core is halted */
#  define DBGMCU_APB1_I2C1STOP    (1 << 21)  /* Bit 21: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_I2C2STOP    (1 << 22)  /* Bit 22: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_I2C3STOP    (1 << 23)  /* Bit 23: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_CAN1STOP    (1 << 25)  /* Bit 25: CAN1 stopped when core is halted */
#  define DBGMCU_APB1_CAN2STOP    (1 << 26)  /* Bit 26: CAN2 stopped when core is halted */
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
      defined(CONFIG_STM32_STM32L15XX)
#  define DBGMCU_APB1_TIM2STOP    (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#  define DBGMCU_APB1_TIM3STOP    (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#  define DBGMCU_APB1_TIM4STOP    (1 << 2)   /* Bit 2: TIM4 stopped when core is halted */
#  define DBGMCU_APB1_TIM6STOP    (1 << 4)   /* Bit 4: TIM6 stopped when core is halted */
#  define DBGMCU_APB1_TIM7STOP    (1 << 5)   /* Bit 5: TIM7 stopped when core is halted */
#  define DBGMCU_APB1_RTCSTOP     (1 << 10)  /* Bit 10: RTC stopped when Core is halted */
#  define DBGMCU_APB1_WWDGSTOP    (1 << 11)  /* Bit 11: Window Watchdog stopped when core is halted */
#  define DBGMCU_APB1_IWDGSTOP    (1 << 12)  /* Bit 12: Independent Watchdog stopped when core is halted */
#  define DBGMCU_APB1_I2C1STOP    (1 << 21)  /* Bit 21: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1_I2C2STOP    (1 << 22)  /* Bit 22: SMBUS timeout mode stopped when Core is halted */
#  if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX)
#    define DBGMCU_APB1_CAN1STOP  (1 << 25)  /* Bit 25: CAN1 stopped when core is halted */
#  endif
#endif

/* Debug MCU APB2 freeze register */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define DBGMCU_APB2_TIM1STOP    (1 << 0)   /* Bit 0:  TIM1 stopped when core is halted */
#  define DBGMCU_APB2_TIM8STOP    (1 << 1)   /* Bit 1:  TIM8 stopped when core is halted */
#  define DBGMCU_APB2_TIM9STOP    (1 << 16)  /* Bit 16: TIM9 stopped when core is halted */
#  define DBGMCU_APB2_TIM10STOP   (1 << 17)  /* Bit 17: TIM10 stopped when core is halted */
#  define DBGMCU_APB2_TIM11STOP   (1 << 18)  /* Bit 18: TIM11 stopped when core is halted */
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX)
#  define DBGMCU_APB2_TIM1STOP    (1 << 0)   /* Bit 0:  TIM1 stopped when core is halted */
#  define DBGMCU_APB2_TIM8STOP    (1 << 1)   /* Bit 1:  TIM8 stopped when core is halted */
#  define DBGMCU_APB2_TIM15STOP   (1 << 2)   /* Bit 2:  TIM15 stopped when core is halted */
#  define DBGMCU_APB2_TIM16STOP   (1 << 3)   /* Bit 3:  TIM16 stopped when core is halted */
#  define DBGMCU_APB2_TIM17STOP   (1 << 4)   /* Bit 4:  TIM17 stopped when core is halted */
#elif defined(CONFIG_STM32_STM32L15XX)
#  define DBGMCU_APB2_TIM9STOP    (1 << 2)   /* Bit 2:  TIM9 stopped when core is halted */
#  define DBGMCU_APB2_TIM10STOP   (1 << 3)   /* Bit 3:  TIM10 stopped when core is halted */
#  define DBGMCU_APB2_TIM11STOP   (1 << 4)   /* Bit 4:  TIM11 stopped when core is halted */
#endif
#endif  /* CONFIG_STM32_HAVE_IP_DBGMCU_V2 */

#ifdef CONFIG_STM32_HAVE_IP_DBGMCU_V3

/* Debug MCU APB1 freeze 1 register */

#  define DBGMCU_APB1FZ1_TIM2STOP    (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#  define DBGMCU_APB1FZ1_TIM3STOP    (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#  define DBGMCU_APB1FZ1_TIM4STOP    (1 << 2)   /* Bit 2: TIM4 stopped when core is halted */
#  define DBGMCU_APB1FZ1_TIM6STOP    (1 << 4)   /* Bit 4: TIM6 stopped when core is halted */
#  define DBGMCU_APB1FZ1_TIM7STOP    (1 << 5)   /* Bit 5: TIM7 stopped when core is halted */
#  define DBGMCU_APB1FZ1_RTCSTOP     (1 << 10)  /* Bit 10: RTC stopped when Core is halted */
#  define DBGMCU_APB1FZ1_WWDGSTOP    (1 << 11)  /* Bit 11: Window Watchdog stopped when core is halted */
#  define DBGMCU_APB1FZ1_IWDGSTOP    (1 << 12)  /* Bit 12: Independent Watchdog stopped when core is halted */
#  define DBGMCU_APB1FZ1_I2C1STOP    (1 << 21)  /* Bit 21: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1FZ1_I2C2STOP    (1 << 22)  /* Bit 22: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1FZ1_I2C3STOP    (1 << 30)  /* Bit 30: SMBUS timeout mode stopped when Core is halted */
#  define DBGMCU_APB1FZ1_LPTIM1STOP  (1 << 31)  /* Bit 31: LPTIM1 counter stopped when Core is halted */

/* Debug MCU APB1 freeze 2 register */

#  define DBGMCU_APB1FZ2_I2C4STOP    (1 << 1)   /* Bit 30: SMBUS timeout mode stopped when Core is halted */

/* Debug MCU APB2 freeze register */

#  define DBGMCU_APB2_TIM1STOP       (1 << 11)  /* Bit 11:  TIM1 stopped when core is halted */
#  define DBGMCU_APB2_TIM8STOP       (1 << 13)  /* Bit 14:  TIM8 stopped when core is halted */
#  define DBGMCU_APB2_TIM15STOP      (1 << 16)  /* Bit 16:  TIM15 stopped when core is halted */
#  define DBGMCU_APB2_TIM16STOP      (1 << 17)  /* Bit 17:  TIM16 stopped when core is halted */
#  define DBGMCU_APB2_TIM17STOP      (1 << 18)  /* Bit 18:  TIM17 stopped when core is halted */
#  define DBGMCU_APB2_TIM20STOP      (1 << 20)  /* Bit 20:  TIM20 stopped when core is halted */
#  define DBGMCU_APB2_HRTIMSTOP      (1 << 26)  /* Bit 20:  HRTIM stopped when core is halted */

#endif  /* CONFIG_STM32_HAVE_IP_DBGMCU_V3 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32_DBGMCU_H */
