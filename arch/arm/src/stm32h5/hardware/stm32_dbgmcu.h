/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DBGMCU_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define STM32_DBGMCU_IDCODE           0xe0044000  /* MCU identifier */
#define STM32_DBGMCU_CR               0xe0044004  /* MCU debug */
#define STM32_DBGMCU_APB1L_FZ         0xe0044008  /* Debug MCU APB1L freeze register */
#define STM32_DBGMCU_APB1H_FZ         0xe004400c  /* Debug MCU APB1H freeze register */
#define STM32_DBGMCU_APB2_FZ          0xe0044010  /* Debug MCU APB2 freeze register */
#define STM32_DBGMCU_APB3_FZ          0xe0044014  /* Debug MCU APB3 freeze register */
#define STM32_DBGMCU_AHB1_FZ          0xe0044020  /* Debug MCU AHB1 freeze register */
#define STM32_DBGMCU_SR               0xe00440fc  /* Debug MCU Status Register */
#define STM32_DBGMCU_DBG_AUTH_HOST    0xe0044100  /* Debug MCU Authorization Host Register */
#define STM32_DBGMCU_DBG_AUTH_DEVICE  0xe0044104  /* Debug MCU Authorization Device Register */
#define STM32_DBGMCU_DBG_AUTH_ACK     0xe0044108  /* Debug MCU Authorization Acknowledge Register */
#define STM32_DBGMCU_PIDR4            0xe0044fd0  /* Debug MCU CoreSight Peripheral Identity Register 4 */
#define STM32_DBGMCU_PIDR0            0xe0044fe0  /* Debug MCU CoreSight Peripheral Identity Register 0 */
#define STM32_DBGMCU_PIDR1            0xe0044fe4  /* Debug MCU CoreSight Peripheral Identity Register 1 */
#define STM32_DBGMCU_PIDR2            0xe0044fe8  /* Debug MCU CoreSight Peripheral Identity Register 2 */
#define STM32_DBGMCU_PIDR3            0xe0044fec  /* Debug MCU CoreSight Peripheral Identity Register 3 */
#define STM32_DBGMCU_CIDR0            0xe0044ff0  /* Debug MCU CoreSight Component Identity Register 0 */
#define STM32_DBGMCU_CIDR1            0xe0044ff4  /* Debug MCU CoreSight Component Identity Register 1 */
#define STM32_DBGMCU_CIDR2            0xe0044ff8  /* Debug MCU CoreSight Component Identity Register 2 */
#define STM32_DBGMCU_CIDR3            0xe0044ffc  /* Debug MCU CoreSight Component Identity Register 3 */

/* Register Bitfield Definitions ********************************************/

/* MCU identifier */

#define DBGMCU_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DBGMCU_IDCODE_DEVID_MASK  (0x0fff << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DBGMCU_IDCODE_REVID_MASK  (0xffff << DBGMCU_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DBGMCU_CR_STOP            (1 << 1)  /* Bit 1: Allows debug in Stop mode */
#define DBGMCU_CR_STANDBY         (1 << 2)  /* Bit 2: Allows debug in Standby mode */
#define DBGMCU_CR_TRACEIOEN       (1 << 4)  /* Bit 4: Trace pin enable */
#define DBGMCU_CR_TRACEEN         (1 << 5)  /* Bit 5: Trace port and clock enable */
#define DBGMCU_CR_TRACEMODE_SHIFT (6)       /* Bits 7-6: Trace mode pin assignment */
#define DBGMCU_CR_TRACEMODE_MASK  (3 << DBGMCU_CR_TRACEMODE_SHIFT)
#define DBGMCU_CR_ASYNCH          (0 << DBGMCU_CR_TRACEMODE_SHIFT) /* Asynchronous Mode */
#define DBGMCU_CR_SYNCH1          (1 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=1 */
#define DBGMCU_CR_SYNCH2          (2 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=2 */
#define DBGMCU_CR_SYNCH4          (3 << DBGMCU_CR_TRACEMODE_SHIFT) /* Synchronous Mode, TRACEDATA size=4 */

#define DBGMCU_CR_DCRT            (1 << 16)  /* Bit 16: Debug credentials reset type */

/* Debug MCU APB1L freeze register */

#define DBGMCU_APB1L_TIM2_STOP      (1 << 0)   /* Bit 0: TIM2 stopped when core is halted */
#define DBGMCU_APB1L_TIM3_STOP      (1 << 1)   /* Bit 1: TIM3 stopped when core is halted */
#define DBGMCU_APB1L_TIM4_STOP      (1 << 2)   /* Bit 2: TIM4 stopped when core is halted */
#define DBGMCU_APB1L_TIM5_STOP      (1 << 3)   /* Bit 3: TIM5 stopped when core is halted */
#define DBGMCU_APB1L_TIM6_STOP      (1 << 4)   /* Bit 4: TIM6 stopped when core is halted */
#define DBGMCU_APB1L_TIM7_STOP      (1 << 5)   /* Bit 5: TIM7 stopped when core is halted */
#define DBGMCU_APB1L_TIM12_STOP     (1 << 6)   /* Bit 6: TIM12 stopped when core is halted */
#define DBGMCU_APB1L_TIM13_STOP     (1 << 7)   /* Bit 6: TIM12 stopped when core is halted */
#define DBGMCU_APB1L_TIM14_STOP     (1 << 8)   /* Bit 6: TIM12 stopped when core is halted */
#define DBGMCU_APB1L_WWDG_STOP      (1 << 11)  /* Bit 11: Window Watchdog stopped when core is halted */
#define DBGMCU_APB1L_IWDG_STOP      (1 << 12)  /* Bit 12: Independent Watchdog stopped when core is halted */
#define DBGMCU_APB1L_I2C1_STOP      (1 << 21)  /* Bit 21: I2C1 SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_APB1L_I2C2_STOP      (1 << 22)  /* Bit 22: I2C2 SMBUS timeout mode stopped when Core is halted */
#define DBGMCU_APB1L_I3C1_STOP      (1 << 23)  /* Bit 23: I2C3 SMBUS timeout mode stopped when Core is halted */

/* Debug MCU APB1H freeze register */

#define DBGMCU_APB1H_LPTIM2_STOP    (1 << 5)   /* Bit 5: LPTIM2 stopped when core is halted */

/* Debug MCU APB2 freeze register */

#define DBGMCU_APB2_TIM1_STOP      (1 << 11)  /* Bit 11: TIM1 stopped when core is halted */
#define DBGMCU_APB2_TIM8_STOP      (1 << 13)  /* Bit 13: TIM8 stopped when core is halted */
#define DBGMCU_APB2_TIM15_STOP     (1 << 16)  /* Bit 16: TIM15 stopped when core is halted */
#define DBGMCU_APB2_TIM16_STOP     (1 << 17)  /* Bit 17: TIM16 stopped when core is halted */
#define DBGMCU_APB2_TIM17_STOP     (1 << 18)  /* Bit 18: TIM17 stopped when core is halted */

/* Debug MCU APB3 freeze register */

#define DBGMCU_APB3_I2C3_STOP     (1 << 10)    /* Bit 10: I2C3 SMBUS stop in debug */
#define DBGMCU_APB3_I2C4_STOP     (1 << 11)    /* Bit 11: I2C4 SMBUS stop in debug */
#define DBGMCU_APB3_LPTIM1_STOP   (1 << 17)    /* Bit 17: LPTIM1 stopped in debug */
#define DBGMCU_APB3_LPTIM3_STOP   (1 << 18)    /* Bit 18: LPTIM3 stopped in debug */
#define DBGMCU_APB3_LPTIM4_STOP   (1 << 19)    /* Bit 19: LPTIM4 stopped in debug */
#define DBGMCU_APB3_LPTIM5_STOP   (1 << 20)    /* Bit 20: LPTIM5 stopped in debug */
#define DBGMCU_APB3_LPTIM6_STOP   (1 << 21)    /* Bit 21: LPTIM6 stopped in debug */
#define DBGMCU_APB3_DBG_RTC_STOP  (1 << 30)    /* Bit 30: RTC stopped in debug */

/* Debug MCU AHB1 freeze register */

#define DBGMCU_AHB1_GPDMA1_0_STOP (1 << 0)    /* Bit 0:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_1_STOP (1 << 1)    /* Bit 1:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_2_STOP (1 << 2)    /* Bit 2:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_3_STOP (1 << 3)    /* Bit 3:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_4_STOP (1 << 4)    /* Bit 4:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_5_STOP (1 << 5)    /* Bit 5:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_6_STOP (1 << 6)    /* Bit 6:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA1_7_STOP (1 << 7)    /* Bit 7:  GPDMA1 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA2_0_STOP (1 << 16)   /* Bit 16: GPDMA2 Channel 0 stop in debug */
#define DBGMCU_AHB1_GPDMA2_1_STOP (1 << 17)   /* Bit 17: GPDMA2 Channel 1 stop in debug */
#define DBGMCU_AHB1_GPDMA2_2_STOP (1 << 18)   /* Bit 18: GPDMA2 Channel 2 stop in debug */
#define DBGMCU_AHB1_GPDMA2_3_STOP (1 << 19)   /* Bit 19: GPDMA2 Channel 3 stop in debug */
#define DBGMCU_AHB1_GPDMA2_4_STOP (1 << 20)   /* Bit 20: GPDMA2 Channel 4 stop in debug */
#define DBGMCU_AHB1_GPDMA2_5_STOP (1 << 21)   /* Bit 21: GPDMA2 Channel 5 stop in debug */
#define DBGMCU_AHB1_GPDMA2_6_STOP (1 << 22)   /* Bit 22: GPDMA2 Channel 6 stop in debug */
#define DBGMCU_AHB1_GPDMA2_7_STOP (1 << 23)   /* Bit 23: GPDMA2 Channel 7 stop in debug */

/* Debug MCU Status Register */

#define DBGMCU_SR_AP_PRESENT_SHIFT (0)
#define DBGMCU_SR_AP_PRESENT_MASK  (0xffff << DBGMCU_SR_AP_PRESENT_SHIFT)
#define DBGMCU_SR_AP_PRESENT       DBGMCU_SR_AP_PRESENT_MASK
#define DBGMCU_SR_AP_ENABLED_SHIFT (16)
#define DBGMCU_SR_AP_ENABLED_MASK  (0xffff << DBGMCU_SR_AP_ENABLED_SHIFT)
#define DBGMCU_SR_AP_ENABLED       DBGMCU_SR_AP_ENABLED_MASK

/* Debug MCU Authorization Acknowledge Register */

#define DBGMCU_DBG_AUTH_ACK_HOST_ACK_SHIFT (0)
#define DBGMCU_DBG_AUTH_ACK_HOST_ACK_MASK  (1 << DBGMCU_DBG_AUTH_ACK_HOST_ACK_SHIFT)
#define DBGMCU_DBG_AUTH_ACK_HOST_ACK       DBGMCU_DBG_AUTH_ACK_HOST_ACK_MASK
#define DBGMCU_DBG_AUTH_ACK_DEV_ACK_SHIFT  (1)
#define DBGMCU_DBG_AUTH_ACK_DEV_ACK_MASK   (1 << DBGMCU_DBG_AUTH_ACK_DEV_ACK_SHIFT)
#define DBGMCU_DBG_AUTH_ACK_DEV_ACK        DBGMCU_DBG_AUTH_ACK_DEV_ACK_MASK

/* Debug MCU CoreSight Peripheral Identity Register 4 */

#define DBGMCU_PIDR4_JEP106CON_SHIFT (0)
#define DBGMCU_PIDR4_JEP106CON_MASK  (0xf << DBGMCU_PIDR4_JEP106CON_SHIFT)
#define DBGMCU_PIDR4_JEP106CON       DBGMCU_PIDR4_JEP106CON_MASK
#define DBGMCU_PIDR4_SIZE_SHIFT     (0)
#define DBGMCU_PIDR4_SIZE_MASK      (0xf << DBGMCU_PIDR4_SIZE_SHIFT)
#define DBGMCU_PIDR4_SIZE           DBGMCU_PIDR4_SIZE_MASK

/* Debug MCU CoreSight Peripheral Identity Register 0 */

#define DBGMCU_PIDR0_PARTNUM_SHIFT (0)
#define DBGMCU_PIDR0_PARTNUM_MASK  (0xff << DBGMCU_PIDR0_PARTNUM_SHIFT)
#define DBGMCU_PIDR0_PARTNUM       DBGMCU_PIDR0_PARTNUM_MASK

/* Debug MCU CoreSight Peripheral Identity Register 1 */

#define DBGMCU_PIDR1_PARTNUM_SHIFT  (0)
#define DBGMCU_PIDR1_PARTNUM_MASK   (0xf << DBGMCU_PIDR1_PARTNUM_SHIFT)
#define DBGMCU_PIDR1_PARTNUM        DBGMCU_PIDR1_PARTNUM_MASK
#define DBGMCU_PIDR1_JEP106ID_SHIFT (4)
#define DBGMCU_PIDR1_JEP106ID_MASK  (0xf << DBGMCU_PIDR1_JEP106ID_SHIFT)
#define DBGMCU_PIDR1_JEP106ID       DBGMCU_PIDR1_JEP106ID_MASK

/* Debug MCU CoreSight Peripheral Identity Register 2 */

#define DBGMCU_PIDR2_JEP106ID_SHIFT  (0)
#define DBGMCU_PIDR2_JEP106ID_MASK   (0x7 << DBGMCU_PIDR2_JEP106ID_SHIFT)
#define DBGMCU_PIDR2_JEP106ID        DBGMCU_PIDR2_JEP106ID_MASK
#define DBGMCU_PIDR2_JEDEC_SHIFT     (3)
#define DBGMCU_PIDR2_JEDEC_MASK      (0x1 << DBGMCU_PIDR2_JEDEC_SHIFT)
#define DBGMCU_PIDR2_JEDEC           DBGMCU_PIDR2_JEDEC_MASK
#define DBGMCU_PIDR2_REVISION_SHIFT  (4)
#define DBGMCU_PIDR2_REVISION_MASK   (0xf << DBGMCU_PIDR2_REVISION_SHIFT)
#define DBGMCU_PIDR2_REVISION        DBGMCU_PIDR2_REVISION_MASK

/* Debug MCU CoreSight Peripheral Identity Register 3 */

#define DBGMCU_PIDR3_CMOD_SHIFT   (0)
#define DBGMCU_PIDR3_CMOD_MASK    (0xf << DBGMCU_PIDR3_CMOD_SHIFT)
#define DBGMCU_PIDR3_CMOD         DBGMCU_PIDR3_CMOD_MASK
#define DBGMCU_PIDR3_REVAND_SHIFT (4)
#define DBGMCU_PIDR3_REVAND_MASK  (0xf << DBGMCU_PIDR3_REVAND_SHIFT)
#define DBGMCU_PIDR3_REVAND       DBGMCU_PIDR3_REVAND_MASK

/* Debug MCU CoreSight Component  Identity Register 0 */

#define DBGMCU_CIDR0_PREAMBLE_SHIFT  (0)
#define DBGMCU_CIDR0_PREAMBLE_MASK   (0xff << DBGMCU_CIDR0_PREAMBLE_SHIFT)
#define DBGMCU_CIDR0_PREAMBLE        DBGMCU_CIDR0_PREAMBLE_MASK

/* Debug MCU CoreSight Component  Identity Register 1 */

#define DBGMCU_CIDR1_PREAMBLE_SHIFT  (0)
#define DBGMCU_CIDR1_PREAMBLE_MASK   (0xf << DBGMCU_CIDR1_PREAMBLE_SHIFT)
#define DBGMCU_CIDR1_PREAMBLE        DBGMCU_CIDR1_PREAMBLE_MASK
#define DBGMCU_CIDR1_CLASS_SHIFT     (4)
#define DBGMCU_CIDR1_CLASS_MASK      (0xf << DBGMCU_CIDR1_CLASS_SHIFT)
#define DBGMCU_CIDR1_CLASS           DBGMCU_CIDR1_CLASS_MASK

/* Debug MCU CoreSight Component  Identity Register 2 */

#define DBGMCU_CIDR2_PREAMBLE_SHIFT  (0)
#define DBGMCU_CIDR2_PREAMBLE_MASK   (0xff << DBGMCU_CIDR2_PREAMBLE_SHIFT)
#define DBGMCU_CIDR2_PREAMBLE        DBGMCU_CIDR2_PREAMBLE_MASK

/* Debug MCU CoreSight Component  Identity Register 3 */

#define DBGMCU_CIDR3_PREAMBLE_SHIFT  (0)
#define DBGMCU_CIDR3_PREAMBLE_MASK   (0xff << DBGMCU_CIDR3_PREAMBLE_SHIFT)
#define DBGMCU_CIDR3_PREAMBLE        DBGMCU_CIDR3_PREAMBLE_MASK

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32_DBGMCU_H */
