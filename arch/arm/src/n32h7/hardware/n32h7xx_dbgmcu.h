/****************************************************************************
 * arch/arm/src/n32h7/hardware/n32h7xx_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7XX_DBGMCU_H
#define __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7XX_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define N32_DBG_ID_OFFSET              0x00  /* ID register */
#define N32_DBG_CTRL_OFFSET            0x04  /* Control register */
#define N32_DBG_M7APB1FZ_OFFSET        0x08  /* M7 APB1 freeze register */
#define N32_DBG_M4APB1FZ_OFFSET        0x0c  /* M4 APB1 freeze register */
#define N32_DBG_M7APB2FZ_OFFSET        0x10  /* M7 APB2 freeze register */
#define N32_DBG_M4APB2FZ_OFFSET        0x14  /* M4 APB2 freeze register */
#define N32_DBG_M7APB5FZ_OFFSET        0x18  /* M7 APB5 freeze register */
#define N32_DBG_M4APB5FZ_OFFSET        0x1c  /* M4 APB5 freeze register */
#define N32_DBG_M7APB6FZ_OFFSET        0x20  /* M7 APB6 freeze register */
#define N32_DBG_M4APB6FZ_OFFSET        0x24  /* M4 APB6 freeze register */

/* Register Addresses *******************************************************/

#define N32_DBG_ID                      (N32_DEBUGMCU_BASE + N32_DBG_ID_OFFSET)
#define N32_DBG_CTRL                    (N32_DEBUGMCU_BASE + N32_DBG_CTRL_OFFSET)
#define N32_DBG_M7APB1FZ                (N32_DEBUGMCU_BASE + N32_DBG_M7APB1FZ_OFFSET)
#define N32_DBG_M4APB1FZ                (N32_DEBUGMCU_BASE + N32_DBG_M4APB1FZ_OFFSET)
#define N32_DBG_M7APB2FZ                (N32_DEBUGMCU_BASE + N32_DBG_M7APB2FZ_OFFSET)
#define N32_DBG_M4APB2FZ                (N32_DEBUGMCU_BASE + N32_DBG_M4APB2FZ_OFFSET)
#define N32_DBG_M7APB5FZ                (N32_DEBUGMCU_BASE + N32_DBG_M7APB5FZ_OFFSET)
#define N32_DBG_M4APB5FZ                (N32_DEBUGMCU_BASE + N32_DBG_M4APB5FZ_OFFSET)
#define N32_DBG_M7APB6FZ                (N32_DEBUGMCU_BASE + N32_DBG_M7APB6FZ_OFFSET)
#define N32_DBG_M4APB6FZ                (N32_DEBUGMCU_BASE + N32_DBG_M4APB6FZ_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* ID register (N32_DBG_ID) */

#define DBG_ID_REV_NUM_H_SHIFT          (28)      /* Bits 31-28: MCU revision number high */
#define DBG_ID_REV_NUM_H_MASK           (0xf << DBG_ID_REV_NUM_H_SHIFT)
#define DBG_ID_REV_NUM_L_SHIFT          (24)      /* Bits 27-24: MCU revision number low */
#define DBG_ID_REV_NUM_L_MASK           (0xf << DBG_ID_REV_NUM_L_SHIFT)
#define DBG_ID_DEV_NUM_H_SHIFT          (20)      /* Bits 23-20: Device number high */
#define DBG_ID_DEV_NUM_H_MASK           (0xf << DBG_ID_DEV_NUM_H_SHIFT)
#define DBG_ID_DEV_NUM_M_SHIFT          (16)      /* Bits 19-16: Device number middle */
#define DBG_ID_DEV_NUM_M_MASK           (0xf << DBG_ID_DEV_NUM_M_SHIFT)
#define DBG_ID_DEV_NUM_L_SHIFT          (12)      /* Bits 15-12: Device number low */
#define DBG_ID_DEV_NUM_L_MASK           (0xf << DBG_ID_DEV_NUM_L_SHIFT)

/* Device number values (DEV_NUM_H + DEV_NUM_M + DEV_NUM_L) */
#define DBG_ID_DEV_NUM_N32H760          0x760
#define DBG_ID_DEV_NUM_N32H762          0x762
#define DBG_ID_DEV_NUM_N32H765          0x765
#define DBG_ID_DEV_NUM_N32H785          0x785
#define DBG_ID_DEV_NUM_N32H787          0x787
#define DBG_ID_DEV_NUM_N32H788          0x788

#define DBG_ID_PINS_SHIFT               (8)       /* Bits 11-8: Pin count */
#define DBG_ID_PINS_MASK                (0xf << DBG_ID_PINS_SHIFT)
#define DBG_ID_PINS_100                 0x2
#define DBG_ID_PINS_144                 0x4
#define DBG_ID_PINS_169                 0x6
#define DBG_ID_PINS_176                 0x8
#define DBG_ID_PINS_208                 0xa
#define DBG_ID_PINS_240                 0xc

#define DBG_ID_FLASH_SHIFT              (6)       /* Bits 7-6: Flash size */
#define DBG_ID_FLASH_MASK               (0x3 << DBG_ID_FLASH_SHIFT)
#define DBG_ID_FLASH_2MB                (0x1 << DBG_ID_FLASH_SHIFT)
#define DBG_ID_FLASH_4MB                (0x2 << DBG_ID_FLASH_SHIFT)

#define DBG_ID_ENC_SHIFT                (4)       /* Bits 5-4: Package type */
#define DBG_ID_ENC_MASK                 (0x3 << DBG_ID_ENC_SHIFT)
#define DBG_ID_ENC_BGA                  (0x1 << DBG_ID_ENC_SHIFT)
#define DBG_ID_ENC_LQFP                 (0x2 << DBG_ID_ENC_SHIFT)

/* Control register (N32_DBG_CTRL) */

#define DBG_CTRL_M4SLEEP                (1 << 0)  /* Bit 0:  Debug sleep mode for CM4 */
#define DBG_CTRL_M4STOP                 (1 << 1)  /* Bit 1:  Debug stop mode for CM4 */
#define DBG_CTRL_M4STBY                 (1 << 2)  /* Bit 2:  Debug standby mode for CM4 */
#define DBG_CTRL_M7SLEEP                (1 << 3)  /* Bit 3:  Debug sleep mode for CM7 */
#define DBG_CTRL_M7STOP                 (1 << 4)  /* Bit 4:  Debug stop mode for CM7 */
#define DBG_CTRL_M7STBY                 (1 << 5)  /* Bit 5:  Debug standby mode for CM7 */
#define DBG_CTRL_TRGOEN                 (1 << 6)  /* Bit 6:  External trigger output enable */
#define DBG_CTRL_TRACE_MODE_SHIFT       (7)       /* Bits 9-7: Trace mode */
#define DBG_CTRL_TRACE_MODE_MASK        (0x7 << DBG_CTRL_TRACE_MODE_SHIFT)
#define DBG_CTRL_TRACE_IOEN             (1 << 7)  /* Bit 7:  Trace I/O enable */
#define DBG_CTRL_TRACE_MODE_ASYNC       (0 << 8)  /* Bits 9-8: Asynchronous mode */
#define DBG_CTRL_TRACE_MODE_SYNC1       (1 << 8)  /* Bits 9-8: Sync mode, 1-bit data */
#define DBG_CTRL_TRACE_MODE_SYNC2       (2 << 8)  /* Bits 9-8: Sync mode, 2-bit data */
#define DBG_CTRL_TRACE_MODE_SYNC4       (3 << 8)  /* Bits 9-8: Sync mode, 4-bit data */
#define DBG_CTRL_TRACE_CFG              (1 << 10) /* Bit 10: Trace source selection (0=CM7, 1=CM4) */

/* M7 APB1 freeze register (N32_DBG_M7APB1FZ) - also used for M4 APB1 */

#define DBG_M7APB1FZ_BTIM1_STOP         (1 << 0)  /* Bit 0:  BTIM1 stopped when halted */
#define DBG_M7APB1FZ_BTIM2_STOP         (1 << 1)  /* Bit 1:  BTIM2 stopped when halted */
#define DBG_M7APB1FZ_BTIM3_STOP         (1 << 2)  /* Bit 2:  BTIM3 stopped when halted */
#define DBG_M7APB1FZ_BTIM4_STOP         (1 << 3)  /* Bit 3:  BTIM4 stopped when halted */
#define DBG_M7APB1FZ_GTIMA4_STOP        (1 << 4)  /* Bit 4:  GTIMA4 stopped when halted */
#define DBG_M7APB1FZ_GTIMA5_STOP        (1 << 5)  /* Bit 5:  GTIMA5 stopped when halted */
#define DBG_M7APB1FZ_GTIMA6_STOP        (1 << 6)  /* Bit 6:  GTIMA6 stopped when halted */
#define DBG_M7APB1FZ_GTIMA7_STOP        (1 << 7)  /* Bit 7:  GTIMA7 stopped when halted */
#define DBG_M7APB1FZ_GTIMB1_STOP        (1 << 8)  /* Bit 8:  GTIMB1 stopped when halted */
#define DBG_M7APB1FZ_GTIMB2_STOP        (1 << 9)  /* Bit 9:  GTIMB2 stopped when halted */
#define DBG_M7APB1FZ_GTIMB3_STOP        (1 << 10) /* Bit 10: GTIMB3 stopped when halted */
#define DBG_M7APB1FZ_I2C1_STOP          (1 << 11) /* Bit 11: I2C1 SMBus timeout stopped */
#define DBG_M7APB1FZ_I2C2_STOP          (1 << 12) /* Bit 12: I2C2 SMBus timeout stopped */
#define DBG_M7APB1FZ_I2C3_STOP          (1 << 13) /* Bit 13: I2C3 SMBus timeout stopped */
#define DBG_M7APB1FZ_WWDG2_STOP         (1 << 14) /* Bit 14: WWDG2 stopped when halted */
#define DBG_M7APB1FZ_CANFD1_STOP        (1 << 15) /* Bit 15: FDCAN1 stopped when halted */
#define DBG_M7APB1FZ_CANFD2_STOP        (1 << 16) /* Bit 16: FDCAN2 stopped when halted */
#define DBG_M7APB1FZ_CANFD5_STOP        (1 << 17) /* Bit 17: FDCAN5 stopped when halted */
#define DBG_M7APB1FZ_CANFD6_STOP        (1 << 18) /* Bit 18: FDCAN6 stopped when halted */

/* M4 APB1 freeze register (N32_DBG_M4APB1FZ) */

#define DBG_M4APB1FZ_BTIM1_STOP         DBG_M7APB1FZ_BTIM1_STOP
#define DBG_M4APB1FZ_BTIM2_STOP         DBG_M7APB1FZ_BTIM2_STOP
#define DBG_M4APB1FZ_BTIM3_STOP         DBG_M7APB1FZ_BTIM3_STOP
#define DBG_M4APB1FZ_BTIM4_STOP         DBG_M7APB1FZ_BTIM4_STOP
#define DBG_M4APB1FZ_GTIMA4_STOP        DBG_M7APB1FZ_GTIMA4_STOP
#define DBG_M4APB1FZ_GTIMA5_STOP        DBG_M7APB1FZ_GTIMA5_STOP
#define DBG_M4APB1FZ_GTIMA6_STOP        DBG_M7APB1FZ_GTIMA6_STOP
#define DBG_M4APB1FZ_GTIMA7_STOP        DBG_M7APB1FZ_GTIMA7_STOP
#define DBG_M4APB1FZ_GTIMB1_STOP        DBG_M7APB1FZ_GTIMB1_STOP
#define DBG_M4APB1FZ_GTIMB2_STOP        DBG_M7APB1FZ_GTIMB2_STOP
#define DBG_M4APB1FZ_GTIMB3_STOP        DBG_M7APB1FZ_GTIMB3_STOP
#define DBG_M4APB1FZ_I2C1_STOP          DBG_M7APB1FZ_I2C1_STOP
#define DBG_M4APB1FZ_I2C2_STOP          DBG_M7APB1FZ_I2C2_STOP
#define DBG_M4APB1FZ_I2C3_STOP          DBG_M7APB1FZ_I2C3_STOP
#define DBG_M4APB1FZ_WWDG2_STOP         DBG_M7APB1FZ_WWDG2_STOP
#define DBG_M4APB1FZ_CANFD1_STOP        DBG_M7APB1FZ_CANFD1_STOP
#define DBG_M4APB1FZ_CANFD2_STOP        DBG_M7APB1FZ_CANFD2_STOP
#define DBG_M4APB1FZ_CANFD5_STOP        DBG_M7APB1FZ_CANFD5_STOP
#define DBG_M4APB1FZ_CANFD6_STOP        DBG_M7APB1FZ_CANFD6_STOP

/* M7 APB2 freeze register (N32_DBG_M7APB2FZ) */

#define DBG_M7APB2FZ_SHRTIM1_STOP       (1 << 0)  /* Bit 0:  SHRTIM1 stopped when halted */
#define DBG_M7APB2FZ_SHRTIM2_STOP       (1 << 1)  /* Bit 1:  SHRTIM2 stopped when halted */
#define DBG_M7APB2FZ_GTIMA1_STOP        (1 << 2)  /* Bit 2:  GTIMA1 stopped when halted */
#define DBG_M7APB2FZ_GTIMA2_STOP        (1 << 3)  /* Bit 3:  GTIMA2 stopped when halted */
#define DBG_M7APB2FZ_GTIMA3_STOP        (1 << 4)  /* Bit 4:  GTIMA3 stopped when halted */
#define DBG_M7APB2FZ_ATIM1_STOP         (1 << 5)  /* Bit 5:  ATIM1 stopped when halted */
#define DBG_M7APB2FZ_ATIM2_STOP         (1 << 6)  /* Bit 6:  ATIM2 stopped when halted */
#define DBG_M7APB2FZ_I2C4_STOP          (1 << 7)  /* Bit 7:  I2C4 SMBus timeout stopped */
#define DBG_M7APB2FZ_I2C5_STOP          (1 << 8)  /* Bit 8:  I2C5 SMBus timeout stopped */
#define DBG_M7APB2FZ_I2C6_STOP          (1 << 9)  /* Bit 9:  I2C6 SMBus timeout stopped */
#define DBG_M7APB2FZ_CANFD3_STOP        (1 << 10) /* Bit 10: FDCAN3 stopped when halted */
#define DBG_M7APB2FZ_CANFD4_STOP        (1 << 11) /* Bit 11: FDCAN4 stopped when halted */
#define DBG_M7APB2FZ_CANFD7_STOP        (1 << 12) /* Bit 12: FDCAN7 stopped when halted */
#define DBG_M7APB2FZ_CANFD8_STOP        (1 << 13) /* Bit 13: FDCAN8 stopped when halted */

/* M4 APB2 freeze register (N32_DBG_M4APB2FZ) */

#define DBG_M4APB2FZ_SHRTIM1_STOP       DBG_M7APB2FZ_SHRTIM1_STOP
#define DBG_M4APB2FZ_SHRTIM2_STOP       DBG_M7APB2FZ_SHRTIM2_STOP
#define DBG_M4APB2FZ_GTIMA1_STOP        DBG_M7APB2FZ_GTIMA1_STOP
#define DBG_M4APB2FZ_GTIMA2_STOP        DBG_M7APB2FZ_GTIMA2_STOP
#define DBG_M4APB2FZ_GTIMA3_STOP        DBG_M7APB2FZ_GTIMA3_STOP
#define DBG_M4APB2FZ_ATIM1_STOP         DBG_M7APB2FZ_ATIM1_STOP
#define DBG_M4APB2FZ_ATIM2_STOP         DBG_M7APB2FZ_ATIM2_STOP
#define DBG_M4APB2FZ_I2C4_STOP          DBG_M7APB2FZ_I2C4_STOP
#define DBG_M4APB2FZ_I2C5_STOP          DBG_M7APB2FZ_I2C5_STOP
#define DBG_M4APB2FZ_I2C6_STOP          DBG_M7APB2FZ_I2C6_STOP
#define DBG_M4APB2FZ_CANFD3_STOP        DBG_M7APB2FZ_CANFD3_STOP
#define DBG_M4APB2FZ_CANFD4_STOP        DBG_M7APB2FZ_CANFD4_STOP
#define DBG_M4APB2FZ_CANFD7_STOP        DBG_M7APB2FZ_CANFD7_STOP
#define DBG_M4APB2FZ_CANFD8_STOP        DBG_M7APB2FZ_CANFD8_STOP

/* M7 APB5 freeze register (N32_DBG_M7APB5FZ) - ATIM3/ATIM4 and LPTIM */

#define DBG_M7APB5FZ_ATIM3_STOP         (1 << 0)  /* Bit 0:  ATIM3 stopped when halted */
#define DBG_M7APB5FZ_ATIM4_STOP         (1 << 1)  /* Bit 1:  ATIM4 stopped when halted */
#define DBG_M7APB5FZ_I2C7_STOP          (1 << 2)  /* Bit 2:  I2C7 SMBus timeout stopped */
#define DBG_M7APB5FZ_I2C8_STOP          (1 << 3)  /* Bit 3:  I2C8 SMBus timeout stopped */
#define DBG_M7APB5FZ_I2C9_STOP          (1 << 4)  /* Bit 4:  I2C9 SMBus timeout stopped */
#define DBG_M7APB5FZ_I2C10_STOP         (1 << 5)  /* Bit 5:  I2C10 SMBus timeout stopped */
#define DBG_M7APB5FZ_IWDG1_STOP         (1 << 6)  /* Bit 6:  IWDG1 stopped when halted */
#define DBG_M7APB5FZ_IWDG2_STOP         (1 << 7)  /* Bit 7:  IWDG2 stopped when halted */
#define DBG_M7APB5FZ_LPTIM1_STOP        (1 << 8)  /* Bit 8:  LPTIM1 stopped when halted */
#define DBG_M7APB5FZ_LPTIM2_STOP        (1 << 9)  /* Bit 9:  LPTIM2 stopped when halted */
#define DBG_M7APB5FZ_LPTIM3_STOP        (1 << 10) /* Bit 10: LPTIM3 stopped when halted */
#define DBG_M7APB5FZ_LPTIM4_STOP        (1 << 11) /* Bit 11: LPTIM4 stopped when halted */
#define DBG_M7APB5FZ_LPTIM5_STOP        (1 << 12) /* Bit 12: LPTIM5 stopped when halted */
#define DBG_M7APB5FZ_RTC_STOP           (1 << 13) /* Bit 13: RTC stopped when halted */

/* M4 APB5 freeze register (N32_DBG_M4APB5FZ) */

#define DBG_M4APB5FZ_ATIM3_STOP         DBG_M7APB5FZ_ATIM3_STOP
#define DBG_M4APB5FZ_ATIM4_STOP         DBG_M7APB5FZ_ATIM4_STOP
#define DBG_M4APB5FZ_I2C7_STOP          DBG_M7APB5FZ_I2C7_STOP
#define DBG_M4APB5FZ_I2C8_STOP          DBG_M7APB5FZ_I2C8_STOP
#define DBG_M4APB5FZ_I2C9_STOP          DBG_M7APB5FZ_I2C9_STOP
#define DBG_M4APB5FZ_I2C10_STOP         DBG_M7APB5FZ_I2C10_STOP
#define DBG_M4APB5FZ_IWDG1_STOP         DBG_M7APB5FZ_IWDG1_STOP
#define DBG_M4APB5FZ_IWDG2_STOP         DBG_M7APB5FZ_IWDG2_STOP
#define DBG_M4APB5FZ_LPTIM1_STOP        DBG_M7APB5FZ_LPTIM1_STOP
#define DBG_M4APB5FZ_LPTIM2_STOP        DBG_M7APB5FZ_LPTIM2_STOP
#define DBG_M4APB5FZ_LPTIM3_STOP        DBG_M7APB5FZ_LPTIM3_STOP
#define DBG_M4APB5FZ_LPTIM4_STOP        DBG_M7APB5FZ_LPTIM4_STOP
#define DBG_M4APB5FZ_LPTIM5_STOP        DBG_M7APB5FZ_LPTIM5_STOP
#define DBG_M4APB5FZ_RTC_STOP           DBG_M7APB5FZ_RTC_STOP

/* M7 APB6 freeze register (N32_DBG_M7APB6FZ) */

#define DBG_M7APB6FZ_WWDG1_STOP         (1 << 0)  /* Bit 0: WWDG1 stopped when halted */

/* M4 APB6 freeze register (N32_DBG_M4APB6FZ) */

#define DBG_M4APB6FZ_WWDG1_STOP         (1 << 0)  /* Bit 0: WWDG1 stopped when halted */

#endif /* __ARCH_ARM_SRC_N32H7_HARDWARE_N32H7XX_DBGMCU_H */
