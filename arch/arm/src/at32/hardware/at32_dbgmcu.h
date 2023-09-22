/****************************************************************************
 * arch/arm/src/at32/hardware/at32_dbgmcu.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DBGMCU_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DBGMCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define AT32_DEBUG_IDCODE           0xE0042000
#define AT32_DEBUG_CTRL             0xE0042004
#define AT32_DEBUG_APB1_PAUSE       0xE0042008
#define AT32_DEBUG_APB2_PAUSE       0xE004200C
#define AT32_DEBUG_SER_ID           0xE0042020

/* Register Bitfield Definitions ********************************************/

/* MCU identifier */

#define DEBUG_IDCODE_DEVID_SHIFT (0)       /* Bits 11-0: Device Identifier */
#define DEBUG_IDCODE_DEVID_MASK  (0x0fff << DEBUG_IDCODE_DEVID_SHIFT)
#define DEBUG_IDCODE_REVID_SHIFT (16)      /* Bits 31-16:  Revision Identifier */
#define DEBUG_IDCODE_REVID_MASK  (0xffff << DEBUG_IDCODE_REVID_SHIFT)

/* MCU debug */

#define DEBUG_CTRL_SLEEP_DEBUG      (1 << 0) /* Debug sleep mode*/
#define DEBUG_CTRL_DEEPSLEEP_DEBUG  (1 << 1) /* Debug deep sleep mode */
#define DEBUG_CTRL_STANDBY_DEBUG    (1 << 2) /* Debug standby mode */

/* APB1 pause */

#define DEBUG_APB1_APUSE_TMR2_PAUSE             (1 << 0)
#define DEBUG_APB1_APUSE_TMR3_PAUSE             (1 << 1)
#define DEBUG_APB1_APUSE_TMR4_PAUSE             (1 << 2)
#define DEBUG_APB1_APUSE_TMR5_PAUSE             (1 << 3)
#define DEBUG_APB1_APUSE_TMR6_PAUSE             (1 << 4)             
#define DEBUG_APB1_APUSE_TMR7_PAUSE             (1 << 5)
#define DEBUG_APB1_APUSE_TMR12_PAUSE            (1 << 6)
#define DEBUG_APB1_APUSE_TMR13_PAUSE            (1 << 7)
#define DEBUG_APB1_APUSE_TMR14_PAUSE            (1 << 8)
#define DEBUG_APB1_APUSE_ERTC_PAUSE             (1 << 10)
#define DEBUG_APB1_APUSE_WWDT_PAUSE             (1 << 11)
#define DEBUG_APB1_APUSE_WDT_PAUSE              (1 << 12)
#define DEBUG_APB1_APUSE_ERTC_512_PAUSE         (1 << 15)
#define DEBUG_APB1_APUSE_I2C1_SMBUS_TIMEOUT     (1 << 24)
#define DEBUG_APB1_APUSE_CAN1_PAUSE             (1 << 25)
#define DEBUG_APB1_APUSE_CAN2_PAUSE             (1 << 26)
#define DEBUG_APB1_APUSE_I2C2_SMBUS_TIMEOUT     (1 << 27)
#define DEBUG_APB1_APUSE_I2C3_SMBUS_TIMEOUT     (1 << 28)

/* APB2 pause */

#define DEBUG_APB2_APUSE_TMR1_PAUSE             (1 << 0)
#define DEBUG_APB2_APUSE_TMR8_PAUSE             (1 << 1)
#define DEBUG_APB2_APUSE_TMR20_PAUSE            (1 << 6)
#define DEBUG_APB2_APUSE_TMR9_PAUSE             (1 << 16)
#define DEBUG_APB2_APUSE_TMR10_PAUSE            (1 << 17)
#define DEBUG_APB2_APUSE_TMR11_PAUSE            (1 << 18)

/* SER ID */
#define DEBUG_SER_ID_SHIFT                      (8)
#  define DEBUG_SER_ID_F435                     (0x0D << DEBUG_SER_ID_SHIFT)
#  define DEBUG_SER_ID_F437                     (0x0E << DEBUG_SER_ID_SHIFT)

#define DEBUG_REV_ID_SHIFT                      (0)
#  define DEBUG_REV_ID                          (0 << DEBUG_REV_ID_SHIFT)               

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DBGMCU_H */
