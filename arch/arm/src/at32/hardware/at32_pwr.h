/****************************************************************************
 * arch/arm/src/at32/hardware/at32_pwr.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_PWR_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_PWR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define AT32_PWC_CTRL_OFFSET        0x0000  /* Power control register */
#define AT32_PWC_CTRLSTS_OFFSET     0x0004  /* Power control/status register */
#define AT32_PWC_LDOOV_OFFSET       0x0010  /* Ldo output voltage select */

/* Register Addresses *******************************************************/

#define AT32_PWC_CTRL           (AT32_PWC_BASE+AT32_PWC_CTRL_OFFSET)
#define AT32_PWC_CTRLSTS        (AT32_PWC_BASE+AT32_PWC_CTRLSTS_OFFSET)
#define AT32_PWC_LDOOV          (AT32_PWC_BASE+AT32_PWC_LDOOV_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Power control register */

#define PWC_CTRL_VRSEL         (1 << 0) /* Voltage regulator state select when deepsleep mode */
#define PWC_CTRL_LPSEL         (1 << 1) /* Low power mode select when Cortex™-M4F sleepdeep*/
#define PWC_CTRL_CLSWEF        (1 << 2) /* Clear SWEF flag */
#define PWC_CTRL_CLSEF         (1 << 3) /* Clear SEF flag */
#define PWC_CTRL_PVMEN         (1 << 4) /* Power voltage monitoring enable */

#define PWC_CTRL_PVMSEL_SHIFT  (5)                            /* Bits 7-5: PVD Level Selection */
#define PWC_CTRL_PVMSEL_MASK   (7 << PWC_CTRL_PVMSEL_SHIFT)   /* Power voltage monitoring boundary select */
#    define PWC_CTRL_None        (0 << PWC_CTRL_PVMSEL_SHIFT) /* 000: None */
#    define PWC_CTRL_2p3V        (1 << PWC_CTRL_PVMSEL_SHIFT) /* 001: 2.3V */
#    define PWC_CTRL_2p4V        (2 << PWC_CTRL_PVMSEL_SHIFT) /* 010: 2.4V */
#    define PWC_CTRL_2p5V        (3 << PWC_CTRL_PVMSEL_SHIFT) /* 011: 2.5V */
#    define PWC_CTRL_2p6V        (4 << PWC_CTRL_PVMSEL_SHIFT) /* 100: 2.6V */
#    define PWC_CTRL_2p7V        (5 << PWC_CTRL_PVMSEL_SHIFT) /* 101: 2.7V */
#    define PWC_CTRL_2p8V        (6 << PWC_CTRL_PVMSEL_SHIFT) /* 110: 2.8V */
#    define PWC_CTRL_2p9V        (7 << PWC_CTRL_PVMSEL_SHIFT) /* 111: 2.9V */

#define PWC_CTRL_BPWEN          (1 << 8) /* Battery powered domain write enable */

/* Power control/status register */

#define PWC_CTRLSTS_SWEF   (1 << 0) /* Standby wake-up event flag */
#define PWC_CTRLSTS_SEF    (1 << 1) /* Standby mode entry flag */
#define PWC_CTRLSTS_PVMOF  (1 << 2) /* Power voltage monitoring output flag */
#define PWC_CTRLSTS_SWPEN1 (1 << 8) /* Standby wake-up pin1 enable */
#define PWC_CTRLSTS_SWPEN2 (1 << 9) /* Standby wake-up pin2 enable */

/* Power ldo output register */

#define PWC_LDOOV_SEL_SHIFT    (0)
#define PWC_LDOOV_SEL_MASK     (7 << PWC_LDOOV_SEL_SHIFT) /* LDO output voltage select*/
# define PWC_LDOOV_1V0         (5 << PWC_LDOOV_SEL_SHIFT) /* 1.0V */
# define PWC_LDOOV_1V1         (4 << PWC_LDOOV_SEL_SHIFT) /* 1.1V */
# define PWC_LDOOV_1V3         (1 << PWC_LDOOV_SEL_SHIFT) /* 1.3V */
# define PWC_LDOOV_1V2         (0 << PWC_LDOOV_SEL_SHIFT) /* 1.2V */

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_PWR_H */