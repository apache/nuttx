/****************************************************************************
 * arch/arm/src/nrf54l/hardware/nrf54l_usbhs.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_USBHS_H
#define __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_USBHS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nrf54l_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USBHS wrapper register offsets *******************************************/

#define NRF54L_USBHS_TASKS_START_OFFSET        0x0000 /* Start USB core */
#define NRF54L_USBHS_TASKS_STOP_OFFSET         0x0004 /* Stop USB core */
#define NRF54L_USBHS_PUBLISH_SOF_OFFSET        0x0180 /* Publish start-of-frame event */
#define NRF54L_USBHS_ENABLE_OFFSET             0x0400 /* Enable USB core and PHY */
#define NRF54L_USBHS_PHY_CONFIG_OFFSET         0x0440 /* PHY tuning configuration */
#define NRF54L_USBHS_PHY_CLOCK_OFFSET          0x0444 /* PHY clock configuration */
#define NRF54L_USBHS_PHY_BATTCHRG_OFFSET       0x0448 /* Battery charger detection control */
#define NRF54L_USBHS_PHY_BATTCHRGSTATUS_OFFSET 0x044c /* Battery charger detection status */
#define NRF54L_USBHS_PHY_INPUTOVERRIDE_OFFSET  0x0458 /* PHY input override enable */
#define NRF54L_USBHS_PHY_OVERRIDEVALUES_OFFSET 0x045c /* PHY input override values */
#define NRF54L_USBHS_PHY_RTUNE_OFFSET          0x0464 /* PHY resistance tuning */

/* USBHS wrapper register addresses *****************************************/

#define NRF54L_USBHS_TASKS_START        (NRF54L_USBHS_BASE + NRF54L_USBHS_TASKS_START_OFFSET)
#define NRF54L_USBHS_TASKS_STOP         (NRF54L_USBHS_BASE + NRF54L_USBHS_TASKS_STOP_OFFSET)
#define NRF54L_USBHS_PUBLISH_SOF        (NRF54L_USBHS_BASE + NRF54L_USBHS_PUBLISH_SOF_OFFSET)
#define NRF54L_USBHS_ENABLE             (NRF54L_USBHS_BASE + NRF54L_USBHS_ENABLE_OFFSET)
#define NRF54L_USBHS_PHY_CONFIG         (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_CONFIG_OFFSET)
#define NRF54L_USBHS_PHY_CLOCK          (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_CLOCK_OFFSET)
#define NRF54L_USBHS_PHY_BATTCHRG       (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_BATTCHRG_OFFSET)
#define NRF54L_USBHS_PHY_BATTCHRGSTATUS (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_BATTCHRGSTATUS_OFFSET)
#define NRF54L_USBHS_PHY_INPUTOVERRIDE  (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_INPUTOVERRIDE_OFFSET)
#define NRF54L_USBHS_PHY_OVERRIDEVALUES (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_OVERRIDEVALUES_OFFSET)
#define NRF54L_USBHS_PHY_RTUNE          (NRF54L_USBHS_BASE + NRF54L_USBHS_PHY_RTUNE_OFFSET)

/* USBHS wrapper register bit definitions ***********************************/

#define USBHS_TASKS_START                      (1 << 0)
#define USBHS_TASKS_STOP                       (1 << 0)
#define USBHS_PUBLISH_SOF_CHIDX_SHIFT          0
#define USBHS_PUBLISH_SOF_CHIDX_MASK           (0xff << USBHS_PUBLISH_SOF_CHIDX_SHIFT)
#define USBHS_PUBLISH_SOF_EN                   (1 << 31)
#define USBHS_ENABLE_CORE                      (1 << 0)
#define USBHS_ENABLE_PHY                       (1 << 1)

/* PHY.CONFIG */

#define USBHS_PHY_CONFIG_PLLITUNE_SHIFT        0
#define USBHS_PHY_CONFIG_PLLITUNE_MASK         (3 << USBHS_PHY_CONFIG_PLLITUNE_SHIFT)
#define USBHS_PHY_CONFIG_PLLPTUNE_SHIFT        2
#define USBHS_PHY_CONFIG_PLLPTUNE_MASK         (15 << USBHS_PHY_CONFIG_PLLPTUNE_SHIFT)
#define USBHS_PHY_CONFIG_COMPDISTUNE_SHIFT     6
#define USBHS_PHY_CONFIG_COMPDISTUNE_MASK      (7 << USBHS_PHY_CONFIG_COMPDISTUNE_SHIFT)
#define USBHS_PHY_CONFIG_SQRXTUNE_SHIFT        9
#define USBHS_PHY_CONFIG_SQRXTUNE_MASK         (7 << USBHS_PHY_CONFIG_SQRXTUNE_SHIFT)
#define USBHS_PHY_CONFIG_VDATREFTUNE_SHIFT     12
#define USBHS_PHY_CONFIG_VDATREFTUNE_MASK      (3 << USBHS_PHY_CONFIG_VDATREFTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXHSXVTUNE_SHIFT      14
#define USBHS_PHY_CONFIG_TXHSXVTUNE_MASK       (3 << USBHS_PHY_CONFIG_TXHSXVTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXFSLSTUNE_SHIFT      16
#define USBHS_PHY_CONFIG_TXFSLSTUNE_MASK       (15 << USBHS_PHY_CONFIG_TXFSLSTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXVREFTUNE_SHIFT      20
#define USBHS_PHY_CONFIG_TXVREFTUNE_MASK       (15 << USBHS_PHY_CONFIG_TXVREFTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXRISETUNE_SHIFT      24
#define USBHS_PHY_CONFIG_TXRISETUNE_MASK       (3 << USBHS_PHY_CONFIG_TXRISETUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXRESTUNE_SHIFT       26
#define USBHS_PHY_CONFIG_TXRESTUNE_MASK        (3 << USBHS_PHY_CONFIG_TXRESTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXPREEMPAMPTUNE_SHIFT 28
#define USBHS_PHY_CONFIG_TXPREEMPAMPTUNE_MASK  (3 << USBHS_PHY_CONFIG_TXPREEMPAMPTUNE_SHIFT)
#define USBHS_PHY_CONFIG_TXPREEMPPULSETUNE     (1 << 30)

/* PHY.CLOCK */

#define USBHS_PHY_CLOCK_FSEL_SHIFT             0
#define USBHS_PHY_CLOCK_FSEL_MASK              (7 << USBHS_PHY_CLOCK_FSEL_SHIFT)
#define USBHS_PHY_CLOCK_PLLBTUNE               (1 << 3)
#define USBHS_PHY_CLOCK_COMMONONN              (1 << 4)

/* PHY.BATTCHRG and PHY.BATTCHRGSTATUS */

#define USBHS_PHY_BATTCHRG_CHRGSEL             (1 << 0)
#define USBHS_PHY_BATTCHRG_VDATENB             (1 << 1)
#define USBHS_PHY_BATTCHRG_VDATSRCENB          (1 << 2)
#define USBHS_PHY_BATTCHRGSTATUS_CHGDET        (1 << 1)
#define USBHS_PHY_BATTCHRGSTATUS_FSVPLUS       (1 << 2)
#define USBHS_PHY_BATTCHRGSTATUS_FSVMINUS      (1 << 3)

/* PHY.INPUTOVERRIDE */

#define USBHS_PHY_INPUTOVERRIDE_OPMODE_SHIFT   18
#define USBHS_PHY_INPUTOVERRIDE_OPMODE_MASK    (3 << USBHS_PHY_INPUTOVERRIDE_OPMODE_SHIFT)
#define USBHS_PHY_INPUTOVERRIDE_XCVRSEL_SHIFT  20
#define USBHS_PHY_INPUTOVERRIDE_XCVRSEL_MASK   (3 << USBHS_PHY_INPUTOVERRIDE_XCVRSEL_SHIFT)
#define USBHS_PHY_INPUTOVERRIDE_DPPULLDOWN     (1 << 23)
#define USBHS_PHY_INPUTOVERRIDE_DMPULLDOWN     (1 << 24)
#define USBHS_PHY_INPUTOVERRIDE_SUSPENDM       (1 << 25)
#define USBHS_PHY_INPUTOVERRIDE_VBUSVALID      (1 << 30)
#define USBHS_PHY_INPUTOVERRIDE_ID             (1 << 31)

/* PHY.OVERRIDEVALUES */

#define USBHS_PHY_OVERRIDEVALUES_OPMODE_SHIFT  18
#define USBHS_PHY_OVERRIDEVALUES_OPMODE_MASK   (3 << USBHS_PHY_OVERRIDEVALUES_OPMODE_SHIFT)
#define USBHS_PHY_OVERRIDEVALUES_XCVRSEL_SHIFT 20
#define USBHS_PHY_OVERRIDEVALUES_XCVRSEL_MASK  (3 << USBHS_PHY_OVERRIDEVALUES_XCVRSEL_SHIFT)
#define USBHS_PHY_OVERRIDEVALUES_DPPULLDOWN    (1 << 23)
#define USBHS_PHY_OVERRIDEVALUES_DMPULLDOWN    (1 << 24)
#define USBHS_PHY_OVERRIDEVALUES_SUSPENDM      (1 << 25)
#define USBHS_PHY_OVERRIDEVALUES_VBUSVALID     (1 << 30)
#define USBHS_PHY_OVERRIDEVALUES_ID            (1 << 31)

/* PHY.RTUNE */

#define USBHS_PHY_RTUNE_RTUNESEL                (1 << 0)
#define USBHS_PHY_RTUNE_RCALCODE_SHIFT          1
#define USBHS_PHY_RTUNE_RCALCODE_MASK           (15 << USBHS_PHY_RTUNE_RCALCODE_SHIFT)

#endif /* __ARCH_ARM_SRC_NRF54L_HARDWARE_NRF54L_USBHS_H */
