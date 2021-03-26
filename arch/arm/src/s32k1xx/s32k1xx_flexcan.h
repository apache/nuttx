/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_flexcan.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/s32k1xx_flexcan.h"

#ifdef CONFIG_S32K1XX_FLEXCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#if !defined(CONFIG_NETDEV_LATEINIT)

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   Initialize the enabled CAN device interfaces.  If there are more
 *   different network devices in the chip, then board-specific logic will
 *   have to provide this function to determine which, if any, network
 *   devices should be initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called very early in the initialization sequence.
 *
 ****************************************************************************/

void arm_netinitialize(void);

#else

/****************************************************************************
 * Function: s32k1xx_caninitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN devices, this value
 *          identifies which CAN device is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int s32k1xx_caninitialize(int intf);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_S32K1XX_FLEXCAN */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_FLEXCAN_H */
