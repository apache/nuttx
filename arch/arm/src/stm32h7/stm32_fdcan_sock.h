/****************************************************************************
 * arch/arm/src/stm32h7/stm32_fdcan_sock.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_SOCK_H
#define __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_SOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32_fdcan.h"

#ifdef CONFIG_STM32H7_FDCAN

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

#ifndef CONFIG_NETDEV_LATEINIT

/****************************************************************************
 * Function: arm_netinitialize
 *
 * Description:
 *   Initialize the first network interface.  If there is more than one
 *   interface in the chip, then board-specific logic will have to provide
 *   this function to determine which, if any, CAN interfaces should be
 *   initialized.  Also prototyped in up_internal.h.
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
 * Function: stm32_fdcansockinitialize
 *
 * Description:
 *   Initialize the CAN controller and driver
 *
 * Input Parameters:
 *   intf - In the case where there are multiple CAN interfaces, this value
 *          identifies which CAN interface is to be initialized.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int stm32_fdcansockinitialize(int intf);

#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32H7_FDCAN */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_FDCAN_SOCK_H */
