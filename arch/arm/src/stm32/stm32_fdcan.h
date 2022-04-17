/****************************************************************************
 * arch/arm/src/stm32/stm32_fdcan.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_FDCAN_H
#define __ARCH_ARM_SRC_STM32_STM32_FDCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_fdcan.h"

#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port numbers for use with stm32_fdcan_initialize() */

#define FDCAN1 1
#define FDCAN2 2
#define FDCAN3 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_STM32_FDCAN_CHARDRIVER

/****************************************************************************
 * Name: stm32_fdcaninitialize
 *
 * Description:
 *   Initialize the selected FDCAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple FDCAN interfaces)
 *
 * Returned Value:
 *   Valid FDCAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s *stm32_fdcaninitialize(int port);
#endif

#ifdef CONFIG_STM32_FDCAN_SOCKET

/****************************************************************************
 * Name: stm32_fdcansockinitialize
 *
 * Description:
 *   Initialize the selected FDCAN port as SocketCAN interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple FDCAN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int stm32_fdcansockinitialize(int port);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_FDCAN_H */
