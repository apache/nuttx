/****************************************************************************
 * arch/arm/src/stm32/stm32_can.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_CAN_H
#define __ARCH_ARM_SRC_STM32_STM32_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_can.h"

#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 2 CAN interfaces are supported */

#if STM32_NCAN < 2
#  undef CONFIG_STM32_CAN2
#endif

#if STM32_NCAN < 1
#  undef CONFIG_STM32_CAN1
#endif

/* CAN BAUD */

#if defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN1_BAUD)
#  error "CONFIG_STM32_CAN1_BAUD is not defined"
#endif

#if defined(CONFIG_STM32_CAN2) && !defined(CONFIG_STM32_CAN2_BAUD)
#  error "CONFIG_STM32_CAN2_BAUD is not defined"
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_STM32_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_STM32_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA   = The number of CAN time quanta in on bit time
 */

#ifndef CONFIG_STM32_CAN_TSEG1
#  define CONFIG_STM32_CAN_TSEG1 6
#endif

#if CONFIG_STM32_CAN_TSEG1 < 1 || CONFIG_STM32_CAN_TSEG1 > CAN_BTR_TSEG1_MAX
#  error "CONFIG_STM32_CAN_TSEG1 is out of range"
#endif

#ifndef CONFIG_STM32_CAN_TSEG2
#  define CONFIG_STM32_CAN_TSEG2 7
#endif

#if CONFIG_STM32_CAN_TSEG2 < 1 || CONFIG_STM32_CAN_TSEG2 > CAN_BTR_TSEG2_MAX
#  error "CONFIG_STM32_CAN_TSEG2 is out of range"
#endif

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

#ifdef CONFIG_STM32_CAN_CHARDRIVER

/****************************************************************************
 * Name: stm32_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port as character device
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
struct can_dev_s *stm32_caninitialize(int port);
#endif

#ifdef CONFIG_STM32_CAN_SOCKET

/****************************************************************************
 * Name: stm32_cansockinitialize
 *
 * Description:
 *   Initialize the selected CAN port as SocketCAN interface
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int stm32_cansockinitialize(int port);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_CAN_H */
