/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_can.h
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

#ifndef __ARCH_ARM_SRC_GD32F4_GD32F4XX_CAN_H
#define __ARCH_ARM_SRC_GD32F4_GD32F4XX_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/gd32f4xx_can.h"

#include <nuttx/can/can.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Up to 2 CAN interfaces are supported */

#if GD32_NCAN < 2
#  undef CONFIG_GD32F4_CAN1
#endif

#if GD32_NCAN < 1
#  undef CONFIG_GD32F4_CAN0
#endif

/* CAN BAUD */

#if defined(CONFIG_GD32F4_CAN0) && !defined(CONFIG_GD32F4_CAN0_BAUD)
#  error "CONFIG_GD32F4_CAN0_BAUD is not defined"
#endif

#if defined(CONFIG_GD32F4_CAN1) && !defined(CONFIG_GD32F4_CAN1_BAUD)
#  error "CONFIG_GD32F4_CAN1_BAUD is not defined"
#endif

/* User-defined TSEG1 and TSEG2 settings may be used.
 *
 * CONFIG_GD32F4_CAN_TSEG1 = the number of CAN time quanta in segment 1
 * CONFIG_GD32F4_CAN_TSEG2 = the number of CAN time quanta in segment 2
 * CAN_BIT_QUANTA          = The number of CAN time quanta in one bit time
 */

#ifndef CONFIG_GD32F4_CAN_TSEG1
#  define CONFIG_GD32F4_CAN_TSEG1 6
#endif

#if CONFIG_GD32F4_CAN_TSEG1 < 1 || CONFIG_GD32F4_CAN_TSEG1 > CAN_BTR_TSEG1_MAX
#  error "CONFIG_GD32F4_CAN_TSEG1 is out of range"
#endif

#ifndef CONFIG_GD32F4_CAN_TSEG2
#  define CONFIG_GD32F4_CAN_TSEG2 7
#endif

#if CONFIG_GD32F4_CAN_TSEG2 < 1 || CONFIG_GD32F4_CAN_TSEG2 > CAN_BTR_TSEG2_MAX
#  error "CONFIG_GD32F4_CAN_TSEG2 is out of range"
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

#if defined(CONFIG_GD32F4_CAN0) || defined(CONFIG_GD32F4_CAN1)

/****************************************************************************
 * Name: gd32_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port as character device
 *
 * Input Parameters:
 *   Port number (0 = CAN0, 1 = CAN1)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
struct can_dev_s *gd32_caninitialize(int port);

#endif /* CONFIG_GD32F4_CAN0 || CONFIG_GD32F4_CAN1 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32F4_GD32F4XX_CAN_H */
