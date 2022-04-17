/****************************************************************************
 * arch/arm/src/sama5/sam_can.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_CAN_H
#define __ARCH_ARM_SRC_SAMA5_SAM_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_can.h"

#include <nuttx/can/can.h>

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_CAN0) || defined(CONFIG_SAMA5_CAN1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CAN BAUD */

#if defined(CONFIG_SAMA5_CAN0) && !defined(CONFIG_SAMA5_CAN0_BAUD)
#  error "CONFIG_SAMA5_CAN0_BAUD is not defined"
#endif

#if defined(CONFIG_SAMA5_CAN1) && !defined(CONFIG_SAMA5_CAN1_BAUD)
#  error "CONFIG_SAMA5_CAN1_BAUD is not defined"
#endif

/* There must be at least one but not more than three receive mailboxes */

#ifdef CONFIG_SAMA5_CAN0
#  if !defined(CONFIG_SAMA5_CAN0_NRECVMB) || CONFIG_SAMA5_CAN0_NRECVMB < 1
#    undef  CONFIG_SAMA5_CAN0_NRECVMB
#    define CONFIG_SAMA5_CAN0_NRECVMB 1
#  endif
#  if CONFIG_SAMA5_CAN0_NRECVMB > 3
#    warning Current implementation only supports up to three receive mailboxes
#    undef  CONFIG_SAMA5_CAN0_NRECVMB
#    define CONFIG_SAMA5_CAN0_NRECVMB 3
#  endif
#else
#  undef  CONFIG_SAMA5_CAN0_NRECVMB
#  define CONFIG_SAMA5_CAN0_NRECVMB 0
#endif

#ifdef CONFIG_SAMA5_CAN1
#  if !defined(CONFIG_SAMA5_CAN1_NRECVMB) || CONFIG_SAMA5_CAN1_NRECVMB < 1
#    undef  CONFIG_SAMA5_CAN1_NRECVMB
#    define CONFIG_SAMA5_CAN1_NRECVMB 1
#  endif
#  if CONFIG_SAMA5_CAN1_NRECVMB > 3
#    warning Current implementation only supports up to three receive mailboxes
#    undef  CONFIG_SAMA5_CAN1_NRECVMB
#    define CONFIG_SAMA5_CAN1_NRECVMB 3
#  endif
#else
#  undef  CONFIG_SAMA5_CAN1_NRECVMB
#  define CONFIG_SAMA5_CAN1_NRECVMB 0
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sama5_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number: 0=CAN0, 1=CAN1
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
struct can_dev_s *sam_caninitialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_SAMA5_CAN0 || CONFIG_SAMA5_CAN1) */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_CAN_H */
