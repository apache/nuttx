/****************************************************************************
 * arch/arm/src/sama5/sam_mcan.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_MCAN_H
#define __ARCH_ARM_SRC_SAMA5_SAM_MCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_mcan.h"

#include <nuttx/can/can.h>

#if defined(CONFIG_CAN) && (defined(CONFIG_SAMA5_MCAN0) || \
    defined(CONFIG_SAMA5_MCAN1))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port numbers for use with sam_mcan_initialize() */

#define MCAN0 0
#define MCAN1 1

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
 * Name: sam_mcan_initialize
 *
 * Description:
 *   Initialize the selected MCAN port
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple CAN interfaces),
 *          0=MCAN0, 1=NCAN1
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct can_dev_s;
struct can_dev_s *sam_mcan_initialize(int port);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_CAN && (CONFIG_SAMA5_MCAN0 || CONFIG_SAMA5_MCAN1) */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_MCAN_H */
