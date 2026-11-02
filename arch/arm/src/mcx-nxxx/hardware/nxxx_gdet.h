/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_gdet.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GDET_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GDET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define NXXX_GDET_ENABLE1_OFFSET         0x0008

/* Register Addresses *******************************************************/

#define NXXX_GDET0_ENABLE1               (NXXX_GDET0_BASE + NXXX_GDET_ENABLE1_OFFSET)
#define NXXX_GDET1_ENABLE1               (NXXX_GDET1_BASE + NXXX_GDET_ENABLE1_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* GDET_ENABLE1 - GDET Enable Register */

#define GDET_ENABLE1_REN1_SHIFT          (0)
#define GDET_ENABLE1_REN1_MASK           (0x1u << GDET_ENABLE1_REN1_SHIFT)
#define GDET_ENABLE1_REN1(x)             (((x) << GDET_ENABLE1_REN1_SHIFT) & \
                                          GDET_ENABLE1_REN1_MASK)

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_GDET_H */
