/****************************************************************************
 * arch/arm/src/xmc4/hardware/xmc4_eru_pinmap.h
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

/* Reference: XMC4[87]00 Reference Manual V1.5 2014-07 Microcontrollers.
 * Reference: XMC4500 Reference Manual V1.0 2012-02 Microcontrollers.
 */

#ifndef __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_PINMAP_H
#define __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "xmc4_eru.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* XMC4 GPIO ERU interconnect */

#define ERU0_ETL0_INPUTA_P0_1               XMC_ERU_ETL_INPUT_A0
#define ERU0_ETL0_INPUTA_P2_5               XMC_ERU_ETL_INPUT_A2
#define ERU0_ETL0_INPUTA_P3_2               XMC_ERU_ETL_INPUT_A1
#define ERU0_ETL0_INPUTB_P0_0               XMC_ERU_ETL_INPUT_B0
#define ERU0_ETL0_INPUTB_P2_0               XMC_ERU_ETL_INPUT_B3
#define ERU0_ETL0_INPUTB_P2_4               XMC_ERU_ETL_INPUT_B2
#define ERU0_ETL0_INPUTB_P3_1               XMC_ERU_ETL_INPUT_B1
#define ERU0_ETL1_INPUTA_P0_10              XMC_ERU_ETL_INPUT_A0
#define ERU0_ETL1_INPUTA_P2_3               XMC_ERU_ETL_INPUT_A2
#define ERU0_ETL1_INPUTB_P0_9               XMC_ERU_ETL_INPUT_B0
#define ERU0_ETL1_INPUTB_P2_2               XMC_ERU_ETL_INPUT_B2
#define ERU0_ETL1_INPUTB_P2_6               XMC_ERU_ETL_INPUT_B3
#define ERU0_ETL2_INPUTA_P0_8               XMC_ERU_ETL_INPUT_A1
#define ERU0_ETL2_INPUTA_P1_5               XMC_ERU_ETL_INPUT_A0
#define ERU0_ETL2_INPUTB_P0_12              XMC_ERU_ETL_INPUT_B2
#define ERU0_ETL2_INPUTB_P0_4               XMC_ERU_ETL_INPUT_B3
#define ERU0_ETL2_INPUTB_P0_7               XMC_ERU_ETL_INPUT_B1
#define ERU0_ETL2_INPUTB_P1_4               XMC_ERU_ETL_INPUT_B0
#define ERU0_ETL3_INPUTA_P0_11              XMC_ERU_ETL_INPUT_A2
#define ERU0_ETL3_INPUTA_P1_1               XMC_ERU_ETL_INPUT_A0
#define ERU0_ETL3_INPUTA_P3_6               XMC_ERU_ETL_INPUT_A1
#define ERU0_ETL3_INPUTB_P0_2               XMC_ERU_ETL_INPUT_B3
#define ERU0_ETL3_INPUTB_P0_6               XMC_ERU_ETL_INPUT_B2
#define ERU0_ETL3_INPUTB_P1_0               XMC_ERU_ETL_INPUT_B0
#define ERU0_ETL3_INPUTB_P3_5               XMC_ERU_ETL_INPUT_B1
#define ERU1_ETL0_INPUTA_P1_5               XMC_ERU_ETL_INPUT_A0
#define ERU1_ETL0_INPUTB_P2_1               XMC_ERU_ETL_INPUT_B0
#define ERU1_ETL1_INPUTA_P1_15              XMC_ERU_ETL_INPUT_A0
#define ERU1_ETL1_INPUTB_P2_7               XMC_ERU_ETL_INPUT_B0
#define ERU1_ETL2_INPUTA_P1_3               XMC_ERU_ETL_INPUT_A0
#define ERU1_ETL2_INPUTB_P1_2               XMC_ERU_ETL_INPUT_B0
#define ERU1_ETL3_INPUTA_P0_5               XMC_ERU_ETL_INPUT_A0
#define ERU1_ETL3_INPUTB_P0_3               XMC_ERU_ETL_INPUT_B0

/*  LQPF144 & BGA196 variants */

#define ERU0_ETL2_INPUTA_P0_13              XMC_ERU_ETL_INPUT_A2

#endif /* __ARCH_ARM_SRC_XMC4_HARDWARE_XMC4_ERU_PINMAP_H */
