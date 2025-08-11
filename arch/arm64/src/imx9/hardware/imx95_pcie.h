/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx95_pcie.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_PCIE_H_
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_PCIE_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define PCIE1_DBI_BASE   0x4c380000
#define PCIE1_DBI_SIZE   0x10000
#define PCIE1_ATU_BASE   0x4c3e0000
#define PCIE1_ATU_SIZE   0x10000
#define PCIE1_DBI2_BASE  0x4c3a0000
#define PCIE1_DBI2_SIZE  0x10000
#define PCIE1_APP_BASE   0x4c3c0000
#define PCIE1_APP_SIZE   0x4000
#define PCIE1_OB_BASE    0xa00000000
#define PCIE1_OB_SIZE    0x100000000
#define PCIE1_DMA_BASE   0xa8100000
#define PCIE1_DMA_SIZE   0x2000000
#define PCIE1_LINK_SPEED 3
#define PCIE1_LINK_LANES 1

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX95_PCIE_H_ */
