/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_sha256.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SHA256_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SHA256_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_SHA256_CSR_OFFSET    0x00000000
#define RP23XX_SHA256_WDATA_OFFSET  0x00000004
#define RP23XX_SHA256_SUM_OFFSET(n) ((n) * 4 + 0x000008)

/* Register definitions *****************************************************/

#define RP23XX_SHA256_CSR    (RP23XX_SHA256_BASE + RP23XX_SHA256_CSR_OFFSET)
#define RP23XX_SHA256_WDATA  (RP23XX_SHA256_BASE + RP23XX_SHA256_WDATA_OFFSET)
#define RP23XX_SHA256_SUM(n) (RP23XX_SHA256_BASE + RP23XX_SHA256_SUM_OFFSET(n))

/* Register bit definitions *************************************************/

#define RP23XX_SHA256_CSR_MASK               (0x00001317)
#define RP23XX_SHA256_CSR_BSWAP              (1 << 18)
#define RP23XX_SHA256_CSR_DMA_SIZE_MASK      (0x00000300)
#define RP23XX_SHA256_CSR_ERR_WDATA_NOT_RDY  (1 << 4)
#define RP23XX_SHA256_CSR_SUM_VLD            (1 << 2)
#define RP23XX_SHA256_CSR_WDATA_RDY          (1 << 1)
#define RP23XX_SHA256_CSR_START              (1 << 0)
#define RP23XX_SHA256_WDATA_MASK             (0xffffffff)
#define RP23XX_SHA256_SUM_MASK               (0xffffffff)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_SHA256_H */
