/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_tspc.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_TSPC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_TSPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TSPC Register Offsets ****************************************************/

#define S32K3XX_TSPC_GRP_EN_OFFSET    (0x00) /* Group Enable Register (GRP_EN) */
#define S32K3XX_TSPC_GRP1_OBE1_OFFSET (0x50) /* Group OBE (GRP1_OBE1) */
#define S32K3XX_TSPC_GRP1_OBE2_OFFSET (0x54) /* Group OBE (GRP1_OBE2) */
#define S32K3XX_TSPC_GRP2_OBE1_OFFSET (0xa0) /* Group OBE (GRP2_OBE1) */
#define S32K3XX_TSPC_GRP2_OBE2_OFFSET (0xa4) /* Group OBE (GRP2_OBE2) */

/* TSPC Register Addresses **************************************************/

#define S32K3XX_TSPC_GRP_EN           (S32K3XX_TSPC_BASE + S32K3XX_TSPC_GRP_EN_OFFSET)
#define S32K3XX_TSPC_GRP1_OBE1        (S32K3XX_TSPC_BASE + S32K3XX_TSPC_GRP1_OBE1_OFFSET)
#define S32K3XX_TSPC_GRP1_OBE2        (S32K3XX_TSPC_BASE + S32K3XX_TSPC_GRP1_OBE2_OFFSET)
#define S32K3XX_TSPC_GRP2_OBE1        (S32K3XX_TSPC_BASE + S32K3XX_TSPC_GRP2_OBE1_OFFSET)
#define S32K3XX_TSPC_GRP2_OBE2        (S32K3XX_TSPC_BASE + S32K3XX_TSPC_GRP2_OBE2_OFFSET)

/* TSPC Register Bitfield Definitions ***************************************/

/* Group Enable Register (GRP_EN) */

#define S32K3XX_TSPC_GRP_EN_GRP1_EN   (1 << 0)      /* Bit 0: Enable for GRP1_OBEn Register (GRP1_EN) */
#define S32K3XX_TSPC_GRP_EN_GRP2_EN   (1 << 1)      /* Bit 1: Enable for GRP2_OBEn Register (GRP2_EN) */
                                                    /* Bits 2-31: Reserved */

/* Group OBE (GRPn_OBE1) */

#define S32K3XX_TSPC_GRP_OBE1_OBE(b)  (1 << (b))    /* Bit b: Output Buffer Enable (OBE) */

/* Group OBE (GRPn_OBE2) */

#define S32K3XX_TSPC_GRP_OBE2_OBE(b)  (1 << ((b) - 32)) /* Bit (b-32): Output Buffer Enable (OBE) */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_TSPC_H */
