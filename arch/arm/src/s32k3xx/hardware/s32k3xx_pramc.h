/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_pramc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PRAMC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PRAMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PRAMC Register Offsets ***************************************************/

#define S32K3XX_PRAMC_PRCR1_OFFSET (0x00) /* Platform RAM Configuration Register 1 (PRCR1) */

/* PRAMC Register Addresses *************************************************/

#define S32K3XX_PRAMC0_PRCR1       (S32K3XX_PRAMC0_BASE + S32K3XX_PRAMC_PRCR1_OFFSET)

#define S32K3XX_PRAMC1_PRCR1       (S32K3XX_PRAMC1_BASE + S32K3XX_PRAMC_PRCR1_OFFSET)

/* PRAMC Register Bitfield Definitions **************************************/

/* Platform RAM Configuration Register 1 (PRCR1) */

#define PRAMC_FT_DIS               (1 << 0)  /* Bit 0: Flow-through disabled (FT_DIS) */
                                             /* Bits 1-5: Reserved */
#define PRAMC_P0_BO_DIS            (1 << 6)  /* Bit 6: Port 0 read burst optimization disable (P0_BO_DIS) */
                                             /* Bit 7-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_PRAMC_H */
