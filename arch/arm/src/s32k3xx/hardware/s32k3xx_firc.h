/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_firc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FIRC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FIRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FIRC Register Offsets ****************************************************/

#define S32K3XX_FIRC_STATUS_OFFSET   (0x04) /* Status Register (STATUS) */
#define S32K3XX_FIRC_STDBY_EN_OFFSET (0x08) /* Standby Enable Register (STDBY_EN) */

/* FIRC Register Addresses **************************************************/

#define S32K3XX_FIRC_STATUS          (S32K3XX_FIRC_BASE + S32K3XX_FIRC_STATUS_OFFSET)
#define S32K3XX_FIRC_STDBY_EN        (S32K3XX_FIRC_BASE + S32K3XX_FIRC_STDBY_EN_OFFSET)

/* FIRC Register Bitfield Definitions ***************************************/

/* Status Register (STATUS) */

#define FIRC_STATUS                  (1 << 0) /* Bit 0: Status bit for FIRC (STATUS) */
#  define FIRC_STATUS_OFF            (0 << 0) /*        FIRC is off or unstable */
#  define FIRC_STATUS_ON             (1 << 0) /*        FIRC is on and stable */
                                              /* Bits 1-31: Reserved */

/* Standby Enable Register (STDBY_EN) */

#define FIRC_STDBY_EN                (1 << 0) /* Bit 0: Enables FIRC in standby mode (STDBY_EN) */
#  define FIRC_STDBY_DIS             (0 << 0) /*        Disables FIRC in standby mode */
                                              /* Bit 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FIRC_H */
