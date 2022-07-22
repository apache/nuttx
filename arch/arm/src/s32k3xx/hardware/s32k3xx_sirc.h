/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_sirc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIRC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SIRC Register Offsets ****************************************************/

#define S32K3XX_SIRC_SR_OFFSET               (0x04) /* Status Register (SR) */
#define S32K3XX_SIRC_MISCELLANEOUS_IN_OFFSET (0x0c) /* Miscellaneous Input Register (MISCELLANEOUS_IN) */

/* SIRC Register Addresses **************************************************/

#define S32K3XX_SIRC_SR                      (S32K3XX_SIRC_BASE + S32K3XX_SIRC_SR_OFFSET)
#define S32K3XX_SIRC_MISCELLANEOUS_IN        (S32K3XX_SIRC_BASE + S32K3XX_SIRC_MISCELLANEOUS_IN_OFFSET)

/* SIRC Register Bitfield Definitions ***************************************/

/* Status Register (SR) */

#define SIRC_SR_STATUS                     (1 << 0) /* Bit 0: Status bit for SIRC (STATUS) */
#  define SIRC_SR_STATUS_OFF               (0 << 0) /*        SIRC is off or unstable */
#  define SIRC_SR_STATUS_ON                (1 << 0) /*        SIRC is on and stable */
                                                    /* Bits 1-31: Reserved */

/* Miscellaneous Input Register (MISCELLANEOUS_IN) */

                                                    /* Bits 0-7: Reserved */

#define SIRC_MISCELLANEOUS_IN_STANDBY_ENABLE    (1 << 8) /* Bit 8: SIRC enabled in standby mode (STANDBY_ENABLE) */
#  define SIRC_MISCELLANEOUS_IN_STANDBY_DISABLE (0 << 8) /*        SIRC disabled in standby mode */

                                                    /* Bit 9-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SIRC_H */
