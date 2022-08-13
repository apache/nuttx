/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_sxosc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SXOSC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SXOSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SXOSC Register Offsets ***************************************************/

#define S32K3XX_SXOSC_CTRL_OFFSET      (0x00) /* Oscillator Control Register (CTRL) */
#define S32K3XX_SXOSC_STAT_OFFSET      (0x04) /* Oscillator Status Register (STAT) */

/* SXOSC Register Addresses *************************************************/

#define S32K3XX_SXOSC_CTRL             (S32K3XX_SXOSC_BASE + S32K3XX_SXOSC_CTRL_OFFSET)
#define S32K3XX_SXOSC_STAT             (S32K3XX_SXOSC_BASE + S32K3XX_SXOSC_STAT_OFFSET)

/* SXOSC Register Bitfield Definitions **************************************/

/* SXOSC Control Register (CTRL) */

#define SXOSC_CTRL_OSCON               (1 << 0)  /* Bit 0: Enables SXOSC (OSCON) */
#  define SXOSC_CTRL_OSCOFF            (0 << 0)  /*        Disables SXOSC */
                                                 /* Bits 1-15: Reserved */
#define SXOSC_CTRL_EOCV_SHIFT          (16)      /* Bits 16-23: End of count value (EOCV) */
#define SXOSC_CTRL_EOCV_MASK           (0xff << SXOSC_CTRL_EOCV_SHIFT)
                                                 /* Bits 24-31: Reserved */

/* SXOSC Status Register (STAT) */

                                                 /* Bits 0-30: Reserved */
#define SXOSC_STAT_OSC_STAT            (1 << 31) /* Bit 31: Crystal oscilator status (OSC_STAT) */
#  define SXOSC_STAT_OSC_STAT_UNSTABLE (0 << 31) /*         Crystal oscillator is unstable */
#  define SXOSC_STAT_OSC_STAT_STABLE   (1 << 31) /*         Crystal oscillator is stable */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_SXOSC_H */
