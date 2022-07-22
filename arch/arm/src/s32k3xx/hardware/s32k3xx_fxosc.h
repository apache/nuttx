/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_fxosc.h
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

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FXOSC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FXOSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FXOSC Register Offsets ***************************************************/

#define S32K3XX_FXOSC_CTRL_OFFSET   (0x00) /* FXOSC Control Register (CTRL) */
#define S32K3XX_FXOSC_STAT_OFFSET   (0x04) /* Oscillator Status Register (STAT) */

/* FXOSC Register Addresses *************************************************/

#define S32K3XX_FXOSC_CTRL          (S32K3XX_FXOSC_BASE + S32K3XX_FXOSC_CTRL_OFFSET)
#define S32K3XX_FXOSC_STAT          (S32K3XX_FXOSC_BASE + S32K3XX_FXOSC_STAT_OFFSET)

/* FXOSC Register Bitfield Definitions **************************************/

/* FXOSC Control Register (CTRL) */

#define FXOSC_CTRL_OSCON            (1 << 0)  /* Bit 0: Enables FXOSC (OSCON) */
#  define FXOSC_CTRL_OSCOFF         (0 << 0)  /*        Disables FXOSC */
                                              /* Bits 1-3: Reserved */
#define FXOSC_CTRL_GM_SEL_SHIFT     (4)       /* Bits 4-7: Crystal overdrive protection, transconductance selection (GM_SEL) */
#define FXOSC_CTRL_GM_SEL_MASK      (0x0f << FXOSC_CTRL_GM_SEL_SHIFT)
#  define FXOSC_CTRL_GM_SEL_0X      (0x00 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0x */
#  define FXOSC_CTRL_GM_SEL_0_1004X (0x01 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.1004x */
#  define FXOSC_CTRL_GM_SEL_0_2009X (0x02 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.2009x */
#  define FXOSC_CTRL_GM_SEL_0_3013X (0x03 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.3013x */
#  define FXOSC_CTRL_GM_SEL_0_2343X (0x04 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.2343x */
#  define FXOSC_CTRL_GM_SEL_0_3348X (0x05 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.3348x */
#  define FXOSC_CTRL_GM_SEL_0_4345X (0x06 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.4345x */
#  define FXOSC_CTRL_GM_SEL_0_5349X (0x07 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.5349x */
#  define FXOSC_CTRL_GM_SEL_0_4679X (0x08 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.4679x */
#  define FXOSC_CTRL_GM_SEL_0_5684X (0x09 << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.5684x */
#  define FXOSC_CTRL_GM_SEL_0_6681X (0x0a << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.6681x */
#  define FXOSC_CTRL_GM_SEL_0_7678X (0x0b << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.7678x */
#  define FXOSC_CTRL_GM_SEL_0_7016X (0x0c << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.7016x */
#  define FXOSC_CTRL_GM_SEL_0_8013X (0x0d << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.8013x */
#  define FXOSC_CTRL_GM_SEL_0_9003X (0x0e << FXOSC_CTRL_GM_SEL_SHIFT) /* 0.9003x */
#  define FXOSC_CTRL_GM_SEL_1X      (0x0f << FXOSC_CTRL_GM_SEL_SHIFT) /* 1x */

                                              /* Bits 8-15: Reserved */
#define FXOSC_CTRL_EOCV_SHIFT       (16)      /* Bits 16-23: End of count value (EOCV) */
#define FXOSC_CTRL_EOCV_MASK        (0xff << FXOSC_CTRL_EOCV_SHIFT)
#define FXOSC_CTRL_EOCV(n)          ((n << FXOSC_CTRL_EOCV_SHIFT) & FXOSC_CTRL_EOCV_MASK)
#define FXOSC_CTRL_COMP_EN          (1 << 24) /* Bit 24: Comparator enable (COMP_EN) */
#  define FXOSC_CTRL_COMP_DIS       (0 << 24) /*         Comparator disable */
                                              /* Bits 25-30: Reserved */
#define FXOSC_CTRL_OSC_BYP          (1 << 31) /* Bit 31: Oscillator bypass (OSC_BYP) */

/* Oscillator Status Register (STAT) */

                                              /* Bits 0-30: Reserved */
#define FXOSC_STAT_OSC_STAT         (1 << 31) /* Bit 31: Crystal oscilator status (OSC_STAT) */
#  define FXOSC_STAT_OSC_STAT_OFF   (0 << 31) /*         Crystal oscillator is off or not stable */
#  define FXOSC_STAT_OSC_STAT_ON    (1 << 31) /*         Crystal oscillator is on and providing a stable clock */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_FXOSC_H */
