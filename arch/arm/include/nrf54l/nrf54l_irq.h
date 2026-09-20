/****************************************************************************
 * arch/arm/include/nrf54l/nrf54l_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_NRF54L_NRF54L_IRQ_H
#define __ARCH_ARM_INCLUDE_NRF54L_NRF54L_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cortex-M33 external interrupts (vectors >= 16) */

#define NRF54L_IRQ_SWI00       (NRF54L_IRQ_EXTINT + 28)
#define NRF54L_IRQ_SWI01       (NRF54L_IRQ_EXTINT + 29)
#define NRF54L_IRQ_SWI02       (NRF54L_IRQ_EXTINT + 30)
#define NRF54L_IRQ_SWI03       (NRF54L_IRQ_EXTINT + 31)
#define NRF54L_IRQ_SPU00       (NRF54L_IRQ_EXTINT + 64)
#define NRF54L_IRQ_MPC00       (NRF54L_IRQ_EXTINT + 65)
#define NRF54L_IRQ_VPR00       (NRF54L_IRQ_EXTINT + 76)
#define NRF54L_IRQ_CTRLAP      (NRF54L_IRQ_EXTINT + 82)
#define NRF54L_IRQ_CM33SS      (NRF54L_IRQ_EXTINT + 84)
#define NRF54L_IRQ_TIMER00     (NRF54L_IRQ_EXTINT + 85)
#define NRF54L_IRQ_AXONS       (NRF54L_IRQ_EXTINT + 86)
#define NRF54L_IRQ_EGU00       (NRF54L_IRQ_EXTINT + 88)
#define NRF54L_IRQ_USBHS       (NRF54L_IRQ_EXTINT + 90)
#define NRF54L_IRQ_SPU10       (NRF54L_IRQ_EXTINT + 128)
#define NRF54L_IRQ_TIMER10     (NRF54L_IRQ_EXTINT + 133)
#define NRF54L_IRQ_EGU10       (NRF54L_IRQ_EXTINT + 135)
#define NRF54L_IRQ_RADIO_0     (NRF54L_IRQ_EXTINT + 138)
#define NRF54L_IRQ_RADIO_1     (NRF54L_IRQ_EXTINT + 139)
#define NRF54L_IRQ_SPU20       (NRF54L_IRQ_EXTINT + 192)
#define NRF54L_IRQ_SERIAL20    (NRF54L_IRQ_EXTINT + 198)
#define NRF54L_IRQ_SERIAL21    (NRF54L_IRQ_EXTINT + 199)
#define NRF54L_IRQ_SERIAL22    (NRF54L_IRQ_EXTINT + 200)
#define NRF54L_IRQ_EGU20       (NRF54L_IRQ_EXTINT + 201)
#define NRF54L_IRQ_TIMER20     (NRF54L_IRQ_EXTINT + 202)
#define NRF54L_IRQ_TIMER21     (NRF54L_IRQ_EXTINT + 203)
#define NRF54L_IRQ_TIMER22     (NRF54L_IRQ_EXTINT + 204)
#define NRF54L_IRQ_TIMER23     (NRF54L_IRQ_EXTINT + 205)
#define NRF54L_IRQ_TIMER24     (NRF54L_IRQ_EXTINT + 206)
#define NRF54L_IRQ_PDM20       (NRF54L_IRQ_EXTINT + 208)
#define NRF54L_IRQ_PDM21       (NRF54L_IRQ_EXTINT + 209)
#define NRF54L_IRQ_PWM20       (NRF54L_IRQ_EXTINT + 210)
#define NRF54L_IRQ_PWM21       (NRF54L_IRQ_EXTINT + 211)
#define NRF54L_IRQ_PWM22       (NRF54L_IRQ_EXTINT + 212)
#define NRF54L_IRQ_SAADC       (NRF54L_IRQ_EXTINT + 213)
#define NRF54L_IRQ_NFCT        (NRF54L_IRQ_EXTINT + 214)
#define NRF54L_IRQ_TEMP        (NRF54L_IRQ_EXTINT + 215)
#define NRF54L_IRQ_GPIOTE20_0  (NRF54L_IRQ_EXTINT + 218)
#define NRF54L_IRQ_GPIOTE20_1  (NRF54L_IRQ_EXTINT + 219)
#define NRF54L_IRQ_I2S20       (NRF54L_IRQ_EXTINT + 221)
#define NRF54L_IRQ_QDEC20      (NRF54L_IRQ_EXTINT + 224)
#define NRF54L_IRQ_QDEC21      (NRF54L_IRQ_EXTINT + 225)
#define NRF54L_IRQ_GRTC_0      (NRF54L_IRQ_EXTINT + 226)
#define NRF54L_IRQ_GRTC_1      (NRF54L_IRQ_EXTINT + 227)
#define NRF54L_IRQ_GRTC_2      (NRF54L_IRQ_EXTINT + 228)
#define NRF54L_IRQ_GRTC_3      (NRF54L_IRQ_EXTINT + 229)
#define NRF54L_IRQ_TDM         (NRF54L_IRQ_EXTINT + 232)
#define NRF54L_IRQ_SERIAL23    (NRF54L_IRQ_EXTINT + 237)
#define NRF54L_IRQ_SERIAL24    (NRF54L_IRQ_EXTINT + 238)
#define NRF54L_IRQ_SPU30       (NRF54L_IRQ_EXTINT + 256)
#define NRF54L_IRQ_SERIAL30    (NRF54L_IRQ_EXTINT + 260)
#define NRF54L_IRQ_COMP_LPCOMP (NRF54L_IRQ_EXTINT + 262)
#define NRF54L_IRQ_WDT30       (NRF54L_IRQ_EXTINT + 264)
#define NRF54L_IRQ_WDT31       (NRF54L_IRQ_EXTINT + 265)
#define NRF54L_IRQ_GPIOTE30_0  (NRF54L_IRQ_EXTINT + 268)
#define NRF54L_IRQ_GPIOTE30_1  (NRF54L_IRQ_EXTINT + 269)
#define NRF54L_IRQ_VREGUSB     (NRF54L_IRQ_EXTINT + 289)

#ifdef CONFIG_ARCH_CHIP_NRF54L15
#  define NRF54L_IRQ_AAR00_CCM00 (NRF54L_IRQ_EXTINT + 70)
#  define NRF54L_IRQ_ECB00       (NRF54L_IRQ_EXTINT + 71)
#  define NRF54L_IRQ_CRACEN      (NRF54L_IRQ_EXTINT + 72)
#  define NRF54L_IRQ_SERIAL00    (NRF54L_IRQ_EXTINT + 74)
#  define NRF54L_IRQ_RRAMC       (NRF54L_IRQ_EXTINT + 75)
#  define NRF54L_IRQ_TAMPC       (NRF54L_IRQ_EXTINT + 220)
#  define NRF54L_IRQ_CLOCK_POWER (NRF54L_IRQ_EXTINT + 261)
#  define NRF54L_IRQ_NEXTINT     (270)
#else
#  define NRF54L_IRQ_AAR00_CCM00 (NRF54L_IRQ_EXTINT + 74)
#  define NRF54L_IRQ_ECB00       (NRF54L_IRQ_EXTINT + 75)
#  define NRF54L_IRQ_SERIAL00    (NRF54L_IRQ_EXTINT + 77)
#  define NRF54L_IRQ_RRAMC       (NRF54L_IRQ_EXTINT + 78)
#  define NRF54L_IRQ_CRACEN      (NRF54L_IRQ_EXTINT + 89)
#  define NRF54L_IRQ_TAMPC       (NRF54L_IRQ_EXTINT + 239)
#  define NRF54L_IRQ_CLOCK_POWER (NRF54L_IRQ_EXTINT + 270)
#  define NRF54L_IRQ_NEXTINT     (290)
#endif

#define NRF54L_IRQ_NIRQS (NRF54L_IRQ_EXTINT + NRF54L_IRQ_NEXTINT)

/* Total number of IRQ numbers */

#define NR_IRQS NRF54L_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_NRF54L_NRF54L_IRQ_H */
