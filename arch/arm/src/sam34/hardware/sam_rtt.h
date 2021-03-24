/****************************************************************************
 * arch/arm/src/sam34/hardware/sam_rtt.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTT_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTT register offsets *****************************************************/

#define SAM_RTT_MR_OFFSET      0x00 /* Mode Register */
#define SAM_RTT_AR_OFFSET      0x04 /* Alarm Register */
#define SAM_RTT_VR_OFFSET      0x08 /* Value Register */
#define SAM_RTT_SR_OFFSET      0x0c /* Status Register */

/* RTT register addresses ***************************************************/

#define SAM_RTT_MR             (SAM_RTT_BASE+SAM_RTT_MR_OFFSET)
#define SAM_RTT_AR             (SAM_RTT_BASE+SAM_RTT_AR_OFFSET)
#define SAM_RTT_VR             (SAM_RTT_BASE+SAM_RTT_VR_OFFSET)
#define SAM_RTT_SR             (SAM_RTT_BASE+SAM_RTT_SR_OFFSET)

/* RTT register bit definitions *********************************************/

/* Real-time Timer Mode Register */

#define RTT_MR_RTPRES_SHIFT    (0)       /* Bits 0-15:  Real-time Timer Prescaler Value */
#define RTT_MR_RTPRES_MASK     (0xffff << RTT_MR_RTPRES_SHIFT)
#  define RTT_MR_RTPRES(n)     ((uint32_t)(n) << RTT_MR_RTPRES_SHIFT)
#define RTT_MR_ALMIEN          (1 << 16) /* Bit 16: Alarm Interrupt Enable */
#define RTT_MR_RTTINCIEN       (1 << 17) /* Bit 17: Real-time Timer Increment Int Enable */
#define RTT_MR_RTTRST          (1 << 18) /* Bit 18: Real-time Timer Restart */

#if defined(CONFIG_ARCH_CHIP_SAM4S) || defined(CONFIG_ARCH_CHIP_SAM4E)
#  define RTT_MR_RTTDIS        (1 << 20) /* Bit 20: Real-time Timer Disable */
#  define RTT_MR_RTC1HZ        (1 << 24) /* Bit 24: Real-Time Clock 1Hz Clock Selection */
#endif

/* Real-time Timer Alarm Register (32-bit alarm value) */

/* Real-time Timer Value Register (32-bit timer value) */

/* Real-time Timer Status Register */

#define RTT_SR_ALMS            (1 << 0)  /* Bit 0:  Real-time Alarm Status */
#define RTT_SR_RTTINC          (1 << 1)  /* Bit 1:  Real-time Timer Increment */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM_RTT_H */
