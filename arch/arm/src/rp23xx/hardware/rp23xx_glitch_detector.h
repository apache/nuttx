/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_glitch_detector.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_GLITCH_DETECTOR_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_GLITCH_DETECTOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_GLITCH_DETECTOR_ARM_OFFSET           0x00000000
#define RP23XX_GLITCH_DETECTOR_DISARM_OFFSET        0x00000004
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_OFFSET   0x00000008
#define RP23XX_GLITCH_DETECTOR_LOCK_OFFSET          0x0000000c
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_OFFSET   0x00000010
#define RP23XX_GLITCH_DETECTOR_TRIG_FORCE_OFFSET    0x00000014

/* Register definitions *****************************************************/

#define RP23XX_GLITCH_DETECTOR_ARM          (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_ARM_OFFSET)
#define RP23XX_GLITCH_DETECTOR_DISARM       (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_DISARM_OFFSET)
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY  (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_SENSITIVITY_OFFSET)
#define RP23XX_GLITCH_DETECTOR_LOCK         (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_LOCK_OFFSET)
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS  (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_TRIG_STATUS_OFFSET)
#define RP23XX_GLITCH_DETECTOR_TRIG_FORCE   (RP23XX_GLITCH_DETECTOR_BASE + RP23XX_GLITCH_DETECTOR_TRIG_FORCE_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_GLITCH_DETECTOR_ARM_MASK                     0x0000ffff
#define RP23XX_GLITCH_DETECTOR_DISARM_MASK                  0x0000ffff
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_MASK             0xff00ffff
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DEFAULT_MASK     0xff000000
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET3_INV_MASK    0x0000c000
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET2_INV_MASK    0x00003000
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET1_INV_MASK    0x00000c00
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET0_INV_MASK    0x00000300
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET3_MASK        0x000000c0
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET2_MASK        0x00000030
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET1_MASK        0x0000000c
#define RP23XX_GLITCH_DETECTOR_SENSITIVITY_DET0_MASK        0x00000003
#define RP23XX_GLITCH_DETECTOR_LOCK_MASK                    0x000000ff
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_MASK             0x0000000f
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_DET3_MASK        0x00000008
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_DET2_MASK        0x00000004
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_DET1_MASK        0x00000002
#define RP23XX_GLITCH_DETECTOR_TRIG_STATUS_DET0_MASK        0x00000001
#define RP23XX_GLITCH_DETECTOR_TRIG_FORCE_MASK              0x0000000f

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_GLITCH_DETECTOR_H */
