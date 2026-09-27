/****************************************************************************
 * arch/arm/src/am67/am67_pwm_hw.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_PWM_HW_H
#define __ARCH_ARM_SRC_AM67_AM67_PWM_HW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Base addresses */

#define AM67_EPWM0_BASE                 0x23000000
#define AM67_EPWM1_BASE                 0x23010000

/* Expected values */

#define AM67_EPWM_PID_EXPECTED          0x44d10903u

/* AQ Module Actions */
#define AM67_EPWM_AQ_NOP             0
#define AM67_EPWM_AQ_CLEAR           1
#define AM67_EPWM_AQ_SET             2
#define AM67_EPWM_AQ_TOGGLE          3

/* CSFA Actions */
#define AM67_EPWM_CSFA_DISABLE                0
#define AM67_EPWM_CSFA_FORCE_LOW              1
#define AM67_EPWM_CSFA_FORCE_HIGH             2
#define AM67_EPWM_CSFA_FORCE_DISABLE          3

/* CTR Modes */
#define AM67_EPWM_TBCTL_CTRMODE_UP                 0
#define AM67_EPWM_TBCTL_CTRMODE_DOWN               1
#define AM67_EPWM_TBCTL_CTRMODE_UP_DOWN            2
#define AM67_EPWM_TBCTL_CTRMODE_STOP_FREEZE        3

/* Register offsets *********************************************************/

#define AM67_EPWM_PID_OFFSET            0x05c

/* TB module */

#define AM67_EPWM_TBCTL_OFFSET          0x000
#define AM67_EPWM_TBSTS_OFFSET          0x002
#define AM67_EPWM_TBPHS_OFFSET          0x006
#define AM67_EPWM_TBCNT_OFFSET          0x008
#define AM67_EPWM_TBPRD_OFFSET          0x00a

/* CC module */

#define AM67_EPWM_CMPCTL_OFFSET         0x00e
#define AM67_EPWM_CMPA_OFFSET           0x012
#define AM67_EPWM_CMPB_OFFSET           0x014

/* AQ module */

#define AM67_EPWM_AQCTLA_OFFSET         0x016
#define AM67_EPWM_AQCTLB_OFFSET         0x018
#define AM67_EPWM_AQSFRC_OFFSET         0x01a
#define AM67_EPWM_AQCSFRC_OFFSET        0x01c

/* Register bit field definitions *******************************************/

/* Time-Base Control Register (TBCTL) */

#define AM67_EPWM_TBCTL_CTRMODE_SHIFT                   (0)
#define AM67_EPWM_TBCTL_CTRMODE_MASK                    (3u << 0)
#define AM67_EPWM_TBCTL_PHSEN_SHIFT                     (2)
#define AM67_EPWM_TBCTL_PHSEN_MASK                      (1u << 2)
#define AM67_EPWM_TBCTL_PRDLD_IMMEDIATE                 (1u << 3)
#define AM67_EPWM_TBCTL_SYNCOSEL_SHIFT                  (4)
#define AM67_EPWM_TBCTL_SYNCOSEL_MASK                   (3u << 4)
#define AM67_EPWM_TBCTL_SWFSYNC_SHIFT                   (6)
#define AM67_EPWM_TBCTL_SWFSYNC_MASK                    (1u << 6)
#define AM67_EPWM_TBCTL_HSPCLKDIV_SHIFT                 (7)
#define AM67_EPWM_TBCTL_HSPCLKDIV_MASK                  (7u << 7)
#define AM67_EPWM_TBCTL_CLKDIV_SHIFT                    (10)
#define AM67_EPWM_TBCTL_CLKDIV_MASK                     (7u << 10)
#define AM67_EPWM_TBCTL_PHSDIR_SHIFT                    (13)
#define AM67_EPWM_TBCTL_PHSDIR_MASK                     (1u << 13)
#define AM67_EPWM_TBCTL_FREE_SOFT_SHIFT                 (14)
#define AM67_EPWM_TBCTL_FREE_SOFT_MASK                  (3u << 14)

/* Counter-Compare Control Register (CMPCTL) */

#define AM67_EPWM_CMPCTL_LOADAMODE_SHIFT                (0)
#define AM67_EPWM_CMPCTL_LOADBMODE_SHIFT                (2)
#define AM67_EPWM_CMPCTL_LOADAMODE_MASK                 (3u << 0)
#define AM67_EPWM_CMPCTL_LOADBMODE_MASK                 (3u << 2)
#define AM67_EPWM_CMPCTL_SHDWAMODE_IMMEDIATE_SHIFT      (4)
#define AM67_EPWM_CMPCTL_SHDWBMODE_IMMEDIATE_SHIFT      (6)
#define AM67_EPWM_CMPCTL_SHDWAMODE_IMMEDIATE            (1u << 4)
#define AM67_EPWM_CMPCTL_SHDWBMODE_IMMEDIATE            (1u << 6)

/* AQ Registers */

#define AM67_EPWM_AQCTLA_ZRO_SHIFT                      (0)
#define AM67_EPWM_AQCTLA_ZRO_MASK                       (3u << 0)
#define AM67_EPWM_AQCTLA_CAU_SHIFT                      (4)
#define AM67_EPWM_AQCTLA_CAU_MASK                       (3u << 4)

/* AQCTLB has the same event field layout as AQCTLA; output B's compare
 * event is CBU (CMPB, up-count), bits 9:8.
 */

#define AM67_EPWM_AQCTLB_ZRO_SHIFT                      (0)
#define AM67_EPWM_AQCTLB_ZRO_MASK                       (3u << 0)
#define AM67_EPWM_AQCTLB_CBU_SHIFT                      (8)
#define AM67_EPWM_AQCTLB_CBU_MASK                       (3u << 8)

/* AQCSFRC: CSFA and CSFB share this register; per-channel writes must be
 * read-modify-write of the 2-bit field only.  The CSFA_* action codes
 * above apply to both fields.
 */

#define AM67_EPWM_AQCSFRC_CSFA_SHIFT                    (0)
#define AM67_EPWM_AQCSFRC_CSFA_MASK                     (3u << 0)
#define AM67_EPWM_AQCSFRC_CSFB_SHIFT                    (2)
#define AM67_EPWM_AQCSFRC_CSFB_MASK                     (3u << 2)
#define AM67_EPWM_AQSFRC_RLDCSF_SHIFT                   (6)
#define AM67_EPWM_AQSFRC_RLDCSF_MASK                    (3u << 6)
#define AM67_EPWM_AQSFRC_RLDCSF_IMMEDIATE               (3)

#endif /* __ARCH_ARM_SRC_AM67_AM67_PWM_HW_H */
