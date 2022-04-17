/****************************************************************************
 * arch/arm/src/lc823450/lc823450_pwm.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_PWM_H
#define __ARCH_ARM_SRC_LC823450_LC823450_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Addresses *******************************************************/

#define LC823450_MTM0_REGBASE   0x40043000
#define LC823450_MTM1_REGBASE   0x40044000

#define LC823450_MTM_OPR        0x00
#define LC823450_MTM_SYNC       0x04
#define LC823450_MTM_PWM        0x08
#define LC823450_MTM_FCTL       0x0c

#define LC823450_MTM_0CTL       0x40
#define LC823450_MTM_0IOCL      0x44
#define LC823450_MTM_0STS       0x4C
#define LC823450_MTM_0CNT       0x50
#define LC823450_MTM_0A         0x54
#define LC823450_MTM_0B         0x58
#define LC823450_MTM_0SOL       0x5C
#define LC823450_MTM_0BA        0x60
#define LC823450_MTM_0BB        0x64
#define LC823450_MTM_0PSCL      0x68
#define LC823450_MTM_0TIER      0x6C
#define LC823450_MTM_0TISR      0x70

#define LC823450_MTM_1CTL       0x80
#define LC823450_MTM_1IOCL      0x84
#define LC823450_MTM_1STS       0x8C
#define LC823450_MTM_1CNT       0x90
#define LC823450_MTM_1A         0x94
#define LC823450_MTM_1B         0x98
#define LC823450_MTM_1SOL       0x9C
#define LC823450_MTM_1BA        0xA0
#define LC823450_MTM_1BB        0xA4
#define LC823450_MTM_1PSCL      0xA8
#define LC823450_MTM_1TIER      0xAC
#define LC823450_MTM_1TISR      0xB0

/* PWM Identifier ***********************************************************/

#define LC823450_PWMTIMER0_CH0  0
#define LC823450_PWMTIMER0_CH1  1
#define LC823450_PWMTIMER1_CH0  2
#define LC823450_PWMTIMER1_CH1  3

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN struct pwm_lowerhalf_s *lc823450_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_PWM_H */
