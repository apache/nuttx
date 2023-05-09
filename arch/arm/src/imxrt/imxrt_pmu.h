/****************************************************************************
 * arch/arm/src/imxrt/imxrt_pmu.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_PMU_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_PMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"

typedef enum dcdc_1p0_buck_mode_target_voltage
{
    dcdc_1p0bucktarget0p6v = 0u, /* in BUCK mode, the target voltage value of vdd1p0 is 0.6v. */
    dcdc_1p0bucktarget0p625v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.625v. */
    dcdc_1p0bucktarget0p65v,     /* in BUCK mode, the target voltage value of vdd1p0 is 0.65v. */
    dcdc_1p0bucktarget0p675v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.675v. */

    dcdc_1p0bucktarget0p7v,      /* in BUCK mode, the target voltage value of vdd1p0 is 0.7v. */
    dcdc_1p0bucktarget0p725v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.725v. */
    dcdc_1p0bucktarget0p75v,     /* in BUCK mode, the target voltage value of vdd1p0 is 0.75v. */
    dcdc_1p0bucktarget0p775v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.775v. */

    dcdc_1p0bucktarget0p8v,      /* in BUCK mode, the target voltage value of vdd1p0 is 0.8v. */
    dcdc_1p0bucktarget0p825v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.825v. */
    dcdc_1p0bucktarget0p85v,     /* in BUCK mode, the target voltage value of vdd1p0 is 0.85v. */
    dcdc_1p0bucktarget0p875v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.875v. */

    dcdc_1p0bucktarget0p9v,      /* in BUCK mode, the target voltage value of vdd1p0 is 0.9v. */
    dcdc_1p0bucktarget0p925v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.925v. */
    dcdc_1p0bucktarget0p95v,     /* in BUCK mode, the target voltage value of vdd1p0 is 0.95v. */
    dcdc_1p0bucktarget0p975v,    /* in BUCK mode, the target voltage value of vdd1p0 is 0.975v. */

    dcdc_1p0bucktarget1p0v,      /* in BUCK mode, the target voltage value of vdd1p0 is 1.0v. */
    dcdc_1p0bucktarget1p025v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.025v. */
    dcdc_1p0bucktarget1p05v,     /* in BUCK mode, the target voltage value of vdd1p0 is 1.05v. */
    dcdc_1p0bucktarget1p075v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.075v. */

    dcdc_1p0bucktarget1p1v,      /* in BUCK mode, the target voltage value of vdd1p0 is 1.1v. */
    dcdc_1p0bucktarget1p125v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.125v. */
    dcdc_1p0bucktarget1p15v,     /* in BUCK mode, the target voltage value of vdd1p0 is 1.15v. */
    dcdc_1p0bucktarget1p175v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.175v. */

    dcdc_1p0bucktarget1p2v,      /* in BUCK mode, the target voltage value of vdd1p0 is 1.2v. */
    dcdc_1p0bucktarget1p225v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.225v. */
    dcdc_1p0bucktarget1p25v,     /* in BUCK mode, the target voltage value of vdd1p0 is 1.25v. */
    dcdc_1p0bucktarget1p275v,    /* in BUCK mode, the target voltage value of vdd1p0 is 1.275v. */

    dcdc_1p0bucktarget1p3v,           /* in BUCK mode, the target voltage value of vdd1p0 is 1.3v. */
    dcdc_1p0bucktarget1p325v,         /* in BUCK mode, the target voltage value of vdd1p0 is 1.325v. */
    dcdc_1p0bucktarget1p35v,          /* in BUCK mode, the target voltage value of vdd1p0 is 1.35v. */
    dcdc_1p0bucktarget1p375v = 0x1fu, /* in BUCK mode, the target voltage value of vdd1p0 is 1.375v. */
} dcdc_1p0_buck_mode_target_voltage;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void imxrt_modify_pll1g(uint8_t addr, uint32_t set, uint32_t clr);
void imxrt_pmu_enable_pll_ldo(void);
void imxrt_pmu_enable_body_bias_fbb_cm7(uint32_t enable);
void imxrt_pmu_vdd1p0_buckmode_targetvoltage(
    dcdc_1p0_buck_mode_target_voltage voltage);
void imxrt_pmu_lpsr_ana_ldo_bypassmode(uint32_t enable);
void imxrt_pmu_lpsr_dig_ldo_bypassmode(uint32_t enable);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_PMU_H */
