/****************************************************************************
 * arch/arm/src/am67/am67_ecap.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_ECAP_H
#define __ARCH_ARM_SRC_AM67_AM67_ECAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_AM67_ECAP0) || defined(CONFIG_AM67_ECAP1) || \
    defined(CONFIG_AM67_ECAP2)

/* The eCAP module is driven as an Auxiliary PWM (APWM) generator, so it
 * binds to the PWM upper half (pwm_register), not the capture upper half.
 */

#include <nuttx/timers/pwm.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am67_ecap_init
 *
 * Description:
 *   Boot-time preparation: unlock MAIN_CTRL_MMR partition 1, mirroring
 *   am67_epwm_init.  Must run before pwm_register().
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int am67_ecap_init(void);

/****************************************************************************
 * Name: am67_ecapinitialize
 *
 * Description:
 *   Return the lower-half instance for the given module so the board bringup
 *   can bind it with pwm_register().  No hardware is touched here.
 *
 * Input Parameters:
 *   ecap - eCAP module number: 0, 1 or 2.
 *
 * Returned Value:
 *   Lower-half pointer on success; NULL on an unsupported or unconfigured
 *   module number.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *am67_ecapinitialize(int ecap);

#endif /* CONFIG_AM67_ECAP0 || CONFIG_AM67_ECAP1 || CONFIG_AM67_ECAP2 */
#endif /* __ARCH_ARM_SRC_AM67_AM67_ECAP_H */
