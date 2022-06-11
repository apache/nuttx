/****************************************************************************
 * arch/arm/src/imxrt/imxrt_flexpwm.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXPWM_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>

#include "hardware/imxrt_flexpwm.h"

#ifdef CONFIG_IMXRT_FLEXPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IMXRT_FLEXPWM1_MOD1
#define FLEXPWM1_MOD1 1
#else
#define FLEXPWM1_MOD1 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM1_MOD2
#define FLEXPWM1_MOD2 1
#else
#define FLEXPWM1_MOD2 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM1_MOD3
#define FLEXPWM1_MOD3 1
#else
#define FLEXPWM1_MOD3 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM1_MOD4
#define FLEXPWM1_MOD4 1
#else
#define FLEXPWM1_MOD4 0
#endif

#define FLEXPWM1_NMODULES (FLEXPWM1_MOD1 + FLEXPWM1_MOD2 + \
                           FLEXPWM1_MOD3 + FLEXPWM1_MOD4)

#ifdef CONFIG_IMXRT_FLEXPWM2_MOD1
#define FLEXPWM2_MOD1 1
#else
#define FLEXPWM2_MOD1 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM2_MOD2
#define FLEXPWM2_MOD2 1
#else
#define FLEXPWM2_MOD2 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM2_MOD3
#define FLEXPWM2_MOD3 1
#else
#define FLEXPWM2_MOD3 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM2_MOD4
#define FLEXPWM2_MOD4 1
#else
#define FLEXPWM2_MOD4 0
#endif

#define FLEXPWM2_NMODULES (FLEXPWM2_MOD1 + FLEXPWM2_MOD2 + \
                           FLEXPWM2_MOD3 + FLEXPWM2_MOD4)

#ifdef CONFIG_IMXRT_FLEXPWM3_MOD1
#define FLEXPWM3_MOD1 1
#else
#define FLEXPWM3_MOD1 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM3_MOD2
#define FLEXPWM3_MOD2 1
#else
#define FLEXPWM3_MOD2 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM3_MOD3
#define FLEXPWM3_MOD3 1
#else
#define FLEXPWM3_MOD3 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM3_MOD4
#define FLEXPWM3_MOD4 1
#else
#define FLEXPWM3_MOD4 0
#endif

#define FLEXPWM3_NMODULES (FLEXPWM3_MOD1 + FLEXPWM3_MOD2 + \
                           FLEXPWM3_MOD3 + FLEXPWM3_MOD4)

#ifdef CONFIG_IMXRT_FLEXPWM4_MOD1
#define FLEXPWM4_MOD1 1
#else
#define FLEXPWM4_MOD1 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM4_MOD2
#define FLEXPWM4_MOD2 1
#else
#define FLEXPWM4_MOD2 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM4_MOD3
#define FLEXPWM4_MOD3 1
#else
#define FLEXPWM4_MOD3 0
#endif

#ifdef CONFIG_IMXRT_FLEXPWM4_MOD4
#define FLEXPWM4_MOD4 1
#else
#define FLEXPWM4_MOD4 0
#endif

#define FLEXPWM4_NMODULES (FLEXPWM4_MOD1 + FLEXPWM4_MOD2 + \
                           FLEXPWM4_MOD3 + FLEXPWM4_MOD4)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Function: imxrt_pwminitialize
 *
 * Description:
 *   Initialize the PWM module for use with the upper level PWM driver.
 *
 * Input Parameters:
 *   pwm - a number identifying the PWM driver.
 *
 * Returned Value:
 *   A pointer to the lower half PWM driver is returned on success,
 *   NULL on failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *imxrt_pwminitialize(int pwm);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMXRT_FLEXPWM */
#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_FLEXPWM_H */
