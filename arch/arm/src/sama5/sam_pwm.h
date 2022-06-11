/****************************************************************************
 * arch/arm/src/sama5/sam_pwm.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_SAM_PWM_H
#define __ARCH_ARM_SRC_SAMA5_SAM_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_pwm.h"

#ifdef CONFIG_SAMA5_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Do we have any PWM channels enabled?
 * If not, then why is the PWM enabled?
 */

#if !defined(CONFIG_SAMA5_PWM_CHAN0) && !defined(CONFIG_SAMA5_PWM_CHAN1) && \
    !defined(CONFIG_SAMA5_PWM_CHAN2) && !defined(CONFIG_SAMA5_PWM_CHAN3)
#  error "No PWM channels configured"
#  undef CONFIG_SAMA5_PWM
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sam_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel use.
 *
 * Returned Value:
 *   On success, a pointer to the SAMA5 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *sam_pwminitialize(int channel);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_SAMA5_PWM */
#endif /* __ARCH_ARM_SRC_SAMA5_SAM_PWM_H */
