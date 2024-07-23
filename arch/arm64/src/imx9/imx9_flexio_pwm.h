/****************************************************************************
 * arch/arm64/src/imx9/imx9_flexio_pwm.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_PWM_H
#define __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check if PWM support for any channel is enabled. */

#ifdef CONFIG_IMX9_FLEXIO_PWM

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/board/board.h>
#include "hardware/imx9_flexio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum
{
  PWM_FLEXIO1 = 0,
  PWM_FLEXIO2 = 1,
} flexio_pwm_id_t;

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_flexio_pwm_init
 *
 * Description:
 *   Initialize a FLEXIO block for EPWM usage.
 *
 * Input Parameters:
 *   pwmid - A number identifying the pwm block.
 *
 * Returned Value:
 *   On success, a pointer to the lower half of the PWM driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *imx9_flexio_pwm_init(flexio_pwm_id_t pwmid);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_IMX9_FLEXIO_PWM */
#endif /* __ARCH_ARM64_SRC_IMX9_IMX9_FLEXIO_PWM_H */
