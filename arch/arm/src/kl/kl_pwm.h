/****************************************************************************
 * arch/arm/src/kl/kl_pwm.h
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

#ifndef __ARCH_ARM_SRC_KL_KL_PWM_H
#define __ARCH_ARM_SRC_KL_KL_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_KL_TPM0_PWM)  || defined(CONFIG_KL_TPM1_PWM)  || \
    defined(CONFIG_KL_TPM2_PWM)

#include <arch/board/board.h>
#include "hardware/kl_pinmux.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.
 * If CONFIG_KL_TPMn is defined then the CONFIG_KL_TPMn_PWM must also be
 * defined to indicate that timer "n" is intended to be used for pulsed
 * output signal generation.
 */

#ifndef CONFIG_KL_TPM0
#  undef CONFIG_KL_TPM0_PWM
#endif
#ifndef CONFIG_KL_TPM1
#  undef CONFIG_KL_TPM1_PWM
#endif
#ifndef CONFIG_KL_TPM2
#  undef CONFIG_KL_TPM2_PWM
#endif

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_KL_TPM0_PWM)  || defined(CONFIG_KL_TPM1_PWM)  || \
    defined(CONFIG_KL_TPM2_PWM)

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_KL_TPMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_TPMx_CHn - One of the values defined in chip/kl*_pinmap.h.  In the
 *   case where there are multiple pin selections, the correct setting must
 *   be provided in the arch/board/board.h file.
 */

#ifdef CONFIG_KL_TPM0_PWM
#  if !defined(CONFIG_KL_TPM0_CHANNEL)
#    error "CONFIG_KL_TPM0_CHANNEL must be provided"
#  elif CONFIG_KL_TPM0_CHANNEL == 0
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH0OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 1
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH1OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 2
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH2OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 3
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH3OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 4
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH4OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 5
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH5OUT
#  else
#    error "Unsupported value of CONFIG_KL_TPM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_KL_TPM1_PWM
#  if !defined(CONFIG_KL_TPM1_CHANNEL)
#    error "CONFIG_KL_TPM1_CHANNEL must be provided"
#  elif CONFIG_KL_TPM1_CHANNEL == 0
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH0OUT
#  elif CONFIG_KL_TPM1_CHANNEL == 1
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH1OUT
#  elif CONFIG_KL_TPM1_CHANNEL == 2
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH2OUT
#  elif CONFIG_KL_TPM1_CHANNEL == 3
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH3OUT
#  elif CONFIG_KL_TPM1_CHANNEL == 4
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH4OUT
#  elif CONFIG_KL_TPM1_CHANNEL == 5
#    define PWM_TPM1_PINCFG GPIO_TPM1_CH5OUT
#  else
#    error "Unsupported value of CONFIG_KL_TPM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_KL_TPM2_PWM
#  if !defined(CONFIG_KL_TPM2_CHANNEL)
#    error "CONFIG_KL_TPM2_CHANNEL must be provided"
#  elif CONFIG_KL_TPM2_CHANNEL == 0
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH0OUT
#  elif CONFIG_KL_TPM2_CHANNEL == 1
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH1OUT
#  elif CONFIG_KL_TPM2_CHANNEL == 2
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH2OUT
#  elif CONFIG_KL_TPM2_CHANNEL == 3
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH3OUT
#  elif CONFIG_KL_TPM2_CHANNEL == 4
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH4OUT
#  elif CONFIG_KL_TPM2_CHANNEL == 5
#    define PWM_TPM2_PINCFG GPIO_TPM2_CH5OUT
#  else
#    error "Unsupported value of CONFIG_KL_TPM3_CHANNEL"
#  endif
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
 * Name: kl_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the KL lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *kl_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_KL_TPMx_PWM */
#endif /* __ARCH_ARM_SRC_KL_KL_PWM_H */
