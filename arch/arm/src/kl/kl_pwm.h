/************************************************************************************
 * arch/arm/src/kl/kl_pwm.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KL_KINETIS_PWM_H
#define __ARCH_ARM_SRC_KL_KINETIS_PWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If CONFIG_KL_TPMn
 * is defined then the CONFIG_KL_TPMn_PWM must also be defined to indicate that
 * timer "n" is intended to be used for pulsed output signal generation.
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

#include <arch/board/board.h>
#include "chip/kl_pinmux.h"

/* For each timer that is enabled for PWM usage, we need the following additional
 * configuration settings:
 *
 * CONFIG_KL_TPMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_TPMx_CHn - One of the values defined in chip/kl*_pinmap.h.  In the case
 *   where there are multiple pin selections, the correct setting must be provided
 *   in the arch/board/board.h file.
 */

#ifdef CONFIG_KL_TPM0_PWM
#  if !defined(CONFIG_KL_TPM0_CHANNEL)
#    error "CONFIG_KL_TPM0_CHANNEL must be provided"
#  elif CONFIG_KL_TPM0_CHANNEL == 0
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH0OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 1
#    define PWM_TPM0_PINCFG GPIO_TPM0_CH1OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 2
#    define PWM_TPM0_PINCFG GPIO_TPM1_CH2OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 3
#    define PWM_TPM0_PINCFG GPIO_TPM1_CH3OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 4
#    define PWM_TPM0_PINCFG GPIO_TPM1_CH4OUT
#  elif CONFIG_KL_TPM0_CHANNEL == 5
#    define PWM_TPM0_PINCFG GPIO_TPM1_CH5OUT
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

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 ************************************************************************************/

FAR struct pwm_lowerhalf_s *kl_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_KL_TPMx_PWM */
#endif /* __ARCH_ARM_SRC_KL_KINETIS_PWM_H */
