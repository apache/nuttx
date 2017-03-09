/************************************************************************************
 * arch/arm/src/kinetis/kinetis_pwm.h
 *
 *   Copyright (C) 2013, 2016, 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Alan Carvalho de Assis <acassis@gmail.com>
 *            Ken Fazzone <kfazz01@gmail.com>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_PWM_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_PWM_H

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
 * to generate modulated outputs for such things as motor control.  If CONFIG_KINETIS_FTMn
 * is defined then the CONFIG_KINETIS_FTMn_PWM must also be defined to indicate that
 * timer "n" is intended to be used for pulsed output signal generation.
 */

#ifndef CONFIG_KINETIS_FTM0
#  undef CONFIG_KINETIS_FTM0_PWM
#endif
#ifndef CONFIG_KINETIS_FTM1
#  undef CONFIG_KINETIS_FTM1_PWM
#endif
#ifndef CONFIG_KINETIS_FTM2
#  undef CONFIG_KINETIS_FTM2_PWM
#endif
#ifndef CONFIG_KINETIS_FTM3
#  undef CONFIG_KINETIS_FTM3_PWM
#endif

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_KINETIS_FTM0_PWM)  || defined(CONFIG_KINETIS_FTM1_PWM)  || \
    defined(CONFIG_KINETIS_FTM2_PWM)  || defined(CONFIG_KINETIS_FTM3_PWM)

#include <arch/board/board.h>
#include "chip/kinetis_pinmux.h"

/* For each timer that is enabled for PWM usage, we need the following additional
 * configuration settings:
 *
 * CONFIG_KINETIS_FTMx_CHANNEL - Specifies the timer output channel {1,..,4}
 * PWM_FTMx_CHn - One of the values defined in kinetis*_pinmap.h.  In the case
 *   where there are multiple pin selections, the correct setting must be provided
 *   in the arch/board/board.h file.
 */

#ifdef CONFIG_KINETIS_FTM0_PWM
#  if !defined(CONFIG_KINETIS_FTM0_CHANNEL)
#    error "CONFIG_KINETIS_FTM0_CHANNEL must be provided"
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 0
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH0OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 1
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH1OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 2
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH2OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 3
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH3OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 4
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH4OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 5
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH5OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 6
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH6OUT
#  elif CONFIG_KINETIS_FTM0_CHANNEL == 7
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH7OUT
#  else
#    error "Unsupported value of CONFIG_KINETIS_FTM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_KINETIS_FTM1_PWM
#  if !defined(CONFIG_KINETIS_FTM1_CHANNEL)
#    error "CONFIG_KINETIS_FTM1_CHANNEL must be provided"
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 0
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH0OUT
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 1
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH1OUT
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 2
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH2OUT
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 3
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH3OUT
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 4
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH4OUT
#  elif CONFIG_KINETIS_FTM1_CHANNEL == 5
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH5OUT
#  else
#    error "Unsupported value of CONFIG_KINETIS_FTM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_KINETIS_FTM2_PWM
#  if !defined(CONFIG_KINETIS_FTM2_CHANNEL)
#    error "CONFIG_KINETIS_FTM2_CHANNEL must be provided"
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 0
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH0OUT
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 1
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH1OUT
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 2
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH2OUT
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 3
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH3OUT
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 4
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH4OUT
#  elif CONFIG_KINETIS_FTM2_CHANNEL == 5
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH5OUT
#  else
#    error "Unsupported value of CONFIG_KINETIS_FTM3_CHANNEL"
#  endif
#endif

#ifdef CONFIG_KINETIS_FTM3_PWM
#  if !defined(CONFIG_KINETIS_FTM3_CHANNEL)
#    error "CONFIG_KINETIS_FTM3_CHANNEL must be provided"
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 0
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH0OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 1
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH1OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 2
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH2OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 3
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH3OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 4
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH4OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 5
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH5OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 6
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH6OUT
#  elif CONFIG_KINETIS_FTM3_CHANNEL == 7
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH7OUT
#  else
#    error "Unsupported value of CONFIG_KINETIS_FTM3_CHANNEL"
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
 * Name: kinetis_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the kinetis lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ************************************************************************************/

FAR struct pwm_lowerhalf_s *kinetis_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_KINETIS_FTMx_PWM */
#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_PWM_H */
