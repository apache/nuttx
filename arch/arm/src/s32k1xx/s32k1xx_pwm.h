/*****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_pwm.h
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
 *****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_PWM_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_PWM_H

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Configuration *************************************************************/

/* Timer devices may be used for different purposes.  One special purpose is
 * to generate modulated outputs for such things as motor control.  If
 * CONFIG_S32K1XX_FTMn is defined then the CONFIG_S32K1XX_FTMn_PWM must also
 * be defined to indicate that timer "n" is intended to be used for pulsed
 * output signal generation.
 */

#ifndef CONFIG_S32K1XX_FTM0
#  undef CONFIG_S32K1XX_FTM0_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM1
#  undef CONFIG_S32K1XX_FTM1_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM2
#  undef CONFIG_S32K1XX_FTM2_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM3
#  undef CONFIG_S32K1XX_FTM3_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM4
#  undef CONFIG_S32K1XX_FTM4_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM5
#  undef CONFIG_S32K1XX_FTM5_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM6
#  undef CONFIG_S32K1XX_FTM6_PWM
#endif
#ifndef CONFIG_S32K1XX_FTM7
#  undef CONFIG_S32K1XX_FTM7_PWM
#endif

/* Check if PWM support for any channel is enabled. */

#if defined(CONFIG_S32K1XX_FTM0_PWM) || defined(CONFIG_S32K1XX_FTM1_PWM) || \
    defined(CONFIG_S32K1XX_FTM2_PWM) || defined(CONFIG_S32K1XX_FTM3_PWM) || \
    defined(CONFIG_S32K1XX_FTM4_PWM) || defined(CONFIG_S32K1XX_FTM5_PWM) || \
    defined(CONFIG_S32K1XX_FTM6_PWM) || defined(CONFIG_S32K1XX_FTM7_PWM)

#include <arch/board/board.h>
#include "hardware/s32k1xx_pinmux.h"

/* For each timer that is enabled for PWM usage, we need the following
 * additional configuration settings:
 *
 * CONFIG_S32K1XX_FTMx_CHANNEL - Specifies the timer output channel {0,..,7}
 * PWM_FTMx_PINCFG - One of the values defined in s32k1*_pinmux.h.  In the
 *   case where there are multiple pin selections, the correct setting must
 *   be provided in the arch/board/board.h file.
 */

#ifdef CONFIG_S32K1XX_FTM0_PWM
#  if !defined(CONFIG_S32K1XX_FTM0_CHANNEL)
#    error "CONFIG_S32K1XX_FTM0_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 0
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH0OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 1
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH1OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 2
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH2OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 3
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH3OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 4
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH4OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 5
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH5OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 6
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH6OUT
#  elif CONFIG_S32K1XX_FTM0_CHANNEL == 7
#    define PWM_FTM0_PINCFG GPIO_FTM0_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM0_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM1_PWM
#  if !defined(CONFIG_S32K1XX_FTM1_CHANNEL)
#    error "CONFIG_S32K1XX_FTM1_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 0
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH0OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 1
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH1OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 2
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH2OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 3
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH3OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 4
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH4OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 5
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH5OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 6
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH6OUT
#  elif CONFIG_S32K1XX_FTM1_CHANNEL == 7
#    define PWM_FTM1_PINCFG GPIO_FTM1_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM1_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM2_PWM
#  if !defined(CONFIG_S32K1XX_FTM2_CHANNEL)
#    error "CONFIG_S32K1XX_FTM2_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 0
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH0OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 1
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH1OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 2
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH2OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 3
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH3OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 4
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH4OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 5
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH5OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 6
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH6OUT
#  elif CONFIG_S32K1XX_FTM2_CHANNEL == 7
#    define PWM_FTM2_PINCFG GPIO_FTM2_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM2_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM3_PWM
#  if !defined(CONFIG_S32K1XX_FTM3_CHANNEL)
#    error "CONFIG_S32K1XX_FTM3_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 0
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH0OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 1
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH1OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 2
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH2OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 3
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH3OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 4
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH4OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 5
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH5OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 6
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH6OUT
#  elif CONFIG_S32K1XX_FTM3_CHANNEL == 7
#    define PWM_FTM3_PINCFG GPIO_FTM3_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM3_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM4_PWM
#  if !defined(CONFIG_S32K1XX_FTM4_CHANNEL)
#    error "CONFIG_S32K1XX_FTM4_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 0
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH0OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 1
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH1OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 2
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH2OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 3
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH3OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 4
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH4OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 5
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH5OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 6
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH6OUT
#  elif CONFIG_S32K1XX_FTM4_CHANNEL == 7
#    define PWM_FTM4_PINCFG GPIO_FTM4_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM4_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM5_PWM
#  if !defined(CONFIG_S32K1XX_FTM5_CHANNEL)
#    error "CONFIG_S32K1XX_FTM5_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 0
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH0OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 1
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH1OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 2
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH2OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 3
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH3OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 4
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH4OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 5
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH5OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 6
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH6OUT
#  elif CONFIG_S32K1XX_FTM5_CHANNEL == 7
#    define PWM_FTM5_PINCFG GPIO_FTM5_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM5_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM6_PWM
#  if !defined(CONFIG_S32K1XX_FTM6_CHANNEL)
#    error "CONFIG_S32K1XX_FTM6_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 0
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH0OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 1
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH1OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 2
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH2OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 3
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH3OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 4
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH4OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 5
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH5OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 6
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH6OUT
#  elif CONFIG_S32K1XX_FTM6_CHANNEL == 7
#    define PWM_FTM6_PINCFG GPIO_FTM6_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM6_CHANNEL"
#  endif
#endif

#ifdef CONFIG_S32K1XX_FTM7_PWM
#  if !defined(CONFIG_S32K1XX_FTM7_CHANNEL)
#    error "CONFIG_S32K1XX_FTM7_CHANNEL must be provided"
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 0
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH0OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 1
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH1OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 2
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH2OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 3
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH3OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 4
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH4OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 5
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH5OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 6
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH6OUT
#  elif CONFIG_S32K1XX_FTM7_CHANNEL == 7
#    define PWM_FTM7_PINCFG GPIO_FTM7_CH7OUT
#  else
#    error "Unsupported value of CONFIG_S32K1XX_FTM7_CHANNEL"
#  endif
#endif

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/*****************************************************************************
 * Public Data
 *****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/*****************************************************************************
 * Public Function Prototypes
 *****************************************************************************/

/*****************************************************************************
 * Name: s32k1xx_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.
 *
 * Returned Value:
 *   On success, a pointer to the S32K1XX lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 *****************************************************************************/

FAR struct pwm_lowerhalf_s *s32k1xx_pwminitialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_S32K1XX_FTMx_PWM */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_PWM_H */
