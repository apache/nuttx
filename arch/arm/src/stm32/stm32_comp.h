/************************************************************************************
 * arch/arm/src/stm32/stm32_comp.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_COMP_H
#define __ARCH_ARM_SRC_STM32_STM32_COMP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_STM32_COMP

#if defined(CONFIG_STM32_STM32F30XX)
#  error "COMP support for STM32F30XX not implemented yet"
#elif defined(CONFIG_STM32_STM32F33XX)
#  include "chip/stm32f33xxx_comp.h"
#elif defined(CONFIG_STM32_STM32F37XX)
#  error "COMP support for STM32F37XX ot implemented yet"
#endif

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

#define COMP_BLANKING_DEFAULT COMP_BLANKING_DIS   /* No blanking */
#define COMP_POL_DEFAULT      COMP_POL_NONINVERT  /* Output is not inverted */
#define COMP_INM_DEFAULT      COMP_INMSEL_1P4VREF /* 1/4 of Vrefint as INM */
#define COMP_OUTSEL_DEFAULT   COMP_OUTSEL_NOSEL   /* Output not selected */
#define COMP_LOCK_DEFAULT     COMP_LOCK_RW        /* Do not lock CSR register */

#ifndef CONFIG_STM32_STM32F33XX
#define COMP_MODE_DEFAULT
#define COMP_HYST_DEFAULT
#define COMP_WINMODE_DEFAULT
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* Blanking source */

enum stm32_comp_blanking_e
{
  COMP_BLANKING_DIS,
#if defined(CONFIG_STM32_STM32F33XX)
  COMP_BLANKING_T1OC5,
  COMP_BLANKING_T3OC4,
  COMP_BLANKING_T2OC3,
  COMP_BLANKING_T3OC3,
  COMP_BLANKING_T15OC1,
  COMP_BLANKING_T2OC4,
  COMP_BLANKING_T15OC2,
#endif
};

/* Output polarisation */

enum stm32_comp_pol_e
{
  COMP_POL_NONINVERT,
  COMP_POL_INVERTED
};

/* Inverting input */

enum stm32_comp_inm_e
{
  COMP_INMSEL_1P4VREF,
  COMP_INMSEL_1P2VREF,
  COMP_INMSEL_3P4VREF,
  COMP_INMSEL_VREF,
  COMP_INMSEL_DAC1CH1,
  COMP_INMSEL_DAC1CH2,
  COMP_INMSEL_PIN
};

/* Output selection */

enum stm32_comp_outsel_e
{
  COMP_OUTSEL_NOSEL,
#if defined(CONFIG_STM32_STM32F33XX)
  COMP_OUTSEL_BRKACTH,
  COMP_OUTSEL_BRK2,
  COMP_OUTSEL_T1OCC,          /* COMP2 only */
  COMP_OUTSEL_T3CAP3,         /* COMP4 only */
  COMP_OUTSEL_T2CAP2,         /* COMP6 only */
  COMP_OUTSEL_T1CAP1,         /* COMP2 only */
  COMP_OUTSEL_T2CAP4,         /* COMP2 only */
  COMP_OUTSEL_T15CAP2,        /* COMP4 only */
  COMP_OUTSEL_T2OCC,          /* COMP6 only */
  COMP_OUTSEL_T16OCC,         /* COMP2 only */
  COMP_OUTSEL_T3CAP1,         /* COMP2 only */
  COMP_OUTSEL_T15OCC,         /* COMP4 only */
  COMP_OUTSEL_T16CAP1,        /* COMP6 only */
  COMP_OUTSEL_T3OCC,          /* COMP2 and COMP4 only */
#endif
};

/* CSR register lock state */

enum stm32_comp_lock_e
{
  COMP_LOCK_RW,
  COMP_LOCK_RO
};

#ifndef CONFIG_STM32_STM32F33XX

/* Hysteresis  */

enum stm32_comp_hyst_e
{
  COMP_HYST_DIS,
  COMP_HYST_LOW,
  COMP_HYST_MEDIUM,
  COMP_HYST_HIGH
};

/* Power/Speed Modes */

enum stm32_comp_mode_e
{
  COMP_MODE_HIGHSPEED,
  COMP_MODE_MEDIUMSPEED,
  COMP_MODE_LOWPOWER,
  COMP_MODE_ULTRALOWPOWER
};

/* Window mode */

enum stm32_comp_winmode_e
{
  COMP_WINMODE_DIS,
  COMP_WINMODE_EN
};

#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
* Name: stm32_compinitialize
*
* Description:
*   Initialize the COMP.
*
* Input Parameters:
*   intf - The COMP interface number.
*
* Returned Value:
*   Valid COMP device structure reference on succcess; a NULL on failure.
*
* Assumptions:
*   1. Clock to the COMP block has enabled,
*   2. Board-specific logic has already configured
*
****************************************************************************/

FAR struct comp_dev_s* stm32_compinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM23_COMP */
#endif /* __ARCH_ARM_SRC_STM32_STM32_COMP_H */
