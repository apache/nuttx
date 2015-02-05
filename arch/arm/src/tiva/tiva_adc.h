/****************************************************************************
 * arch/arm/src/tiva/tiva_adc.h
 *
 *   Copyright (C) 2015 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_TIVA_ADC_H
#define __ARCH_ARM_SRC_TIVA_TIVA_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/analog/adc.h>
#include <nuttx/fs/ioctl.h>

#include "chip.h"
#include "chip/tiva_adc.h"

#ifdef CONFIG_TIVA_ADC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define TIVA_ADC_PWM_TRIG_IOCTL _ANIOC(0x00F0)

/* PWM trigger support definitions *************************************************/

#define ADC_TRIG_PWM_MASK \
    (0xE0000000 & ADC_EMUX_PWM0 & ADC_EMUX_PWM1 & ADC_EMUX_PWM2 & ADC_EMUX_PWM3)

/* Encodes the PWM generator (0 to 3) value to the 2 MSB's of 32-bits;
 * PWM module (0 or 1) value in the 3rd MSB:
 *      0bGGM0.0000.0000.0000,0000.0000.0000.0000
 * To be OR'd with a trigger register value.
 */

#define PWM_GEN_SHIFT (31)
#define PWM_MOD_SHIFT (30)
#  define ADC_TRIG_PWM_IOCTL(gen,mod) \
    (((gen) << PWM_GEN_SHIFT) | ((mod) << PWM_MOD_SHIFT))

/* Decodes PWM generator number from trigger */

#define PWM_GEN_MASK             (0xC0000000)
#  define ADC_TRIG_PWM_GEN(trig) \
    (((trig)&PWM_GEN_MASK)>>(PWM_GEN_SHIFT-1))

/* Decodes PWM module number from trigger */

#define PWM_MOD_MASK             (0x20000000)
#  define ADC_TRIG_PWM_MOD(trig) \
    (((trig)&PWM_MOD_MASK)>>(PWM_MOD_SHIFT-1))

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__
#  ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#  else
#  define EXTERN extern
#endif

/****************************************************************************
 * Name: tiva_adc_initialize
 *
 * Description:
 *   Initialize the ADC
 *
 * Returned Value:
 *   Valid can device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *tiva_adc_initialize(int adc_num);

/****************************************************************************
 * Interfaces exported from the ADC driver
 ****************************************************************************/

struct tiva_adc_s;

/****************************************************************************
 * Name: tiva_adc_lock
 *
 * Description:
 *   Get exclusive access to the ADC interface
 *
 ****************************************************************************/

void tiva_adc_lock(FAR struct tiva_adc_s *priv, int sse);

/****************************************************************************
 * Name: tiva_adc_unlock
 *
 * Description:
 *   Relinquish the lock on the ADC interface
 *
 ****************************************************************************/

void tiva_adc_unlock(FAR struct tiva_adc_s *priv, int sse);

/****************************************************************************
 * Name: tiva_adc_getreg
 *
 * Description:
 *   Read any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC_REGDEBUG
uint32_t tiva_adc_getreg(FAR struct tiva_adc_s *priv, uintptr_t address);
#else
#  define tiva_adc_getreg(handle,addr) getreg32(addr)
#endif

/****************************************************************************
 * Name: tiva_adc_putreg
 *
 * Description:
 *   Write to any 32-bit register using an absolute address.
 *
 ****************************************************************************/

#ifdef CONFIG_TIVA_ADC_REGDEBUG
void tiva_adc_putreg(FAR struct tiva_adc_s *priv, uintptr_t address,
                    uint32_t regval);
#else
#  define tiva_adc_putreg(handle,addr,val) putreg32(val,addr)
#endif

#  undef EXTERN
#  ifdef __cplusplus
}
#  endif
#endif /* __ASSEMBLY__ */
#endif /* CONFIG_TIVA_ADC */
#endif /* __ARCH_ARM_SRC_TIVA_TIVA_ADC_H */
