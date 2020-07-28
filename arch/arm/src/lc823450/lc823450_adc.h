/****************************************************************************
 * arch/arm/src/lc823450/chip/lc823450_adc.h
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Masayuki Ishikawa <Masayuki.Ishikawa@jp.sony.com>
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_ADC_H
#define __ARCH_ARM_SRC_LC823450_LC823450_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Register Addresses *******************************************************/

#define ADC_REGBASE 0x40087000
#define ADC0DT     (ADC_REGBASE + 0x00)
#define ADC1DT     (ADC_REGBASE + 0x04)
#define ADC2DT     (ADC_REGBASE + 0x08)
#define ADC3DT     (ADC_REGBASE + 0x0C)
#define ADC4DT     (ADC_REGBASE + 0x10)
#define ADC5DT     (ADC_REGBASE + 0x14)
#define ADCCTL     (ADC_REGBASE + 0x28)
#define ADCSTS     (ADC_REGBASE + 0x2C)
#define ADCSMPL    (ADC_REGBASE + 0x30)
#define ADCSTBY    (ADC_REGBASE + 0x34)

/* Register Bitfield Definitions ********************************************/

/* ADC Control Register */

#define ADCCTL_ADCNTNU        (1 << 9)  /* Bit 9: ADC continuous conversion enable */
#define ADCCTL_ADACT          (1 << 8)  /* Bit 8: ADC activate enable */
#define ADCCTL_ADCHSCN        (1 << 7)  /* Bit 7: ADC channel scan enable */

#define ADCCTL_ADCNVCK_SHIFT  (4)
#define ADCCTL_ADCNVCK_DIV2   (0 << ADCCTL_ADCNVCK_SHIFT)
#define ADCCTL_ADCNVCK_DIV4   (1 << ADCCTL_ADCNVCK_SHIFT)
#define ADCCTL_ADCNVCK_DIV8   (2 << ADCCTL_ADCNVCK_SHIFT)
#define ADCCTL_ADCNVCK_DIV16  (3 << ADCCTL_ADCNVCK_SHIFT)
#define ADCCTL_ADCNVCK_DIV32  (4 << ADCCTL_ADCNVCK_SHIFT)
#define ADCCTL_ADCNVCK_DIV64  (5 << ADCCTL_ADCNVCK_SHIFT)

#define ADCCTL_ADCHST_SHIFT   (0)

/* ADC Status Register */

#define ADCSTS_ADCMPL         (1 << 0)  /* Bit 0: ADC Conversion Completion Flag */

/* ADC Standby Register */

#define ADCSTBY_STBY          (1 << 0)    /* Bit 0: Standby enable */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct adc_dev_s *lc823450_adcinitialize(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_ADC_H */
