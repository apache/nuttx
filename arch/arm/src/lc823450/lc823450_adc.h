/****************************************************************************
 * arch/arm/src/lc823450/lc823450_adc.h
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

struct adc_dev_s *lc823450_adcinitialize(void);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_LC823450_LC823450_ADC_H */
